#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <netdb.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <survive.h>

#ifndef UMI_DEFAULT_SURVIVE_PLUGIN_DIR
#define UMI_DEFAULT_SURVIVE_PLUGIN_DIR ""
#endif

#define MAX_OBJECTS 32
#define MAX_PACKET_BYTES 16384
#define MAX_CONFIG_BYTES (1024 * 1024)

typedef enum {
    POSE_ACCEPTED = 0,
    POSE_REJECTED_NONFINITE,
    POSE_REJECTED_TIMECODE,
    POSE_REJECTED_CONFIDENCE,
    POSE_REJECTED_OPTICAL_CLOCK,
    POSE_REJECTED_OPTICAL_AGE,
    POSE_REJECTED_QUATERNION,
} PoseRejectionReason;

typedef struct {
    SurviveObject *object;
    bool connected;
    bool configured;
    bool reacquiring;
    bool has_candidate;
    uint64_t last_valid_monotonic_ns;
    uint64_t continuity_id;
    uint64_t rejected_pose_count;
    PoseRejectionReason last_rejection_reason;
    survive_long_timecode timecode;
    survive_long_timecode candidate_timecode;
    survive_long_timecode reacquire_start_timecode;
    SurvivePose pose;
    SurvivePose candidate_pose;
} ObjectState;

typedef struct {
    char target_host[256];
    char config_dir[1024];
    char poser[64];
    int target_port;
    double rate_hz;
    double max_pose_age_s;
    double max_optical_age_s;
    double max_linear_speed_m_s;
    double max_angular_speed_rad_s;
    double reacquire_s;
    bool calibrate;
    bool self_test;
} Options;

static volatile sig_atomic_t keep_running = 1;
static volatile sig_atomic_t signal_received = 0;
static ObjectState objects[MAX_OBJECTS];
static pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
static int udp_socket_fd = -1;
static struct sockaddr_storage target_address;
static socklen_t target_address_length;
static Options options;
static uint64_t packet_sequence;

static uint64_t clock_ns(clockid_t clock_id) {
    struct timespec value = {0};
    if (clock_gettime(clock_id, &value) != 0) {
        return 0;
    }
    return (uint64_t)value.tv_sec * 1000000000ULL + (uint64_t)value.tv_nsec;
}

static void handle_signal(int signal_number) {
    (void)signal_number;
    signal_received = 1;
    keep_running = 0;
}

static void log_to_stderr(SurviveContext *context, SurviveLogLevel level, const char *message) {
    (void)context;
    if (!options.calibrate && level != SURVIVE_LOG_LEVEL_ERROR &&
        level != SURVIVE_LOG_LEVEL_WARNING) {
        return;
    }
    if (signal_received && level == SURVIVE_LOG_LEVEL_WARNING &&
        strstr(message, "Libusb poll failed") != NULL) {
        return;
    }
    const char *name = "info";
    if (level == SURVIVE_LOG_LEVEL_ERROR) {
        name = "error";
    } else if (level == SURVIVE_LOG_LEVEL_WARNING) {
        name = "warning";
    }
    fprintf(stderr, "libsurvive[%s]: %s\n", name, message);
    fflush(stderr);
}

static ObjectState *state_for(SurviveObject *object, bool create) {
    ObjectState *empty = NULL;
    for (size_t index = 0; index < MAX_OBJECTS; ++index) {
        if (objects[index].object == object) {
            return &objects[index];
        }
        if (empty == NULL && objects[index].object == NULL) {
            empty = &objects[index];
        }
    }
    if (create && empty != NULL) {
        empty->object = object;
        return empty;
    }
    return NULL;
}

static bool stable_tracker_serial(const SurviveObject *object) {
    return object != NULL && strncmp(object->serial_number, "LHR-", 4) == 0;
}

static bool finite_pose(const SurvivePose *pose, double *quaternion_norm) {
    double norm_squared = 0.0;
    for (size_t index = 0; index < 3; ++index) {
        if (!isfinite(pose->Pos[index])) {
            return false;
        }
    }
    for (size_t index = 0; index < 4; ++index) {
        if (!isfinite(pose->Rot[index])) {
            return false;
        }
        norm_squared += pose->Rot[index] * pose->Rot[index];
    }
    *quaternion_norm = sqrt(norm_squared);
    return true;
}

static double optical_age_seconds(survive_long_timecode timecode,
                                  survive_long_timecode last_light, int32_t timebase_hz) {
    if (timebase_hz > 0 && last_light > 0) {
        /* The threaded poser can report a pose while the USB thread has already advanced
         * last_light. That means fresh optical data, not a stale pose. */
        survive_long_timecode age_ticks = timecode > last_light ? timecode - last_light : 0;
        return (double)age_ticks / (double)timebase_hz;
    }
    return INFINITY;
}

static PoseRejectionReason pose_rejection_reason(
    const SurvivePose *pose, survive_long_timecode timecode, survive_long_timecode last_light,
    int32_t timebase_hz, double confidence, double *quaternion_norm, double *optical_age_s) {
    *optical_age_s = optical_age_seconds(timecode, last_light, timebase_hz);
    if (!finite_pose(pose, quaternion_norm)) {
        return POSE_REJECTED_NONFINITE;
    }
    if (timecode <= 0) {
        return POSE_REJECTED_TIMECODE;
    }
    if (!isfinite(confidence) || confidence <= 0.0) {
        return POSE_REJECTED_CONFIDENCE;
    }
    if (timebase_hz <= 0 || last_light <= 0) {
        return POSE_REJECTED_OPTICAL_CLOCK;
    }
    if (*optical_age_s > options.max_optical_age_s) {
        return POSE_REJECTED_OPTICAL_AGE;
    }
    if (*quaternion_norm < 0.99 || *quaternion_norm > 1.01) {
        return POSE_REJECTED_QUATERNION;
    }
    return POSE_ACCEPTED;
}

static bool pose_is_valid(const SurvivePose *pose, survive_long_timecode timecode,
                          survive_long_timecode last_light, int32_t timebase_hz,
                          double confidence) {
    double quaternion_norm = 0.0;
    double optical_age_s = INFINITY;
    return pose_rejection_reason(pose, timecode, last_light, timebase_hz, confidence,
                                 &quaternion_norm, &optical_age_s) == POSE_ACCEPTED;
}

static const char *pose_rejection_name(PoseRejectionReason reason) {
    switch (reason) {
    case POSE_REJECTED_NONFINITE:
        return "non-finite pose";
    case POSE_REJECTED_TIMECODE:
        return "invalid timecode";
    case POSE_REJECTED_CONFIDENCE:
        return "non-positive confidence";
    case POSE_REJECTED_OPTICAL_CLOCK:
        return "missing optical clock";
    case POSE_REJECTED_OPTICAL_AGE:
        return "stale optical data";
    case POSE_REJECTED_QUATERNION:
        return "invalid quaternion norm";
    case POSE_ACCEPTED:
        return "accepted";
    }
    return "unknown";
}

static double quaternion_angle_rad(const SurvivePose *left, const SurvivePose *right) {
    double left_norm_squared = 0.0;
    double right_norm_squared = 0.0;
    double dot = 0.0;
    for (size_t index = 0; index < 4; ++index) {
        left_norm_squared += left->Rot[index] * left->Rot[index];
        right_norm_squared += right->Rot[index] * right->Rot[index];
        dot += left->Rot[index] * right->Rot[index];
    }
    double denominator = sqrt(left_norm_squared * right_norm_squared);
    if (!isfinite(denominator) || denominator <= 0.0) {
        return INFINITY;
    }
    double normalized_dot = fabs(dot / denominator);
    normalized_dot = fmin(1.0, fmax(0.0, normalized_dot));
    return 2.0 * acos(normalized_dot);
}

static bool pose_step_is_plausible(const SurvivePose *previous,
                                   survive_long_timecode previous_timecode,
                                   const SurvivePose *current,
                                   survive_long_timecode current_timecode, int32_t timebase_hz,
                                   double *linear_speed_m_s,
                                   double *angular_speed_rad_s) {
    *linear_speed_m_s = INFINITY;
    *angular_speed_rad_s = INFINITY;
    if (timebase_hz <= 0 || current_timecode <= previous_timecode) {
        return false;
    }
    double delta_s = (double)(current_timecode - previous_timecode) / (double)timebase_hz;
    if (!isfinite(delta_s) || delta_s <= 0.0 || delta_s > options.max_pose_age_s) {
        return false;
    }
    double distance_squared = 0.0;
    for (size_t index = 0; index < 3; ++index) {
        double delta = current->Pos[index] - previous->Pos[index];
        distance_squared += delta * delta;
    }
    *linear_speed_m_s = sqrt(distance_squared) / delta_s;
    *angular_speed_rad_s = quaternion_angle_rad(previous, current) / delta_s;
    return isfinite(*linear_speed_m_s) && isfinite(*angular_speed_rad_s) &&
           *linear_speed_m_s <= options.max_linear_speed_m_s &&
           *angular_speed_rad_s <= options.max_angular_speed_rad_s;
}

static void begin_reacquisition(ObjectState *state, const char *serial, const char *reason,
                                double linear_speed_m_s, double angular_speed_rad_s) {
    if (!state->reacquiring) {
        state->continuity_id++;
        fprintf(stderr,
                "Vive tracker %s continuity lost (%s, linear=%.3fm/s angular=%.1fdeg/s); "
                "publishing invalid until %.0fms stable\n",
                serial, reason, linear_speed_m_s, angular_speed_rad_s * 180.0 / acos(-1.0),
                options.reacquire_s * 1000.0);
    }
    state->reacquiring = true;
    state->has_candidate = false;
    state->last_valid_monotonic_ns = 0;
    state->rejected_pose_count++;
}

static void set_reacquisition_candidate(ObjectState *state, survive_long_timecode timecode,
                                        const SurvivePose *pose) {
    state->candidate_pose = *pose;
    state->candidate_timecode = timecode;
    state->reacquire_start_timecode = timecode;
    state->has_candidate = true;
}

static bool append_format(char **cursor, size_t *remaining, const char *format, ...) {
    va_list arguments;
    va_start(arguments, format);
    int written = vsnprintf(*cursor, *remaining, format, arguments);
    va_end(arguments);
    if (written < 0 || (size_t)written >= *remaining) {
        return false;
    }
    *cursor += written;
    *remaining -= (size_t)written;
    return true;
}

static bool append_json_string(char **cursor, size_t *remaining, const char *value) {
    if (!append_format(cursor, remaining, "\"")) {
        return false;
    }
    for (const unsigned char *item = (const unsigned char *)value; *item != '\0'; ++item) {
        if (*item == '\"' || *item == '\\') {
            if (!append_format(cursor, remaining, "\\%c", *item)) {
                return false;
            }
        } else if (*item < 0x20) {
            if (!append_format(cursor, remaining, "\\u%04x", *item)) {
                return false;
            }
        } else if (!append_format(cursor, remaining, "%c", *item)) {
            return false;
        }
    }
    return append_format(cursor, remaining, "\"");
}

static size_t build_packet(char *buffer, size_t capacity, uint64_t now_monotonic_ns,
                           uint64_t now_realtime_ns) {
    char *cursor = buffer;
    size_t remaining = capacity;
    double unix_seconds = (double)now_realtime_ns / 1000000000.0;
    if (!append_format(&cursor, &remaining,
                       "{\"t\":%.9f,\"seq\":%llu,\"frame\":\"libsurvive_tracking\","
                       "\"trackers\":[",
                       unix_seconds, (unsigned long long)packet_sequence++)) {
        return 0;
    }

    bool first = true;
    pthread_mutex_lock(&state_mutex);
    for (size_t index = 0; index < MAX_OBJECTS; ++index) {
        ObjectState *state = &objects[index];
        if (state->object == NULL || !state->configured ||
            !stable_tracker_serial(state->object)) {
            continue;
        }
        double pose_age_s = INFINITY;
        if (state->last_valid_monotonic_ns > 0 &&
            now_monotonic_ns >= state->last_valid_monotonic_ns) {
            pose_age_s = (double)(now_monotonic_ns - state->last_valid_monotonic_ns) /
                         1000000000.0;
        }
        bool valid = state->connected && !state->reacquiring && isfinite(pose_age_s) &&
                     pose_age_s <= options.max_pose_age_s;
        double quaternion_norm = 0.0;
        valid = valid && finite_pose(&state->pose, &quaternion_norm) &&
                quaternion_norm >= 0.99 && quaternion_norm <= 1.01;

        if (!first && !append_format(&cursor, &remaining, ",")) {
            pthread_mutex_unlock(&state_mutex);
            return 0;
        }
        first = false;
        if (!append_format(&cursor, &remaining, "{\"serial\":") ||
            !append_json_string(&cursor, &remaining, state->object->serial_number) ||
            !append_format(&cursor, &remaining,
                           ",\"valid\":%s,\"continuity_id\":%llu,\"rejected_poses\":%llu,"
                           "\"timebase_hz\":%d",
                           valid ? "true" : "false",
                           (unsigned long long)state->continuity_id,
                           (unsigned long long)state->rejected_pose_count,
                           state->object->timebase_hz)) {
            pthread_mutex_unlock(&state_mutex);
            return 0;
        }
        if (valid) {
            const double inverse_norm = 1.0 / quaternion_norm;
            if (!append_format(
                    &cursor, &remaining,
                    ",\"pose_age_s\":%.9f,\"device_timecode\":%llu,"
                    "\"pos\":[%.12g,%.12g,%.12g],\"quat\":[%.12g,%.12g,%.12g,%.12g]",
                    pose_age_s, (unsigned long long)state->timecode, state->pose.Pos[0],
                    state->pose.Pos[1], state->pose.Pos[2],
                    state->pose.Rot[0] * inverse_norm, state->pose.Rot[1] * inverse_norm,
                    state->pose.Rot[2] * inverse_norm, state->pose.Rot[3] * inverse_norm)) {
                pthread_mutex_unlock(&state_mutex);
                return 0;
            }
        }
        if (!append_format(&cursor, &remaining, "}")) {
            pthread_mutex_unlock(&state_mutex);
            return 0;
        }
    }
    pthread_mutex_unlock(&state_mutex);

    if (!append_format(&cursor, &remaining, "]}")) {
        return 0;
    }
    return (size_t)(cursor - buffer);
}

static void *sender_loop(void *unused) {
    (void)unused;
    const uint64_t period_ns = (uint64_t)llround(1000000000.0 / options.rate_hz);
    uint64_t next_send_ns = clock_ns(CLOCK_MONOTONIC);
    uint64_t sent = 0;
    uint64_t errors = 0;
    while (keep_running) {
        uint64_t monotonic_ns = clock_ns(CLOCK_MONOTONIC);
        uint64_t realtime_ns = clock_ns(CLOCK_REALTIME);
        char packet[MAX_PACKET_BYTES];
        size_t packet_size = build_packet(packet, sizeof(packet), monotonic_ns, realtime_ns);
        if (packet_size == 0) {
            fprintf(stderr, "failed to build Vive packet: buffer exhausted\n");
            keep_running = 0;
            break;
        }
        ssize_t result = sendto(udp_socket_fd, packet, packet_size, 0,
                                (const struct sockaddr *)&target_address,
                                target_address_length);
        if (result == (ssize_t)packet_size) {
            sent++;
        } else {
            errors++;
            if (errors == 1 || errors % 100 == 0) {
                fprintf(stderr, "Vive UDP send failed: %s\n", strerror(errno));
            }
        }

        next_send_ns += period_ns;
        uint64_t now_ns = clock_ns(CLOCK_MONOTONIC);
        if (next_send_ns <= now_ns) {
            next_send_ns = now_ns + period_ns;
        }
        struct timespec deadline = {
            .tv_sec = (time_t)(next_send_ns / 1000000000ULL),
            .tv_nsec = (long)(next_send_ns % 1000000000ULL),
        };
        while (keep_running && clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, NULL) ==
                                   EINTR) {
        }
    }
    fprintf(stderr, "Vive UDP sender stopped: sent=%llu errors=%llu\n",
            (unsigned long long)sent, (unsigned long long)errors);
    return NULL;
}

static void new_object_process(SurviveObject *object) {
    survive_default_new_object_process(object);
    pthread_mutex_lock(&state_mutex);
    ObjectState *state = state_for(object, true);
    if (state != NULL) {
        state->connected = true;
    }
    pthread_mutex_unlock(&state_mutex);
}

static void disconnect_process(SurviveObject *object) {
    pthread_mutex_lock(&state_mutex);
    ObjectState *state = state_for(object, false);
    if (state != NULL) {
        state->connected = false;
        state->last_valid_monotonic_ns = 0;
        if (state->configured && keep_running) {
            begin_reacquisition(state, object->serial_number, "device disconnected", INFINITY,
                                INFINITY);
        }
    }
    pthread_mutex_unlock(&state_mutex);
    survive_default_disconnect_process(object);
}

static int config_process(SurviveObject *object, char *config, int length) {
    int result = survive_default_config_process(object, config, length);
    pthread_mutex_lock(&state_mutex);
    ObjectState *state = state_for(object, true);
    if (state != NULL) {
        state->connected = true;
        state->configured = stable_tracker_serial(object);
    }
    if (stable_tracker_serial(object)) {
        fprintf(stderr, "configured Vive tracker %s (%s)\n", object->serial_number,
                SurviveObjectSubtypeStr(object->object_subtype));
    }
    pthread_mutex_unlock(&state_mutex);
    return result;
}

static void pose_process(SurviveObject *object, survive_long_timecode timecode,
                         const SurvivePose *pose) {
    survive_default_pose_process(object, timecode, pose);
    survive_long_timecode last_light = object->activations.last_light;
    int32_t timebase_hz = object->timebase_hz;
    double confidence = object->poseConfidence;
    double quaternion_norm = 0.0;
    double optical_age_s = INFINITY;
    PoseRejectionReason rejection =
        pose_rejection_reason(pose, timecode, last_light, timebase_hz, confidence,
                              &quaternion_norm, &optical_age_s);
    pthread_mutex_lock(&state_mutex);
    ObjectState *state = state_for(object, true);
    if (state != NULL) {
        state->connected = true;
        state->configured = stable_tracker_serial(object);
        if (!state->configured) {
            pthread_mutex_unlock(&state_mutex);
            return;
        }
        if (rejection != POSE_ACCEPTED) {
            if (rejection != state->last_rejection_reason) {
                fprintf(stderr,
                        "Vive tracker %s pose callback rejected (%s, confidence=%.6g "
                        "optical_age=%.6gs timecode=%llu last_light=%llu qnorm=%.6g)\n",
                        object->serial_number, pose_rejection_name(rejection), confidence,
                        optical_age_s, (unsigned long long)timecode,
                        (unsigned long long)last_light, quaternion_norm);
            }
            state->last_rejection_reason = rejection;
            pthread_mutex_unlock(&state_mutex);
            return;
        }
        state->last_rejection_reason = POSE_ACCEPTED;

        if (state->last_valid_monotonic_ns == 0 && !state->reacquiring) {
            begin_reacquisition(state, object->serial_number, "initial acquisition", 0.0, 0.0);
        }

        if (state->reacquiring) {
            if (!state->has_candidate) {
                set_reacquisition_candidate(state, timecode, pose);
            } else {
                double linear_speed_m_s = INFINITY;
                double angular_speed_rad_s = INFINITY;
                bool plausible = pose_step_is_plausible(
                    &state->candidate_pose, state->candidate_timecode, pose, timecode,
                    object->timebase_hz, &linear_speed_m_s, &angular_speed_rad_s);
                if (!plausible) {
                    state->rejected_pose_count++;
                    set_reacquisition_candidate(state, timecode, pose);
                } else {
                    state->candidate_pose = *pose;
                    state->candidate_timecode = timecode;
                    double stable_s =
                        (double)(timecode - state->reacquire_start_timecode) /
                        (double)object->timebase_hz;
                    if (stable_s >= options.reacquire_s) {
                        state->pose = state->candidate_pose;
                        state->timecode = state->candidate_timecode;
                        state->last_valid_monotonic_ns = clock_ns(CLOCK_MONOTONIC);
                        state->reacquiring = false;
                        state->has_candidate = false;
                        fprintf(stderr, "Vive tracker %s continuity reacquired after %.0fms\n",
                                object->serial_number, stable_s * 1000.0);
                    }
                }
            }
        } else {
            double linear_speed_m_s = INFINITY;
            double angular_speed_rad_s = INFINITY;
            bool plausible = pose_step_is_plausible(
                &state->pose, state->timecode, pose, timecode, object->timebase_hz,
                &linear_speed_m_s, &angular_speed_rad_s);
            if (!plausible) {
                begin_reacquisition(state, object->serial_number, "impossible pose step",
                                    linear_speed_m_s, angular_speed_rad_s);
                set_reacquisition_candidate(state, timecode, pose);
            } else {
                state->last_valid_monotonic_ns = clock_ns(CLOCK_MONOTONIC);
                state->timecode = timecode;
                state->pose = *pose;
            }
        }
    }
    pthread_mutex_unlock(&state_mutex);
}

static int resolve_target(const char *host, int port) {
    char service[16];
    snprintf(service, sizeof(service), "%d", port);
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_DGRAM,
    };
    struct addrinfo *addresses = NULL;
    int result = getaddrinfo(host, service, &hints, &addresses);
    if (result != 0 || addresses == NULL) {
        fprintf(stderr, "cannot resolve target %s:%d: %s\n", host, port,
                gai_strerror(result));
        return -1;
    }
    if (addresses->ai_addrlen > sizeof(target_address)) {
        freeaddrinfo(addresses);
        return -1;
    }
    memcpy(&target_address, addresses->ai_addr, addresses->ai_addrlen);
    target_address_length = (socklen_t)addresses->ai_addrlen;
    freeaddrinfo(addresses);

    udp_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_fd < 0) {
        perror("socket");
        return -1;
    }
    int enabled = 1;
    if (setsockopt(udp_socket_fd, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled)) != 0) {
        perror("setsockopt(SO_BROADCAST)");
        close(udp_socket_fd);
        udp_socket_fd = -1;
        return -1;
    }
    return 0;
}

static int mkdir_if_needed(const char *path) {
    if (mkdir(path, 0755) == 0 || errno == EEXIST) {
        return 0;
    }
    fprintf(stderr, "cannot create config directory %s: %s\n", path, strerror(errno));
    return -1;
}

static bool completed_calibration_exists(const char *config_dir) {
    char path[1200];
    int length = snprintf(path, sizeof(path), "%s/libsurvive/config.json", config_dir);
    if (length < 0 || (size_t)length >= sizeof(path)) {
        return false;
    }
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        return false;
    }
    char *contents = malloc(MAX_CONFIG_BYTES + 1);
    if (contents == NULL) {
        fclose(file);
        return false;
    }
    size_t size = fread(contents, 1, MAX_CONFIG_BYTES, file);
    fclose(file);
    contents[size] = '\0';
    bool complete = strstr(contents, "\"lighthouse0\"") != NULL &&
                    strstr(contents, "\"PositionSet\":\"1\"") != NULL;
    free(contents);
    return complete;
}

static void print_usage(FILE *stream, const char *program) {
    fprintf(stream,
            "Usage: %s [options]\n"
            "  --target-host HOST          UDP destination (default 127.0.0.1)\n"
            "  --target-port PORT          UDP destination port (default 9001)\n"
            "  --rate-hz HZ                Packet rate, 1-240 (default 60)\n"
            "  --max-pose-age-ms MS        Hold last optically valid pose this long (default 200)\n"
            "  --max-optical-age-ms MS     Reject poses older than this optically (default 200)\n"
            "  --max-linear-speed-m-s MPS  Reject faster position steps (default 8)\n"
            "  --max-angular-speed-deg-s D Reject faster orientation steps (default 1440)\n"
            "  --reacquire-ms MS           Stable tracking required after rejection (default 250)\n"
            "  --poser NAME                KalmanOnly (default) or MPFIT\n"
            "  --config-dir PATH           Isolated XDG config root\n"
            "  --calibrate                 Run libsurvive's native Lighthouse solver\n"
            "  --self-test                 Run deterministic validity tests and exit\n"
            "  --help                      Show this help\n",
            program);
}

static int parse_port(const char *text) {
    char *end = NULL;
    errno = 0;
    long value = strtol(text, &end, 10);
    return errno == 0 && end != text && *end == '\0' && value > 0 && value <= 65535
               ? (int)value
               : -1;
}

static double parse_positive(const char *text) {
    char *end = NULL;
    errno = 0;
    double value = strtod(text, &end);
    return errno == 0 && end != text && *end == '\0' && isfinite(value) && value > 0.0
               ? value
               : -1.0;
}

static int parse_options(int argc, char **argv) {
    snprintf(options.target_host, sizeof(options.target_host), "127.0.0.1");
    const char *home = getenv("HOME");
    if (home == NULL || home[0] == '\0') {
        home = ".";
    }
    snprintf(options.config_dir, sizeof(options.config_dir), "%s/.config/umi-teleop", home);
    options.target_port = 9001;
    options.rate_hz = 60.0;
    options.max_pose_age_s = 0.2;
    options.max_optical_age_s = 0.2;
    options.max_linear_speed_m_s = 8.0;
    options.max_angular_speed_rad_s = 1440.0 * acos(-1.0) / 180.0;
    options.reacquire_s = 0.25;
    snprintf(options.poser, sizeof(options.poser), "KalmanOnly");

    enum {
        OPTION_TARGET_HOST = 1000,
        OPTION_TARGET_PORT,
        OPTION_RATE_HZ,
        OPTION_MAX_POSE_AGE_MS,
        OPTION_MAX_OPTICAL_AGE_MS,
        OPTION_MAX_LINEAR_SPEED_M_S,
        OPTION_MAX_ANGULAR_SPEED_DEG_S,
        OPTION_REACQUIRE_MS,
        OPTION_POSER,
        OPTION_CONFIG_DIR,
        OPTION_CALIBRATE,
        OPTION_SELF_TEST,
    };
    static const struct option long_options[] = {
        {"target-host", required_argument, NULL, OPTION_TARGET_HOST},
        {"target-port", required_argument, NULL, OPTION_TARGET_PORT},
        {"rate-hz", required_argument, NULL, OPTION_RATE_HZ},
        {"max-pose-age-ms", required_argument, NULL, OPTION_MAX_POSE_AGE_MS},
        {"max-optical-age-ms", required_argument, NULL, OPTION_MAX_OPTICAL_AGE_MS},
        {"max-linear-speed-m-s", required_argument, NULL, OPTION_MAX_LINEAR_SPEED_M_S},
        {"max-angular-speed-deg-s", required_argument, NULL, OPTION_MAX_ANGULAR_SPEED_DEG_S},
        {"reacquire-ms", required_argument, NULL, OPTION_REACQUIRE_MS},
        {"poser", required_argument, NULL, OPTION_POSER},
        {"config-dir", required_argument, NULL, OPTION_CONFIG_DIR},
        {"calibrate", no_argument, NULL, OPTION_CALIBRATE},
        {"self-test", no_argument, NULL, OPTION_SELF_TEST},
        {"help", no_argument, NULL, 'h'},
        {NULL, 0, NULL, 0},
    };

    int option = 0;
    while ((option = getopt_long(argc, argv, "h", long_options, NULL)) != -1) {
        double value = 0.0;
        switch (option) {
        case OPTION_TARGET_HOST:
            if (snprintf(options.target_host, sizeof(options.target_host), "%s", optarg) >=
                (int)sizeof(options.target_host)) {
                return -1;
            }
            break;
        case OPTION_TARGET_PORT:
            options.target_port = parse_port(optarg);
            if (options.target_port < 0) {
                return -1;
            }
            break;
        case OPTION_RATE_HZ:
            value = parse_positive(optarg);
            if (value < 1.0 || value > 240.0) {
                return -1;
            }
            options.rate_hz = value;
            break;
        case OPTION_MAX_POSE_AGE_MS:
            value = parse_positive(optarg);
            if (value < 1.0 || value > 5000.0) {
                return -1;
            }
            options.max_pose_age_s = value / 1000.0;
            break;
        case OPTION_MAX_OPTICAL_AGE_MS:
            value = parse_positive(optarg);
            if (value < 1.0 || value > 5000.0) {
                return -1;
            }
            options.max_optical_age_s = value / 1000.0;
            break;
        case OPTION_MAX_LINEAR_SPEED_M_S:
            value = parse_positive(optarg);
            if (value < 0.1 || value > 100.0) {
                return -1;
            }
            options.max_linear_speed_m_s = value;
            break;
        case OPTION_MAX_ANGULAR_SPEED_DEG_S:
            value = parse_positive(optarg);
            if (value < 10.0 || value > 10000.0) {
                return -1;
            }
            options.max_angular_speed_rad_s = value * acos(-1.0) / 180.0;
            break;
        case OPTION_REACQUIRE_MS:
            value = parse_positive(optarg);
            if (value < 10.0 || value > 5000.0) {
                return -1;
            }
            options.reacquire_s = value / 1000.0;
            break;
        case OPTION_POSER:
            if (strcmp(optarg, "KalmanOnly") != 0 && strcmp(optarg, "MPFIT") != 0) {
                return -1;
            }
            snprintf(options.poser, sizeof(options.poser), "%s", optarg);
            break;
        case OPTION_CONFIG_DIR:
            if (snprintf(options.config_dir, sizeof(options.config_dir), "%s", optarg) >=
                (int)sizeof(options.config_dir)) {
                return -1;
            }
            break;
        case OPTION_CALIBRATE:
            options.calibrate = true;
            break;
        case OPTION_SELF_TEST:
            options.self_test = true;
            break;
        case 'h':
            print_usage(stdout, argv[0]);
            exit(0);
        default:
            return -1;
        }
    }
    return optind == argc ? 0 : -1;
}

static int run_self_tests(void) {
    options.max_optical_age_s = 0.2;
    options.max_pose_age_s = 0.2;
    options.max_linear_speed_m_s = 8.0;
    options.max_angular_speed_rad_s = 1440.0 * acos(-1.0) / 180.0;
    SurvivePose pose = {.Pos = {1.0, 2.0, 3.0}, .Rot = {1.0, 0.0, 0.0, 0.0}};
    survive_long_timecode now = 48000000;
    int failures = 0;
#define CHECK(expression)                                                                      \
    do {                                                                                       \
        if (!(expression)) {                                                                   \
            fprintf(stderr, "self-test failed at line %d: %s\n", __LINE__, #expression);      \
            failures++;                                                                        \
        }                                                                                      \
    } while (0)
    CHECK(pose_is_valid(&pose, now, now - 480000, 48000000, 1.0));
    CHECK(pose_is_valid(&pose, now, now + 480000, 48000000, 1.0));
    CHECK(!pose_is_valid(&pose, now, 0, 48000000, 1.0));
    CHECK(!pose_is_valid(&pose, now, now - 24000000, 48000000, 1.0));
    CHECK(!pose_is_valid(&pose, now, now - 480000, 48000000, 0.0));
    pose.Rot[0] = 0.5;
    CHECK(!pose_is_valid(&pose, now, now - 480000, 48000000, 1.0));
    pose.Rot[0] = 1.0;
    pose.Pos[1] = NAN;
    CHECK(!pose_is_valid(&pose, now, now - 480000, 48000000, 1.0));
    pose.Pos[1] = 2.0;
    SurvivePose moved = pose;
    moved.Pos[0] += 0.04;
    double linear_speed_m_s = 0.0;
    double angular_speed_rad_s = 0.0;
    CHECK(pose_step_is_plausible(&pose, now, &moved, now + 480000, 48000000,
                                 &linear_speed_m_s, &angular_speed_rad_s));
    CHECK(fabs(linear_speed_m_s - 4.0) < 1e-9);
    moved.Pos[0] = pose.Pos[0] + 0.2;
    CHECK(!pose_step_is_plausible(&pose, now, &moved, now + 480000, 48000000,
                                  &linear_speed_m_s, &angular_speed_rad_s));
    moved = pose;
    double half_angle = 5.0 * acos(-1.0) / 180.0;
    moved.Rot[0] = cos(half_angle);
    moved.Rot[3] = sin(half_angle);
    CHECK(pose_step_is_plausible(&pose, now, &moved, now + 480000, 48000000,
                                 &linear_speed_m_s, &angular_speed_rad_s));
    half_angle = 15.0 * acos(-1.0) / 180.0;
    moved.Rot[0] = cos(half_angle);
    moved.Rot[3] = sin(half_angle);
    CHECK(!pose_step_is_plausible(&pose, now, &moved, now + 480000, 48000000,
                                  &linear_speed_m_s, &angular_speed_rad_s));
    CHECK(!pose_step_is_plausible(&pose, now, &pose, now, 48000000, &linear_speed_m_s,
                                  &angular_speed_rad_s));
#undef CHECK
    if (failures == 0) {
        puts("umi-vive-linux self-test: PASS");
    }
    return failures == 0 ? 0 : 1;
}

int main(int argc, char **argv) {
    if (parse_options(argc, argv) != 0) {
        print_usage(stderr, argv[0]);
        return 2;
    }
    if (options.self_test) {
        return run_self_tests();
    }

    if (mkdir_if_needed(options.config_dir) != 0) {
        return 2;
    }
    char libsurvive_config_dir[1200];
    int config_length = snprintf(libsurvive_config_dir, sizeof(libsurvive_config_dir),
                                 "%s/libsurvive", options.config_dir);
    if (config_length < 0 || (size_t)config_length >= sizeof(libsurvive_config_dir) ||
        mkdir_if_needed(libsurvive_config_dir) != 0) {
        return 2;
    }
    if (!options.calibrate && !completed_calibration_exists(options.config_dir)) {
        fprintf(stderr,
                "No completed native Lighthouse calibration under %s. Run this command once "
                "with --calibrate while every tracker is still and visible.\n",
                options.config_dir);
        return 2;
    }
    if (setenv("XDG_CONFIG_HOME", options.config_dir, 1) != 0) {
        perror("setenv(XDG_CONFIG_HOME)");
        return 2;
    }
    if (getenv("SURVIVE_PLUGINS") == NULL && UMI_DEFAULT_SURVIVE_PLUGIN_DIR[0] != '\0' &&
        setenv("SURVIVE_PLUGINS", UMI_DEFAULT_SURVIVE_PLUGIN_DIR, 1) != 0) {
        perror("setenv(SURVIVE_PLUGINS)");
        return 2;
    }
    if (resolve_target(options.target_host, options.target_port) != 0) {
        return 2;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    char *survive_argv[8] = {argv[0], "--globalscenesolver",
                             options.calibrate ? "1" : "0", "-p", options.poser,
                             NULL,    NULL,                    NULL};
    int survive_argc = 5;
    if (options.calibrate) {
        survive_argv[survive_argc++] = "--force-calibrate";
        survive_argv[survive_argc++] = "1";
    }
    SurviveContext *context =
        survive_init_with_logger(survive_argc, survive_argv, NULL, log_to_stderr);
    if (context == NULL) {
        close(udp_socket_fd);
        return 2;
    }
    survive_install_new_object_fn(context, new_object_process);
    survive_install_disconnect_fn(context, disconnect_process);
    survive_install_config_fn(context, config_process);

    int result = survive_startup(context);
    if (result == 0) {
        /* Match libsurvive's SurviveSimple API: install pose delivery only after startup has
         * finished constructing every USB object and threaded poser. */
        survive_install_pose_fn(context, pose_process);
    }
    pthread_t sender_thread;
    bool sender_started = false;
    if (result == 0) {
        if (pthread_create(&sender_thread, NULL, sender_loop, NULL) != 0) {
            fprintf(stderr, "failed to start UDP sender thread\n");
            result = -1;
        } else {
            sender_started = true;
            fprintf(stderr,
                    "Streaming native Vive poses to %s:%d at %.1f Hz; config=%s; mode=%s; "
                    "poser=%s\n",
                    options.target_host, options.target_port, options.rate_hz,
                    options.config_dir, options.calibrate ? "calibrate" : "tracking",
                    options.poser);
        }
    }
    while (keep_running && result == 0) {
        result = survive_poll(context);
    }
    keep_running = 0;
    if (sender_started) {
        pthread_join(sender_thread, NULL);
    }
    survive_close(context);
    close(udp_socket_fd);
    return signal_received ? 0 : (result == 0 ? 0 : 1);
}
