ARG BASE_IMAGE=koide3/glim_ros2:jazzy_cuda12.5
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

RUN bash -c 'set -e; \
    source /opt/ros/jazzy/setup.bash; \
    GLIM_WS=""; \
    for ws in /root/ros2_ws /ros2_ws /opt/glim_ws; do \
        [ -f "$ws/install/setup.bash" ] && GLIM_WS="$ws" && break; \
    done; \
    if [ -z "$GLIM_WS" ]; then \
        GLIM_WS=$(find / -maxdepth 4 -path "*/install/setup.bash" 2>/dev/null \
                  | head -n1 | sed "s|/install/setup.bash||"); \
    fi; \
    [ -n "$GLIM_WS" ] && source "$GLIM_WS/install/setup.bash"; \
    echo "${GLIM_WS:-none}" > /etc/glim_ws_path; \
    CFG="$(ros2 pkg prefix glim)/share/glim/config"; \
    [ -d "$CFG" ] || { echo "GLIM stock config not found" >&2; exit 1; }; \
    cp -r "$CFG" /glim_config'

COPY config_ros.json /glim_config/config_ros.json
COPY config_sensors.json /glim_config/config_sensors.json

RUN apt-get update && apt-get install -y --no-install-recommends \
      git libpcap-dev libyaml-cpp-dev python3 \
    && rm -rf /var/lib/apt/lists/*

ARG RSLIDAR_SDK_REF=v1.5.20
ARG RSLIDAR_MSG_REF=v1.5.10
WORKDIR /rslidar_ws/src
RUN git clone --depth 1 --branch ${RSLIDAR_SDK_REF} --recurse-submodules --shallow-submodules \
      https://github.com/RoboSense-LiDAR/rslidar_sdk \
    && git clone --depth 1 --branch ${RSLIDAR_MSG_REF} https://github.com/RoboSense-LiDAR/rslidar_msg
RUN grep -q 'set(CMAKE_CXX_STANDARD 14)' rslidar_sdk/CMakeLists.txt \
    && sed -i 's/set(CMAKE_CXX_STANDARD 14)/set(CMAKE_CXX_STANDARD 17)/' rslidar_sdk/CMakeLists.txt \
    && grep -q 'set(POINT_TYPE XYZI)' rslidar_sdk/CMakeLists.txt \
    && sed -i 's/set(POINT_TYPE XYZI)/set(POINT_TYPE XYZIRT)/' rslidar_sdk/CMakeLists.txt
RUN test "$(grep -c 'imu_msg.linear_acceleration.x = data->linear_acceleration_x;' rslidar_sdk/src/source/source_pointcloud_ros.hpp)" = 2 \
    && sed -i -e 's#imu_msg.linear_acceleration.\([xyz]\) = data->linear_acceleration_\([xyz]\);#imu_msg.linear_acceleration.\1 = data->linear_acceleration_\2 * 9.80665;#' \
        rslidar_sdk/src/source/source_pointcloud_ros.hpp
WORKDIR /rslidar_ws
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_IMU_DATA_PARSE=ON

COPY rslidar_live.yaml /rslidar_live.yaml
COPY box_unpack.py /box_unpack.py
COPY pose_sub.py /pose_sub.py
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    ROS_LOCALHOST_ONLY=1

ENTRYPOINT ["/entrypoint.sh"]
