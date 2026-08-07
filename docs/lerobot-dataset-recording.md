# LeRobot Dataset Recording Requirements

## Goal

Record UMI-driven bimanual YAM demonstrations as LeRobot v3 datasets that can be
used directly with `allenai/MolmoAct2-BimanualYAM`, then optionally upload the
finalized dataset to the Hugging Face Hub.

## Dataset Contract

- Record at exactly 30 Hz on the YAM controller computer.
- Store three ordered RGB views named `top`, `left`, and `right`.
- Store every image as RGB `uint8`. The default RealSense profile stores all three views with shape
  `(270, 480, 3)`. The optional ultrawide profile stores `top` at `(270, 480, 3)` and `left`/`right`
  at `(312, 416, 3)`.
- Capture the top D435 at `640x480`, center-crop it to `640x360`, and downsample it to `480x270`.
  Capture the default side D405 cameras natively at `480x270`, or capture the optional side
  ultrawides as MJPEG at `640x480` with a requested 60 fps setting that delivers 30 fps on the
  tested hardware, then downsample them to `416x312` for storage and encoding. This exact 4:3
  frame has 129,792 pixels, within 0.15% of each stored `480x270` RealSense frame.
- Store `observation.state` as 14 `float32` values in this order:
  left joints 0-5, left gripper, right joints 0-5, right gripper.
- Store `action` in the same order. The action is the final joint/gripper target
  sent after IK, joint limits, and per-tick slew limits, not the raw UMI target.
- Read joint and gripper observations from the robot before sending that frame's
  action.
- Store one nonempty, fixed task string for every episode.
- Store yuv420p video. Default to H.264 on the Raspberry Pi; allow AV1 when
  file-level codec parity with the released dataset is required.

## Episode Contract

- Automatic startup motion is disabled by default. The arms hold their physical starting pose, and
  a RECORD tap after a valid ZERO starts teleoperation and the first episode together.
- The idle wait for ZERO and RECORD is not recorded or padded with duplicate frames.
- An intentional RECORD stop commits the active episode and holds both arms by default.
- `--dataset.post-episode-action neutral` explicitly returns both arms to the all-zero base posture
  after the commit is queued. The return is not recorded, and the old Cartesian reference is
  invalidated so a new ZERO is required before the next RECORD start.
- Repeated ZERO/RECORD/stop cycles start and commit additional episodes without restarting the
  source, controller, cameras, CAN connections, or dataset writer.
- Passing `--robot.move-to-ready` to the one-command `umi-teleop` launcher (or
  `--move-to-ready` to `umi-yam` directly) explicitly performs an unrecorded startup motion.
  RECORD still starts teleoperation and the first dataset episode together after a valid ZERO.
- Passing `--move-to-base` to `umi-yam` directly performs an unrecorded startup parking operation.
- ZERO while armed, CANCEL, source restart, stale telemetry, tracking loss, control failure, camera
  failure, or process shutdown discards the active episode. Ordinary degraded IK remains armed and
  records the final bounded action actually sent; exhausting the bounded IK fallback search
  disarms both arms and discards the episode.
- Episodes shorter than the configured minimum are discarded.
- A discarded episode must not change the count of saved episodes.
- Apart from an explicitly requested automatic startup episode, a new episode can start only after
  a new disabled-to-enabled transition.

## Safety And Failure Contract

- Recording is opt-in and must not import LeRobot, open cameras, create files, or
  change the normal controller path when disabled.
- Dataset metadata and all three cameras are validated before CAN is opened or
  either robot is constructed.
- RealSense remains the default side-camera backend. Ultrawide mode requires distinct stable
  `/dev/v4l/by-path` paths because the two cameras expose identical USB serials.
- Camera capture and dataset writes use a bounded asynchronous queue so video or
  Parquet work does not run in the 30 Hz motor-control thread.
- Any frame drop reported by a streaming encoder invalidates and discards the
  episode rather than committing camera streams of different lengths.
- A writer backlog invalidates and discards the active episode before the bounded queue can fill,
  leaving the controller alive and the motors holding position. Unexpected queue overflow and
  background writer failures still fail closed.
- Require a configurable free-disk reserve before initialization and before each
  episode.
- Per-arm IK and bounded fallback search run in latest-target background workers. The foreground
  loop accepts only fresh, validated results, applies its existing joint slew limit at 100 Hz, and
  holds the last validated goal while a worker is busy.
- A transient scheduling backlog over the configured timing limit is allowed to recover. Only a
  backlog that remains over the limit for the configured grace period, or a single cycle exceeding
  the combined limit and grace, discards the episode. LeRobot timestamps are otherwise synthesized
  at the declared 30 Hz.
- Use two H.264 encoder workers per camera. A one-worker hardware trial saturated an ultrawide
  encoder after roughly 32 seconds, earlier than the two-worker baseline.
- Convert RGB arrays directly into PyAV frames and defer per-camera image statistics until each
  completed MP4 can be decoded after the episode. This keeps statistics complete without spending
  histogram CPU in the live encoder threads.
- A bimanual frame is queued only after both arm commands return successfully.
- Previously committed episodes remain local if capture, finalization, login, or
  upload fails.
- Final validation checks contiguous episode row ranges and requires every encoded camera video
  file to have exactly the combined length of all episodes that reference it before Hub upload.
  This covers LeRobot's multi-episode shared-video layout without weakening frame-count checks.
- Finalization is mandatory and idempotent. Upload begins only after both robots
  have been closed and the local dataset has finalized. Writer shutdown is
  bounded; a hung encoder reports an error and leaves the local files untouched
  instead of hanging robot shutdown indefinitely.
- Hub upload is explicit, private by default, and never deletes the local copy.

## Compatibility

The integration is pinned to the tested AfterQuery LeRobot revision used by the
YAM policy client. The recording extra requires Python 3.12. Existing Windows,
UMI source Pi, monitor-only, simulation, and non-recording YAM commands retain
their current dependencies and behavior.

Dataset creation, append/resume, frame insertion, episode save/discard, finalization, reopening,
and Hub upload use the public `LeRobotDataset` API documented by Hugging Face. The default PNG
staging path does not patch LeRobot. The opt-in streaming encoder optimization adapts the pinned
revision's private `_CameraEncoderThread` to defer RGB statistics, and live queue-depth diagnostics
inspect the writer's internal queues. Those version-pinned optimizations do not replace LeRobot's
public dataset lifecycle or on-disk format.

## Verification

- Unit tests cover feature order, frame validation, episode commit/discard rules,
  queue failure, writer failure, and idempotent cleanup.
- Integration tests cover asynchronous fake cameras and a dataset backend across
  multiple saved/discarded episodes.
- A LeRobot integration test writes, finalizes, reopens, and checks a real local
  v3 dataset when the recording extra is installed.
- The end-to-end test drives the existing Windows-to-source UDP path into the
  recording session with simulated robot state and synthetic camera frames.
- Hardware acceptance is one short supervised episode with the selected camera profile, followed
  by local frame-count/shape validation, Hub upload, and remote reopen verification.
