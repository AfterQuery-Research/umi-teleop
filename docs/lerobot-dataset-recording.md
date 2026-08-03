# LeRobot Dataset Recording Requirements

## Goal

Record UMI-driven bimanual YAM demonstrations as LeRobot v3 datasets that can be
used directly with `allenai/MolmoAct2-BimanualYAM`, then optionally upload the
finalized dataset to the Hugging Face Hub.

## Dataset Contract

- Record at exactly 30 Hz on the YAM controller computer.
- Store three ordered RGB views named `top`, `left`, and `right`.
- Store every image as RGB `uint8` with shape `(360, 640, 3)`.
- Capture the top D435 at `640x480` and center-crop it to `640x360`; capture the
  two side D405 cameras natively at `640x360`.
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

- A successful `E` enable transition starts one episode.
- An intentional `E` disable transition commits the episode.
- ZERO, CANCEL, source restart, stale telemetry, tracking loss, workspace limit,
  IK failure, camera failure, or process shutdown discards the active episode.
- Episodes shorter than the configured minimum are discarded.
- A discarded episode must not change the count of saved episodes.
- A new episode can start only after a new disabled-to-enabled transition.

## Safety And Failure Contract

- Recording is opt-in and must not import LeRobot, open cameras, create files, or
  change the normal controller path when disabled.
- Dataset metadata and all three cameras are validated before CAN is opened or
  either robot is constructed.
- Camera capture and dataset writes use a bounded asynchronous queue so video or
  Parquet work does not run in the 30 Hz motor-control thread.
- Queue overflow and background writer failures fail closed.
- Require a configurable free-disk reserve before initialization and before each
  episode.
- A control-loop delay over the configured timing limit discards the episode;
  LeRobot timestamps are otherwise synthesized at the declared 30 Hz.
- A bimanual frame is queued only after both arm commands return successfully.
- Previously committed episodes remain local if capture, finalization, login, or
  upload fails.
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

## Verification

- Unit tests cover feature order, frame validation, episode commit/discard rules,
  queue failure, writer failure, and idempotent cleanup.
- Integration tests cover asynchronous fake cameras and a dataset backend across
  multiple saved/discarded episodes.
- A LeRobot integration test writes, finalizes, reopens, and checks a real local
  v3 dataset when the recording extra is installed.
- The end-to-end test drives the existing Windows-to-source UDP path into the
  recording session with simulated robot state and synthetic camera frames.
- Hardware acceptance is one short supervised episode with all three RealSense
  cameras, followed by private Hub upload and remote reopen verification.
