# `robotd --plant` — a MuJoCo robot behind `RobotIo`

Status: draft · Date: 2026-09-07 · Owner: liko

Fills the row [`robotd-design.md`](robotd-design.md) §8 defers as "MuJoCo backend and the
`RemoteIo` protocol", and the decision-table row "sim after slice 2". `--fake` was the
stand-in ("there is no simulator yet, and this is what stands in for one" — `robotd --help`);
this is the simulator.

**Two simulators, since 2026-09.** Upstream landed its own the same week this was written:
`robotd --sim host:port` → `duck_control::sim::RemoteIo` → `microduck_rl`'s `duck-body`,
designed for containers and many ducks (`docs/design/simulation.md`). This one stayed as
`robotd --plant SOCKET` → `robotd::sim::SimIo` → `scripts/plant_server.py`: single duck,
lockstep, unix socket, with the depth-sensor server, the maze and room scenes, and the
recorder the brain's bench uses. Same seam, two plants; the brain does not know which.

## 1. Why

Two things need a robot that is not a robot:

- **The brain** ([`brain-design.md`](brain-design.md), M9). "A duck left alone in a room does
  something worth watching for ten minutes" cannot be iterated on with `FakeIo`, which never
  falls, never moves, and reports whatever it was told. It needs gravity, a floor, and a
  depth sensor that sees the floor.
- **Policy acceptance at the daemon level.** Every policy so far was rehearsed in
  `microduck_rl/scripts/infer_policy.py`, a Python re-implementation of the runtime's
  observation builder, skill chain and gain handling. Each of those re-implementations has
  drifted from `robotd` at least once (action low-pass: `control.rs` applies 0.5/0.7,
  `infer_policy.py` applies none). Running the *actual* daemon against physics removes the
  copy.

## 2. The shape of it

```text
   robotd (Rust)                                   plant (Python, microduck_rl)
   ┌──────────────────────────────┐                ┌──────────────────────────────┐
   │ control thread, 50 Hz        │   unix socket  │ MuJoCo 3.10, CPU             │
   │  safety.read()  ──────────── │ ◄──── reply ── │  scene_{walk,rollers,ball}   │
   │  controller.step()           │                │  200 Hz physics, 4 substeps  │
   │  safety.apply() ─ write() ── │ ── step ─────► │  per control tick            │
   │                              │                │  optional viewer window      │
   │ SimIo: RobotIo               │                │  optional tof.stream server  │
   └──────────────────────────────┘                └──────────────────────────────┘
```

**The plant is a separate process, and it is Python.** The robot model, its scenes, the
actuator fits and the whole sim2real recipe live in `microduck_rl`; the plant is one more
script there (`scripts/plant_server.py`), next to `infer_policy.py`, sharing its constants.
Embedding MuJoCo in `robotd` would mean a Rust binding, a second copy of the model, and a
`duck-control` that links a physics engine on the board. None of that is wanted.

**`SimIo` is the third `RobotIo`.** It lives in `robotd/src/sim.rs`, not in `duck-control`:
that crate's rule is "no tokio, no socket, no systemd", and a socket is a socket even when it
is blocking. `robotd` already owns every other socket. The implementation is a blocking
`std::os::unix::net::UnixStream`, because the control thread is a blocking thread by design
(§4.1 of the robotd doc). `Safety` owns it exactly as it owns the bus.

**Lockstep, not free-running.** The plant advances physics only when `robotd` writes targets.
One `write()` = one control period of physics (4 × 5 ms) and one reply carrying the sensors
that period produced. `read()` returns that reply. Consequences, all wanted:

- Sim time is `robotd`'s tick count. A slow laptop makes the sim slower than real time, never
  wrong. Nothing races.
- The 20 ms budget is spent as: socket round trip (~0.1 ms) + 4 `mj_step` on a 20-DoF model
  (~0.5 ms) + policy inference. Well inside 45 Hz on any machine that builds the workspace.
- Determinism: same targets in, same sensors out. Golden traces become possible.
- Pausing the viewer pauses the robot, which is what a human at the window expects.

**The tick order in `robotd` is read → step → write.** So the first `read()` of a run
precedes any `write()`. `SimIo::read` with nothing buffered sends a `read` request; the plant
answers with the current state and steps nothing. Every later `read()` returns the reply to
the previous `write()`, without touching the socket.

## 3. The protocol

NDJSON over a unix socket, one request → one reply, no pipelining — the same framing as
every other socket in this repo (`duck-ipc-proto` §1), so it can be watched with `socat` and
written by hand. It is **not** JSON-RPC: there is no `id`, because there is never more than
one request in flight, and no method namespace, because there is one peer.

Requests, from `robotd`:

| `op` | fields | reply |
|---|---|---|
| `hello` | `version: 1` | `model` (`walk`/`rollers`/`ball`), `joints` (14 names, actuator order), `dt` (0.02), `physics_dt` (0.005) |
| `read` | — | a sensor frame (below); steps nothing |
| `step` | `targets: [f64; 15]` (radians, `JOINT_NAMES` order, mouth at 9) | a sensor frame, after one control period |
| `gain` | `kp: u16` (firmware units, 0–1023) | `ok` |
| `torque` | `on: bool` | `ok` |
| `slow` | — | `volts: f64`, `temps_c: [f64; 15]` |
| `reset` | `pose: "stand"\|"prone"\|"supine"\|"sit"`, `z?: f64` | a sensor frame |

The sensor frame:

```json
{"ok": true, "t": 12.34,
 "pos": [15 f64], "vel": [15 f64],
 "gyro": [3 f64], "gravity": [3 f64], "quat": [4 f64],
 "trunk": {"pos": [3 f64], "yaw": f64}}
```

- `pos`/`vel`: radians and rad/s, `JOINT_NAMES` order. The mouth (index 9) has no joint in
  the model; the plant echoes the last commanded mouth target, as `FakeIo` does. Servo
  angles come from `qpos[jnt_qposadr[actuator_trnid[i]]]`, never a contiguous slice — the
  roller model interleaves passive wheels (`AGENTS.md`, `microduck_rl`).
- `gyro`: the `imu_ang_vel` sensor on site `imu`, trunk frame, rad/s. Same source as
  `infer_policy.py` and the training env.
- `gravity`: world `[0,0,−1]` rotated into the trunk frame by the trunk quaternion, unit
  length. This is what `imu.rs` derives from the SFLP quaternion on hardware; the
  accelerometer is not used by either.
- `quat`: trunk→world, scalar-first `[w,x,y,z]`, `ImuData::quat` convention.
- `trunk`: ground truth, for tests and the viewer HUD. `robotd` ignores it; odometry stays
  what the loop computes from joints and the quaternion, so the sim exercises it.
- Errors: `{"ok": false, "error": "..."}`. `SimIo` maps them to `IoError::Bus`, a closed
  socket to `IoError::Port` — the same two errors the real bus produces, so the health
  verdict (`consecutive_errors ≥ 10` → unhealthy) needs no new case.

### 3.1 What the plant does with `gain` and `torque`

Position servos in the model are MuJoCo `<position kp=0.55>` actuators, the "marc" fit for
firmware kp 200 (`joints_properties.xml`). The plant scales linearly:
`kp_sim = 0.55 · kp / 200`, written to `actuator_gainprm[:,0]` and `actuator_biasprm[:,1]`
(the pattern `infer_policy.py --record` already uses). `gain_limp = 50` therefore yields a
robot that sags, which is what "limp" means on hardware.

`torque off` zeroes both, and the robot collapses under gravity. `torque on` restores the
last gain. Neither is on the tick path; both cost one round trip.

Force is clipped at the XL330's 1.75 A current limit (`kt · 1.75 = 0.64 Nm`), as
`infer_policy.py` does. This plant is the **deployment twin**, the softer of the two
actuator models. `--actuator bam` is the other one — the training env's voltage-controlled
XL330 (pure-numpy `bam.mujoco.MujocoController`, motors instead of position servos, the
`microduck_constants.py` values for kp, supply drop and command delay).

**Measured on day one, and the reason `position` is the default:** the shipped `alpha_*`
policies walk on the position servo (0.39 m in 4.5 s at 0.2 m/s) and stand still on BAM
(0.01 m); a policy trained here under BAM (the spin) does the reverse (360° on BAM, 134° on
the position servo). Each policy family prefers the actuator it was trained against, and
the daemon exists to run the shipped set. Which model the hardware matches is still the
open question it was.

**Neither model stands open-loop.** Holding the HOME pose with no policy tips the robot
forward in about a second on both — the CoM sits ahead of the feet on purpose (STAND2) and
`alpha_stand` does the balancing. Two consequences the plant is built around: it spawns
**seated** by default (the SIT keyframe; `robotd` reads the folded legs as a seated boot and
rises through the sitstand policy on `robot.enable`, exactly the hardware path), and for
spawns that must start standing (the roller model rolls onto its back when seated) there is
`--freeze-root SECONDS`, a bench aid that pins the trunk until the policy is driving.

### 3.2 IMU readiness and staleness

`imu_ready()` is `true` after 25 frames (0.5 s at 50 Hz), mirroring `SflpDecoder::ready`'s
25 quaternion samples. Until then `robotd` does not drive and cannot declare a fall — the sim
should exercise that gate, not bypass it. `imu_stale()` stays at the trait default.

### 3.3 Slow sensors

`volts` defaults to 7.4 and `temps_c` to 32 °C, `FakeIo`'s numbers; both are plant flags
(`--volts`, `--temp`). Two things in `robotd` act on voltage and must be remembered when
setting it: `voltage_adapt` multiplies the action scale by `nominal / measured`, and
`battery_empty_shutdown` **powers the machine off** at 6.6 V. The plant refuses `--volts`
below 6.7 unless `--i-mean-it`.

### 3.4 The depth sensor (for the brain)

With `--tof`, the plant also serves **`tofd`'s protocol** — `tof.stream` request,
`tof.frame` notifications at 15 Hz, `TofFrame { seq, at_us, rows: 8, cols: 8, distance_mm,
status }` — on a second socket, by casting 64 rays from the `tof` site (`sensors.xml` has
the site; the ST VL53L5CX's 45° × 45° field is split 8 × 8). Status is 5 (valid) for a hit,
255 for no return within 4 m. A brain that consumes `tof.frame` then runs unchanged against
the sim and the robot. The ball in `scene_ball.xml` is visible to it, which is how BallPlay
gets tested.

## 4. Touch points in `robotd`

From the audit of `main.rs`:

1. `Args`: `--plant <SOCKET>` (`Option<PathBuf>`, `conflicts_with = "fake"`), beside `--fake`.
2. `spawn_control_thread`: a third arm — `control_loop(SimIo::connect_waiting(..), …)`. The
   third monomorphisation of `control_loop`; nothing else in the loop changes.
3. `SimIo::connect_waiting` retries on `STARTUP_RETRY_INTERVAL` and publishes
   `startup_bus_failures`, so a plant that is not up yet reads as "degraded: no robot on
   the motor bus after n attempts", the same words as an unpowered board. It ignores the
   `cfg!(target_os = "linux")` early return in `open_bus_waiting`, which is about serial
   ports, not this.
4. `robotd init --sim` is not provided. `init` is a bus-maintenance subcommand that talks to
   servos the loop is not running; the sim has no such state. `robot.init`/`robot.enable`
   over the socket bring a sim robot up like any other.
5. `robotd.toml` gains nothing. The socket is a flag, like `--fake`: a laptop concern, never
   a board's.

## 5. Running it

```bash
# terminal 1 — the plant, from microduck_rl
uv run mjpython scripts/plant_server.py --socket /tmp/plant.sock --model rollers --viewer --tof /tmp/tof.sock

# terminal 2 — the daemon, from microduck
ORT_DYLIB_PATH=…/libonnxruntime.1.24.4.dylib \
  cargo run -p robotd -- --plant /tmp/plant.sock --socket /tmp/robotd.sock --params dev/robotd-mac.toml

# terminal 3 — drive it
cargo run -p padd -- --socket /tmp/robotd.sock          # a gamepad
cargo run -p robotctl -- --robot-socket /tmp/robotd.sock health
```

`mjpython` is required for the viewer on macOS (`launch_passive` must own the main thread);
headless runs use plain `python`. `dev/robotd-mac.toml` points the seven `[policy]` paths at
`microduck_rl/policies/`.

## 6. Testing

- **`duck-control`**: `SimIo` against an in-process thread speaking the protocol over a
  socketpair — the `FakeIo` tests' shape, one level down. Pins: framing, the 15↔14 mouth
  mapping, error mapping, `imu_ready` after 25 frames, the read-before-write case.
- **Plant, Python** (`microduck_rl/tests/test_plant_server.py`): a client that says `hello`,
  `read`, `step` ×100 with the home pose, asserts the trunk settles near `STAND_Z` and
  gravity near `[0,0,−1]`; `torque off` makes it fall; `reset prone` puts it on its face.
- **End to end** (`robotd/tests/sim.rs`, `#[ignore]` unless `MICRODUCK_PLANT` names a plant
  executable): spawn the plant headless, spawn `robotd --plant`, wait healthy, `robot.enable`,
  `robot.move vx=0.2` for 5 s, assert `odom.position[0] > 0.5` and `safety.fallen == false`.
  This is the daemon-level walking acceptance that has never existed.
- **Golden traces** (later): record `(targets, frame)` per tick; replay through `SimIo` with
  the plant swapped for the recording. `robotd`'s observation builder against a recorded sim
  is a unit test of the whole control path with no physics engine in CI.

## 7. Decisions recorded

| decision | why |
|---|---|
| plant in Python, in `microduck_rl` | the model, scenes and actuator fits live there; one copy |
| lockstep, one `step` per `write` | determinism; the tick budget is bounded by physics cost, not wall time |
| NDJSON, not binary, not JSON-RPC | inspectable; one peer, one request in flight |
| `SimIo` in `robotd`, blocking `UnixStream` | `duck-control` speaks to no socket; the control thread is blocking by design |
| `--plant` a flag, not a params key | a laptop concern, like `--fake`; the board never sees it |
| position-servo plant first, BAM second | the deployment twin is the conservative model; which is right is a hardware question |
| the plant speaks `tofd`'s protocol | a brain written against `tof.frame` runs unchanged on both |
| ground truth in the frame, ignored by `robotd` | tests need it; odometry must not be fed it |

## 8. Deferred, deliberately

BAM actuator option · IMU noise/misalignment injection (the training DR ranges are the
obvious values) · a `ball` body pose in the frame · pushes from the viewer (`P` in
`infer_policy.py`) as a protocol op · recording/replay · a camera image for `mediad`'s duck
detector · running the plant on the board (it is not for the board).

## 9. Open

- Whether `infer_policy.py` should eventually become a thin client of the plant rather than
  a second stepping loop. It should; not now.
- The action low-pass discrepancy (`control.rs` 0.5/0.7 vs unfiltered training/`infer_policy`)
  showed up on day one: under the daemon's filter and 0.9 scale, `alpha_walking` at
  0.2 m/s does not start stepping from a still stand (5/5 seeds), while 0.3 m/s, a yaw
  nudge, or the unfiltered action all start it (5/5). `padd`'s full stick is 0.3, so people
  never see it; a brain that creeps at 0.2 will. Resolve by measurement on hardware, not by
  picking a side; until then the brain should command ≥ 0.3 or add a yaw component.
- **The yaw command is only trustworthy inside about ±0.6 rad/s.** `alpha_walking` on the
  deployment plant (with or without the daemon's filter): from a stand, +1.0 turns left
  ~27°/s but −1.0 barely moves; while walking, −1.0 turns *left* (+150°) and ±1.5 both go
  the wrong way — the trained range is ±1.0 and the sign flips at its edge. With 0.3 m/s
  forward and ±0.6, both directions turn correctly on a ~0.15 m radius (−132° in 4 s under
  `robotd`). The brain therefore turns left in place at +1.0 and right as a walking arc at
  −0.6, and clamps steering to ±0.6. `padd` maps a full stick to 1.5 rad/s, which on this
  plant turns the wrong way; the robot is the place to check whether that is real.
- Contact odometry reads zero on rollers (nothing steps). The brain on rollers needs wheel
  odometry or the plant's ground truth; on feet it has what `robotctl monitor` has.

## 10. Status (2026-09-07)

Built and passing: `SimIo` (`robotd/src/sim.rs`, wire tests), `--plant`, the plant with both
actuators and the `tof.stream` server, `microduck_rl/tests/test_plant_server.py` (9), and
`robotd/tests/sim.rs` — seated boot → `robot.enable` → rise → stand → walk 0.3 m/s → stop,
healthy at 50 Hz, `#[ignore]` unless `MICRODUCK_RL` is set. Roller mode drives at the 0.6
push command (1.9 m in 6 s). `dev/robotd-mac*.toml` point the policies at `microduck_rl`;
`scripts/sim_drive.py` is a keyboard `padd`.
