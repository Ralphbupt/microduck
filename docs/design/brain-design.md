# `braind` — the autonomous behaviour layer

Status: draft · Date: 2026-09-07 · Owner: liko

Covers [`roadmap.md`](../project/roadmap.md) M9. The holding pen it drains is
[`ideas/autonomous_behavior.md`](../ideas/autonomous_behavior.md); the prototype being
re-created (not ported — its source is not available here) is the runtime's `autonomous.rs`,
a 16-state machine on an energy/mood model.

Done when, from the roadmap: *a duck left alone in a room does something worth watching for
ten minutes, and the chorale is something it decides rather than something a command starts.*

## 1. Where it lives: its own daemon

`architecture.md` open question 3 asks whether the brain is part of `robotd` or its own
service. **Its own service**, `braind`, an ordinary socket client of `robotd` the way `padd`
is. The reasons are the ones the roadmap already gives:

- "The chorale is 55 KB of `robotd` — that is the pattern this milestone exists to stop
  repeating." The brain is the largest piece of work left and the one most likely to grow;
  it must not grow inside the 50 Hz loop.
- `padd` proved the shape: no privileged access, intents over the socket, a hop of tens of
  microseconds against a 20 ms tick, killable without touching a motor.
- It runs unchanged against `robotd --sim` ([`sim-backend-design.md`](sim-backend-design.md))
  and a real robot, because it never sees a bus.
- `updater-design.md` §5.7 already anticipates the behaviour layer as its own artifact and
  channel. Learned state (habits, friends) is that channel's business, not `robotd`'s.

What it costs: two inputs that today are in-process to `robotd` — ambient sound events and
petting — have to be put on the wire (§4.2). That is a small addition to the state stream
and the right one regardless: `robotctl monitor` wants them too.

## 2. The shape of it

```text
            robot.state (50 Hz)  ┐
            tof.frame  (15 Hz)   ├──►  senses  ──►  drives  ──►  arbiter  ──►  behaviour  ──►  intents
            robot.event (sparse) ┘       │            │                          (one active)      │
                                          │            │                                            ▼
                                     world model    energy · mood · curiosity          robot.move / head / pose /
                                    (obstacles,      (slow scalars)                     do / sound / mouth  → robotd
                                     novelty grid,
                                     last pet, last voice, ducks nearby)
```

Three layers, each a plain data transform, each testable without a socket:

1. **Senses** turn wire messages into a `World`: trunk-frame obstacle zones
   (`kinematics::tof::Reprojector` joined with `robot.state`'s gravity and trunk height, as
   `robotctl monitor` already does), odometry, whether the robot is fallen/limp/homing, which
   policy drove, the last sound/pet/beacon events with ages, and a **novelty grid** — a 2-D
   grid of visit counts over odometry, 0.25 m cells, decaying.
2. **Drives** are slow scalars in `[0, 1]`: `energy` (drains with motion, recovers in Chill
   and Nap), `curiosity` (rises with time since the last new cell, drops when Wander pays
   out), `social` (rises when a duck is nearby or a voice is heard), `comfort` (rises when
   petted, drops on startle). They are the whole "mood model": behaviours are chosen by
   scoring against drives, never by a fixed sequence.
3. **The arbiter** picks one behaviour at a time. Every 250 ms it scores each candidate
   (`score(world, drives) → Option<f64>`, `None` = not applicable) and switches when a
   candidate beats the current one by a margin *and* the current one's minimum dwell has
   passed. Reflexes (Startle, Petted, fallen) score high and dwell short; Nap scores high only
   when energy is low and dwells long. Randomness enters as a per-candidate jitter so the
   duck is not a clock.

A **behaviour** is a small state machine with `enter / tick(world, drives) → Intents / exit`.
`Intents` is what goes to `robotd`: twist, head, pose, an optional skill, an optional sound.
The daemon sends twist and head as notifications at 20 Hz (inside the 500 ms deadman with
room to spare), skills and sounds as requests once.

## 3. The behaviours

The prototype's sixteen, kept as names so the vocabulary stays shared, in the order they
land:

| behaviour | v1 | what it does | needs |
|---|---|---|---|
| Chill | ✓ | stand, breathe (slow body-pose z), glance | — |
| LookAround | ✓ | head sweeps to random targets, dwells | — |
| Wander | ✓ | walk toward the least-visited reachable heading, avoid obstacles, stop before walls | ToF, odom |
| TurnInPlace | ✓ | yaw toward a new heading, used by Wander when boxed in | — |
| Zoomies | ✓ | short burst at max speed in an open direction, only when energy is high | ToF |
| Stretch · Ruffle · Preen · Sneeze | ✓ | body-pose and head gestures, scripted, a few seconds, plus a sound | — |
| Nap | ✓ | sit (`sit_toggle`), quiet, energy recovers, wakes on sound/pet | — |
| Startle | ✓ | freeze, head up, `alarm`; from a sudden close ToF hit or a loud noise | ToF, sound |
| GroundPick | ✓ | the skill, when curiosity is high and the floor ahead is clear | — |
| Petted | ✓ | lean in, `coo`, stay still while it lasts; outbids a startle | pet events |
| Startle by sound | ✓ | a loud noise, unless being petted | sound events |
| Dance | ✓ | bob, sway and nod on the beat while beats keep coming | beat events |
| Greet | ✓ | a duck's beacon appears: `greet` for a stranger, `chirp` for a friend, then it is known | duck events |
| Lonely | ✓ | alone for three minutes with low social drive: an `inquire` call, rarely | events (absence) |
| BallPlay | v2 | approach / line up / kick, from a ball the ToF sees as a low, round hit | ToF (sim: ball scene) |
| Held | dropped | pickup detection is deprecated in the runtime; nothing feeds it | — |
| Sing | v2 | the chorale as a decision: rare, gated on company and mood | `robot.chorale` |

v1 is what runs in the sim on day one and what "ten minutes worth watching" is measured
against. Everything in v2 needs one of the wire additions in §4.2.

## 4. Inputs

### 4.1 Already on the wire

- `robot.subscribe` → `robot.state`: `policy` label (`walk`/`stand`/`sit`/`limp_fall`/
  `homing`/`held`…), `safety.fallen/limp/gravity`, `odom.position/yaw`, `joints`, `head`,
  `move.requested/applied/limited_by`, `chorale`, `theremin`.
- `tof.stream` → `tof.frame`, from `tofd` on the robot or the plant in the sim.
- `robot.mode` (walk/roller) shapes speeds exactly as `padd` shapes sticks.

### 4.2 Events on the state frame (API v17, built)

Not a new notification: a new field. `RobotState.events` carries what happened since the
last frame, absent from the wire when empty:

```json
"events": [{"kind":"sound_voice"}, {"kind":"duck_seen","id":7}]
```

`kind ∈ sound_noise · sound_voice · pet_start · pet_end · beat · duck_seen · duck_lost`.
`robotd` had every one in hand (`pet.try_recv_sound`, the petting events, the chorale's
peers and beat); they now ride on the frame, in order with the state they belong to. A
subscriber that skips frames (`hz`) loses none: events on skipped frames accumulate onto the
next frame it is sent. Riding on the frame rather than a separate notification kept the
subscriber loop's nested selects untouched and the wire additive.

On a bench, `braind --events-from FILE` feeds the same events from appended lines
(`echo pet_start >> FILE`), since the plant has no microphone.

### 4.3 Not available, and not waited for

RSSI (not carried by `btd`), the camera detector (WebRTC data channel only), hand distance
(inside the theremin). Each is a later `robot.event` kind or a later stream; none blocks v1.

## 5. Yielding to people

`robotd` has no authority arbitration: `robot.move` is last-writer-wins at 50 Hz, and a
brain and a gamepad would fight. Arbitration proper is `architecture.md` open question 3 and
stays there. The brain's rule until then:

- **It watches `move.requested` in `robot.state`.** If it differs from what the brain last
  sent, someone else is driving: the brain stops sending, enters `Yielded`, and stays there
  until the stream shows no foreign intent for 5 s. Same for `head` and for a skill it did
  not ask for.
- **It never enables the robot.** A limp robot stays limp until a person sends
  `robot.enable`; the brain drives a robot someone stood up. (Flag `--autonomous-enable` for
  the sim bench.)
- **It stops on a fall** and waits for `homing`/`stand` before scoring again.
- `braind` starts **paused** on a real robot and is switched on by `robotctl brain on` (its
  own tiny socket, `/run/braind/brain.sock`); on the sim bench, `--on`. Off means silent —
  no intents at all — the same "off = invisible" rule the chorale set.

## 6. The daemon

- Crate `braind`: `lib` (senses, drives, arbiter, behaviours — no sockets, no tokio) + `bin`
  (two client connections to `robotd` — one stream, one prompt lane, as `padd` and the
  theremin's ToF client do — and one to `tofd`; reconnect with `RECONNECT = 2 s`, the
  theremin's pattern). Config in `braind.toml`, defaults commented, same conventions as
  `robotd.toml`; speeds default to `padd`'s (0.3 m/s, 1.5 rad/s) and a lower
  `--max-linear` for a room.
- Packaging: the six places a daemon must be named (workspace `members` and
  `default-members`, `braind/systemd/braind.service` + `sysusers.d`, both build workflows,
  `configd`'s `MANAGED`, `scripts/install.sh`). `SupplementaryGroups=robot`,
  `RestrictAddressFamilies=AF_UNIX`, `RuntimeDirectory=braind`, `Restart=always`.
- Learned state (novelty grid, friends by beacon id, met counts) in
  `/var/lib/robot/brain/`, written on change, loaded on start, absent = fresh duck.

## 7. Testing

- **Library**: every behaviour as a table of `(world, drives) → score` and `tick → intents`
  cases; the arbiter's dwell/margin/jitter with a seeded RNG; the novelty grid; the yield
  rule from recorded `robot.state` frames. No sockets.
- **In the sim**: `plant_server.py --tof --model rollers` + `robotd --sim` + `braind --on
  --autonomous-enable`, scripted scenes — an empty room, a wall ahead, a ball, a loud noise
  injected via a `--fake-event` on the plant. Assertions on the `robot.state` log:
  distance travelled, no fall, no wall contact, at least N distinct behaviours in ten
  minutes, Nap reached from low energy. This is the "ten minutes" test, run headless.
- **On the robot**: `robotctl brain on` in a room, watch `robotctl monitor`, which gains a
  line for the active behaviour and the drives.

## 8. Decisions recorded

| decision | why |
|---|---|
| its own daemon, a client of `robotd` | the loop must not grow; killable; same code on sim and robot |
| drives + scored behaviours, not a fixed graph | "worth watching" needs variety; scores are tunable, graphs are not |
| one behaviour active at a time | the intent surface is single-writer per slot; blending is `robotd`'s deferred business |
| sound/pet/beat events over `robot.event` | the smallest additive change that unblocks v2 |
| yield by observation, no claim | arbitration is an open question above this layer; the rule needs no protocol |
| starts paused on a robot | a duck that wanders because a daemon restarted is a fault |
| `Held` dropped | nothing feeds it |

## 9. Deferred, deliberately

Authority arbitration · RSSI and the camera detector as inputs · the social behaviours'
protocol (telephone, voting, follow-the-leader) · learned preferences beyond visit counts ·
a behaviour editor · anything that needs the speaker beyond the existing sound tags.

## 10. Status (2026-09-07)

Built and passing: crate `braind` (lib: `world`, `drives`, `behaviours`, `arbiter`; bin: the
daemon), 20 unit tests, clippy clean. v1 behaviours from §3 all present. Not yet: the
`robot.event` notification (§4.2, API v17) and the packaging list (§6) — v1 is a bench daemon.

Three-minute sim run (`plant_server.py --tof`, `robotd --sim`, `braind --on
--autonomous-enable`, seed 5): wander · stretch · zoomies · ruffle · preen · chill · look
around · sneeze, 60 novelty cells, ~5 m travelled, no fall, no false yield or startle.

Learned on the way, each now a rule in the code:

- **An active `robot.pose` keeps `robotd` in the standing policy and a twist is ignored.**
  A behaviour that leaves its body pose on (Chill's breathing) makes the next Wander stand
  still for its whole dwell. Every switch now resets pose, head and mouth.
- **The state stream echoes our own twist a frame late.** A yield rule that compares
  `move.requested` with only the last value sent yields to itself after every stop. It
  compares against the last two, with a 0.4 s grace after a change.
- **The depth sensor hangs off the measured head joints (`joints[5..9]`), not the head
  intent (`head`).** With the intent, the floor at HOME's 40° head pitch reprojects as a
  wall 0.39 m away in every sector.
- **A startle needs a straight head.** A closing range while the head is turning is a wall
  coming into view, not a thing coming at the duck.

### 10.1 The maze mission (2026-09-07)

A first mission on top of v1: `braind --maze` against `plant_server.py --model maze`
(`microduck_rl/scripts/make_maze.py` generates a walled maze; the duck boots seated at the
entrance and knows nothing of the layout). Two generations:

- **Reactive left-hand rule** (glance left/ahead/right, decide, turn, walk a cell). Got out
  of a 4×4 in 9 moves / 113 s and a 6×6 in 41 moves / 473 s, but any misread of a side loops
  forever — it has no memory.
- **Map-driven** (`maze.rs`): cells with the walls seen and visit counts, odometry snapped to
  the grid, depth-first with backtracking, glances only at unknown sides, a wall of the
  current cell used as a ruler to correct odometry drift, "nothing within reach" as the
  outside (verified by looking around on arrival). 4×4: **6 moves, the shortest path, 83 s**.
  6×6: not solved — odometry drifts 0.3–0.45 m within ~8 moves and the map stops matching
  the walls; stopped there by decision. The fix, when wanted, is re-localising on every
  cell from all the walls seen, not only when a wall is judged.

Facts the mission established about the walking policy on the deployment plant, each now
a rule in the code: the yaw command is trustworthy only inside ±0.6 rad/s (right turns are
walking arcs); the gait sometimes does not start at 0.3 m/s from a still stand (a yaw nudge
starts it); the sensor's 30° bins graze the corridor's own walls (judge along the ±8° axis).

### 10.2 The room map (2026-09-09)

`room.rs`: an occupancy grid of 0.1 m cells, sparse so it grows from wherever the duck
booted, log-odds per cell — a hit raises it, every ray through it lowers it, a level beam
that saw nothing clears 2 m. Fed from every fresh depth frame while standing, placed in
the world by odometry (no correction: on feet the drift is a few percent, which a room
tolerates; the maze needed better and got wall-ruler corrections of its own).

Wander became frontier exploration: every two seconds a breadth-first search over walkable
cells (free, and not within 0.2 m of anything occupied) to the nearest free cell touching
the unknown, then steering at the waypoint 0.35 m along the path. With no frontier — nothing
mapped yet, or everything seen — the novelty heading stands. The map is written to
`braind-room.svg` every ten seconds and printed as ASCII in the log; `make_room.py` makes a
two-room flat with a doorway and furniture for the plant (`--model room`).

## 11. Open

- Whether `braind`'s config belongs in `robotd.toml` under `[brain]` (one file to edit) or its
  own file (its own update channel). Own file, until the channel exists.
- The cadence: 250 ms arbiter, 20 Hz intents. The prototype's numbers are unknown; these are
  starting points to tune in the sim.
