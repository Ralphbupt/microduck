//! The behaviours: each a small state machine with a score, a minimum dwell, and a tick
//! that yields intents. One is active at a time — the arbiter's business.
//!
//! Names are the prototype's, so the vocabulary stays shared (`brain-design.md` §3). v1 is
//! what runs with `robot.state` and `tof.frame` alone.

use duck_ipc_proto::{Skill, SoundTag};

use crate::drives::Drives;
use crate::maze::{Dir, MazeMap, Plan, Side};
use crate::world::{Mode, Radar, Stale, World};

/// What a behaviour wants sent this tick. `None` fields are "leave it alone".
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Intents {
    /// vx, vy, vyaw — trunk frame. Always sent while a behaviour is active (zero stops).
    pub twist: [f64; 3],
    /// neck_pitch, head_pitch, head_yaw, head_roll, radians, joint space.
    pub head: Option<[f64; 4]>,
    /// z (metres, negative crouches), roll, pitch — body-pose mode.
    pub pose: Option<(f64, f64, f64)>,
    pub skill: Option<Skill>,
    pub sound: Option<SoundTag>,
    /// 0 closed .. 1 open.
    pub mouth: Option<f64>,
}

/// Speeds a behaviour may use. The daemon fills these from the mode and its flags — the
/// walk numbers are `padd`'s full stick, the roller numbers its push/yaw shaping.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    pub linear: f64,
    pub angular: f64,
}

impl Limits {
    pub fn for_mode(mode: Mode, max_linear: f64) -> Self {
        match mode {
            // A walking duck at 0.2 m/s under the daemon's action filter sometimes never
            // starts stepping (sim-backend-design.md §9); 0.3 always does.
            Mode::Walk => Self {
                linear: max_linear.max(0.3),
                angular: 1.0,
            },
            Mode::Roller => Self {
                linear: 0.6,
                angular: 0.3,
            },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    Chill,
    LookAround,
    Wander,
    TurnInPlace,
    Zoomies,
    Stretch,
    Ruffle,
    Preen,
    Sneeze,
    Nap,
    Startle,
    GroundPick,
    /// The mission: get out of the maze by the right-hand rule, glancing with the head.
    Maze,
    /// Someone is stroking the duck: lean in, coo, stay still.
    Petted,
    /// Beats are arriving: bob, sway, nod on them.
    Dance,
    /// Another duck's beacon just appeared: a sound for a stranger, another for a friend.
    Greet,
    /// Nobody around for a long while: call out.
    Lonely,
}

impl Kind {
    pub const ALL: [Kind; 17] = [
        Kind::Chill,
        Kind::LookAround,
        Kind::Wander,
        Kind::TurnInPlace,
        Kind::Zoomies,
        Kind::Stretch,
        Kind::Ruffle,
        Kind::Preen,
        Kind::Sneeze,
        Kind::Nap,
        Kind::Startle,
        Kind::GroundPick,
        Kind::Maze,
        Kind::Petted,
        Kind::Dance,
        Kind::Greet,
        Kind::Lonely,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Kind::Chill => "chill",
            Kind::LookAround => "look_around",
            Kind::Wander => "wander",
            Kind::TurnInPlace => "turn_in_place",
            Kind::Zoomies => "zoomies",
            Kind::Stretch => "stretch",
            Kind::Ruffle => "ruffle",
            Kind::Preen => "preen",
            Kind::Sneeze => "sneeze",
            Kind::Nap => "nap",
            Kind::Startle => "startle",
            Kind::GroundPick => "ground_pick",
            Kind::Maze => "maze",
            Kind::Petted => "petted",
            Kind::Dance => "dance",
            Kind::Greet => "greet",
            Kind::Lonely => "lonely",
        }
    }

    /// Seconds the arbiter keeps a behaviour before a better score may replace it.
    pub fn min_dwell(self) -> f64 {
        match self {
            Kind::Chill => 4.0,
            Kind::LookAround => 3.0,
            Kind::Wander => 6.0,
            Kind::TurnInPlace => 1.0,
            Kind::Zoomies => 2.0,
            Kind::Stretch | Kind::Ruffle | Kind::Preen => 3.0,
            Kind::Sneeze => 1.5,
            Kind::Nap => 40.0,
            Kind::Startle => 1.5,
            Kind::GroundPick => 6.0,
            Kind::Maze => 600.0,
            Kind::Petted => 2.0,
            Kind::Dance => 6.0,
            Kind::Greet => 2.5,
            Kind::Lonely => 3.0,
        }
    }

    /// A reflex may cut a dwell short.
    pub fn is_reflex(self) -> bool {
        matches!(self, Kind::Startle | Kind::Petted | Kind::Greet)
    }

    /// Energy cost per second at full tilt (drives::ENERGY_DRAIN_PER_ACTIVITY × this).
    pub fn activity(self) -> f64 {
        match self {
            Kind::Chill | Kind::Nap => 0.0,
            Kind::LookAround | Kind::Stretch | Kind::Ruffle | Kind::Preen | Kind::Sneeze => 0.1,
            Kind::TurnInPlace | Kind::Startle => 0.3,
            Kind::Wander | Kind::GroundPick | Kind::Maze => 0.5,
            Kind::Zoomies => 1.0,
            Kind::Petted => 0.0,
            Kind::Dance => 0.3,
            Kind::Greet | Kind::Lonely => 0.1,
        }
    }
}

/// The score a behaviour bids with this round. `None`: not applicable now.
pub fn score(
    kind: Kind,
    w: &World,
    d: &Drives,
    rng: &mut fastrand::Rng,
    prev_center: Option<f64>,
) -> Option<f64> {
    if !w.standing() && kind != Kind::Nap {
        return None;
    }
    let o = &w.obstacles;
    let mut jitter = |amp: f64| rng.f64() * amp;
    Some(match kind {
        Kind::Chill => 0.30 + 0.30 * (1.0 - d.energy) + jitter(0.15),
        Kind::LookAround => 0.30 + 0.30 * d.curiosity + jitter(0.20),
        Kind::Wander => {
            if !o.fresh() || o.center.is_some_and(|r| r < 0.35) {
                return None;
            }
            0.25 + 0.55 * d.curiosity + 0.20 * d.energy + jitter(0.10)
        }
        Kind::TurnInPlace => {
            if !o.fresh() {
                return None;
            }
            // Boxed in ahead: turning is the only way to keep exploring.
            if o.center.is_some_and(|r| r < 0.35) {
                0.75 + jitter(0.10)
            } else {
                0.10 + 0.2 * d.curiosity + jitter(0.15)
            }
        }
        Kind::Zoomies => {
            if d.energy < 0.7 || !o.fresh() || o.center.is_none_or(|r| r < 1.2) {
                return None;
            }
            0.35 + 0.45 * d.energy + jitter(0.35)
        }
        Kind::Stretch | Kind::Ruffle | Kind::Preen => 0.20 + 0.10 * d.comfort + jitter(0.35),
        Kind::Sneeze => 0.05 + jitter(0.45),
        Kind::Nap => {
            if w.fallen || w.limp || w.policy == "held" || w.policy == "homing" {
                return None;
            }
            if d.energy < 0.25 {
                0.95
            } else {
                0.05 + 0.25 * (1.0 - d.energy) + jitter(0.05)
            }
        }
        Kind::Startle => {
            // A loud noise startles, unless the duck is being petted (then it is a pat).
            if w.noise_age().is_some_and(|a| a < 0.4) && !w.petting() {
                return Some(1.5);
            }
            // A thing that *came at* the duck, not a wall the head turned toward: only with
            // the head near straight ahead does a closing range count.
            let head_straight = w.head[2].abs() < 0.3 && w.head[1] < 0.3;
            let jumped = head_straight
                && match (prev_center, o.center) {
                    (Some(before), Some(now)) => before - now > 0.30 && now < 0.5,
                    _ => false,
                };
            if o.fresh() && (o.too_close || jumped) {
                1.5
            } else {
                return None;
            }
        }
        Kind::GroundPick => {
            if w.mode == Mode::Roller || !o.fresh() || !o.floor_ahead || d.curiosity < 0.5 {
                return None;
            }
            0.25 + 0.30 * d.curiosity + jitter(0.30)
        }
        // A mission outbids everything, reflexes included: the walls are the point.
        Kind::Maze => {
            if !w.maze || !o.fresh() {
                return None;
            }
            2.0
        }
        // Being petted beats a startle: a hand on the back is not a threat.
        Kind::Petted => {
            if w.petting() {
                1.6
            } else {
                return None;
            }
        }
        Kind::Dance => {
            w.beat_period()?;
            0.9 + 0.2 * d.energy + jitter(0.1)
        }
        Kind::Greet => {
            w.greet_pending?;
            1.4
        }
        Kind::Lonely => {
            if w.alone_for() < 180.0 || d.social > 0.3 || w.company() > 0 {
                return None;
            }
            0.25 + jitter(0.25)
        }
    })
}

/// A running behaviour. `enter` samples what it needs; `tick` is called at the daemon's
/// rate and says whether it is finished.
#[derive(Debug, Clone)]
pub struct Active {
    pub kind: Kind,
    pub since: f64,
    /// Behaviour-local scalars, sampled in `enter`.
    target_yaw: f64,
    head_target: [f64; 4],
    head_at: f64,
    phase: u32,
    fired: bool,
    speed: f64,
    /// Maze: where the current leg started (odom x, y), the map, what we are looking at
    /// or heading for, and how many sides of the current cell looked like open country.
    anchor: [f64; 2],
    pub map: MazeMap,
    /// Correction added to odometry, learned from walls: a wall of this cell seen at range
    /// r must be at the cell's edge, so the difference is odometry drift along that axis.
    odom_fix: [f64; 2],
    stucks: u32,
    /// Walk phase: odometry at the last check and whether a gait kick has been given.
    walk_check: (f64, [f64; 2]),
    kicked: bool,
    retries: u32,
    /// Robot time since which the level beam straight ahead has read under 0.3 m.
    blocked_since: Option<f64>,
    /// Wander: the path to the current frontier target (world points), and when it was
    /// planned; `None` = no frontier known, fall back to a novelty heading.
    pub plan: Option<Vec<(f64, f64)>>,
    planned_at: f64,
    look: Option<Dir>,
    go: Option<Dir>,
    exiting: bool,
    deep_sides: u32,
    phase_since: f64,
}

pub struct Step {
    pub intents: Intents,
    pub done: bool,
}

impl Active {
    pub fn enter(kind: Kind, w: &World, rng: &mut fastrand::Rng) -> Self {
        let mut a = Self {
            kind,
            since: w.t,
            target_yaw: w.yaw,
            head_target: [0.0; 4],
            head_at: w.t,
            phase: 0,
            fired: false,
            speed: 1.0,
            anchor: [w.odom[0], w.odom[1]],
            map: MazeMap::at_entrance(),
            odom_fix: [0.0; 2],
            stucks: 0,
            walk_check: (0.0, [0.0; 2]),
            kicked: false,
            retries: 0,
            blocked_since: None,
            plan: None,
            planned_at: f64::NEG_INFINITY,
            look: None,
            go: None,
            exiting: false,
            deep_sides: 0,
            phase_since: w.t,
        };
        match kind {
            Kind::Wander => a.target_yaw = pick_heading(w, rng),
            Kind::TurnInPlace => {
                // Turn away from the nearer side; a random amount between 60° and 150°.
                let left_room = w.obstacles.left.unwrap_or(9.0);
                let right_room = w.obstacles.right.unwrap_or(9.0);
                let sign = if left_room >= right_room { 1.0 } else { -1.0 };
                a.target_yaw = wrap(w.yaw + sign * (1.0 + rng.f64() * 1.6));
            }
            Kind::LookAround => a.head_target = random_gaze(rng),
            Kind::Zoomies => a.speed = 1.0,
            // A friend is a negative "speed": the one scalar the gesture needs.
            Kind::Greet => {
                a.speed = if w.greet_pending.is_some_and(|(_, stranger)| stranger) {
                    1.0
                } else {
                    -1.0
                }
            }
            _ => {}
        }
        a
    }

    pub fn tick(&mut self, w: &World, limits: Limits, rng: &mut fastrand::Rng) -> Step {
        let age = w.t - self.since;
        let mut i = Intents::default();
        let mut done = false;
        match self.kind {
            Kind::Chill => {
                // Breathe: a slow 3 mm body-pose sway; a glance every few seconds.
                let breath = (age * 0.8).sin() * 0.003;
                i.pose = Some((breath, 0.0, 0.0));
                if w.t - self.head_at > 3.0 + rng.f64() * 3.0 {
                    self.head_target = random_gaze(rng);
                    self.head_at = w.t;
                }
                i.head = Some(self.head_target.map(|v| v * 0.4));
            }
            Kind::LookAround => {
                if w.t - self.head_at > 1.2 + rng.f64() * 1.2 {
                    self.head_target = random_gaze(rng);
                    self.head_at = w.t;
                }
                i.head = Some(self.head_target);
                done = age > 8.0;
            }
            Kind::Wander => {
                let o = &w.obstacles;
                // Frontier exploration: every couple of seconds, a path over the known floor
                // to the nearest edge of the unknown; steer at the waypoint ~0.4 m along it.
                // With no frontier (nothing mapped yet, or everything seen) the novelty
                // heading from `enter` stands.
                if w.t - self.planned_at > 2.0 {
                    self.planned_at = w.t;
                    self.plan = w.room.path_to_frontier((w.odom[0], w.odom[1]), 0.5);
                    if let Some(path) = &self.plan {
                        tracing::debug!(len = path.len(), target = ?path.last(), "wander: frontier");
                    }
                }
                if let Some(path) = &self.plan {
                    let (x, y) = (w.odom[0], w.odom[1]);
                    // The first point on the path at least 0.35 m away, else its end.
                    let aim = path
                        .iter()
                        .copied()
                        .find(|&(px, py)| (px - x).hypot(py - y) >= 0.35)
                        .or_else(|| path.last().copied());
                    if let Some((ax, ay)) = aim {
                        self.target_yaw = (ay - y).atan2(ax - x);
                    }
                }
                let err = wrap(self.target_yaw - w.yaw);
                // Steering while walking stays inside ±0.6: beyond that the sign of the
                // policy's response is not to be trusted (sim-backend-design.md §9).
                let steer = (err * 1.5).clamp(-0.6, 0.6);
                // Slow down near things; stop and let TurnInPlace win when boxed in.
                let room = o.center.unwrap_or(9.0);
                let speed = if room < 0.35 {
                    0.0
                } else if room < 0.7 {
                    0.5
                } else {
                    1.0
                };
                i.twist = [limits.linear * speed, 0.0, steer];
                // Look where we go, slightly down: the sensor must keep seeing the floor.
                i.head = Some([0.0, 0.15, (err * 0.5).clamp(-0.8, 0.8), 0.0]);
                if self.plan.is_none() && age > 5.0 && rng.f64() < 0.01 {
                    self.target_yaw = pick_heading(w, rng);
                }
                done = age > 40.0 || speed == 0.0 && age > 1.0;
            }
            Kind::TurnInPlace => {
                let err = wrap(self.target_yaw - w.yaw);
                // Left in place; right as a tight walking arc (the policy's right turn in
                // place does not start — sim-backend-design.md §9).
                i.twist = if err > 0.0 {
                    [0.0, 0.0, limits.angular]
                } else {
                    [limits.linear, 0.0, -0.6]
                };
                i.head = Some([0.0, 0.1, (err * 0.5).clamp(-0.8, 0.8), 0.0]);
                done = err.abs() < 0.15 || age > 4.0;
            }
            Kind::Zoomies => {
                let room = w.obstacles.center.unwrap_or(9.0);
                let ahead = room > 0.8;
                i.twist = if ahead {
                    [limits.linear * self.speed, 0.0, 0.0]
                } else {
                    [0.0, 0.0, limits.angular]
                };
                i.head = Some([0.0, -0.2, 0.0, 0.0]);
                if !self.fired {
                    i.sound = Some(SoundTag::Wheee);
                    self.fired = true;
                }
                done = age > 3.5;
            }
            Kind::Stretch => {
                // Up on tiptoe, head back, then settle.
                let (z, pitch) = if age < 1.5 { (0.008, -0.2) } else { (0.0, 0.0) };
                i.pose = Some((z, 0.0, pitch));
                i.head = Some([-0.3, -0.3, 0.0, 0.0]);
                done = age > 3.0;
            }
            Kind::Ruffle => {
                let roll = (age * 12.0).sin() * 0.12 * (1.0 - (age / 2.5).min(1.0));
                i.pose = Some((0.0, roll, 0.0));
                i.head = Some([0.0, 0.0, 0.0, (age * 12.0).cos() * 0.25]);
                done = age > 2.5;
            }
            Kind::Preen => {
                // Look under a wing: head down and to one side, mouth working.
                let side = if self.since.rem_euclid(2.0) < 1.0 {
                    1.0
                } else {
                    -1.0
                };
                i.head = Some([0.5, 0.6, side * 1.0, side * 0.3]);
                i.mouth = Some(((age * 6.0).sin() * 0.5 + 0.5) * 0.6);
                done = age > 3.5;
            }
            Kind::Sneeze => {
                if age < 0.5 {
                    i.head = Some([-0.4, -0.4, 0.0, 0.0]);
                } else if age < 0.8 {
                    i.head = Some([0.6, 0.6, 0.0, 0.0]);
                    i.mouth = Some(1.0);
                    if !self.fired {
                        i.sound = Some(SoundTag::Peck);
                        self.fired = true;
                    }
                } else {
                    i.head = Some([0.0; 4]);
                }
                done = age > 1.5;
            }
            Kind::Nap => {
                // Sit once, then be still. Waking is the arbiter's job (energy, a startle).
                if !self.fired && w.standing() {
                    i.skill = Some(Skill::SitToggle);
                    self.fired = true;
                }
                i.head = Some([0.6, 0.5, 0.0, 0.0]);
            }
            Kind::Startle => {
                i.head = Some([-0.4, -0.5, 0.0, 0.0]);
                if !self.fired {
                    i.sound = Some(SoundTag::Alarm);
                    self.fired = true;
                }
                // A hop back, then freeze.
                i.twist = if age < 0.4 {
                    [-limits.linear, 0.0, 0.0]
                } else {
                    [0.0; 3]
                };
                done = age > 1.5;
            }
            Kind::Petted => {
                // Lean into the hand: a little lower, head forward and down, a quiet coo.
                i.pose = Some((-0.008, 0.0, 0.08));
                i.head = Some([0.25, 0.35, 0.0, 0.0]);
                i.mouth = Some(((age * 3.0).sin() * 0.5 + 0.5) * 0.15);
                if !self.fired {
                    i.sound = Some(SoundTag::Coo);
                    self.fired = true;
                }
                done = !w.petting() && w.pet_ended.is_some_and(|t| w.t - t > 1.0);
            }
            Kind::Dance => {
                // Bob and sway on the beat; the phase comes from the last beat heard.
                match (w.beat_period(), w.beats.last()) {
                    (Some(period), Some(&last)) if period > 0.1 => {
                        let phase = ((w.t - last) / period) * std::f64::consts::TAU;
                        i.pose =
                            Some((0.006 * phase.sin() - 0.003, 0.12 * (phase / 2.0).sin(), 0.0));
                        i.head = Some([
                            0.0,
                            0.15 * phase.cos(),
                            0.35 * (phase / 2.0).cos(),
                            0.1 * phase.sin(),
                        ]);
                    }
                    _ => done = true,
                }
                if !self.fired {
                    i.sound = Some(SoundTag::Chirp);
                    self.fired = true;
                }
            }
            Kind::Greet => {
                // Head up, a call — `greet` for a stranger, `chirp` for a friend — then note
                // that this duck is known.
                i.head = Some([-0.2, -0.3, 0.0, 0.0]);
                if !self.fired {
                    self.fired = true;
                    i.sound = Some(if self.speed < 0.0 {
                        SoundTag::Chirp
                    } else {
                        SoundTag::Greet
                    });
                }
                done = age > 2.5;
            }
            Kind::Lonely => {
                i.head = Some([0.0, -0.1, 0.8 * (age * 1.2).sin(), 0.0]);
                if !self.fired {
                    i.sound = Some(SoundTag::Inquire);
                    self.fired = true;
                }
                done = age > 3.0;
            }
            Kind::Maze => {
                let (intents, finished) = self.maze_tick(w, limits);
                i = intents;
                done = finished;
            }
            Kind::GroundPick => {
                match self.phase {
                    0 => {
                        i.skill = Some(Skill::GroundPick);
                        self.phase = 1;
                    }
                    1 => {
                        // Wait for the skill to start, then for the policy to come back.
                        if w.policy == "ground_pick" {
                            self.phase = 2;
                        } else if age > 2.0 {
                            done = true; // it never took: the daemon was busy
                        }
                    }
                    _ => done = w.policy != "ground_pick",
                }
            }
        }
        Step { intents: i, done }
    }

    /// Metres from the duck to the edge of its current cell in direction `d`, given odometry.
    /// Positive inside the cell; the wall (or opening) of this cell lies at exactly this
    /// range, the next cell beyond it.
    fn maze_edge(map: &MazeMap, d: Dir, odom: [f64; 3]) -> f64 {
        use crate::maze::CELL_M;
        let (sign, axis, idx) = match d {
            Dir::East => (1.0, 0usize, map.at.0),
            Dir::West => (-1.0, 0, map.at.0),
            Dir::North => (1.0, 1, map.at.1),
            Dir::South => (-1.0, 1, map.at.1),
        };
        let along = sign * odom[axis];
        let centre = sign * f64::from(idx) * CELL_M;
        centre + CELL_M / 2.0 - along
    }

    /// One step of the maze mission, driven by the map (`maze.rs`): localise on the grid,
    /// ask the explorer what it wants — a glance at an unknown side, a move to an unvisited
    /// neighbour, a backtrack, or the exit — and carry it out. Phases: 0 plan · 1 glance ·
    /// 4 turn · 5 walk one cell.
    fn maze_tick(&mut self, w: &World, limits: Limits) -> (Intents, bool) {
        use crate::maze::CELL_M;
        const GLANCE: f64 = 1.4; // head yaw, radians — the trained range's edge
        const SETTLE_S: f64 = 0.9; // head slew + a few depth frames
        const OUT_M: f64 = 2.5; // sides this deep, two of them: open country, we are out
        const TURN_RATE: f64 = 1.0; // rad/s, the yaw command the walking policy turns left at
        const RIGHT_ARC_RATE: f64 = 0.6; // rad/s with forward speed: a right turn that works
        let mut i = Intents::default();
        let age = w.t - self.phase_since;
        let pos = [
            w.odom[0] + self.odom_fix[0],
            w.odom[1] + self.odom_fix[1],
            w.odom[2],
        ];
        let next = |me: &mut Self, phase: u32| {
            me.phase = phase;
            me.phase_since = w.t;
        };
        match self.phase {
            0 => {
                self.map.localise(pos[0], pos[1], w.yaw);
                // Glances are measured off the body: square up to the cell's axis first,
                // or every look is off by however crooked the last move left us.
                if wrap(self.map.facing.yaw() - w.yaw).abs() > 0.15 {
                    self.target_yaw = self.map.facing.yaw();
                    self.go = None;
                    next(self, 4);
                    return (i, false);
                }
                // Back in the same cell for the fifth time: the map is lying somewhere.
                if self.map.cell(self.map.at).visits > 4 && self.stucks < 3 {
                    self.stucks += 1;
                    tracing::warn!(
                        at = format!("{:?}", self.map.at),
                        "maze: going in circles — forgetting the walls"
                    );
                    self.map.forget_walls();
                }
                if self.deep_sides >= 2 && self.map.moves >= 3 {
                    tracing::info!(
                        moves = self.map.moves,
                        cells = self.map.cells_known(),
                        "maze: open country — out!"
                    );
                    i.sound = Some(SoundTag::Wheee);
                    return (i, true);
                }
                // One deep side means we may be outside: look at every side before moving,
                // so open country is recognised rather than wandered into.
                let plan = match (self.map.plan(), self.map.unknown_ahead()) {
                    (Plan::Go(_), Some(d)) if self.deep_sides >= 1 => Plan::Look(d),
                    (plan, _) => plan,
                };
                match plan {
                    Plan::Look(d) => {
                        self.look = Some(d);
                        next(self, 1);
                    }
                    Plan::Go(d) | Plan::Exit(d) => {
                        self.go = Some(d);
                        self.exiting = matches!(self.map.plan(), Plan::Exit(_));
                        self.target_yaw = d.yaw();
                        self.deep_sides = 0;
                        tracing::info!(
                            at = format!("{:?}", self.map.at),
                            dir = format!("{d:?}"),
                            visits = self.map.cell(d.step(self.map.at)).visits,
                            move_no = self.map.moves + 1,
                            "maze: go"
                        );
                        next(self, 4);
                    }
                    Plan::Stuck => {
                        self.stucks += 1;
                        if self.stucks > 3 {
                            tracing::warn!(
                                at = format!("{:?}", self.map.at),
                                "maze: everything explored, no way out"
                            );
                            return (i, true);
                        }
                        tracing::warn!(
                            at = format!("{:?}", self.map.at),
                            stucks = self.stucks,
                            "maze: contradiction — forgetting the walls, looking again"
                        );
                        if self.stucks == 1 {
                            self.map.forget_here();
                        } else {
                            self.map.forget_walls();
                        }
                    }
                }
            }
            1 => {
                let d = self.look.unwrap_or(self.map.facing);
                let rel_left = d == self.map.facing.left();
                let rel_right = d == self.map.facing.right();
                let yaw = if rel_left {
                    GLANCE
                } else if rel_right {
                    -GLANCE
                } else {
                    0.0
                };
                i.head = Some([0.0, 0.05, yaw, 0.0]);
                if age >= SETTLE_S {
                    // Sides use the perpendicular bin alone: the diagonal bin sees down the
                    // next corridor across a corner and calls a wall an opening.
                    // Ahead likewise uses the centre bin alone (±15°): the wider bins hold
                    // the corridor's own walls at 0.6 m and call every corridor a dead end.
                    let bins: &[usize] = if rel_left {
                        &[Radar::LEFT]
                    } else if rel_right {
                        &[Radar::RIGHT]
                    } else {
                        &[Radar::AHEAD]
                    };
                    let deep = w.radar.deepest(bins.iter().copied(), w.t);
                    // Range straight along the glance, not the bin's grazing edges: an
                    // off-centre duck's side wall otherwise reads as a wall in every direction.
                    let near = if w.radar.fresh(bins[0], w.t) {
                        Ok(w.radar.axis[bins[0]])
                    } else {
                        Err(Stale)
                    };
                    // Open = the sensor sees past this cell's edge into the next cell. The
                    // edge distance comes from where we actually stand, not the cell centre.
                    let edge = Self::maze_edge(&self.map, d, pos).max(0.05);
                    let verdict = match (deep, near) {
                        (Ok(deep), Ok(near)) => {
                            let open = near.is_none_or(|n| n > edge + 0.30) && deep > edge + 0.45;
                            if deep > OUT_M {
                                self.deep_sides += 1;
                            }
                            // Nothing at all within the sensor's reach: not a corridor, the
                            // outside. Verified on arrival by looking around (an unusually
                            // long corridor reads the same way).
                            Some(
                                if near.is_none() && deep >= crate::world::TOF_REACH_M - 0.01 {
                                    Side::Outside
                                } else if open {
                                    Side::Open
                                } else {
                                    Side::Wall
                                },
                            )
                        }
                        _ if age > SETTLE_S + 2.0 => Some(Side::Wall), // no frame: assume a wall
                        _ => None,
                    };
                    if let Some(side) = verdict {
                        // A wall of this cell is a ruler: the odometry drift along that axis
                        // is however far the wall is from where the edge should be.
                        if side == Side::Wall
                            && let Ok(Some(r)) = near
                            && r < 0.9
                        {
                            let delta = (r - edge).clamp(-0.25, 0.25);
                            let (sign, axis) = match d {
                                Dir::East => (1.0, 0),
                                Dir::West => (-1.0, 0),
                                Dir::North => (1.0, 1),
                                Dir::South => (-1.0, 1),
                            };
                            self.odom_fix[axis] -= sign * delta;
                        }
                        self.map.observe(d, side);
                        tracing::info!(
                            at = format!("{:?}", self.map.at),
                            dir = format!("{d:?}"),
                            side = format!("{side:?}"),
                            edge = format!("{edge:.2}"),
                            deep = format!("{:.2}", deep.unwrap_or(0.0)),
                            near = format!("{:.2}", near.ok().flatten().unwrap_or(9.0)),
                            "maze: look"
                        );
                        next(self, 0);
                    }
                }
            }
            4 => {
                // Left: in place at the one yaw rate the gait honours. Right: a tight arc —
                // the policy's response to a yaw command at the trained range's edge (−1.0)
                // is a LEFT turn, but −0.6 with 0.3 m/s forward turns right on a ~0.15 m
                // radius (sim-backend-design.md §9). Back: left, twice.
                let err = wrap(self.target_yaw - w.yaw);
                i.head = Some([0.0, 0.1, 0.0, 0.0]);
                if err.abs() < 0.12 || age > 14.0 {
                    self.anchor = [pos[0], pos[1]];
                    // Squaring up (no move planned) goes back to planning.
                    next(self, if self.go.is_some() { 5 } else { 0 });
                } else if !(-2.6..=0.0).contains(&err) {
                    i.twist = [0.0, 0.0, TURN_RATE];
                } else {
                    i.twist = [limits.linear, 0.0, -RIGHT_ARC_RATE];
                }
            }
            _ => {
                let d = self.go.unwrap_or(self.map.facing);
                // Remaining distance to the next cell's centre: this cell's edge plus half
                // a cell. Walking to the centre re-centres the duck every move.
                let remaining = Self::maze_edge(&self.map, d, pos) + CELL_M / 2.0;
                let err = wrap(self.target_yaw - w.yaw);
                // "Blocked" is the level beam straight ahead (not the floor rows the head
                // sees while walking) under 0.3 m for a third of a second.
                let (ahead, far_ahead) = if w.radar.fresh(Radar::AHEAD, w.t) {
                    (w.radar.axis[Radar::AHEAD], w.radar.far[Radar::AHEAD])
                } else {
                    (None, 9.0)
                };
                // A wall across the way reads short on every beam; a side wall grazed at an
                // angle reads short on one and deep on the rest, and is not "blocked".
                let room = if far_ahead < 0.7 {
                    ahead.unwrap_or(9.0)
                } else {
                    9.0
                };
                if room < 0.30 {
                    self.blocked_since.get_or_insert(w.t);
                } else {
                    self.blocked_since = None;
                }
                let blocked = self.blocked_since.is_some_and(|t0| w.t - t0 >= 0.3);
                let arrived = remaining <= 0.06 || blocked || age > 12.0;
                // The gait sometimes does not start from a still stand at 0.3 m/s; if the
                // odometry has not moved in 2.5 s, a yaw nudge starts it.
                if age < 0.1 {
                    self.walk_check = (w.t, [pos[0], pos[1]]);
                    self.kicked = false;
                }
                let since_check = w.t - self.walk_check.0;
                let moved = ((pos[0] - self.walk_check.1[0]).powi(2)
                    + (pos[1] - self.walk_check.1[1]).powi(2))
                .sqrt();
                let kick = !arrived && since_check > 2.5 && moved < 0.05;
                if kick && !self.kicked {
                    self.kicked = true;
                    tracing::info!("maze: gait did not start — nudging");
                }
                if since_check > 2.5 {
                    self.walk_check = (w.t, [pos[0], pos[1]]);
                }
                if arrived {
                    let d = self.go.take().unwrap_or(self.map.facing);
                    let from = self.map.at;
                    self.map.localise(pos[0], pos[1], w.yaw);
                    let landed = self.map.at;
                    self.map.at = from;
                    if landed == d.step(from) {
                        self.retries = 0;
                        self.map.moved(d, landed);
                        if self.exiting {
                            // Through what looked like the exit: look around before
                            // believing it — two deep sides make it open country.
                            self.exiting = false;
                            self.deep_sides = 1;
                            next(self, 0);
                            return (i, false);
                        }
                    } else if blocked || self.retries >= 2 {
                        // Did not get there and something stood in the way (or it keeps
                        // failing): what looked open was not.
                        tracing::warn!(
                            from = format!("{from:?}"),
                            dir = format!("{d:?}"),
                            landed = format!("{landed:?}"),
                            blocked,
                            "maze: move failed"
                        );
                        self.map.observe(d, Side::Wall);
                        self.map.at = landed;
                        self.retries = 0;
                    } else {
                        // Ran out of time without a wall in the way: the gait stalled. Try again.
                        self.retries += 1;
                        tracing::info!(
                            from = format!("{from:?}"),
                            dir = format!("{d:?}"),
                            retry = self.retries,
                            "maze: move stalled — retrying"
                        );
                        self.map.at = landed;
                    }
                    self.deep_sides = 0;
                    next(self, 0);
                } else {
                    let speed = if room < 0.6 { 0.6 } else { 1.0 };
                    // Hold the corridor's centre line: the lateral offset (left positive)
                    // from the cell's axis becomes a heading bias, so a duck that drifted
                    // toward a wall walks back out instead of scraping along it.
                    let lateral = Self::maze_edge(&self.map, d.left(), pos)
                        - Self::maze_edge(&self.map, d.right(), pos);
                    let centring = (0.5 * lateral).clamp(-0.25, 0.25);
                    let steer = if self.kicked && since_check < 0.6 {
                        0.6
                    } else {
                        (err * 1.5 + centring).clamp(-0.6, 0.6)
                    };
                    i.twist = [limits.linear * speed, 0.0, steer];
                    i.head = Some([0.0, 0.1, 0.0, 0.0]);
                }
            }
        }
        (i, false)
    }

    /// The exit intent: everything to neutral, twist zero. Sent once on leaving.
    pub fn neutral() -> Intents {
        Intents {
            twist: [0.0; 3],
            head: Some([0.0; 4]),
            pose: Some((0.0, 0.0, 0.0)),
            mouth: Some(0.0),
            ..Default::default()
        }
    }
}

/// A world heading that points at the least-visited nearby cell, among those the depth
/// sensor does not object to. Eight candidates, ties broken randomly.
fn pick_heading(w: &World, rng: &mut fastrand::Rng) -> f64 {
    let o = &w.obstacles;
    let mut best = (u32::MAX, w.yaw, 0.0);
    for k in 0..8 {
        let rel = -std::f64::consts::PI + k as f64 * std::f64::consts::FRAC_PI_4;
        // Only the three sectors ahead are seen; sideways/back headings are known only by
        // the grid. Refuse a sector the sensor calls blocked.
        let blocked = if rel.abs() < 0.4 {
            o.center.is_some_and(|r| r < 0.7)
        } else if rel > 0.0 && rel < 1.2 {
            o.left.is_some_and(|r| r < 0.5)
        } else if rel < 0.0 && rel > -1.2 {
            o.right.is_some_and(|r| r < 0.5)
        } else {
            false
        };
        if blocked {
            continue;
        }
        let yaw = wrap(w.yaw + rel);
        let visits = w.grid.visits_ahead(w.odom[0], w.odom[1], yaw, 0.5);
        // Prefer going on over turning around, slightly.
        let tiebreak = rng.f64() + if rel.abs() < 0.4 { -0.3 } else { 0.0 };
        if visits < best.0 || visits == best.0 && tiebreak < best.2 {
            best = (visits, yaw, tiebreak);
        }
    }
    best.1
}

fn random_gaze(rng: &mut fastrand::Rng) -> [f64; 4] {
    [
        0.0,
        -0.3 + rng.f64() * 0.5,
        -0.9 + rng.f64() * 1.8,
        -0.15 + rng.f64() * 0.3,
    ]
}

pub fn wrap(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::Obstacles;

    fn standing() -> World {
        World {
            policy: "stand".into(),
            obstacles: Obstacles {
                age_s: Some(0.1),
                center: Some(2.0),
                floor_ahead: true,
                ..Default::default()
            },
            ..Default::default()
        }
    }

    #[test]
    fn nothing_bids_while_the_robot_is_down_except_nap() {
        let mut rng = fastrand::Rng::with_seed(1);
        let w = World {
            policy: "held".into(),
            ..Default::default()
        };
        let d = Drives::default();
        for k in Kind::ALL {
            assert!(
                score(k, &w, &d, &mut rng, None).is_none(),
                "{k:?} bid on a held robot"
            );
        }
    }

    #[test]
    fn a_wall_ahead_turns_wander_into_turn_in_place() {
        let mut rng = fastrand::Rng::with_seed(2);
        let mut w = standing();
        let d = Drives::default();
        assert!(score(Kind::Wander, &w, &d, &mut rng, None).is_some());
        w.obstacles.center = Some(0.2);
        assert!(score(Kind::Wander, &w, &d, &mut rng, None).is_none());
        assert!(score(Kind::TurnInPlace, &w, &d, &mut rng, None).unwrap() > 0.7);
    }

    #[test]
    fn a_sudden_close_thing_is_a_startle_and_a_stale_frame_is_not() {
        let mut rng = fastrand::Rng::with_seed(3);
        let mut w = standing();
        let d = Drives::default();
        assert!(score(Kind::Startle, &w, &d, &mut rng, Some(2.0)).is_none());
        w.obstacles.center = Some(0.4);
        assert_eq!(score(Kind::Startle, &w, &d, &mut rng, Some(2.0)), Some(1.5));
        w.obstacles.age_s = Some(3.0);
        assert!(score(Kind::Startle, &w, &d, &mut rng, Some(2.0)).is_none());
    }

    #[test]
    fn low_energy_makes_nap_the_obvious_choice() {
        let mut rng = fastrand::Rng::with_seed(4);
        let w = standing();
        let d = Drives {
            energy: 0.1,
            ..Default::default()
        };
        let nap = score(Kind::Nap, &w, &d, &mut rng, None).unwrap();
        for k in Kind::ALL {
            if k == Kind::Nap || k == Kind::Startle {
                continue;
            }
            if let Some(s) = score(k, &w, &d, &mut rng, None) {
                assert!(s < nap, "{k:?} {s} outbid nap {nap}");
            }
        }
    }

    #[test]
    fn wander_drives_toward_its_heading_and_stops_at_a_wall() {
        let mut rng = fastrand::Rng::with_seed(5);
        let mut w = standing();
        let limits = Limits::for_mode(Mode::Walk, 0.3);
        let mut a = Active::enter(Kind::Wander, &w, &mut rng);
        a.target_yaw = w.yaw + 0.5;
        let step = a.tick(&w, limits, &mut rng);
        assert!(step.intents.twist[0] > 0.29 && step.intents.twist[2] > 0.0);
        assert!(!step.done);
        w.obstacles.center = Some(0.3);
        w.t = 2.0;
        let step = a.tick(&w, limits, &mut rng);
        assert_eq!(step.intents.twist[0], 0.0);
        assert!(step.done);
    }

    #[test]
    fn heading_prefers_the_unvisited_cell() {
        let mut rng = fastrand::Rng::with_seed(6);
        let mut w = standing();
        // Visited straight ahead, twice.
        w.grid.visit(0.5, 0.0, 1.0);
        w.grid.visit(0.0, 0.0, 2.0);
        w.grid.visit(0.5, 0.0, 3.0);
        w.grid.visit(0.0, 0.0, 4.0);
        let yaw = pick_heading(&w, &mut rng);
        assert!(
            yaw.abs() > 0.3,
            "went straight into the visited cell: {yaw}"
        );
    }

    #[test]
    fn a_hand_on_the_back_is_a_pet_not_a_startle_and_a_new_duck_is_greeted() {
        let mut rng = fastrand::Rng::with_seed(8);
        let mut w = standing();
        let d = Drives::default();
        w.t = 10.0;
        w.observe_events(&[duck_ipc_proto::RobotEvent {
            kind: duck_ipc_proto::EventKind::SoundNoise,
            id: None,
        }]);
        assert_eq!(score(Kind::Startle, &w, &d, &mut rng, None), Some(1.5));
        w.observe_events(&[duck_ipc_proto::RobotEvent {
            kind: duck_ipc_proto::EventKind::PetStart,
            id: None,
        }]);
        assert!(
            score(Kind::Startle, &w, &d, &mut rng, None).is_none(),
            "petting is not a threat"
        );
        assert_eq!(score(Kind::Petted, &w, &d, &mut rng, None), Some(1.6));
        let limits = Limits::for_mode(Mode::Walk, 0.3);
        let mut pet = Active::enter(Kind::Petted, &w, &mut rng);
        assert_eq!(
            pet.tick(&w, limits, &mut rng).intents.sound,
            Some(SoundTag::Coo)
        );
        w.observe_events(&[duck_ipc_proto::RobotEvent {
            kind: duck_ipc_proto::EventKind::PetEnd,
            id: None,
        }]);
        w.t = 12.0;
        assert!(pet.tick(&w, limits, &mut rng).done);

        w.observe_events(&[duck_ipc_proto::RobotEvent {
            kind: duck_ipc_proto::EventKind::DuckSeen,
            id: Some(3),
        }]);
        assert_eq!(score(Kind::Greet, &w, &d, &mut rng, None), Some(1.4));
        let mut greet = Active::enter(Kind::Greet, &w, &mut rng);
        assert_eq!(
            greet.tick(&w, limits, &mut rng).intents.sound,
            Some(SoundTag::Greet),
            "a stranger"
        );
        w.known_ducks.insert(3);
        w.greet_pending = Some((3, false));
        let mut again = Active::enter(Kind::Greet, &w, &mut rng);
        assert_eq!(
            again.tick(&w, limits, &mut rng).intents.sound,
            Some(SoundTag::Chirp),
            "a friend"
        );

        assert!(
            score(Kind::Dance, &w, &d, &mut rng, None).is_none(),
            "no beat yet"
        );
        for k in 0..4 {
            w.t = 20.0 + 0.5 * k as f64;
            w.observe_events(&[duck_ipc_proto::RobotEvent {
                kind: duck_ipc_proto::EventKind::Beat,
                id: None,
            }]);
        }
        assert!(score(Kind::Dance, &w, &d, &mut rng, None).unwrap() > 0.9);
        let mut dance = Active::enter(Kind::Dance, &w, &mut rng);
        assert!(dance.tick(&w, limits, &mut rng).intents.pose.is_some());
        w.t = 30.0;
        assert!(dance.tick(&w, limits, &mut rng).done, "the music stopped");
    }

    #[test]
    fn nap_sits_once_and_ground_pick_waits_for_the_skill() {
        let mut rng = fastrand::Rng::with_seed(7);
        let mut w = standing();
        let limits = Limits::for_mode(Mode::Walk, 0.3);
        let mut nap = Active::enter(Kind::Nap, &w, &mut rng);
        assert_eq!(
            nap.tick(&w, limits, &mut rng).intents.skill,
            Some(Skill::SitToggle)
        );
        assert_eq!(nap.tick(&w, limits, &mut rng).intents.skill, None);

        let mut pick = Active::enter(Kind::GroundPick, &w, &mut rng);
        assert_eq!(
            pick.tick(&w, limits, &mut rng).intents.skill,
            Some(Skill::GroundPick)
        );
        w.policy = "ground_pick".into();
        assert!(!pick.tick(&w, limits, &mut rng).done);
        w.policy = "stand".into();
        assert!(pick.tick(&w, limits, &mut rng).done);
    }
}
