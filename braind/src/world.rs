//! Senses: wire messages become a `World` the behaviours can reason about.
//!
//! Everything a behaviour asks — is the way ahead clear, am I standing, where have I not
//! been — is answered here, from `robot.state`, `tof.frame`, and memory. Nothing in this
//! module talks to a socket; the daemon feeds it and the tests feed it by hand.

use std::collections::HashMap;

use std::collections::HashSet;

use duck_ipc_proto::{EventKind, RobotEvent, RobotState, TofFrame};
use kinematics::tof::{COLS, Posture, ROWS, Reprojector, Zone};

/// ST status codes `tofd` marks as a trustworthy range — the same set the theremin's hand
/// tracker accepts (`deploy/robotd.toml` `[theremin] statuses`).
const VALID_STATUS: [u8; 7] = [4, 5, 6, 9, 10, 12, 13];

/// A frame older than this is not evidence about the room any more.
pub const TOF_STALE_S: f64 = 0.5;

/// Novelty grid cell, metres. A duck-length; finer would count fidgeting as exploring.
pub const CELL_M: f64 = 0.25;

/// Which drive mode the robot is in — it changes what a "push" means.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    #[default]
    Walk,
    Roller,
}

/// What the depth sensor says about the room, in three sectors, horizontal range in metres
/// from the trunk axis. `None` = nothing in range in that sector (or no return).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Obstacles {
    pub left: Option<f64>,
    pub center: Option<f64>,
    pub right: Option<f64>,
    /// Something inside the sensor's short-range noise band — an object at the beak, or a
    /// hand. Startle material.
    pub too_close: bool,
    /// The floor is confirmed somewhere ahead: the sensor is looking down enough to see it
    /// and nothing is in the way. Ground pick wants this.
    pub floor_ahead: bool,
    /// Age of the frame these came from, seconds. `None` = never had one.
    pub age_s: Option<f64>,
}

impl Obstacles {
    pub fn fresh(&self) -> bool {
        self.age_s.is_some_and(|a| a <= TOF_STALE_S)
    }

    /// The nearest thing in the sector a heading points at: forward → center.
    pub fn nearest(&self) -> Option<f64> {
        [self.left, self.center, self.right]
            .into_iter()
            .flatten()
            .reduce(f64::min)
    }
}

/// Where the duck has been. Visit counts per cell, so Wander can prefer the unvisited.
#[derive(Debug, Default, Clone)]
pub struct NoveltyGrid {
    cells: HashMap<(i32, i32), u32>,
    last: Option<(i32, i32)>,
    /// Sim/robot time of the last time a *new* cell was entered.
    pub last_new_t: f64,
}

impl NoveltyGrid {
    fn key(x: f64, y: f64) -> (i32, i32) {
        ((x / CELL_M).floor() as i32, (y / CELL_M).floor() as i32)
    }

    /// Record the robot at `(x, y)`. Returns true when this is a cell never seen before.
    pub fn visit(&mut self, x: f64, y: f64, t: f64) -> bool {
        let k = Self::key(x, y);
        if self.last == Some(k) {
            return false;
        }
        self.last = Some(k);
        let count = self.cells.entry(k).or_insert(0);
        *count += 1;
        if *count == 1 {
            self.last_new_t = t;
            true
        } else {
            false
        }
    }

    pub fn visits(&self, x: f64, y: f64) -> u32 {
        self.cells.get(&Self::key(x, y)).copied().unwrap_or(0)
    }

    pub fn cells_seen(&self) -> usize {
        self.cells.len()
    }

    /// Score a heading (world yaw) by how unvisited the cell `look_m` ahead is: lower is
    /// more novel. Ties broken by the caller's jitter.
    pub fn visits_ahead(&self, x: f64, y: f64, yaw: f64, look_m: f64) -> u32 {
        self.visits(x + look_m * yaw.cos(), y + look_m * yaw.sin())
    }
}

/// Bearing bins around the trunk, 30° each from −105° to +105°; index 3 is dead ahead,
/// 0 is the right side, 6 the left. The depth sensor covers ~45° at a time, so a full
/// picture is built by turning the head — this is what remembers the glances.
pub const RADAR_BINS: usize = 7;
pub const RADAR_BIN_DEG: f64 = 30.0;
pub const RADAR_MAX_AGE_S: f64 = 3.0;
/// What "saw nothing" means in metres: the sensor's useful reach.
pub const TOF_REACH_M: f64 = 4.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Radar {
    /// Nearest hit per bin, horizontal metres; `None` = looked and saw nothing in range.
    pub range: [Option<f64>; RADAR_BINS],
    /// Farthest a beam in the bin reached: the deepest hit, or `TOF_REACH_M` when some beam
    /// saw nothing at all. An opening in a wall shows up here even when the wall's edge is
    /// the nearest thing.
    pub far: [f64; RADAR_BINS],
    /// Nearest hit among only the beams within ±8° of the bin's centre line — the range
    /// straight along that bearing, unclipped by a side wall the bin's edges graze.
    pub axis: [Option<f64>; RADAR_BINS],
    /// Robot time the bin was last covered by a frame; −∞ = never.
    pub at: [f64; RADAR_BINS],
}

impl Default for Radar {
    fn default() -> Self {
        Self {
            range: [None; RADAR_BINS],
            far: [0.0; RADAR_BINS],
            axis: [None; RADAR_BINS],
            at: [f64::NEG_INFINITY; RADAR_BINS],
        }
    }
}

impl Radar {
    pub fn bin_of(bearing_rad: f64) -> Option<usize> {
        let deg = bearing_rad.to_degrees() + RADAR_BIN_DEG * RADAR_BINS as f64 / 2.0;
        let b = (deg / RADAR_BIN_DEG).floor();
        (b >= 0.0 && b < RADAR_BINS as f64).then_some(b as usize)
    }

    /// The bin a trunk-frame bearing (radians, left positive) falls in.
    pub const RIGHT: usize = 0;
    pub const AHEAD: usize = 3;
    pub const LEFT: usize = 6;

    /// Whether `bin` has been looked at within `RADAR_MAX_AGE_S`.
    pub fn fresh(&self, bin: usize, now: f64) -> bool {
        now - self.at[bin] <= RADAR_MAX_AGE_S
    }

    /// How deep the sensor saw across `bins`; `Err` if any is stale.
    pub fn deepest(&self, bins: impl IntoIterator<Item = usize>, now: f64) -> Result<f64, ()> {
        let mut best = 0.0f64;
        for b in bins {
            if !self.fresh(b, now) {
                return Err(());
            }
            best = best.max(self.far[b]);
        }
        Ok(best)
    }

    /// The nearest thing across `bins`, or `None` if all clear; `Err` if any is stale.
    pub fn nearest(
        &self,
        bins: impl IntoIterator<Item = usize>,
        now: f64,
    ) -> Result<Option<f64>, ()> {
        let mut best: Option<f64> = None;
        for b in bins {
            if !self.fresh(b, now) {
                return Err(());
            }
            if let Some(r) = self.range[b] {
                best = Some(best.map_or(r, |x: f64| x.min(r)));
            }
        }
        Ok(best)
    }
}

/// Everything a behaviour is allowed to know.
#[derive(Debug, Clone, Default)]
pub struct World {
    /// A mission flag: solve the maze the plant put us in (`braind --maze`).
    pub maze: bool,
    pub radar: Radar,
    /// Robot time, seconds (`robot.state.t`).
    pub t: f64,
    pub mode: Mode,
    /// `robot.state.policy`: `walk`, `stand`, `sit`, `held`, `homing`, `rise`, a skill…
    pub policy: String,
    pub fallen: bool,
    pub limp: bool,
    pub gravity: [f64; 3],
    /// Odometry: x, y, z metres in the frame the robot came up in; yaw radians.
    pub odom: [f64; 3],
    pub yaw: f64,
    /// Measured head joints: neck_pitch, head_pitch, head_yaw, head_roll.
    pub head: [f64; 4],
    /// What some client asked for this tick — the yield rule compares it with what we sent.
    pub move_requested: [f64; 3],
    pub obstacles: Obstacles,
    pub grid: NoveltyGrid,
    /// Whether a state frame has ever arrived.
    pub have_state: bool,
    /// Events, as ages: the robot time of the last one of each kind.
    pub last_noise: Option<f64>,
    pub last_voice: Option<f64>,
    pub petting_since: Option<f64>,
    pub pet_ended: Option<f64>,
    /// Ducks by beacon id: when each was last seen; and every duck ever met.
    pub ducks: HashMap<u16, f64>,
    pub known_ducks: HashSet<u16>,
    /// A duck just seen and not yet greeted: its id, and whether it is a stranger.
    pub greet_pending: Option<(u16, bool)>,
    /// Recent beat times, newest last (a handful are kept).
    pub beats: Vec<f64>,
    /// Robot time the robot last had company (a duck or a voice); `None` = never.
    pub last_company: Option<f64>,
}

impl World {
    /// Standing under a policy, ready to take an intent.
    pub fn standing(&self) -> bool {
        matches!(self.policy.as_str(), "stand" | "walk") && !self.fallen && !self.limp
    }

    /// Between things: a skill, a rise, homing, sitting. Wait.
    pub fn busy(&self) -> bool {
        !matches!(self.policy.as_str(), "stand" | "walk")
    }

    pub fn sitting(&self) -> bool {
        self.policy == "sit"
    }

    /// Fold in a state frame.
    pub fn observe_state(&mut self, s: &RobotState) {
        self.have_state = true;
        self.t = s.t;
        self.policy = s.policy.clone();
        self.fallen = s.safety.fallen;
        self.limp = s.safety.limp;
        self.gravity = s.safety.gravity;
        self.odom = s.odom.position;
        self.yaw = s.odom.yaw;
        // Measured head joints, not the head *intent* (`s.head`): the depth sensor hangs off
        // where the head actually is. `robotctl monitor` reads the same four slots.
        if let Some(h) = s.joints.get(5..9) {
            self.head = [h[0], h[1], h[2], h[3]];
        }
        self.move_requested = s.movement.requested;
        if self.standing() {
            self.grid.visit(self.odom[0], self.odom[1], self.t);
        }
        // `s.events` is deliberately not folded in here: the daemon accumulates events across
        // frames and hands them to `observe_events` itself, so none is lost or counted twice.
    }

    /// Fold in events — from a state frame, or injected on a bench.
    pub fn observe_events(&mut self, events: &[RobotEvent]) {
        for e in events {
            match e.kind {
                EventKind::SoundNoise => self.last_noise = Some(self.t),
                EventKind::SoundVoice => {
                    self.last_voice = Some(self.t);
                    self.last_company = Some(self.t);
                }
                EventKind::PetStart => {
                    self.petting_since = Some(self.t);
                    self.pet_ended = None;
                }
                EventKind::PetEnd => {
                    self.petting_since = None;
                    self.pet_ended = Some(self.t);
                }
                EventKind::DuckSeen => {
                    if let Some(id) = e.id {
                        let stranger = !self.known_ducks.contains(&id);
                        self.ducks.insert(id, self.t);
                        self.greet_pending = Some((id, stranger));
                        self.last_company = Some(self.t);
                    }
                }
                EventKind::DuckLost => {
                    if let Some(id) = e.id {
                        self.ducks.remove(&id);
                    }
                }
                EventKind::Beat => {
                    self.beats.push(self.t);
                    if self.beats.len() > 8 {
                        self.beats.remove(0);
                    }
                }
            }
        }
    }

    pub fn petting(&self) -> bool {
        self.petting_since.is_some()
    }

    /// Seconds since the last loud noise; `None` = never.
    pub fn noise_age(&self) -> Option<f64> {
        self.last_noise.map(|t| self.t - t)
    }

    pub fn voice_age(&self) -> Option<f64> {
        self.last_voice.map(|t| self.t - t)
    }

    /// Ducks heard from in the last ten seconds.
    pub fn company(&self) -> usize {
        self.ducks.values().filter(|&&t| self.t - t < 10.0).count()
    }

    /// How long since anyone was around; from boot if never.
    pub fn alone_for(&self) -> f64 {
        self.t - self.last_company.unwrap_or(0.0)
    }

    /// The beat period, if beats have been arriving steadily (three in the last four seconds).
    pub fn beat_period(&self) -> Option<f64> {
        let recent: Vec<f64> = self
            .beats
            .iter()
            .copied()
            .filter(|&t| self.t - t < 4.0)
            .collect();
        if recent.len() < 3 {
            return None;
        }
        let gaps: Vec<f64> = recent.windows(2).map(|w| w[1] - w[0]).collect();
        Some(gaps.iter().sum::<f64>() / gaps.len() as f64)
    }

    /// Fold in a depth frame, reprojected through the head pose the state stream reports.
    pub fn observe_tof(&mut self, frame: &TofFrame, reprojector: &Reprojector, age_s: f64) {
        let mut ranges = [None; ROWS * COLS];
        for (i, slot) in ranges.iter_mut().enumerate() {
            let (Some(&d), Some(&st)) = (frame.distance_mm.get(i), frame.status.get(i)) else {
                continue;
            };
            if d > 0 && VALID_STATUS.contains(&st) {
                *slot = Some(f64::from(d) / 1000.0);
            }
        }
        let posture = Posture {
            gravity: self.gravity,
            trunk_height_m: (self.odom[2] > 0.02).then_some(self.odom[2]),
        };
        let zones = reprojector.project(&ranges, self.head, &posture);
        self.obstacles = summarise(&zones, age_s);

        // The radar: which bins this frame covered, and the nearest hit in each.
        let sensor = reprojector.sensor_in_trunk(self.head);
        let mut covered = [false; RADAR_BINS];
        let mut nearest: [Option<f64>; RADAR_BINS] = [None; RADAR_BINS];
        let mut deepest = [0.0f64; RADAR_BINS];
        let mut on_axis: [Option<f64>; RADAR_BINS] = [None; RADAR_BINS];
        for (i, beam) in reprojector.beams().iter().enumerate() {
            let dir = sensor.quat.rotate(*beam);
            let bearing = dir[1].atan2(dir[0]);
            let Some(bin) = Radar::bin_of(bearing) else {
                continue;
            };
            let centre =
                (bin as f64 - (RADAR_BINS as f64 - 1.0) / 2.0) * RADAR_BIN_DEG.to_radians();
            let near_axis = (bearing - centre).abs() <= 8.0f64.to_radians();
            // Only the beams near the horizon vote: a floor beam looks at the floor, not
            // the room, and a sky beam sees nothing whatever is there.
            // Level or slightly down only: a beam tilted up looks over a wall top and
            // reports "nothing", which is not an opening.
            if !(-0.45..=0.05).contains(&dir[2]) {
                continue;
            }
            covered[bin] = true;
            match zones[i] {
                Zone::Hit { range, .. } => {
                    nearest[bin] = Some(nearest[bin].map_or(range, |r: f64| r.min(range)));
                    deepest[bin] = deepest[bin].max(range);
                    if near_axis {
                        on_axis[bin] = Some(on_axis[bin].map_or(range, |r: f64| r.min(range)));
                    }
                }
                Zone::Empty => deepest[bin] = deepest[bin].max(TOF_REACH_M),
                _ => {}
            }
        }
        for b in 0..RADAR_BINS {
            if covered[b] {
                self.radar.range[b] = nearest[b];
                self.radar.far[b] = deepest[b];
                self.radar.axis[b] = on_axis[b];
                self.radar.at[b] = self.t;
            }
        }
    }
}

/// Three sectors by column — column 0 is the sensor's left — nearest hit each, plus the two
/// flags. Rows are all pooled: a low obstacle and a high one both stop a duck.
pub fn summarise(zones: &[Zone; ROWS * COLS], age_s: f64) -> Obstacles {
    let mut o = Obstacles {
        age_s: Some(age_s),
        ..Default::default()
    };
    let mut floor_center = 0;
    for (i, z) in zones.iter().enumerate() {
        let col = i % COLS;
        let sector = if col < 3 {
            &mut o.left
        } else if col < 5 {
            &mut o.center
        } else {
            &mut o.right
        };
        match z {
            Zone::Hit { range, .. } => {
                *sector = Some(sector.map_or(*range, |r: f64| r.min(*range)));
            }
            Zone::TooClose => o.too_close = true,
            Zone::Floor { .. } if (3..5).contains(&col) => floor_center += 1,
            _ => {}
        }
    }
    o.floor_ahead = floor_center >= 2 && o.center.is_none_or(|r| r > 0.4);
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn novelty_counts_cells_not_ticks() {
        let mut g = NoveltyGrid::default();
        assert!(g.visit(0.0, 0.0, 1.0));
        assert!(!g.visit(0.05, 0.05, 2.0), "same cell, no new visit");
        assert!(g.visit(0.3, 0.0, 3.0), "next cell over");
        assert!(!g.visit(0.0, 0.0, 4.0), "back home: known");
        assert_eq!(g.visits(0.0, 0.0), 2);
        assert_eq!(g.cells_seen(), 2);
        assert_eq!(g.last_new_t, 3.0);
        assert_eq!(g.visits_ahead(0.0, 0.0, 0.0, 0.3), 1);
        assert_eq!(
            g.visits_ahead(0.0, 0.0, std::f64::consts::FRAC_PI_2, 0.3),
            0
        );
    }

    #[test]
    fn sectors_take_the_nearest_hit_and_flag_the_floor() {
        let mut zones = [Zone::Empty; ROWS * COLS];
        zones[0] = Zone::Hit {
            point: [0.0; 3],
            range: 0.9,
        }; // left, row 0
        zones[COLS + 1] = Zone::Hit {
            point: [0.0; 3],
            range: 0.4,
        }; // left, row 1
        zones[3] = Zone::Hit {
            point: [0.0; 3],
            range: 1.2,
        }; // center
        zones[7 * COLS + 3] = Zone::Floor { point: [0.0; 3] };
        zones[7 * COLS + 4] = Zone::Floor { point: [0.0; 3] };
        zones[6] = Zone::TooClose;
        let o = summarise(&zones, 0.1);
        assert_eq!(o.left, Some(0.4));
        assert_eq!(o.center, Some(1.2));
        assert_eq!(o.right, None);
        assert!(o.too_close);
        assert!(o.floor_ahead);
        assert!(o.fresh());
        assert_eq!(o.nearest(), Some(0.4));
        let stale = summarise(&zones, 2.0);
        assert!(!stale.fresh());
    }

    #[test]
    fn events_become_ages_company_and_beats() {
        let mut w = World {
            t: 100.0,
            ..Default::default()
        };
        let ev = |kind, id| RobotEvent { kind, id };
        w.observe_events(&[
            ev(EventKind::PetStart, None),
            ev(EventKind::DuckSeen, Some(7)),
        ]);
        assert!(w.petting());
        assert_eq!(w.company(), 1);
        assert_eq!(w.greet_pending, Some((7, true)), "a stranger");
        w.known_ducks.insert(7);
        w.t = 101.0;
        w.observe_events(&[
            ev(EventKind::PetEnd, None),
            ev(EventKind::DuckSeen, Some(7)),
        ]);
        assert!(!w.petting());
        assert_eq!(w.greet_pending, Some((7, false)), "a friend now");
        for k in 0..4 {
            w.t = 102.0 + 0.5 * k as f64;
            w.observe_events(&[ev(EventKind::Beat, None)]);
        }
        assert!((w.beat_period().unwrap() - 0.5).abs() < 1e-9);
        w.t = 110.0;
        assert!(w.beat_period().is_none(), "beats went stale");
        assert!((w.alone_for() - 9.0).abs() < 1e-9);
        w.observe_events(&[ev(EventKind::DuckLost, Some(7))]);
        assert_eq!(w.company(), 0);
    }

    #[test]
    fn standing_means_a_policy_is_holding_it_up() {
        let mut w = World {
            policy: "stand".into(),
            ..Default::default()
        };
        assert!(w.standing() && !w.busy());
        w.policy = "rise".into();
        assert!(!w.standing() && w.busy());
        w.policy = "walk".into();
        w.fallen = true;
        assert!(!w.standing());
    }
}
