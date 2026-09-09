//! The arbiter: one behaviour at a time, chosen by score, kept for its dwell, and dropped
//! when something else is clearly better or it says it is done. Plus the yield rule — when
//! another client is driving, the brain goes quiet.

use crate::behaviours::{Active, Intents, Kind, Limits, score};
use crate::drives::Drives;
use crate::world::World;

/// How often bids are taken.
pub const ROUND_S: f64 = 0.25;
/// A challenger must beat the incumbent by this much.
pub const MARGIN: f64 = 0.10;
/// After a foreign intent is seen, stay quiet this long past the last one.
pub const YIELD_S: f64 = 5.0;
/// Twist difference that counts as "someone else's".
const FOREIGN_EPS: f64 = 0.02;
/// Energy at which a nap is over.
pub const WAKE_ENERGY: f64 = 0.85;
/// After our own twist changes, how long a mismatching request is still assumed to be the
/// stream lagging rather than someone else.
const SEND_GRACE_S: f64 = 0.4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Not enabled: says nothing, sends nothing.
    Off,
    /// Enabled but the robot is not standing under a policy (held, rising, fallen).
    Waiting,
    /// Someone else is driving.
    Yielded,
    Active(Kind),
}

pub struct Arbiter {
    pub on: bool,
    pub drives: Drives,
    rng: fastrand::Rng,
    active: Option<Active>,
    current_score: f64,
    last_round: f64,
    last_sent_twist: [f64; 3],
    /// The twist before that, and when it changed: the state stream lags our own sends by
    /// a frame, so a request that matches either recent value is ours.
    prev_sent_twist: [f64; 3],
    twist_changed_at: f64,
    foreign_until: f64,
    prev_center: Option<f64>,
    last_t: Option<f64>,
    /// Set by a switch, consumed by the next tick: the newcomer's first intents reset
    /// whatever the last behaviour left on the robot.
    entered: bool,
    /// The maze was solved once; the mission does not restart.
    pub maze_solved: bool,
    /// The map the last maze mission built, kept after it ends for drawing.
    pub last_maze: Option<crate::maze::MazeMap>,
    /// The last frame's novelty count, so a new cell is noticed once.
    cells_seen: usize,
    cooldown: std::collections::HashMap<Kind, f64>,
}

/// What the daemon should do this tick.
pub struct Decision {
    pub status: Status,
    /// `None` = send nothing at all (off, waiting, yielded).
    pub intents: Option<Intents>,
    /// A behaviour change worth a log line.
    pub changed: Option<(Option<Kind>, Option<Kind>)>,
}

impl Arbiter {
    pub fn new(seed: u64, on: bool) -> Self {
        Self {
            on,
            drives: Drives::default(),
            rng: fastrand::Rng::with_seed(seed),
            active: None,
            current_score: 0.0,
            last_round: f64::NEG_INFINITY,
            last_sent_twist: [0.0; 3],
            prev_sent_twist: [0.0; 3],
            twist_changed_at: f64::NEG_INFINITY,
            foreign_until: f64::NEG_INFINITY,
            prev_center: None,
            last_t: None,
            entered: false,
            maze_solved: false,
            last_maze: None,
            cells_seen: 0,
            cooldown: Default::default(),
        }
    }

    pub fn active_kind(&self) -> Option<Kind> {
        self.active.as_ref().map(|a| a.kind)
    }

    /// Where Wander is heading, if it is following a frontier path.
    pub fn wander_target(&self) -> Option<(f64, f64)> {
        match &self.active {
            Some(a) if a.kind == Kind::Wander => a.plan.as_ref().and_then(|p| p.last().copied()),
            _ => None,
        }
    }

    /// The maze map while the mission runs (for the log), else the last one.
    pub fn maze_map(&self) -> Option<&crate::maze::MazeMap> {
        match &self.active {
            Some(a) if a.kind == Kind::Maze => Some(&a.map),
            _ => self.last_maze.as_ref(),
        }
    }

    pub fn tick(&mut self, w: &World, limits: Limits) -> Decision {
        let dt = self.last_t.map_or(0.0, |t| (w.t - t).clamp(0.0, 1.0));
        self.last_t = Some(w.t);
        let was = self.active_kind();

        if !self.on || !w.have_state {
            return self.stop(w, Status::Off, was);
        }

        // Yield: a twist we did not send means a person (or another client) has the robot.
        // Ours are the last two values sent, with a grace after a change for the stream to
        // catch up.
        let differs =
            |sent: &[f64; 3]| (0..3).any(|k| (w.move_requested[k] - sent[k]).abs() > FOREIGN_EPS);
        let foreign = w.move_requested.iter().any(|v| v.abs() > FOREIGN_EPS)
            && differs(&self.last_sent_twist)
            && differs(&self.prev_sent_twist)
            && w.t - self.twist_changed_at > SEND_GRACE_S;
        if foreign {
            self.foreign_until = w.t + YIELD_S;
        }
        if w.t < self.foreign_until {
            return self.stop(w, Status::Yielded, was);
        }

        // Drives move whether or not anything is chosen.
        let napping = self.active_kind() == Some(Kind::Nap);
        let activity = self.active_kind().map_or(0.0, Kind::activity);
        let new_place = w.grid.cells_seen() > self.cells_seen;
        self.cells_seen = w.grid.cells_seen();
        self.drives.update(dt, activity, napping, new_place, w);

        // Only a napping brain may act on a sitting robot; anything else waits to stand.
        if !(w.standing() || napping && w.sitting()) {
            if w.busy() && !w.sitting() {
                // A skill or a rise in progress: keep the active behaviour, send nothing.
                return Decision {
                    status: Status::Waiting,
                    intents: None,
                    changed: None,
                };
            }
            return self.stop(w, Status::Waiting, was);
        }

        // A round of bids.
        if w.t - self.last_round >= ROUND_S {
            self.last_round = w.t;
            let mut best: Option<(Kind, f64)> = None;
            for kind in Kind::ALL {
                if self.cooldown.get(&kind).is_some_and(|&until| w.t < until) {
                    continue;
                }
                if kind == Kind::Maze && self.maze_solved {
                    continue;
                }
                if let Some(s) = score(kind, w, &self.drives, &mut self.rng, self.prev_center)
                    && best.is_none_or(|(_, b)| s > b)
                {
                    best = Some((kind, s));
                }
            }
            self.prev_center = w.obstacles.center;
            if let Some((kind, s)) = best {
                let switch = match &self.active {
                    None => true,
                    Some(a) if a.kind == kind => {
                        self.current_score = s;
                        false
                    }
                    Some(a) => {
                        let dwelt = w.t - a.since >= a.kind.min_dwell();
                        (dwelt || kind.is_reflex()) && s > self.current_score + MARGIN
                    }
                };
                if switch {
                    self.switch_to(kind, s, w);
                }
            }
        }

        // Waking: a nap ends when energy is back, and the duck stands itself up.
        if let Some(a) = &self.active
            && a.kind == Kind::Nap
            && self.drives.energy > WAKE_ENERGY
            && w.t - a.since >= Kind::Nap.min_dwell()
        {
            self.leave(w, Kind::Nap);
            self.current_score = 0.0;
            self.sent([0.0; 3], w.t);
            let mut up = Active::neutral();
            if w.sitting() {
                up.skill = Some(duck_ipc_proto::Skill::from("sit_toggle"));
            }
            return Decision {
                status: Status::Waiting,
                intents: Some(up),
                changed: Some((was, None)),
            };
        }

        let Some(active) = self.active.as_mut() else {
            return self.stop(w, Status::Waiting, was);
        };
        let kind = active.kind;
        let step = active.tick(w, limits, &mut self.rng);
        if kind == Kind::Startle && was != Some(Kind::Startle) {
            self.drives.startled();
        }
        let mut intents = step.intents;
        if step.done {
            if kind == Kind::Maze {
                self.maze_solved = true;
                self.last_maze = Some(active.map.clone());
            }
            self.leave(w, kind);
            let mut out = Active::neutral();
            out.sound = intents.sound; // a parting sound survives the reset
            intents = out;
            self.current_score = 0.0;
        } else if self.entered {
            self.entered = false;
            // A fresh behaviour inherits nothing: the last one's body pose, head and mouth
            // go to neutral unless it sets them itself. An active body pose keeps `robotd`
            // in the standing policy, where a twist is ignored — Wander after Chill would
            // stand still for its whole dwell.
            intents.pose.get_or_insert((0.0, 0.0, 0.0));
            intents.head.get_or_insert([0.0; 4]);
            intents.mouth.get_or_insert(0.0);
        }
        self.sent(intents.twist, w.t);
        Decision {
            status: self.active_kind().map_or(Status::Waiting, Status::Active),
            intents: Some(intents),
            changed: (was != self.active_kind()).then_some((was, self.active_kind())),
        }
    }

    fn sent(&mut self, twist: [f64; 3], t: f64) {
        if twist != self.last_sent_twist {
            self.prev_sent_twist = self.last_sent_twist;
            self.last_sent_twist = twist;
            self.twist_changed_at = t;
        }
    }

    fn switch_to(&mut self, kind: Kind, s: f64, w: &World) {
        if let Some(prev) = self.active.take() {
            self.leave(w, prev.kind);
        }
        self.active = Some(Active::enter(kind, w, &mut self.rng));
        self.current_score = s;
        self.entered = true;
    }

    fn leave(&mut self, w: &World, kind: Kind) {
        self.active = None;
        // Gestures and picks do not repeat straight away.
        let cool = match kind {
            Kind::Stretch | Kind::Ruffle | Kind::Preen | Kind::Sneeze => 20.0,
            Kind::GroundPick | Kind::Zoomies => 40.0,
            Kind::Startle => 3.0,
            Kind::Lonely => 120.0,
            Kind::Dance => 10.0,
            _ => 0.0,
        };
        if cool > 0.0 {
            self.cooldown.insert(kind, w.t + cool);
        }
    }

    fn stop(&mut self, w: &World, status: Status, was: Option<Kind>) -> Decision {
        let had = self.active.is_some();
        if let Some(a) = self.active.take() {
            self.leave(w, a.kind);
        }
        self.current_score = 0.0;
        self.sent([0.0; 3], w.t);
        Decision {
            status,
            // One neutral on the way out, so a twist is not left running.
            intents: had.then(Active::neutral),
            changed: had.then_some((was, None)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::{Mode, Obstacles};

    fn standing(t: f64) -> World {
        World {
            t,
            policy: "stand".into(),
            have_state: true,
            obstacles: Obstacles {
                age_s: Some(0.1),
                center: Some(2.0),
                floor_ahead: true,
                ..Default::default()
            },
            ..Default::default()
        }
    }

    fn limits() -> Limits {
        Limits::for_mode(Mode::Walk, 0.3)
    }

    #[test]
    fn off_sends_nothing_and_on_picks_something() {
        let mut a = Arbiter::new(1, false);
        let d = a.tick(&standing(0.0), limits());
        assert_eq!(d.status, Status::Off);
        assert!(d.intents.is_none());
        a.on = true;
        let d = a.tick(&standing(0.05), limits());
        assert!(matches!(d.status, Status::Active(_)), "{:?}", d.status);
        assert!(d.intents.is_some());
    }

    #[test]
    fn a_foreign_twist_makes_the_brain_yield_and_come_back() {
        let mut a = Arbiter::new(2, true);
        let _ = a.tick(&standing(0.0), limits());
        // Past the send grace, a twist that is not ours: a gamepad.
        let mut w = standing(1.0);
        w.move_requested = [0.12, 0.0, 0.7];
        let d = a.tick(&w, limits());
        assert_eq!(d.status, Status::Yielded);
        assert_eq!(
            d.intents.map(|i| i.twist),
            Some([0.0; 3]),
            "one neutral on the way out"
        );
        let d = a.tick(&standing(4.0), limits());
        assert_eq!(d.status, Status::Yielded, "still inside the yield window");
        let d = a.tick(&standing(6.5), limits());
        assert!(matches!(d.status, Status::Active(_)), "{:?}", d.status);
    }

    #[test]
    fn our_own_previous_twist_echoed_late_is_not_foreign() {
        let mut a = Arbiter::new(9, true);
        let d = a.tick(&standing(0.0), limits());
        let sent = d.intents.unwrap().twist;
        // Next frame the stream still shows what we sent a tick ago: ours.
        let mut w = standing(0.05);
        w.move_requested = sent;
        assert!(matches!(a.tick(&w, limits()).status, Status::Active(_)));
        // The behaviour ends and we send zero; the stream lags with the old value: ours.
        a.stop(&w, Status::Waiting, None);
        let mut w = standing(0.1);
        w.move_requested = sent;
        let d = a.tick(&w, limits());
        assert_ne!(d.status, Status::Yielded, "{:?}", d.status);
    }

    #[test]
    fn a_reflex_preempts_a_dwell_and_the_incumbent_otherwise_holds() {
        let mut a = Arbiter::new(3, true);
        let mut w = standing(0.0);
        let d = a.tick(&w, limits());
        let first = match d.status {
            Status::Active(k) => k,
            s => panic!("{s:?}"),
        };
        // Within the dwell, no ordinary challenger replaces it.
        for k in 1..8 {
            w.t = k as f64 * 0.3;
            let d = a.tick(&w, limits());
            if first.min_dwell() > w.t {
                assert!(
                    matches!(d.status, Status::Active(x) if x == first)
                        || first == Kind::TurnInPlace,
                    "{:?} at {}",
                    d.status,
                    w.t
                );
            }
        }
        // A thing appears at the beak: Startle now, dwell or not.
        w.t += 0.3;
        w.obstacles.too_close = true;
        let d = a.tick(&w, limits());
        assert_eq!(d.status, Status::Active(Kind::Startle));
        assert!(a.drives.comfort < 0.5);
    }

    #[test]
    fn a_new_behaviour_clears_the_old_ones_body_pose() {
        let mut a = Arbiter::new(11, true);
        let mut w = standing(0.0);
        // Force a Chill (pose active) then a Wander by hand.
        a.switch_to(Kind::Chill, 1.0, &w);
        let d = a.tick(&w, limits());
        assert!(d.intents.unwrap().pose.is_some());
        w.t = 0.1;
        a.switch_to(Kind::Wander, 1.0, &w);
        let d = a.tick(&w, limits());
        let i = d.intents.unwrap();
        assert_eq!(
            i.pose,
            Some((0.0, 0.0, 0.0)),
            "pose released so the twist is obeyed"
        );
        assert!(i.twist[0] > 0.0);
    }

    #[test]
    fn it_waits_while_the_robot_is_rising_and_stops_when_it_falls() {
        let mut a = Arbiter::new(4, true);
        let _ = a.tick(&standing(0.0), limits());
        let mut w = standing(0.1);
        w.policy = "rise".into();
        let d = a.tick(&w, limits());
        assert_eq!(d.status, Status::Waiting);
        assert!(d.intents.is_none(), "nothing while a skill runs");
        w.policy = "walk".into();
        w.fallen = true;
        let d = a.tick(&w, limits());
        assert_eq!(d.status, Status::Waiting);
        assert!(d.intents.is_some(), "a neutral on the way down");
        assert!(a.active_kind().is_none());
    }

    #[test]
    fn a_tired_duck_naps_and_wakes_rested() {
        let mut a = Arbiter::new(5, true);
        a.drives.energy = 0.1;
        let mut w = standing(0.0);
        let d = a.tick(&w, limits());
        assert_eq!(d.status, Status::Active(Kind::Nap));
        assert_eq!(
            d.intents.unwrap().skill,
            Some(duck_ipc_proto::Skill::from("sit_toggle"))
        );
        w.policy = "sit".into();
        let mut t = 0.1;
        let mut woke = None;
        while t < 400.0 {
            w.t = t;
            let d = a.tick(&w, limits());
            if d.status != Status::Active(Kind::Nap) {
                woke = Some((t, d.status));
                break;
            }
            t += 0.5;
        }
        let (t, status) = woke.expect("never woke");
        assert!(t > 40.0, "woke before the dwell: {t}");
        assert!(a.drives.energy > 0.5, "{}", a.drives.energy);
        // On a sitting robot nothing but Nap applies; it waits to be stood up.
        assert_eq!(status, Status::Waiting);
    }
}
