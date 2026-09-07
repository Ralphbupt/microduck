//! Drives: the slow scalars behaviours are scored against. This is the whole "mood model".
//!
//! Each is clamped to `0..=1`. They move on the order of minutes, which is what makes a
//! duck look like it has a day rather than a random number generator.

use crate::world::World;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Drives {
    /// Drains with motion, recovers at rest and fastest asleep.
    pub energy: f64,
    /// Rises with time since the last new place, drops when Wander finds one.
    pub curiosity: f64,
    /// Rises with company (a duck, a voice), decays alone. v1 has no inputs for it.
    pub social: f64,
    /// Rises when petted, drops on a startle, drifts back to the middle.
    pub comfort: f64,
}

impl Default for Drives {
    fn default() -> Self {
        Self {
            energy: 0.8,
            curiosity: 0.5,
            social: 0.3,
            comfort: 0.5,
        }
    }
}

/// Per-second rates. Tuned so a full-energy duck wanders for a few minutes, chills a while,
/// naps for a couple, and does not repeat itself on the minute.
pub const ENERGY_DRAIN_PER_ACTIVITY: f64 = 1.0 / 240.0;
pub const ENERGY_REST_RECOVERY: f64 = 1.0 / 400.0;
pub const ENERGY_NAP_RECOVERY: f64 = 1.0 / 90.0;
pub const CURIOSITY_RISE: f64 = 1.0 / 120.0;
pub const CURIOSITY_PAYOUT: f64 = 0.15;
pub const COMFORT_RELAX: f64 = 1.0 / 180.0;

impl Drives {
    /// `activity` is 0 (still) to 1 (zoomies); `napping` switches recovery to the fast
    /// rate. `new_place` is the novelty grid's verdict for this tick.
    pub fn update(
        &mut self,
        dt: f64,
        activity: f64,
        napping: bool,
        new_place: bool,
        world: &World,
    ) {
        let drain = ENERGY_DRAIN_PER_ACTIVITY * activity;
        let recover = if napping {
            ENERGY_NAP_RECOVERY
        } else if activity < 0.05 {
            ENERGY_REST_RECOVERY
        } else {
            0.0
        };
        self.energy = clamp(self.energy + (recover - drain) * dt);

        if new_place {
            self.curiosity = clamp(self.curiosity - CURIOSITY_PAYOUT);
        } else if world.standing() {
            self.curiosity = clamp(self.curiosity + CURIOSITY_RISE * dt);
        }

        // Comfort drifts back to the middle from either side.
        let toward = 0.5 - self.comfort;
        self.comfort = clamp(self.comfort + toward.signum() * toward.abs().min(COMFORT_RELAX * dt));
    }

    pub fn startled(&mut self) {
        self.comfort = clamp(self.comfort - 0.3);
    }
}

fn clamp(v: f64) -> f64 {
    v.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn energy_drains_moving_and_recovers_napping() {
        let world = World {
            policy: "stand".into(),
            ..Default::default()
        };
        let mut d = Drives::default();
        for _ in 0..600 {
            d.update(0.1, 1.0, false, false, &world); // a minute of zoomies
        }
        assert!(d.energy < 0.6, "{}", d.energy);
        for _ in 0..600 {
            d.update(0.1, 0.0, true, false, &world); // a minute asleep
        }
        assert!(
            d.energy > 0.95,
            "a minute asleep should nearly fill it: {}",
            d.energy
        );
        assert!(d.energy <= 1.0);
    }

    #[test]
    fn curiosity_builds_standing_still_and_pays_out_on_a_new_place() {
        let world = World {
            policy: "stand".into(),
            ..Default::default()
        };
        let mut d = Drives {
            curiosity: 0.0,
            ..Default::default()
        };
        for _ in 0..600 {
            d.update(0.1, 0.0, false, false, &world);
        }
        assert!(d.curiosity > 0.4, "{}", d.curiosity);
        let before = d.curiosity;
        d.update(0.1, 0.5, false, true, &world);
        assert!(d.curiosity < before - 0.1);
    }

    #[test]
    fn comfort_returns_to_the_middle() {
        let world = World::default();
        let mut d = Drives::default();
        d.startled();
        assert!(d.comfort < 0.3);
        for _ in 0..3000 {
            d.update(0.1, 0.0, false, false, &world);
        }
        assert!((d.comfort - 0.5).abs() < 1e-9);
    }
}
