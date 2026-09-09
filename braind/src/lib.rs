//! The autonomous behaviour layer, minus the sockets.
//!
//! Three plain data transforms — senses ([`world`]), drives ([`drives`]), and the arbiter
//! over scored behaviours ([`arbiter`], [`behaviours`]) — that the daemon feeds from the
//! wire and the tests feed by hand. Design: `docs/design/brain-design.md`.

pub mod arbiter;
pub mod behaviours;
pub mod drives;
pub mod maze;
pub mod room;
pub mod world;

pub use arbiter::{Arbiter, Decision, Status};
pub use behaviours::{Intents, Kind, Limits};
pub use drives::Drives;
pub use world::{Mode, Obstacles, World};
