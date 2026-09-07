//! `braind` — the autonomous behaviour daemon.
//!
//! An ordinary client of `robotd`, like `padd`: one connection parked on the state stream,
//! one for intents, one on `tofd`'s depth stream. Twenty times a second it folds the newest
//! frames into a `World`, lets the arbiter pick, and sends what the active behaviour wants.
//! It enables nothing on a real robot and starts paused there (`brain-design.md` §5); on a
//! sim bench, `--on --autonomous-enable`.
//!
//! Laptop use:
//!
//! ```text
//! cargo run -p braind -- --socket /tmp/robotd.sock --tof /tmp/tof.sock --on --autonomous-enable
//! ```

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use braind::{Arbiter, Kind, Limits, Mode, Status, World};
use clap::Parser;
use duck_ipc_proto::{self as proto, Call, Request, Response};

const RECONNECT: Duration = Duration::from_secs(2);
const TICK: Duration = Duration::from_millis(50);

#[derive(Parser, Debug)]
#[command(name = "braind", about = "Autonomous behaviour for the duck", version)]
struct Args {
    /// robotd's socket.
    #[arg(long, default_value = proto::socket::ROBOT)]
    socket: PathBuf,

    /// tofd's socket (or the sim plant's `--tof`). Without it the brain has no eyes: no
    /// Wander, no Zoomies, no Startle — it chills, looks around, preens and naps.
    #[arg(long)]
    tof: Option<PathBuf>,

    /// Start driving immediately. Off by default on purpose: a duck that wanders because a
    /// daemon restarted is a fault.
    #[arg(long)]
    on: bool,

    /// Send `robot.enable` when the robot is `held` — a bench convenience. Never on a
    /// robot with people around it.
    #[arg(long)]
    autonomous_enable: bool,

    /// Walking speed cap, m/s. Below 0.3 the gait may not start; the brain floors it there.
    #[arg(long, default_value_t = 0.3)]
    max_linear: f64,

    /// The mission: the plant's `--model maze`. Solve it by the right-hand rule, then live.
    #[arg(long)]
    maze: bool,

    /// Where the maze mission writes the map it built (`braind-maze.txt` and `.svg`).
    #[arg(long, default_value = "/tmp")]
    maze_map_dir: PathBuf,

    /// Random seed. Same seed, same frames, same decisions.
    #[arg(long, default_value_t = 7)]
    seed: u64,

    /// Log every behaviour change and the drives, once a second.
    #[arg(long)]
    verbose: bool,
}

struct Prompt {
    reader: BufReader<UnixStream>,
    writer: UnixStream,
    next_id: u64,
}

impl Prompt {
    fn connect(socket: &Path) -> std::io::Result<Self> {
        let stream = UnixStream::connect(socket)?;
        stream.set_read_timeout(Some(Duration::from_secs(2)))?;
        Ok(Self {
            writer: stream.try_clone()?,
            reader: BufReader::new(stream),
            next_id: 1,
        })
    }

    fn call(&mut self, call: &Call) -> std::io::Result<Response> {
        let id = self.next_id;
        self.next_id += 1;
        let line = serde_json::to_string(&Request::call(proto::Id::Number(id), call))?;
        self.writer.write_all(format!("{line}\n").as_bytes())?;
        let mut reply = String::new();
        self.reader.read_line(&mut reply)?;
        serde_json::from_str(&reply).map_err(std::io::Error::other)
    }

    fn notify(&mut self, call: &Call) -> std::io::Result<()> {
        let line = serde_json::to_string(&Request::notify(call))?;
        self.writer.write_all(format!("{line}\n").as_bytes())
    }
}

/// Park on `robot.state`, forever; the newest frame lands in the slot.
fn state_stream(socket: PathBuf, slot: Arc<Mutex<Option<proto::RobotState>>>) {
    loop {
        match stream_states(&socket, &slot) {
            Ok(()) => tracing::warn!("state stream ended"),
            Err(e) => tracing::warn!(error = %e, "state stream"),
        }
        *slot.lock().unwrap() = None;
        std::thread::sleep(RECONNECT);
    }
}

fn stream_states(socket: &Path, slot: &Mutex<Option<proto::RobotState>>) -> std::io::Result<()> {
    let stream = UnixStream::connect(socket)?;
    let mut writer = stream.try_clone()?;
    let sub = Request::call(
        proto::Id::Number(1),
        &Call::RobotSubscribe(proto::SubscribeParams { hz: Some(20) }),
    );
    writer.write_all(format!("{}\n", serde_json::to_string(&sub)?).as_bytes())?;
    let reader = BufReader::new(stream);
    for line in reader.lines() {
        let line = line?;
        if let Ok(req) = serde_json::from_str::<Request>(&line)
            && let Some(state) = req.as_state()
        {
            *slot.lock().unwrap() = Some(state);
        }
    }
    Ok(())
}

/// Park on `tof.frame`, forever — the theremin's pattern, one level up.
fn tof_stream(socket: PathBuf, slot: Arc<Mutex<Option<(proto::TofFrame, Instant)>>>) {
    loop {
        match stream_tof(&socket, &slot) {
            Ok(()) => tracing::warn!("depth stream ended"),
            Err(e) => tracing::warn!(error = %e, "depth stream"),
        }
        *slot.lock().unwrap() = None;
        std::thread::sleep(RECONNECT);
    }
}

fn stream_tof(
    socket: &Path,
    slot: &Mutex<Option<(proto::TofFrame, Instant)>>,
) -> std::io::Result<()> {
    let stream = UnixStream::connect(socket)?;
    let mut writer = stream.try_clone()?;
    let sub = Request::call(proto::Id::Number(1), &Call::TofStream);
    writer.write_all(format!("{}\n", serde_json::to_string(&sub)?).as_bytes())?;
    let reader = BufReader::new(stream);
    for line in reader.lines() {
        let line = line?;
        if let Ok(req) = serde_json::from_str::<Request>(&line)
            && let Some(frame) = req.as_tof_frame()
        {
            *slot.lock().unwrap() = Some((frame, Instant::now()));
        }
    }
    Ok(())
}

fn main() -> std::process::ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive("info".parse().unwrap()),
        )
        .init();

    let mut prompt = match Prompt::connect(&args.socket) {
        Ok(p) => p,
        Err(e) => {
            tracing::error!(socket = %args.socket.display(), error = %e, "cannot reach robotd");
            return std::process::ExitCode::FAILURE;
        }
    };
    let mode = prompt
        .call(&Call::RobotMode)
        .ok()
        .and_then(|r| r.result_as::<proto::ModeResult>().ok())
        .map_or(Mode::Walk, |m| {
            if m.mode == "roller" {
                Mode::Roller
            } else {
                Mode::Walk
            }
        });
    let limits = Limits::for_mode(mode, args.max_linear);
    tracing::warn!(?mode, on = args.on, tof = args.tof.is_some(), "braind up");

    let states = Arc::new(Mutex::new(None));
    std::thread::spawn({
        let socket = args.socket.clone();
        let slot = states.clone();
        move || state_stream(socket, slot)
    });
    let frames = Arc::new(Mutex::new(None));
    if let Some(tof) = args.tof.clone() {
        let slot = frames.clone();
        std::thread::spawn(move || tof_stream(tof, slot));
    }
    let reprojector = kinematics::tof::Reprojector::alpha();

    let mut world = World {
        mode,
        maze: args.maze,
        ..Default::default()
    };
    let mut arbiter = Arbiter::new(args.seed, args.on);
    let mut enabled_once = false;
    let mut last_log = Instant::now();
    let mut last_map_log = Instant::now();
    let mut last_map_moves = u32::MAX;
    let mut last_twist_sent = false;
    let mut last_status = Status::Off;

    loop {
        let tick_start = Instant::now();
        if let Some(state) = states.lock().unwrap().as_ref() {
            world.observe_state(state);
        }
        if let Some((frame, at)) = frames.lock().unwrap().as_ref() {
            world.observe_tof(frame, &reprojector, at.elapsed().as_secs_f64());
        } else {
            world.obstacles.age_s = None;
        }

        if args.autonomous_enable && !enabled_once && world.have_state && world.policy == "held" {
            enabled_once = true;
            match prompt.call(&Call::RobotEnable(proto::EnableParams {
                on: true,
                toggle: false,
            })) {
                Ok(r) => tracing::warn!(reply = ?r.result, "--autonomous-enable: robot.enable"),
                Err(e) => tracing::error!(error = %e, "robot.enable failed"),
            }
        }

        let decision = arbiter.tick(&world, limits);
        // The maze mission's map: printed every few moves, written out when it ends.
        if let Some(map) = arbiter.maze_map() {
            let ended = matches!(decision.changed, Some((Some(Kind::Maze), _)));
            if ended {
                let txt = args.maze_map_dir.join("braind-maze.txt");
                let svg = args.maze_map_dir.join("braind-maze.svg");
                let _ = std::fs::write(&txt, map.ascii());
                let _ = std::fs::write(&svg, map.svg());
                tracing::info!(
                    cells = map.cells_known(),
                    moves = map.moves,
                    svg = %svg.display(),
                    "maze: the map as the duck built it\n{}",
                    map.ascii()
                );
            } else if map.moves != last_map_moves
                && last_map_log.elapsed() >= Duration::from_secs(6)
            {
                last_map_moves = map.moves;
                last_map_log = Instant::now();
                tracing::info!(moves = map.moves, "maze: map so far\n{}", map.ascii());
            }
        }
        if let Some((from, to)) = decision.changed {
            tracing::info!(
                from = from.map(Kind::name).unwrap_or("-"),
                to = to.map(Kind::name).unwrap_or("-"),
                energy = format!("{:.2}", arbiter.drives.energy),
                curiosity = format!("{:.2}", arbiter.drives.curiosity),
                "behaviour"
            );
        }
        if decision.status != last_status {
            if matches!(
                decision.status,
                Status::Off | Status::Waiting | Status::Yielded
            ) {
                tracing::info!(status = ?decision.status, "brain");
            }
            last_status = decision.status;
        }

        if let Some(i) = decision.intents {
            let moving = i.twist.iter().any(|v| v.abs() > 1e-9);
            if moving || last_twist_sent {
                let _ = prompt.notify(&Call::RobotMove(proto::MoveParams {
                    vx: i.twist[0],
                    vy: i.twist[1],
                    vyaw: i.twist[2],
                }));
            }
            last_twist_sent = moving;
            if let Some(h) = i.head {
                let _ = prompt.notify(&Call::RobotHead(proto::HeadParams {
                    neck_pitch: h[0],
                    head_pitch: h[1],
                    head_yaw: h[2],
                    head_roll: h[3],
                }));
            }
            if let Some((z, roll, pitch)) = i.pose {
                let active = z != 0.0 || roll != 0.0 || pitch != 0.0;
                let _ = prompt.notify(&Call::RobotPose(proto::PoseParams {
                    z,
                    roll,
                    pitch,
                    active,
                }));
            }
            if let Some(open) = i.mouth {
                let _ = prompt.notify(&Call::RobotMouth(proto::MouthParams { open }));
            }
            if let Some(skill) = i.skill {
                match prompt.call(&Call::RobotDo(proto::DoParams { skill })) {
                    Ok(r) => tracing::info!(?skill, reply = ?r.result, "skill"),
                    Err(e) => tracing::warn!(?skill, error = %e, "skill"),
                }
            }
            if let Some(tag) = i.sound {
                let _ = prompt.notify(&Call::RobotSound(proto::SoundParams { tag, hold: None }));
            }
        }

        if args.verbose && last_log.elapsed() >= Duration::from_secs(1) {
            last_log = Instant::now();
            let o = &world.obstacles;
            tracing::info!(
                status = ?decision.status,
                policy = %world.policy,
                energy = format!("{:.2}", arbiter.drives.energy),
                curiosity = format!("{:.2}", arbiter.drives.curiosity),
                comfort = format!("{:.2}", arbiter.drives.comfort),
                cells = world.grid.cells_seen(),
                odom = format!("({:.2},{:.2}) yaw {:.2}", world.odom[0], world.odom[1], world.yaw),
                ahead = format!("L{} C{} R{}{}", fmt_m(o.left), fmt_m(o.center), fmt_m(o.right), if o.fresh() { "" } else { " (stale)" }),
                "brain"
            );
        }

        if let Some(rest) = TICK.checked_sub(tick_start.elapsed()) {
            std::thread::sleep(rest);
        }
    }
}

fn fmt_m(v: Option<f64>) -> String {
    v.map_or("  - ".to_owned(), |m| format!("{m:.2}"))
}
