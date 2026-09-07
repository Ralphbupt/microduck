//! `robotd --sim` against the MuJoCo plant, end to end: a seated duck is enabled, rises,
//! walks forward on command, and stops. The daemon-level walking acceptance that
//! `docs/design/sim-backend-design.md` §6 promises.
//!
//! Needs the plant and the policies, so it is `#[ignore]` unless `MICRODUCK_RL` names a
//! `microduck_rl` checkout (with its `.venv` synced) — then it runs headless in ~20 s:
//!
//! ```text
//! MICRODUCK_RL=~/microduck/microduck_rl cargo test -p robotd --test sim -- --ignored
//! ```
//!
//! `ORT_DYLIB_PATH` is derived from that checkout's venv when unset.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{Value, json};

struct Procs {
    plant: Child,
    robotd: Child,
}

impl Drop for Procs {
    fn drop(&mut self) {
        let _ = self.robotd.kill();
        let _ = self.plant.kill();
        let _ = self.robotd.wait();
        let _ = self.plant.wait();
    }
}

fn rl_checkout() -> Option<PathBuf> {
    std::env::var_os("MICRODUCK_RL").map(PathBuf::from)
}

fn ort_dylib(rl: &Path) -> Option<PathBuf> {
    if let Some(p) = std::env::var_os("ORT_DYLIB_PATH") {
        return Some(PathBuf::from(p));
    }
    let capi = rl.join(".venv/lib");
    let py = std::fs::read_dir(&capi)
        .ok()?
        .flatten()
        .find(|e| e.file_name().to_string_lossy().starts_with("python3"))?;
    let dir = py.path().join("site-packages/onnxruntime/capi");
    std::fs::read_dir(dir)
        .ok()?
        .flatten()
        .map(|e| e.path())
        .find(|p| {
            let name = p
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_default();
            name.starts_with("libonnxruntime.")
                && (name.ends_with(".dylib") || name.contains(".so"))
        })
}

fn params_file(dir: &Path, rl: &Path) -> PathBuf {
    let p = |n: &str| rl.join("policies").join(n).display().to_string();
    let toml = format!(
        "[policy]\nwalk = \"{}\"\nstand = \"{}\"\nsitstand = \"{}\"\nground_pick = \"{}\"\nkick_left = \"{}\"\nkick_right = \"{}\"\nroulade = \"{}\"\n",
        p("alpha_walking.onnx"),
        p("alpha_stand.onnx"),
        p("alpha_sitstand.onnx"),
        p("alpha_ground_pick.onnx"),
        p("ball_kick_left.onnx"),
        p("ball_kick_right.onnx"),
        p("roulade.onnx"),
    );
    let path = dir.join("robotd.toml");
    std::fs::write(&path, toml).unwrap();
    path
}

struct Client {
    reader: BufReader<UnixStream>,
    writer: UnixStream,
    next_id: u64,
}

impl Client {
    fn connect(socket: &Path) -> Self {
        let stream = UnixStream::connect(socket).unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(5)))
            .unwrap();
        Self {
            writer: stream.try_clone().unwrap(),
            reader: BufReader::new(stream),
            next_id: 1,
        }
    }

    fn call(&mut self, method: &str, params: Value) -> Value {
        let id = self.next_id;
        self.next_id += 1;
        let req = json!({"jsonrpc": "2.0", "id": id, "method": method, "params": params});
        self.writer
            .write_all(format!("{req}\n").as_bytes())
            .unwrap();
        let mut line = String::new();
        self.reader.read_line(&mut line).unwrap();
        let reply: Value = serde_json::from_str(&line).unwrap();
        reply["result"].clone()
    }

    fn notify(&mut self, method: &str, params: Value) {
        let req = json!({"jsonrpc": "2.0", "method": method, "params": params});
        self.writer
            .write_all(format!("{req}\n").as_bytes())
            .unwrap();
    }

    /// Subscribe and keep the newest state frame in a slot, so a reader that looks only
    /// now and then sees *now* rather than the front of a backlog.
    fn subscribe(mut self) -> Latest {
        let _ = self.call("robot.subscribe", json!({"hz": 20}));
        let slot = Arc::new(Mutex::new(Value::Null));
        let writer = slot.clone();
        std::thread::spawn(move || {
            loop {
                let mut line = String::new();
                match self.reader.read_line(&mut line) {
                    Ok(0) | Err(_) => return,
                    Ok(_) => {}
                }
                if let Ok(msg) = serde_json::from_str::<Value>(&line)
                    && msg["method"] == "robot.state"
                {
                    *writer.lock().unwrap() = msg["params"].clone();
                }
            }
        });
        Latest { slot }
    }
}

struct Latest {
    slot: Arc<Mutex<Value>>,
}

impl Latest {
    /// The newest frame, waiting for the first one to land.
    fn now(&self) -> Value {
        let start = Instant::now();
        loop {
            let v = self.slot.lock().unwrap().clone();
            if !v.is_null() {
                return v;
            }
            assert!(
                start.elapsed() < Duration::from_secs(5),
                "no state frame arrived"
            );
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    fn wait_policy(&self, label: &str, timeout: Duration) -> Value {
        let start = Instant::now();
        loop {
            let st = self.now();
            if st["policy"] == label {
                return st;
            }
            assert!(
                start.elapsed() < timeout,
                "policy never became {label}: {st}"
            );
            std::thread::sleep(Duration::from_millis(50));
        }
    }
}

fn wait_for(path: &Path, timeout: Duration) {
    let start = Instant::now();
    while !path.exists() {
        assert!(
            start.elapsed() < timeout,
            "{} never appeared",
            path.display()
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

#[test]
#[ignore = "needs MICRODUCK_RL (a microduck_rl checkout with .venv) and ~20 s"]
fn a_seated_duck_is_enabled_rises_walks_and_stops() {
    let rl = rl_checkout().expect("MICRODUCK_RL");
    let dylib = ort_dylib(&rl).expect("ONNX Runtime dylib in the microduck_rl venv");
    // Short paths on purpose: AF_UNIX names cap at ~104 bytes and the default temp dir on
    // macOS is deep enough to blow it.
    let dir = tempfile::Builder::new()
        .prefix("duck-sim-")
        .tempdir_in("/tmp")
        .unwrap();
    let plant_sock = dir.path().join("plant.sock");
    let robot_sock = dir.path().join("robotd.sock");

    let plant = Command::new("uv")
        .args(["run", "python", "scripts/plant_server.py", "--socket"])
        .arg(&plant_sock)
        .args(["--model", "walk"])
        .current_dir(&rl)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("uv run plant_server.py");
    let robotd = Command::new(env!("CARGO_BIN_EXE_robotd"))
        .arg("--sim")
        .arg(&plant_sock)
        .arg("--socket")
        .arg(&robot_sock)
        .arg("--params")
        .arg(params_file(dir.path(), &rl))
        .env("ORT_DYLIB_PATH", dylib)
        .env("RUST_LOG", "error")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    let _procs = Procs { plant, robotd };

    wait_for(&robot_sock, Duration::from_secs(20));
    let mut prompt = Client::connect(&robot_sock);
    let start = Instant::now();
    loop {
        let health = prompt.call("robot.health", json!({}));
        if health["healthy"] == true && health["control_loop"]["ticks"].as_u64().unwrap_or(0) > 10 {
            break;
        }
        assert!(
            start.elapsed() < Duration::from_secs(30),
            "never healthy: {health}"
        );
        std::thread::sleep(Duration::from_millis(200));
    }

    let stream = Client::connect(&robot_sock).subscribe();
    let boot = stream.now();
    assert_eq!(boot["policy"], "held");

    let enabled = prompt.call("robot.enable", json!({"on": true, "toggle": false}));
    assert_eq!(enabled["accepted"], true, "{enabled}");

    // Rise: the seated boot goes through the sitstand policy and lands on `stand`. Give
    // it a moment to settle before asking it to walk.
    let standing = stream.wait_policy("stand", Duration::from_secs(8));
    std::thread::sleep(Duration::from_secs(1));
    let standing = {
        let now = stream.now();
        assert_eq!(now["policy"], "stand", "{now}");
        let _ = standing;
        now
    };
    assert_eq!(standing["safety"]["fallen"], false);
    assert!(standing["safety"]["gravity"][2].as_f64().unwrap() < -0.95);
    let z0 = standing["odom"]["position"][2].as_f64().unwrap();
    assert!(z0 > 0.10, "standing height {z0}");
    let x0 = standing["odom"]["position"][0].as_f64().unwrap();

    // Walk: 0.3 m/s forward for 6 s, inside the deadman. 0.3 is `padd`'s full stick; at
    // 0.2 the daemon's action low-pass leaves the gait sometimes not starting from a still
    // stand (sim-backend-design.md §9), which is a tuning question, not this test's.
    let start = Instant::now();
    let mut last = standing.clone();
    while start.elapsed() < Duration::from_secs(6) {
        prompt.notify("robot.move", json!({"vx": 0.3, "vy": 0.0, "vyaw": 0.0}));
        std::thread::sleep(Duration::from_millis(50));
        last = stream.now();
        assert_eq!(
            last["safety"]["fallen"], false,
            "fell while walking: {last}"
        );
    }
    assert_eq!(last["policy"], "walk", "{last}");
    let x1 = last["odom"]["position"][0].as_f64().unwrap();
    assert!(x1 - x0 > 0.4, "walked only {:.2} m", x1 - x0);

    // Stop: the deadman zeroes the twist and the stand policy takes over.
    let stopped = stream.wait_policy("stand", Duration::from_secs(5));
    assert_eq!(stopped["safety"]["fallen"], false);
    let health = prompt.call("robot.health", json!({}));
    assert_eq!(health["healthy"], true, "{health}");
    // A debug build sharing a laptop with the plant and a test runner may drop a tick or
    // two in ~10 s; the health gate's own bar is 45 Hz, and that is the one that matters.
    assert!(
        health["control_loop"]["missed"].as_u64().unwrap() <= 2,
        "{health}"
    );
}
