//! `SimIo` — a MuJoCo robot behind [`RobotIo`], for `robotd --sim`.
//!
//! The physics lives in a separate process (`microduck_rl/scripts/plant_server.py`); this
//! is the daemon's end of the wire. One `write` steps the plant one control period and
//! buffers the sensors that period produced; the next `read` hands them over. The loop
//! reads before it ever writes, so a `read` with nothing buffered asks the plant for its
//! current state without stepping. Design: `docs/design/sim-backend-design.md` §2–3.
//!
//! Blocking `std` sockets on purpose: this runs on the control thread, which is a blocking
//! thread by design (§4.1 of the robotd doc), and the plant answers a `step` in well under
//! a millisecond. A plant that stops answering costs the tick its deadline, and the health
//! gate reports that — the same failure the real bus produces.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::time::Duration;

use duck_control::io::{ImuStale, IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use duck_control::{ImuData, NUM_JOINTS};
use serde::Deserialize;
use serde_json::{Value, json};

/// The protocol this build speaks; the plant refuses a mismatch in `hello`.
const PROTOCOL_VERSION: u64 = 1;

/// Longer than any plant step, shorter than the health gate's stall window (500 ms) so a
/// wedged plant surfaces as read errors rather than as a frozen loop.
const REPLY_TIMEOUT: Duration = Duration::from_millis(250);

#[derive(Debug, Deserialize)]
struct Frame {
    pos: Vec<f64>,
    vel: Vec<f64>,
    gyro: [f64; 3],
    gravity: [f64; 3],
    quat: [f64; 4],
    #[serde(default)]
    imu_ready: bool,
}

pub struct SimIo {
    path: PathBuf,
    reader: BufReader<UnixStream>,
    writer: UnixStream,
    line: String,
    /// The sensors produced by the last `step`, not yet handed to the loop.
    pending: Option<Sensors>,
    /// Every request carries a sequence number the plant echoes. A reply that arrives after
    /// its request timed out is then recognised and dropped instead of being taken for the
    /// answer to the next request — which desynchronised the whole stream once.
    seq: u64,
    imu_ready: bool,
    pub model: String,
}

impl std::fmt::Debug for SimIo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SimIo")
            .field("path", &self.path)
            .field("model", &self.model)
            .field("imu_ready", &self.imu_ready)
            .finish_non_exhaustive()
    }
}

impl SimIo {
    /// Connect and shake hands. `Err(IoError::Port)` when nothing listens there yet — the
    /// caller retries, exactly as it waits for an unpowered bus.
    pub fn connect(path: &Path) -> Result<Self> {
        let port = |e: std::io::Error| IoError::Port {
            path: path.display().to_string(),
            source: e,
        };
        let stream = UnixStream::connect(path).map_err(port)?;
        stream.set_read_timeout(Some(REPLY_TIMEOUT)).map_err(port)?;
        let writer = stream.try_clone().map_err(port)?;
        let mut io = Self {
            path: path.to_path_buf(),
            reader: BufReader::new(stream),
            writer,
            line: String::new(),
            pending: None,
            seq: 0,
            imu_ready: false,
            model: String::new(),
        };
        let hello = io.call(json!({"op": "hello", "version": PROTOCOL_VERSION}))?;
        let version = hello.get("version").and_then(Value::as_u64).unwrap_or(0);
        if version != PROTOCOL_VERSION {
            return Err(IoError::Bus(format!(
                "plant speaks protocol {version}, this robotd speaks {PROTOCOL_VERSION}"
            )));
        }
        io.model = hello
            .get("model")
            .and_then(Value::as_str)
            .unwrap_or("?")
            .to_owned();
        Ok(io)
    }

    fn call(&mut self, mut request: Value) -> Result<Value> {
        let port = |path: &Path, e: std::io::Error| IoError::Port {
            path: path.display().to_string(),
            source: e,
        };
        self.seq += 1;
        let seq = self.seq;
        request["seq"] = json!(seq);
        let mut line = serde_json::to_string(&request).map_err(|e| IoError::Bus(e.to_string()))?;
        line.push('\n');
        self.writer
            .write_all(line.as_bytes())
            .map_err(|e| port(&self.path, e))?;
        let reply: Value = loop {
            self.line.clear();
            match self.reader.read_line(&mut self.line) {
                Ok(0) => {
                    return Err(IoError::Port {
                        path: self.path.display().to_string(),
                        source: std::io::Error::other("plant closed the socket"),
                    });
                }
                Ok(_) => {}
                Err(e) => return Err(port(&self.path, e)),
            }
            let reply: Value = serde_json::from_str(&self.line)
                .map_err(|e| IoError::Bus(format!("plant reply: {e}")))?;
            match reply.get("seq").and_then(Value::as_u64) {
                Some(got) if got < seq => continue, // a late answer to an earlier request
                _ => break reply,
            }
        };
        if reply.get("ok").and_then(Value::as_bool) != Some(true) {
            let why = reply
                .get("error")
                .and_then(Value::as_str)
                .unwrap_or("plant said no");
            return Err(IoError::Bus(why.to_owned()));
        }
        Ok(reply)
    }

    fn sensors_from(&mut self, reply: Value) -> Result<Sensors> {
        let frame: Frame =
            serde_json::from_value(reply).map_err(|e| IoError::Bus(format!("plant frame: {e}")))?;
        if frame.pos.len() != NUM_JOINTS || frame.vel.len() != NUM_JOINTS {
            return Err(IoError::ShortRead {
                what: "plant joints",
                expected: NUM_JOINTS,
                got: frame.pos.len().min(frame.vel.len()),
            });
        }
        let mut sensors = Sensors::default();
        sensors.positions.copy_from_slice(&frame.pos);
        sensors.velocities.copy_from_slice(&frame.vel);
        sensors.imu = ImuData {
            gyro: frame.gyro,
            gravity: frame.gravity,
            quat: frame.quat,
        };
        self.imu_ready = frame.imu_ready;
        Ok(sensors)
    }
}

impl RobotIo for SimIo {
    fn read(&mut self) -> Result<Sensors> {
        if let Some(sensors) = self.pending.take() {
            return Ok(sensors);
        }
        let reply = self.call(json!({"op": "read"}))?;
        self.sensors_from(reply)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        let reply = self.call(json!({"op": "step", "targets": targets.positions}))?;
        self.pending = Some(self.sensors_from(reply)?);
        Ok(())
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        self.call(json!({"op": "gain", "kp": kp})).map(|_| ())
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        self.call(json!({"op": "torque", "on": on})).map(|_| ())
    }

    fn reboot(&mut self, id: u8) -> Result<()> {
        tracing::debug!(id, "reboot of a plant servo: nothing to do");
        Ok(())
    }

    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        let reply = self.call(json!({"op": "slow"}))?;
        let volts = reply
            .get("volts")
            .and_then(Value::as_f64)
            .ok_or_else(|| IoError::Bus("plant slow: no volts".into()))?;
        let mut temps_c = [0.0; NUM_JOINTS];
        if let Some(list) = reply.get("temps_c").and_then(Value::as_array) {
            for (slot, v) in temps_c.iter_mut().zip(list) {
                *slot = v.as_f64().unwrap_or(0.0);
            }
        }
        Ok(SlowSensors { volts, temps_c })
    }

    fn imu_stale(&self) -> ImuStale {
        ImuStale::default()
    }

    fn imu_ready(&self) -> bool {
        self.imu_ready
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use duck_control::DEFAULT_POSITION;
    use std::os::unix::net::UnixListener;

    /// A plant made of arithmetic: echoes targets as positions, counts frames for
    /// `imu_ready`, refuses a bad op. Enough to pin the wire, not the physics.
    fn fake_plant(dir: &Path) -> (PathBuf, std::thread::JoinHandle<Vec<String>>) {
        let path = dir.join("plant.sock");
        let listener = UnixListener::bind(&path).unwrap();
        let handle = std::thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            let mut writer = stream.try_clone().unwrap();
            let reader = BufReader::new(stream);
            let mut seen = Vec::new();
            let mut pos = vec![0.0; NUM_JOINTS];
            let mut frames = 0u64;
            for line in reader.lines() {
                let line = line.unwrap();
                let req: Value = serde_json::from_str(&line).unwrap();
                let op = req["op"].as_str().unwrap().to_owned();
                seen.push(op.clone());
                let mut reply = match op.as_str() {
                    "hello" => json!({"ok": true, "version": PROTOCOL_VERSION, "model": "walk"}),
                    "read" | "step" => {
                        if op == "step" {
                            pos = req["targets"]
                                .as_array()
                                .unwrap()
                                .iter()
                                .map(|v| v.as_f64().unwrap())
                                .collect();
                            frames += 1;
                        }
                        json!({"ok": true, "t": frames as f64 * 0.02, "pos": pos, "vel": vec![0.0; NUM_JOINTS],
                               "gyro": [0.0, 0.0, 0.0], "gravity": [0.0, 0.0, -1.0], "quat": [1.0, 0.0, 0.0, 0.0],
                               "imu_ready": frames >= 25})
                    }
                    "gain" | "torque" => json!({"ok": true}),
                    "slow" => json!({"ok": true, "volts": 7.4, "temps_c": vec![32.0; NUM_JOINTS]}),
                    _ => json!({"ok": false, "error": "unknown op"}),
                };
                reply["seq"] = req["seq"].clone();
                writer
                    .write_all(format!("{}\n", serde_json::to_string(&reply).unwrap()).as_bytes())
                    .unwrap();
            }
            seen
        });
        (path, handle)
    }

    #[test]
    fn read_before_write_asks_the_plant_and_reads_after_write_do_not() {
        let dir = tempfile::tempdir().unwrap();
        let (path, handle) = fake_plant(dir.path());
        let mut io = SimIo::connect(&path).unwrap();
        assert_eq!(io.model, "walk");

        let first = io.read().unwrap();
        assert_eq!(first.positions, [0.0; NUM_JOINTS]);
        assert!(!io.imu_ready());

        let targets = JointTargets::new(DEFAULT_POSITION);
        io.write(&targets).unwrap();
        let after = io.read().unwrap();
        assert_eq!(
            after.positions, DEFAULT_POSITION,
            "a step's frame is the next read"
        );
        assert_eq!(after.imu.gravity, [0.0, 0.0, -1.0]);

        for _ in 0..30 {
            io.write(&targets).unwrap();
            io.read().unwrap();
        }
        assert!(io.imu_ready(), "the plant's readiness reaches the trait");

        let slow = io.slow_sensors().unwrap();
        assert_eq!(slow.volts, 7.4);
        io.set_gain(50).unwrap();
        io.set_torque(false).unwrap();
        drop(io);

        let seen = handle.join().unwrap();
        assert_eq!(seen[..3], ["hello", "read", "step"], "{seen:?}");
        assert_eq!(
            seen.iter().filter(|op| *op == "read").count(),
            1,
            "only the first read hit the wire"
        );
    }

    #[test]
    fn nothing_listening_is_a_port_error() {
        let dir = tempfile::tempdir().unwrap();
        let err = SimIo::connect(&dir.path().join("absent.sock")).unwrap_err();
        assert!(matches!(err, IoError::Port { .. }), "{err:?}");
    }
}
