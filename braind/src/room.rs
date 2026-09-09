//! A room in the duck's memory: an occupancy grid built from the depth sensor as it goes,
//! and frontier exploration over it — walk to the nearest edge between known floor and the
//! unknown, and the map grows until there is no edge left.
//!
//! Cells are `RES` metres, stored sparsely so the map can grow in any direction from wherever
//! the duck booted. Each holds a log-odds occupancy: hits push it up, rays passing through
//! push it down. Odometry is the pose; on feet it drifts a few percent, which a room-sized
//! map tolerates (a maze did not, and got wall-based corrections in `maze.rs`).

use std::collections::{HashMap, HashSet, VecDeque};

use kinematics::tof::{COLS, ROWS, Zone};

pub const RES: f64 = 0.10;
/// A beam that saw nothing is trusted to be free this far.
pub const CLEAR_REACH_M: f64 = 2.0;
/// Log-odds steps and clamps.
const HIT: i16 = 6;
const MISS: i16 = -2;
const CLAMP: i16 = 30;
/// Occupied above this, free below its negative, unknown between.
const OCCUPIED: i16 = 8;
const FREE: i16 = -4;
/// Cells this close to an obstacle are not walked through (the duck is ~0.15 m wide).
pub const INFLATE_M: f64 = 0.20;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cell {
    Unknown,
    Free,
    Occupied,
}

#[derive(Debug, Clone, Default)]
pub struct RoomMap {
    cells: HashMap<(i32, i32), i16>,
    /// Where the duck has stood, for drawing.
    pub track: Vec<(f64, f64)>,
    pub integrated: u64,
}

fn key(x: f64, y: f64) -> (i32, i32) {
    ((x / RES).floor() as i32, (y / RES).floor() as i32)
}

fn centre(c: (i32, i32)) -> (f64, f64) {
    ((c.0 as f64 + 0.5) * RES, (c.1 as f64 + 0.5) * RES)
}

impl RoomMap {
    pub fn cell(&self, c: (i32, i32)) -> Cell {
        match self.cells.get(&c) {
            Some(&v) if v >= OCCUPIED => Cell::Occupied,
            Some(&v) if v <= FREE => Cell::Free,
            _ => Cell::Unknown,
        }
    }

    pub fn cell_at(&self, x: f64, y: f64) -> Cell {
        self.cell(key(x, y))
    }

    fn bump(&mut self, c: (i32, i32), by: i16) {
        let v = self.cells.entry(c).or_insert(0);
        *v = (*v + by).clamp(-CLAMP, CLAMP);
    }

    /// Cells along the segment from `a` to `b`, excluding `b`'s own cell.
    fn ray(a: (f64, f64), b: (f64, f64)) -> Vec<(i32, i32)> {
        let n = ((b.0 - a.0).hypot(b.1 - a.1) / (RES * 0.5)).ceil().max(1.0) as usize;
        let end = key(b.0, b.1);
        let mut out = Vec::with_capacity(n);
        let mut last = None;
        for i in 0..n {
            let t = i as f64 / n as f64;
            let c = key(a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
            if c == end {
                break;
            }
            if last != Some(c) {
                out.push(c);
                last = Some(c);
            }
        }
        out
    }

    /// Fold in one reprojected depth frame. `pose` is the trunk in the world (x, y, yaw);
    /// `sensor` its position and the beam directions in the trunk frame; `zones` the frame.
    pub fn integrate(
        &mut self,
        pose: (f64, f64, f64),
        sensor_pos: [f64; 3],
        beam_dirs: &[[f64; 3]; ROWS * COLS],
        zones: &[Zone; ROWS * COLS],
    ) {
        let (px, py, yaw) = pose;
        let (cy, sy) = (yaw.cos(), yaw.sin());
        let to_world = |v: [f64; 3]| (px + cy * v[0] - sy * v[1], py + sy * v[0] + cy * v[1]);
        let origin = to_world(sensor_pos);
        for (i, zone) in zones.iter().enumerate() {
            let dir = beam_dirs[i];
            match zone {
                Zone::Hit { point, .. } => {
                    let p = to_world(*point);
                    for c in Self::ray(origin, p) {
                        self.bump(c, MISS);
                    }
                    self.bump(key(p.0, p.1), HIT);
                }
                Zone::Floor { point } => {
                    let p = to_world(*point);
                    for c in Self::ray(origin, p) {
                        self.bump(c, MISS);
                    }
                    self.bump(key(p.0, p.1), MISS);
                }
                Zone::Empty => {
                    // Only near-level beams say anything about the room; a sky beam sees
                    // nothing because there is nothing up there.
                    if dir[2].abs() > 0.35 {
                        continue;
                    }
                    let far = [
                        sensor_pos[0] + dir[0] * CLEAR_REACH_M,
                        sensor_pos[1] + dir[1] * CLEAR_REACH_M,
                        sensor_pos[2] + dir[2] * CLEAR_REACH_M,
                    ];
                    let p = to_world(far);
                    for c in Self::ray(origin, p) {
                        self.bump(c, MISS / 2);
                    }
                }
                Zone::TooClose => {
                    let p = to_world([
                        sensor_pos[0] + dir[0] * 0.08,
                        sensor_pos[1] + dir[1] * 0.08,
                        0.0,
                    ]);
                    self.bump(key(p.0, p.1), HIT / 2);
                }
            }
        }
        // The duck's own footprint is free: it is standing there.
        self.bump(key(px, py), MISS);
        if self
            .track
            .last()
            .is_none_or(|&(x, y)| (x - px).hypot(y - py) > 0.05)
        {
            self.track.push((px, py));
        }
        self.integrated += 1;
    }

    /// Free and not within `INFLATE_M` of anything occupied: a cell the duck may stand in.
    pub fn walkable(&self, c: (i32, i32)) -> bool {
        if self.cell(c) != Cell::Free {
            return false;
        }
        let r = (INFLATE_M / RES).ceil() as i32;
        for dx in -r..=r {
            for dy in -r..=r {
                if (dx * dx + dy * dy) as f64 * RES * RES <= INFLATE_M * INFLATE_M
                    && self.cell((c.0 + dx, c.1 + dy)) == Cell::Occupied
                {
                    return false;
                }
            }
        }
        true
    }

    /// A free cell with an unknown neighbour: the edge of what is known.
    pub fn is_frontier(&self, c: (i32, i32)) -> bool {
        self.cell(c) == Cell::Free
            && [(1, 0), (-1, 0), (0, 1), (0, -1)]
                .iter()
                .any(|d| self.cell((c.0 + d.0, c.1 + d.1)) == Cell::Unknown)
    }

    /// Breadth-first over walkable cells from `from` to the nearest frontier at least
    /// `min_m` away; the path back, from the cell after `from` to the frontier.
    pub fn path_to_frontier(&self, from: (f64, f64), min_m: f64) -> Option<Vec<(f64, f64)>> {
        let start = key(from.0, from.1);
        let mut prev: HashMap<(i32, i32), (i32, i32)> = HashMap::new();
        let mut seen: HashSet<(i32, i32)> = HashSet::from([start]);
        let mut q = VecDeque::from([start]);
        let min_cells = (min_m / RES).round() as i64;
        while let Some(c) = q.pop_front() {
            let far_enough = (i64::from(c.0 - start.0).pow(2) + i64::from(c.1 - start.1).pow(2))
                >= min_cells * min_cells;
            if c != start && far_enough && self.is_frontier(c) {
                let mut path = vec![centre(c)];
                let mut at = c;
                while let Some(&p) = prev.get(&at) {
                    if p == start {
                        break;
                    }
                    path.push(centre(p));
                    at = p;
                }
                path.reverse();
                return Some(path);
            }
            for d in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let n = (c.0 + d.0, c.1 + d.1);
                // The start cell itself may be inflated (the duck stands next to a wall);
                // walk out of it through free cells regardless.
                let ok = if c == start {
                    self.cell(n) == Cell::Free
                } else {
                    self.walkable(n)
                };
                if ok && seen.insert(n) {
                    prev.insert(n, c);
                    q.push_back(n);
                }
            }
        }
        None
    }

    pub fn counts(&self) -> (usize, usize) {
        let free = self.cells.values().filter(|&&v| v <= FREE).count();
        let occ = self.cells.values().filter(|&&v| v >= OCCUPIED).count();
        (free, occ)
    }

    fn bounds(&self) -> Option<((i32, i32), (i32, i32))> {
        let known: Vec<&(i32, i32)> = self
            .cells
            .iter()
            .filter(|(_, v)| **v <= FREE || **v >= OCCUPIED)
            .map(|(k, _)| k)
            .collect();
        if known.is_empty() {
            return None;
        }
        let xs = known.iter().map(|c| c.0);
        let ys = known.iter().map(|c| c.1);
        Some((
            (xs.clone().min().unwrap(), xs.max().unwrap()),
            (ys.clone().min().unwrap(), ys.max().unwrap()),
        ))
    }

    /// The room from above, one character per cell: `#` a thing, `.` floor, ` ` unknown,
    /// `@` the duck, `*` its track.
    pub fn ascii(&self, duck: (f64, f64)) -> String {
        let Some(((x0, x1), (y0, y1))) = self.bounds() else {
            return String::from("(nothing known yet)\n");
        };
        let track: HashSet<(i32, i32)> = self.track.iter().map(|&(x, y)| key(x, y)).collect();
        let at = key(duck.0, duck.1);
        let mut out = String::new();
        for y in (y0..=y1).rev() {
            for x in x0..=x1 {
                let c = (x, y);
                out.push(if c == at {
                    '@'
                } else {
                    match self.cell(c) {
                        Cell::Occupied => '#',
                        Cell::Free if track.contains(&c) => '*',
                        Cell::Free => '.',
                        Cell::Unknown => ' ',
                    }
                });
            }
            out.push('\n');
        }
        out
    }

    /// The same as an SVG: dark = things, pale = floor, blank = unknown, the track in orange,
    /// the duck a gold dot, an optional target in green.
    pub fn svg(&self, duck: (f64, f64, f64), target: Option<(f64, f64)>) -> String {
        const S: f64 = 8.0;
        const M: f64 = 20.0;
        let Some(((x0, x1), (y0, y1))) = self.bounds() else {
            return "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"200\" height=\"40\"><text x=\"10\" y=\"25\">nothing known yet</text></svg>\n".into();
        };
        let w = (x1 - x0 + 1) as f64 * S + 2.0 * M;
        let h = (y1 - y0 + 1) as f64 * S + 2.0 * M + 16.0;
        let px = |x: f64| M + (x / RES - x0 as f64) * S;
        let py = |y: f64| M + (y1 as f64 + 1.0 - y / RES) * S;
        let mut out = format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w}\" height=\"{h}\" viewBox=\"0 0 {w} {h}\" font-family=\"sans-serif\">\n<rect width=\"{w}\" height=\"{h}\" fill=\"#fafaf7\"/>\n"
        );
        for &c in self.cells.keys() {
            let fill = match self.cell(c) {
                Cell::Occupied => "#333",
                Cell::Free => "#dbe9f6",
                Cell::Unknown => continue,
            };
            out.push_str(&format!(
                "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"{S}\" height=\"{S}\" fill=\"{fill}\"/>\n",
                M + (c.0 - x0) as f64 * S,
                M + (y1 - c.1) as f64 * S
            ));
        }
        if self.track.len() > 1 {
            let pts: Vec<String> = self
                .track
                .iter()
                .map(|&(x, y)| format!("{:.1},{:.1}", px(x), py(y)))
                .collect();
            out.push_str(&format!(
                "<polyline points=\"{}\" fill=\"none\" stroke=\"#e07b39\" stroke-width=\"2\" opacity=\"0.8\"/>\n",
                pts.join(" ")
            ));
        }
        if let Some((tx, ty)) = target {
            out.push_str(&format!(
                "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"6\" fill=\"none\" stroke=\"#2a9d3f\" stroke-width=\"2\"/>\n",
                px(tx),
                py(ty)
            ));
        }
        let (dx, dy, yaw) = duck;
        out.push_str(&format!(
            "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"5\" fill=\"#e6b422\" stroke=\"#333\"/>\n<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#333\" stroke-width=\"2\"/>\n",
            px(dx), py(dy), px(dx), py(dy), px(dx + 0.25 * yaw.cos()), py(dy + 0.25 * yaw.sin())
        ));
        let (free, occ) = self.counts();
        out.push_str(&format!(
            "<text x=\"{M}\" y=\"{:.1}\" font-size=\"12\" fill=\"#555\">{:.1} m² floor seen · {occ} obstacle cells · {} frames · {RES} m cells</text>\n</svg>\n",
            h - 6.0,
            free as f64 * RES * RES,
            self.integrated
        ));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame with one level beam straight ahead hitting a wall `r` metres out, the rest
    /// floor at 1 m.
    fn frame(r: f64) -> ([[f64; 3]; ROWS * COLS], [Zone; ROWS * COLS]) {
        let mut dirs = [[1.0, 0.0, 0.0]; ROWS * COLS];
        let mut zones = [Zone::Floor {
            point: [1.0, 0.0, -0.12],
        }; ROWS * COLS];
        dirs[0] = [1.0, 0.0, 0.0];
        zones[0] = Zone::Hit {
            point: [r, 0.0, 0.1],
            range: r,
        };
        (dirs, zones)
    }

    #[test]
    fn a_hit_marks_the_wall_and_clears_the_way_to_it() {
        let mut m = RoomMap::default();
        let (dirs, zones) = frame(1.5);
        for _ in 0..3 {
            m.integrate((0.0, 0.0, 0.0), [0.05, 0.0, 0.2], &dirs, &zones);
        }
        assert_eq!(m.cell_at(1.5, 0.0), Cell::Occupied);
        assert_eq!(m.cell_at(0.8, 0.0), Cell::Free);
        assert_eq!(m.cell_at(0.0, 1.0), Cell::Unknown);
        assert!(!m.walkable(key(1.35, 0.0)), "inflated around the wall");
        assert!(m.walkable(key(0.6, 0.0)));
    }

    #[test]
    fn the_frontier_is_the_edge_of_the_known_floor() {
        let mut m = RoomMap::default();
        let (dirs, zones) = frame(2.5);
        for _ in 0..3 {
            m.integrate((0.0, 0.0, 0.0), [0.05, 0.0, 0.2], &dirs, &zones);
        }
        // Floor known to 1 m ahead in a line; the frontier is out there, not at our feet.
        let path = m.path_to_frontier((0.0, 0.0), 0.3).expect("a frontier");
        let (tx, ty) = *path.last().unwrap();
        assert!(tx > 0.3 && ty.abs() < 0.2, "target {tx},{ty}");
        assert!(path.len() >= 3);
        // Turned around (yaw π) the beams paint the other way; both sides become known.
        let mut back = m.clone();
        for _ in 0..2 {
            back.integrate(
                (0.0, 0.0, std::f64::consts::PI),
                [0.05, 0.0, 0.2],
                &dirs,
                &zones,
            );
        }
        assert_eq!(back.cell_at(-2.5, 0.0), Cell::Occupied);
    }

    #[test]
    fn ascii_and_svg_render_something() {
        let mut m = RoomMap::default();
        assert!(m.ascii((0.0, 0.0)).contains("nothing"));
        let (dirs, zones) = frame(1.6);
        m.integrate((0.0, 0.0, 0.0), [0.05, 0.0, 0.2], &dirs, &zones);
        m.integrate((0.0, 0.0, 0.0), [0.05, 0.0, 0.2], &dirs, &zones);
        let a = m.ascii((0.0, 0.0));
        assert!(a.contains('@') && a.contains('#'), "{a}");
        let s = m.svg((0.0, 0.0, 0.0), Some((1.0, 0.0)));
        assert!(s.contains("<rect") && s.contains("<circle"));
    }
}
