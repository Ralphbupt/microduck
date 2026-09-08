//! A maze in the duck's memory: a grid of cells with the walls it has seen and the cells it
//! has visited, and a depth-first explorer over it that cannot loop.
//!
//! Localisation is the grid itself: cells are `CELL_M` wide, headings are the four axes,
//! and odometry snapped to the nearest cell centre is the position — contact odometry is
//! good to ~10 cm over a maze, far inside half a cell. No socket, no physics; the behaviour
//! feeds it scan results and asks it where to go next.

use std::collections::HashMap;

pub const CELL_M: f64 = 0.8;

/// A heading along the maze's axes. `+x` is where the duck faces on boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Dir {
    #[default]
    East,
    North,
    West,
    South,
}

impl Dir {
    pub const ALL: [Dir; 4] = [Dir::East, Dir::North, Dir::West, Dir::South];

    pub fn from_yaw(yaw: f64) -> Dir {
        let q = (yaw / std::f64::consts::FRAC_PI_2).round().rem_euclid(4.0) as i32;
        [Dir::East, Dir::North, Dir::West, Dir::South][q as usize]
    }

    pub fn yaw(self) -> f64 {
        match self {
            Dir::East => 0.0,
            Dir::North => std::f64::consts::FRAC_PI_2,
            Dir::West => std::f64::consts::PI,
            Dir::South => -std::f64::consts::FRAC_PI_2,
        }
    }

    pub fn left(self) -> Dir {
        match self {
            Dir::East => Dir::North,
            Dir::North => Dir::West,
            Dir::West => Dir::South,
            Dir::South => Dir::East,
        }
    }

    pub fn right(self) -> Dir {
        self.left().left().left()
    }

    pub fn back(self) -> Dir {
        self.left().left()
    }

    pub fn step(self, cell: (i32, i32)) -> (i32, i32) {
        match self {
            Dir::East => (cell.0 + 1, cell.1),
            Dir::North => (cell.0, cell.1 + 1),
            Dir::West => (cell.0 - 1, cell.1),
            Dir::South => (cell.0, cell.1 - 1),
        }
    }

    fn index(self) -> usize {
        match self {
            Dir::East => 0,
            Dir::North => 1,
            Dir::West => 2,
            Dir::South => 3,
        }
    }
}

/// What a side of a cell turned out to be.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Unknown,
    Wall,
    Open,
    /// Open onto the outside: the exit.
    Outside,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Cell {
    sides: [Option<Side>; 4],
    pub visits: u32,
}

impl Cell {
    pub fn side(&self, d: Dir) -> Side {
        self.sides[d.index()].unwrap_or(Side::Unknown)
    }
}

/// The map plus the explorer's own state: where it is, which way it faces, the path it
/// came by (for backtracking).
#[derive(Debug, Clone, Default)]
pub struct MazeMap {
    cells: HashMap<(i32, i32), Cell>,
    pub at: (i32, i32),
    pub facing: Dir,
    /// The cells behind us, most recent last: depth-first backtracking pops these.
    trail: Vec<(i32, i32)>,
    pub moves: u32,
    /// Every cell entered, in order — the route, for drawing.
    pub path: Vec<(i32, i32)>,
}

/// What the explorer wants next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Plan {
    /// Look this way before deciding (the side is unknown).
    Look(Dir),
    /// Go to the neighbouring cell in this direction.
    Go(Dir),
    /// Step out through this side: the exit.
    Exit(Dir),
    /// Everything reachable is explored and none of it opens outside.
    Stuck,
}

impl MazeMap {
    pub fn new() -> Self {
        let mut m = Self::default();
        m.cells.entry((0, 0)).or_default().visits = 1;
        m.path.push((0, 0));
        m
    }

    /// The mission's start: outside the maze, facing the entrance. The way in is ahead; the
    /// other three sides of the start cell are declared walls so exploration and backtracking
    /// never leave the maze by the way they came.
    pub fn at_entrance() -> Self {
        let mut m = Self::new();
        let cell = m.cells.entry((0, 0)).or_default();
        for d in [Dir::East.left(), Dir::East.right(), Dir::East.back()] {
            cell.sides[d.index()] = Some(Side::Wall);
        }
        m
    }

    /// Snap odometry to the grid; the heading to the nearest axis.
    pub fn localise(&mut self, odom_x: f64, odom_y: f64, yaw: f64) {
        self.at = (
            (odom_x / CELL_M).round() as i32,
            (odom_y / CELL_M).round() as i32,
        );
        self.facing = Dir::from_yaw(yaw);
    }

    pub fn cell(&self, c: (i32, i32)) -> Cell {
        self.cells.get(&c).copied().unwrap_or_default()
    }

    /// Record what a glance from the current cell showed in direction `d`. A wall on our
    /// side is a wall on the neighbour's side too.
    pub fn observe(&mut self, d: Dir, side: Side) {
        let here = self.at;
        self.cells.entry(here).or_default().sides[d.index()] = Some(side);
        if side != Side::Outside {
            let other = d.step(here);
            self.cells.entry(other).or_default().sides[d.back().index()] = Some(side);
        }
    }

    /// Arrived in the cell `d` of the last one (call after the move lands).
    pub fn moved(&mut self, d: Dir, arrived: (i32, i32)) {
        let from = self.at;
        // Going forward pushes the trail; a backtrack pops it.
        if self.trail.last() == Some(&arrived) {
            self.trail.pop();
        } else {
            self.trail.push(from);
        }
        self.at = arrived;
        self.facing = d;
        self.path.push(arrived);
        self.cells.entry(arrived).or_default().visits += 1;
        // The way we came is open, whatever we thought.
        self.cells.entry(arrived).or_default().sides[d.back().index()] = Some(Side::Open);
        self.moves += 1;
    }

    /// Depth-first: an exit if seen; else an unvisited open neighbour (left, straight,
    /// right — the order keeps the walk looking deliberate); else look at an unknown side;
    /// else backtrack along the trail.
    pub fn plan(&self) -> Plan {
        let here = self.cell(self.at);
        let order = [
            self.facing.left(),
            self.facing,
            self.facing.right(),
            self.facing.back(),
        ];
        if let Some(d) = order
            .iter()
            .copied()
            .find(|&d| here.side(d) == Side::Outside)
        {
            return Plan::Exit(d);
        }
        for d in order.iter().copied().take(3) {
            if here.side(d) == Side::Open && self.cell(d.step(self.at)).visits == 0 {
                return Plan::Go(d);
            }
        }
        if let Some(d) = order
            .iter()
            .copied()
            .take(3)
            .find(|&d| here.side(d) == Side::Unknown)
        {
            return Plan::Look(d);
        }
        // Back only counts as unexplored when we did not come from there.
        if here.side(self.facing.back()) == Side::Open
            && self.cell(self.facing.back().step(self.at)).visits == 0
        {
            return Plan::Go(self.facing.back());
        }
        // Backtrack along the trail — but only through a side the map does not now call a
        // wall (a localisation slip can contradict the trail; the wall wins).
        if let Some(&prev) = self.trail.last()
            && let Some(d) = Dir::ALL.iter().copied().find(|&d| d.step(self.at) == prev)
            && here.side(d) != Side::Wall
        {
            return Plan::Go(d);
        }
        // Otherwise the least-visited open neighbour, so a slip does not end the mission.
        let mut best: Option<(u32, Dir)> = None;
        for d in order {
            if here.side(d) == Side::Open {
                let v = self.cell(d.step(self.at)).visits;
                if best.is_none_or(|(b, _)| v < b) {
                    best = Some((v, d));
                }
            }
        }
        if let Some((_, d)) = best {
            return Plan::Go(d);
        }
        Plan::Stuck
    }

    /// Forget what this cell's sides looked like and drop the trail: the recovery from a
    /// contradiction, after which the explorer looks again.
    pub fn forget_here(&mut self) {
        self.cells.entry(self.at).or_default().sides = [None; 4];
        self.trail.clear();
    }

    /// Forget every wall in the map but keep the visit counts: the recovery when the map
    /// has painted itself into a corner (a failed move recorded as a wall, say).
    pub fn forget_walls(&mut self) {
        for cell in self.cells.values_mut() {
            cell.sides = [None; 4];
            cell.visits = cell.visits.min(1);
        }
        self.trail.clear();
    }

    /// The first side among left, straight, right that has not been looked at.
    pub fn unknown_ahead(&self) -> Option<Dir> {
        let here = self.cell(self.at);
        [self.facing.left(), self.facing, self.facing.right()]
            .into_iter()
            .find(|&d| here.side(d) == Side::Unknown)
    }

    pub fn cells_known(&self) -> usize {
        self.cells.len()
    }
}

impl MazeMap {
    /// The box around every cell the duck stood in.
    fn bounds(&self) -> ((i32, i32), (i32, i32)) {
        let visited: Vec<(i32, i32)> = self
            .cells
            .iter()
            .filter(|(_, c)| c.visits > 0)
            .map(|(&k, _)| k)
            .collect();
        let xs = visited.iter().map(|c| c.0);
        let ys = visited.iter().map(|c| c.1);
        (
            (xs.clone().min().unwrap_or(0), xs.max().unwrap_or(0)),
            (ys.clone().min().unwrap_or(0), ys.max().unwrap_or(0)),
        )
    }

    fn visited(&self, c: (i32, i32)) -> bool {
        self.cell(c).visits > 0
    }

    /// The edge between cell `c` and its neighbour in direction `d`, as either side recorded
    /// it — a wall seen from one cell is the same wall from the other.
    pub fn edge(&self, c: (i32, i32), d: Dir) -> Side {
        let mine = self.cell(c).side(d);
        if mine != Side::Unknown {
            return mine;
        }
        self.cell(d.step(c)).side(d.back())
    }

    /// The map as the duck believes it, in the generator's notation: `--` a wall, spaces an
    /// opening, `··` a side never looked at; cells show their visit count, `@` is the duck.
    /// Rows run north to south so it reads like the room seen from above.
    pub fn ascii(&self) -> String {
        let ((x0, x1), (y0, y1)) = self.bounds();
        let horizontal = |c: (i32, i32), d: Dir| -> &str {
            if !self.visited(c) && !self.visited(d.step(c)) {
                return "  ";
            }
            match self.edge(c, d) {
                Side::Wall => "--",
                Side::Open => "  ",
                Side::Outside => "^^",
                Side::Unknown => "··",
            }
        };
        let vertical = |c: (i32, i32), d: Dir| -> char {
            if !self.visited(c) && !self.visited(d.step(c)) {
                return ' ';
            }
            match self.edge(c, d) {
                Side::Wall => '|',
                Side::Open => ' ',
                Side::Outside => '>',
                Side::Unknown => ':',
            }
        };
        let mut out = String::new();
        for y in (y0..=y1).rev() {
            let mut top = String::from("+");
            for x in x0..=x1 {
                top.push_str(horizontal((x, y), Dir::North));
                top.push('+');
            }
            out.push_str(&top);
            out.push('\n');
            let mut row = String::new();
            for x in x0..=x1 {
                let c = (x, y);
                row.push(vertical(c, Dir::West));
                let body = if c == self.at {
                    "@ ".to_owned()
                } else if self.visited(c) {
                    format!("{} ", self.cell(c).visits)
                } else {
                    "  ".to_owned()
                };
                row.push_str(&body);
            }
            row.push(vertical((x1, y), Dir::East));
            out.push_str(&row);
            out.push('\n');
        }
        let mut bottom = String::from("+");
        for x in x0..=x1 {
            bottom.push_str(horizontal((x, y0), Dir::South));
            bottom.push('+');
        }
        out.push_str(&bottom);
        out.push('\n');
        out
    }

    /// The same map as an SVG: walls solid, unlooked sides dotted, visited cells shaded by
    /// visits, the route as a line from the green start to the gold end.
    pub fn svg(&self) -> String {
        const S: f64 = 60.0;
        const M: f64 = 30.0;
        let ((x0, x1), (y0, y1)) = self.bounds();
        let w = (x1 - x0 + 1) as f64 * S + 2.0 * M;
        let h = (y1 - y0 + 1) as f64 * S + 2.0 * M + 16.0;
        // Cell (x, y) -> its top-left corner on the page (y up in the maze, down on the page).
        let px = |x: i32| M + (x - x0) as f64 * S;
        let py = |y: i32| M + (y1 - y) as f64 * S;
        let mut out = format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w}\" height=\"{h}\" viewBox=\"0 0 {w} {h}\" font-family=\"sans-serif\">\n<rect width=\"{w}\" height=\"{h}\" fill=\"#fafaf7\"/>\n"
        );
        for y in y0..=y1 {
            for x in x0..=x1 {
                let c = (x, y);
                if !self.visited(c) {
                    continue;
                }
                let (cx, cy) = (px(x), py(y));
                let shade = match self.cell(c).visits {
                    1 => "#dbe9f6",
                    2 => "#b7d3ee",
                    _ => "#8fbbe3",
                };
                out.push_str(&format!(
                    "<rect x=\"{cx}\" y=\"{cy}\" width=\"{S}\" height=\"{S}\" fill=\"{shade}\"/>\n"
                ));
                let edges = [
                    (Dir::North, (cx, cy, cx + S, cy)),
                    (Dir::South, (cx, cy + S, cx + S, cy + S)),
                    (Dir::West, (cx, cy, cx, cy + S)),
                    (Dir::East, (cx + S, cy, cx + S, cy + S)),
                ];
                for (d, (ax, ay, bx, by)) in edges {
                    let style = match self.edge(c, d) {
                        Side::Wall => "stroke=\"#333\" stroke-width=\"4\"",
                        Side::Open => continue,
                        Side::Outside => {
                            "stroke=\"#2a9d3f\" stroke-width=\"4\" stroke-dasharray=\"2 6\""
                        }
                        Side::Unknown => {
                            "stroke=\"#bbb\" stroke-width=\"2\" stroke-dasharray=\"3 5\""
                        }
                    };
                    out.push_str(&format!(
                        "<line x1=\"{ax}\" y1=\"{ay}\" x2=\"{bx}\" y2=\"{by}\" {style} stroke-linecap=\"round\"/>\n"
                    ));
                }
            }
        }
        if self.path.len() > 1 {
            let pts: Vec<String> = self
                .path
                .iter()
                .map(|c| format!("{},{}", px(c.0) + S / 2.0, py(c.1) + S / 2.0))
                .collect();
            out.push_str(&format!(
                "<polyline points=\"{}\" fill=\"none\" stroke=\"#e07b39\" stroke-width=\"3\" stroke-linejoin=\"round\" opacity=\"0.85\"/>\n",
                pts.join(" ")
            ));
        }
        if let Some(first) = self.path.first() {
            out.push_str(&format!(
                "<circle cx=\"{}\" cy=\"{}\" r=\"9\" fill=\"#2a9d3f\"/>\n",
                px(first.0) + S / 2.0,
                py(first.1) + S / 2.0
            ));
        }
        out.push_str(&format!(
            "<circle cx=\"{}\" cy=\"{}\" r=\"9\" fill=\"#e6b422\" stroke=\"#333\" stroke-width=\"2\"/>\n",
            px(self.at.0) + S / 2.0,
            py(self.at.1) + S / 2.0
        ));
        let visited = self.cells.values().filter(|c| c.visits > 0).count();
        out.push_str(&format!(
            "<text x=\"{M}\" y=\"{}\" font-size=\"12\" fill=\"#555\">{visited} cells walked · {} moves · dotted = never looked · green = outside</text>\n</svg>\n",
            h - 6.0,
            self.moves
        ));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_ascii_map_reads_like_the_generator() {
        let mut m = MazeMap::at_entrance();
        m.observe(Dir::East, Side::Open);
        m.moved(Dir::East, (1, 0));
        m.observe(Dir::North, Side::Wall);
        m.observe(Dir::East, Side::Open);
        let text = m.ascii();
        assert!(text.contains('@'), "{text}");
        assert!(text.contains("--"), "{text}");
        assert!(text.contains("··"), "an unlooked side is dotted: {text}");
        let svg = m.svg();
        assert!(
            svg.starts_with("<svg")
                && svg.contains("<polyline")
                && svg.contains("stroke-dasharray")
        );
    }

    /// A 3x3 maze as a wall oracle (exit north of (2,2)):
    ///
    /// ```text
    /// +--+--+  +
    /// |     |  |
    /// +  +  +  +
    /// |  |     |
    /// +  +--+  +
    /// |        |
    /// +--+--+--+
    /// ```
    fn oracle(cell: (i32, i32), d: Dir) -> Side {
        let (x, y) = cell;
        let outside = |c: (i32, i32)| !(0..3).contains(&c.0) || !(0..3).contains(&c.1);
        let n = d.step(cell);
        if outside(n) {
            return if cell == (2, 2) && d == Dir::North {
                Side::Outside
            } else {
                Side::Wall
            };
        }
        // Interior walls: between (1,0)-(1,1)? no. Encode the picture: vertical wall east of
        // (0,1); wall east of (1,1) ... simplest: list the closed pairs.
        let closed = [
            ((0, 1), (1, 1)),
            ((1, 0), (1, 1)),
            ((2, 0), (2, 1)),
            ((1, 2), (2, 2)),
        ];
        let pair = (cell, n);
        let shut = closed
            .iter()
            .any(|&(a, b)| pair == (a, b) || pair == (b, a));
        let _ = (x, y);
        if shut { Side::Wall } else { Side::Open }
    }

    #[test]
    fn depth_first_reaches_the_exit_without_looping() {
        let mut m = MazeMap::new();
        let mut steps = 0;
        loop {
            steps += 1;
            assert!(steps < 60, "looped: {:?}", m.at);
            match m.plan() {
                Plan::Look(d) => m.observe(d, oracle(m.at, d)),
                Plan::Go(d) => {
                    let to = d.step(m.at);
                    m.moved(d, to);
                }
                Plan::Exit(d) => {
                    assert_eq!((m.at, d), ((2, 2), Dir::North));
                    break;
                }
                Plan::Stuck => panic!("stuck at {:?}", m.at),
            }
        }
        // Every cell visited at most a few times: no cycling.
        for (c, cell) in &m.cells {
            assert!(cell.visits <= 3, "{c:?} visited {} times", cell.visits);
        }
    }

    #[test]
    fn a_dead_end_backtracks_along_the_trail() {
        let mut m = MazeMap::new();
        // Corridor east two cells, dead end; the way back must be planned.
        m.observe(Dir::East, Side::Open);
        m.observe(Dir::North, Side::Wall);
        m.observe(Dir::South, Side::Wall);
        m.moved(Dir::East, (1, 0));
        for d in [Dir::East, Dir::North, Dir::South] {
            m.observe(d, Side::Wall);
        }
        assert_eq!(m.plan(), Plan::Go(Dir::West), "back the way we came");
        m.moved(Dir::West, (0, 0));
        assert!(m.trail.is_empty());
        assert_eq!(
            m.plan(),
            Plan::Look(Dir::West),
            "the one unknown side of the start"
        );
    }

    #[test]
    fn localisation_snaps_to_the_grid() {
        let mut m = MazeMap::new();
        m.localise(0.85, -0.1, 1.4);
        assert_eq!(m.at, (1, 0));
        assert_eq!(m.facing, Dir::North);
        assert_eq!(Dir::from_yaw(-3.0), Dir::West);
        assert_eq!(Dir::from_yaw(-1.6), Dir::South);
    }
}
