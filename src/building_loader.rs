use crate::config::{AMBIENT_TEMP_C, ATTRACTOR_OFFSET_M, FLASHOVER_TEMP_C};
use crate::model::hazard::{AlarmStation, ExtinguisherItem, FloorStatus};
use crate::model::topology::{
    Cell, CellKind, Obstacle, Portal, RouteInfo, SpatialGrid, WallSegment,
};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};
use std::fs;

pub const TILE_SIZE_M: f32 = 1.0;

#[derive(Copy, Clone, PartialEq)]
struct MazeNode {
    cost: f32,
    r: usize,
    c: usize,
}
impl Eq for MazeNode {}
impl Ord for MazeNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for MazeNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct LoadedBuilding {
    pub cells: Vec<Cell>,
    pub portals: Vec<Portal>,
    pub alarms: Vec<AlarmStation>,
    pub extinguishers: Vec<ExtinguisherItem>,
    pub floors: Vec<FloorStatus>,
    pub grid_width_m: f32,
    pub grid_height_m: f32,
    pub rows: usize,
    pub cols: usize,
    pub raw_floors: Vec<Vec<Vec<char>>>,
    pub cell_grid: Vec<Vec<Vec<Option<usize>>>>,
    pub portal_at_tile: Vec<Vec<Vec<Option<usize>>>>,
    pub walkable_grid: Vec<Vec<Vec<bool>>>,
    // Matrices termodinámicas por baldosa (1m x 1m) para propagación en ondas reales
    pub tile_temp: Vec<Vec<Vec<f32>>>,
    pub tile_smoke: Vec<Vec<Vec<f32>>>,
    pub tile_co: Vec<Vec<Vec<f32>>>,
    pub tile_co2: Vec<Vec<Vec<f32>>>,
    pub tile_fuel: Vec<Vec<Vec<f32>>>,
    pub tile_fire_time: Vec<Vec<Vec<f32>>>,
    // Campos de navegación del laberinto
    pub corridor_flow: Vec<Vec<Vec<f32>>>,
    pub naive_flow: Vec<Vec<Vec<f32>>>,
    pub optimal_flow: Vec<Vec<Vec<f32>>>,
    pub corridor_waypoints: Vec<Vec<(f32, f32)>>,
}

impl LoadedBuilding {
    pub fn cell_at(&self, floor: usize, x: f32, y: f32) -> Option<usize> {
        if floor >= self.cell_grid.len() || x < 0.0 || y < 0.0 {
            return None;
        }
        let c = (x / TILE_SIZE_M).floor() as usize;
        let r = (y / TILE_SIZE_M).floor() as usize;
        if r < self.rows && c < self.cols {
            self.cell_grid[floor][r][c]
        } else {
            None
        }
    }

    pub fn tile_coords_at(&self, x: f32, y: f32) -> (usize, usize) {
        let c = ((x / TILE_SIZE_M).floor() as isize).clamp(0, self.cols as isize - 1) as usize;
        let r = ((y / TILE_SIZE_M).floor() as isize).clamp(0, self.rows as isize - 1) as usize;
        (r, c)
    }

    /// Inyecta o apaga fuego exactamente en la baldosa clicada (y su vecindad inmediata de 1m)
    pub fn ignite_at_tile(&mut self, floor: usize, r: usize, c: usize, target_temp: f32) {
        if floor >= self.floors.len() || r >= self.rows || c >= self.cols {
            return;
        }
        if !self.walkable_grid[floor][r][c] {
            return;
        }

        if target_temp <= AMBIENT_TEMP_C + 1.0 {
            // Apagar un radio de 2 baldosas alrededor del clic
            for dr in -2..=2isize {
                for dc in -2..=2isize {
                    let nr = r as isize + dr;
                    let nc = c as isize + dc;
                    if nr >= 0 && nr < self.rows as isize && nc >= 0 && nc < self.cols as isize {
                        let (ur, uc) = (nr as usize, nc as usize);
                        self.tile_temp[floor][ur][uc] = AMBIENT_TEMP_C;
                        self.tile_smoke[floor][ur][uc] = 0.0;
                        self.tile_co[floor][ur][uc] = 0.0;
                        self.tile_fire_time[floor][ur][uc] = 0.0;
                    }
                }
            }
        } else {
            // Encender epicentro puntual en la baldosa (r, c) para que crezca en ondas
            self.tile_temp[floor][r][c] = target_temp;
            self.tile_smoke[floor][r][c] = if target_temp >= FLASHOVER_TEMP_C { 0.85 } else { 0.35 };
            self.tile_fire_time[floor][r][c] = if target_temp >= FLASHOVER_TEMP_C { 15.0 } else { 2.0 };

            // Precalentar suavemente las 4 baldosas ortogonales para iniciar el frente de onda circular
            for (dr, dc) in [(-1isize, 0isize), (1, 0), (0, -1), (0, 1)] {
                let nr = r as isize + dr;
                let nc = c as isize + dc;
                if self.is_tile_passable(floor, nr, nc) {
                    let (ur, uc) = (nr as usize, nc as usize);
                    self.tile_temp[floor][ur][uc] =
                    self.tile_temp[floor][ur][uc].max(AMBIENT_TEMP_C + (target_temp - AMBIENT_TEMP_C) * 0.35);
                    self.tile_smoke[floor][ur][uc] = self.tile_smoke[floor][ur][uc].max(0.15);
                }
            }
        }
    }

    pub fn is_tile_passable(&self, floor: usize, r: isize, c: isize) -> bool {
        if floor >= self.walkable_grid.len()
            || r < 0
            || c < 0
            || r >= self.rows as isize
            || c >= self.cols as isize
            {
                return false;
            }
            let ur = r as usize;
        let uc = c as usize;
        if !self.walkable_grid[floor][ur][uc] {
            return false;
        }
        if let Some(pid) = self.portal_at_tile[floor][ur][uc] {
            if self.portals[pid].is_closed_or_blocked {
                return false;
            }
        }
        if let Some(cid) = self.cell_grid[floor][ur][uc] {
            if self.cells[cid].is_collapsed {
                return false;
            }
        }
        true
    }

    pub fn recompute_maze_flow(&mut self) {
        let num_floors = self.floors.len();
        let dirs: [(isize, isize, f32); 8] = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, 1.414),
            (-1, 1, 1.414),
            (1, -1, 1.414),
            (1, 1, 1.414),
        ];

        for f in 0..num_floors {
            // 1. Meta 1: Salir de las oficinas hacia el pasillo (evitando baldosas en llamas)
            let mut c_dist = vec![vec![f32::INFINITY; self.cols]; self.rows];
            let mut c_heap = BinaryHeap::new();
            for r in 0..self.rows {
                for c in 0..self.cols {
                    if !self.is_tile_passable(f, r as isize, c as isize) {
                        continue;
                    }
                    if let Some(cid) = self.cell_grid[f][r][c] {
                        if self.cells[cid].kind == CellKind::Corridor
                            && self.tile_temp[f][r][c] < 120.0
                            {
                                c_dist[r][c] = 0.0;
                                c_heap.push(MazeNode { cost: 0.0, r, c });
                            }
                    }
                }
            }
            while let Some(MazeNode { cost, r, c }) = c_heap.pop() {
                if cost > c_dist[r][c] {
                    continue;
                }
                for (dr, dc, w) in dirs {
                    let nr = r as isize + dr;
                    let nc = c as isize + dc;
                    if !self.is_tile_passable(f, nr, nc) {
                        continue;
                    }
                    if dr != 0
                        && dc != 0
                        && (!self.is_tile_passable(f, r as isize, nc)
                        || !self.is_tile_passable(f, nr, c as isize))
                        {
                            continue;
                        }
                        let (ur, uc) = (nr as usize, nc as usize);
                    let heat_pen = if self.tile_temp[f][ur][uc] > 80.0 {
                        (self.tile_temp[f][ur][uc] - AMBIENT_TEMP_C) * 0.4
                    } else {
                        0.0
                    };
                    let next_c = cost + w + heat_pen;
                    if next_c < c_dist[ur][uc] {
                        c_dist[ur][uc] = next_c;
                        c_heap.push(MazeNode {
                            cost: next_c,
                            r: ur,
                            c: uc,
                        });
                    }
                }
            }
            self.corridor_flow[f] = c_dist;

            // 2. Meta 3: optimal_flow y naive_flow evaluando el calor baldosa por baldosa
            let mut opt_dist = vec![vec![f32::INFINITY; self.cols]; self.rows];
            let mut naive_dist = vec![vec![f32::INFINITY; self.cols]; self.rows];
            let mut opt_heap = BinaryHeap::new();
            let mut naive_heap = BinaryHeap::new();

            for r in 0..self.rows {
                for c in 0..self.cols {
                    if !self.is_tile_passable(f, r as isize, c as isize) {
                        continue;
                    }
                    if let Some(cid) = self.cell_grid[f][r][c] {
                        let kind = self.cells[cid].kind;
                        let is_goal = if f == 0 {
                            kind == CellKind::Exit
                        } else {
                            kind == CellKind::Stairwell && !self.cells[cid].is_collapsed
                        };

                        if is_goal {
                            let crowd_pen = (self.cells[cid].occupant_ids.len() as f32) * 0.45;
                            opt_dist[r][c] = crowd_pen;
                            opt_heap.push(MazeNode {
                                cost: crowd_pen,
                                r,
                                c,
                            });

                            let habit_bias = if c < self.cols / 2 { 0.0 } else { 5.0 };
                            naive_dist[r][c] = habit_bias;
                            naive_heap.push(MazeNode {
                                cost: habit_bias,
                                r,
                                c,
                            });
                        }
                    }
                }
            }

            while let Some(MazeNode { cost, r, c }) = opt_heap.pop() {
                if cost > opt_dist[r][c] {
                    continue;
                }
                for (dr, dc, step_w) in dirs {
                    let (nr, nc) = (r as isize + dr, c as isize + dc);
                    if !self.is_tile_passable(f, nr, nc) {
                        continue;
                    }
                    if dr != 0
                        && dc != 0
                        && (!self.is_tile_passable(f, r as isize, nc)
                        || !self.is_tile_passable(f, nr, c as isize))
                        {
                            continue;
                        }
                        let (ur, uc) = (nr as usize, nc as usize);
                    let mut pen = 0.0_f32;
                    let t_tile = self.tile_temp[f][ur][uc];
                    let s_tile = self.tile_smoke[f][ur][uc];
                    if t_tile > AMBIENT_TEMP_C + 8.0 {
                        pen += (t_tile - AMBIENT_TEMP_C) * 0.45;
                    }
                    if t_tile >= 280.0 {
                        pen += 900.0;
                    }
                    pen += s_tile * 14.0;

                    if let Some(cid) = self.cell_grid[f][ur][uc] {
                        let cell = &self.cells[cid];
                        if f == 0 && cell.kind == CellKind::Stairwell {
                            pen += 8.0;
                        }
                        pen += cell.density() * 0.80;
                    }
                    let next_cost = cost + step_w + pen;
                    if next_cost < opt_dist[ur][uc] {
                        opt_dist[ur][uc] = next_cost;
                        opt_heap.push(MazeNode {
                            cost: next_cost,
                            r: ur,
                            c: uc,
                        });
                    }
                }
            }

            while let Some(MazeNode { cost, r, c }) = naive_heap.pop() {
                if cost > naive_dist[r][c] {
                    continue;
                }
                for (dr, dc, step_w) in dirs {
                    let (nr, nc) = (r as isize + dr, c as isize + dc);
                    if !self.is_tile_passable(f, nr, nc) {
                        continue;
                    }
                    if dr != 0
                        && dc != 0
                        && (!self.is_tile_passable(f, r as isize, nc)
                        || !self.is_tile_passable(f, nr, c as isize))
                        {
                            continue;
                        }
                        let (ur, uc) = (nr as usize, nc as usize);
                    let mut pen = 0.0_f32;
                    // El civil ve las llamas directas en una baldosa (> 220°C) y las rodea, pero no prevé humo ni congestión lejana
                    if self.tile_temp[f][ur][uc] >= 220.0 {
                        pen += 450.0;
                    }
                    if let Some(cid) = self.cell_grid[f][ur][uc] {
                        if f == 0 && self.cells[cid].kind == CellKind::Stairwell {
                            pen += 8.0;
                        }
                    }
                    let next_cost = cost + step_w + pen;
                    if next_cost < naive_dist[ur][uc] {
                        naive_dist[ur][uc] = next_cost;
                        naive_heap.push(MazeNode {
                            cost: next_cost,
                            r: ur,
                            c: uc,
                        });
                    }
                }
            }

            self.optimal_flow[f] = opt_dist;
            self.naive_flow[f] = naive_dist;
        }
    }

    pub fn sample_flow_direction(
        &self,
        flow_map: &[Vec<Vec<f32>>],
        floor: usize,
        x: f32,
        y: f32,
    ) -> (f32, f32) {
        if floor >= flow_map.len() {
            return (0.0, 0.0);
        }
        let (r, c) = self.tile_coords_at(x, y);
        let flow = &flow_map[floor];
        let mut best_target = ((c as f32 + 0.5) * TILE_SIZE_M, (r as f32 + 0.5) * TILE_SIZE_M);
        let mut best_val = flow[r][c];

        for dr in -2..=2isize {
            for dc in -2..=2isize {
                if dr == 0 && dc == 0 {
                    continue;
                }
                let nr = r as isize + dr;
                let nc = c as isize + dc;
                if !self.is_tile_passable(floor, nr, nc) {
                    continue;
                }
                let mid_r = r as isize + dr.signum();
                let mid_c = c as isize + dc.signum();
                if !self.is_tile_passable(floor, mid_r, mid_c)
                    || !self.is_tile_passable(floor, r as isize, mid_c)
                    || !self.is_tile_passable(floor, mid_r, c as isize)
                    {
                        continue;
                    }

                    let val = flow[nr as usize][nc as usize];
                if val < best_val {
                    best_val = val;
                    best_target = (
                        (nc as f32 + 0.5) * TILE_SIZE_M,
                                   (nr as f32 + 0.5) * TILE_SIZE_M,
                    );
                }
            }
        }

        let dx = best_target.0 - x;
        let dy = best_target.1 - y;
        let len = dx.hypot(dy);
        if len > 1e-4 {
            (dx / len, dy / len)
        } else {
            (0.0, 0.0)
        }
    }

    pub fn direction_towards_point(
        &self,
        floor: usize,
        x: f32,
        y: f32,
        tx: f32,
        ty: f32,
    ) -> (f32, f32) {
        let (start_r, start_c) = self.tile_coords_at(x, y);
        let (goal_r, goal_c) = self.tile_coords_at(tx, ty);

        if (start_r as isize - goal_r as isize).abs() <= 1
            && (start_c as isize - goal_c as isize).abs() <= 1
            {
                let dx = tx - x;
                let dy = ty - y;
                let l = dx.hypot(dy).max(1e-4);
                return (dx / l, dy / l);
            }

            let mut dist = vec![vec![u16::MAX; self.cols]; self.rows];
        let mut q = VecDeque::new();
        dist[goal_r][goal_c] = 0;
        q.push_back((goal_r, goal_c));

        while let Some((cr, cc)) = q.pop_front() {
            if (cr, cc) == (start_r, start_c) {
                break;
            }
            let d = dist[cr][cc];
            for (dr, dc) in [(-1isize, 0isize), (1, 0), (0, -1), (0, 1)] {
                let nr = cr as isize + dr;
                let nc = cc as isize + dc;
                if self.is_tile_passable(floor, nr, nc) {
                    let ur = nr as usize;
                    let uc = nc as usize;
                    if dist[ur][uc] == u16::MAX {
                        dist[ur][uc] = d + 1;
                        q.push_back((ur, uc));
                    }
                }
            }
        }

        let mut best_rc = (start_r, start_c);
        let mut best_d = dist[start_r][start_c];
        for (dr, dc) in [
            (-1isize, 0isize),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ] {
            let nr = start_r as isize + dr;
            let nc = start_c as isize + dc;
            if self.is_tile_passable(floor, nr, nc) {
                if dr != 0
                    && dc != 0
                    && (!self.is_tile_passable(floor, start_r as isize, nc)
                    || !self.is_tile_passable(floor, nr, start_c as isize))
                    {
                        continue;
                    }
                    let d = dist[nr as usize][nc as usize];
                if d < best_d {
                    best_d = d;
                    best_rc = (nr as usize, nc as usize);
                }
            }
        }

        let wx = (best_rc.1 as f32 + 0.5) * TILE_SIZE_M;
        let wy = (best_rc.0 as f32 + 0.5) * TILE_SIZE_M;
        let dx = wx - x;
        let dy = wy - y;
        let l = dx.hypot(dy).max(1e-4);
        (dx / l, dy / l)
    }

    pub fn resolve_maze_wall_collisions(
        &self,
        floor: usize,
        x: &mut f32,
        y: &mut f32,
        radius: f32,
    ) {
        let cr = (*y / TILE_SIZE_M).floor() as isize;
        let cc = (*x / TILE_SIZE_M).floor() as isize;

        for dr in -1..=1 {
            for dc in -1..=1 {
                let nr = cr + dr;
                let nc = cc + dc;
                if !self.is_tile_passable(floor, nr, nc) {
                    let box_min_x = (nc as f32) * TILE_SIZE_M;
                    let box_max_x = box_min_x + TILE_SIZE_M;
                    let box_min_y = (nr as f32) * TILE_SIZE_M;
                    let box_max_y = box_min_y + TILE_SIZE_M;

                    let closest_x = (*x).clamp(box_min_x, box_max_x);
                    let closest_y = (*y).clamp(box_min_y, box_max_y);
                    let dx = *x - closest_x;
                    let dy = *y - closest_y;
                    let dist_sq = dx * dx + dy * dy;

                    if dist_sq < radius * radius {
                        let dist = dist_sq.sqrt();
                        if dist > 1e-4 {
                            let overlap = radius - dist;
                            *x += (dx / dist) * overlap;
                            *y += (dy / dist) * overlap;
                        } else {
                            let tile_cx = (cc as f32 + 0.5) * TILE_SIZE_M;
                            let tile_cy = (cr as f32 + 0.5) * TILE_SIZE_M;
                            *x = (*x + (tile_cx - *x) * 0.3).clamp(0.2, self.grid_width_m - 0.2);
                            *y = (*y + (tile_cy - *y) * 0.3).clamp(0.2, self.grid_height_m - 0.2);
                        }
                    }
                }
            }
        }
    }
}

pub fn load_building_from_ascii(path: &str) -> Result<LoadedBuilding, String> {
    let content = fs::read_to_string(path)
    .map_err(|e| format!("No se pudo leer el plano '{}': {}", path, e))?;

    let mut raw_floors: Vec<Vec<Vec<char>>> = Vec::new();
    let mut current_floor: Vec<Vec<char>> = Vec::new();

    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with("//") {
            continue;
        }
        if trimmed.starts_with("=== FLOOR") {
            if !current_floor.is_empty() {
                raw_floors.push(current_floor);
                current_floor = Vec::new();
            }
            continue;
        }
        current_floor.push(trimmed.chars().collect());
    }
    if !current_floor.is_empty() {
        raw_floors.push(current_floor);
    }

    if raw_floors.is_empty() {
        return Err("El archivo de edificio no contiene pisos.".into());
    }

    let num_floors = raw_floors.len();
    let rows = raw_floors[0].len();
    let cols = raw_floors[0][0].len();

    let mut cell_grid: Vec<Vec<Vec<Option<usize>>>> =
    vec![vec![vec![None; cols]; rows]; num_floors];
    let mut portal_at_tile: Vec<Vec<Vec<Option<usize>>>> =
    vec![vec![vec![None; cols]; rows]; num_floors];
    let mut walkable_grid: Vec<Vec<Vec<bool>>> = vec![vec![vec![false; cols]; rows]; num_floors];
    let mut cells: Vec<Cell> = Vec::new();
    let mut alarms: Vec<AlarmStation> = Vec::new();
    let mut extinguishers: Vec<ExtinguisherItem> = Vec::new();

    let is_room_tile = |ch: char| matches!(ch, '.' | 'C' | 'S' | 'V' | 'X' | 'A' | 'E' | 'O');

    for f in 0..num_floors {
        for r in 0..rows {
            for c in 0..cols {
                let ch = raw_floors[f][r][c];
                if ch != '#' && ch != 'O' {
                    walkable_grid[f][r][c] = true;
                }
            }
        }
    }

    for f in 0..num_floors {
        for r in 0..rows {
            for c in 0..cols {
                let ch = raw_floors[f][r][c];
                if is_room_tile(ch) && cell_grid[f][r][c].is_none() {
                    let cell_id = cells.len();
                    let mut queue = VecDeque::new();
                    queue.push_back((r, c));
                    cell_grid[f][r][c] = Some(cell_id);

                    let mut min_r = r;
                    let mut max_r = r;
                    let mut min_c = c;
                    let mut max_c = c;
                    let mut tiles = Vec::new();
                    let mut detected_kind = CellKind::Room;
                    let mut corridor_tiles = 0usize;
                    let mut cell_obstacles = Vec::new();

                    while let Some((cr, cc)) = queue.pop_front() {
                        tiles.push((cr, cc));
                        min_r = min_r.min(cr);
                        max_r = max_r.max(cr);
                        min_c = min_c.min(cc);
                        max_c = max_c.max(cc);

                        let tile_ch = raw_floors[f][cr][cc];
                        let center_x = (cc as f32 + 0.5) * TILE_SIZE_M;
                        let center_y = (cr as f32 + 0.5) * TILE_SIZE_M;

                        match tile_ch {
                            'X' => detected_kind = CellKind::Exit,
                            'S' => detected_kind = CellKind::Stairwell,
                            'V' => detected_kind = CellKind::Elevator { shaft_id: cc },
                            'C' => corridor_tiles += 1,
                            'O' => cell_obstacles.push(Obstacle {
                                x: center_x,
                                y: center_y,
                                radius: 0.45,
                            }),
                            'A' => alarms.push(AlarmStation {
                                id: alarms.len(),
                                               floor: f,
                                               cell_id,
                                               x: center_x,
                                               y: center_y,
                                               is_triggered: false,
                            }),
                            'E' => extinguishers.push(ExtinguisherItem {
                                id: extinguishers.len(),
                                                      floor: f,
                                                      cell_id,
                                                      x: center_x,
                                                      y: center_y,
                                                      is_available: true,
                            }),
                            _ => {}
                        }

                        let neighbors = [
                            (cr.wrapping_sub(1), cc),
                            (cr + 1, cc),
                            (cr, cc.wrapping_sub(1)),
                            (cr, cc + 1),
                        ];
                        for (nr, nc) in neighbors {
                            if nr < rows && nc < cols && cell_grid[f][nr][nc].is_none() {
                                let nch = raw_floors[f][nr][nc];
                                let same_region = match (tile_ch, nch) {
                                    ('S', 'S') | ('V', 'V') | ('X', 'X') => true,
                                    ('S', _) | (_, 'S') | ('V', _) | (_, 'V') | ('X', _)
                                    | (_, 'X') => false,
                                    _ => is_room_tile(nch),
                                };
                                if same_region {
                                    cell_grid[f][nr][nc] = Some(cell_id);
                                    queue.push_back((nr, nc));
                                }
                            }
                        }
                    }

                    if detected_kind == CellKind::Room && corridor_tiles * 2 >= tiles.len() {
                        detected_kind = CellKind::Corridor;
                    }

                    let avg_r =
                    tiles.iter().map(|&(tr, _)| tr as f32).sum::<f32>() / (tiles.len() as f32);
                    let avg_c =
                    tiles.iter().map(|&(_, tc)| tc as f32).sum::<f32>() / (tiles.len() as f32);
                    let &(rep_r, rep_c) = tiles
                    .iter()
                    .filter(|&&(tr, tc)| raw_floors[f][tr][tc] != 'O')
                    .min_by(|&&(r1, c1), &&(r2, c2)| {
                        let d1 = (r1 as f32 - avg_r).hypot(c1 as f32 - avg_c);
                        let d2 = (r2 as f32 - avg_r).hypot(c2 as f32 - avg_c);
                        d1.partial_cmp(&d2).unwrap()
                    })
                    .unwrap_or(&tiles[0]);

                    let rep_point = (
                        (rep_c as f32 + 0.5) * TILE_SIZE_M,
                                     (rep_r as f32 + 0.5) * TILE_SIZE_M,
                    );

                    let mut cell_walls = Vec::new();
                    for &(tr, tc) in &tiles {
                        let x0 = (tc as f32) * TILE_SIZE_M;
                        let y0 = (tr as f32) * TILE_SIZE_M;
                        let x1 = x0 + TILE_SIZE_M;
                        let y1 = y0 + TILE_SIZE_M;

                        if tr == 0 || raw_floors[f][tr - 1][tc] == '#' {
                            cell_walls.push(WallSegment {
                                x1: x0,
                                y1: y0,
                                x2: x1,
                                y2: y0,
                                nx: 0.0,
                                ny: 1.0,
                            });
                        }
                        if tr + 1 == rows || raw_floors[f][tr + 1][tc] == '#' {
                            cell_walls.push(WallSegment {
                                x1: x0,
                                y1: y1,
                                x2: x1,
                                y2: y1,
                                nx: 0.0,
                                ny: -1.0,
                            });
                        }
                        if tc == 0 || raw_floors[f][tr][tc - 1] == '#' {
                            cell_walls.push(WallSegment {
                                x1: x0,
                                y1: y0,
                                x2: x0,
                                y2: y1,
                                nx: 1.0,
                                ny: 0.0,
                            });
                        }
                        if tc + 1 == cols || raw_floors[f][tr][tc + 1] == '#' {
                            cell_walls.push(WallSegment {
                                x1: x1,
                                y1: y0,
                                x2: x1,
                                y2: y1,
                                nx: -1.0,
                                ny: 0.0,
                            });
                        }
                    }

                    let area_m2 = (tiles.len() as f32) * TILE_SIZE_M * TILE_SIZE_M;
                    let name = format!("F{}-{:?}-{}", f, detected_kind, cell_id);
                    cells.push(Cell {
                        id: cell_id,
                        floor: f,
                        name,
                        kind: detected_kind,
                        min_x: (min_c as f32) * TILE_SIZE_M,
                               min_y: (min_r as f32) * TILE_SIZE_M,
                               max_x: ((max_c + 1) as f32) * TILE_SIZE_M,
                               max_y: ((max_r + 1) as f32) * TILE_SIZE_M,
                               rep_point,
                               tiles,
                               area_m2,
                               walls: cell_walls,
                               obstacles: cell_obstacles,
                               portals: Vec::new(),
                               temperature_c: AMBIENT_TEMP_C,
                               fire_time_s: 0.0,
                               co_ppm: 0.0,
                               co2_pct: 0.04,
                               smoke_opacity: 0.0,
                               fuel: 1.0,
                               is_collapsed: false,
                               routes: RouteInfo::default(),
                               occupant_ids: Vec::new(),
                               spatial_grid: SpatialGrid::new(1.0),
                    });
                }
            }
        }
    }

    let mut portals: Vec<Portal> = Vec::new();
    for f in 0..num_floors {
        for r in 0..rows {
            for c in 0..cols {
                let ch = raw_floors[f][r][c];
                if ch == 'D' || ch == 'F' {
                    let resistance = if ch == 'F' { 120.0 } else { 30.0 };
                    let door_width = if ch == 'F' { 1.8 } else { 1.35 };
                    let px = (c as f32 + 0.5) * TILE_SIZE_M;
                    let py = (r as f32 + 0.5) * TILE_SIZE_M;

                    if c > 0 && c + 1 < cols {
                        if let (Some(ca), Some(cb)) =
                            (cell_grid[f][r][c - 1], cell_grid[f][r][c + 1])
                            {
                                if ca != cb {
                                    cell_grid[f][r][c] = Some(ca);
                                    cells[ca].tiles.push((r, c));
                                    let pid = portals.len();
                                    portal_at_tile[f][r][c] = Some(pid);
                                    portals.push(Portal {
                                        id: pid,
                                        cell_a: ca,
                                        cell_b: cb,
                                        floor_a: f,
                                        floor_b: f,
                                        tile_rc: (r, c),
                                                 pos_a: (px, py),
                                                 pos_b: (px, py),
                                                 attractor_a: (px - ATTRACTOR_OFFSET_M, py),
                                                 attractor_b: (px + ATTRACTOR_OFFSET_M, py),
                                                 width_m: door_width,
                                                 fire_resistance_s: resistance,
                                                 max_resistance_s: resistance,
                                                 is_closed_or_blocked: false,
                                                 crossing_agents: Vec::new(),
                                    });
                                    cells[ca].portals.push(pid);
                                    cells[cb].portals.push(pid);
                                    continue;
                                }
                            }
                    }

                    if r > 0 && r + 1 < rows {
                        if let (Some(ca), Some(cb)) =
                            (cell_grid[f][r - 1][c], cell_grid[f][r + 1][c])
                            {
                                if ca != cb {
                                    cell_grid[f][r][c] = Some(ca);
                                    cells[ca].tiles.push((r, c));
                                    let pid = portals.len();
                                    portal_at_tile[f][r][c] = Some(pid);
                                    portals.push(Portal {
                                        id: pid,
                                        cell_a: ca,
                                        cell_b: cb,
                                        floor_a: f,
                                        floor_b: f,
                                        tile_rc: (r, c),
                                                 pos_a: (px, py),
                                                 pos_b: (px, py),
                                                 attractor_a: (px, py - ATTRACTOR_OFFSET_M),
                                                 attractor_b: (px, py + ATTRACTOR_OFFSET_M),
                                                 width_m: door_width,
                                                 fire_resistance_s: resistance,
                                                 max_resistance_s: resistance,
                                                 is_closed_or_blocked: false,
                                                 crossing_agents: Vec::new(),
                                    });
                                    cells[ca].portals.push(pid);
                                    cells[cb].portals.push(pid);
                                }
                            }
                    }
                }
            }
        }
    }

    for f in 0..(num_floors.saturating_sub(1)) {
        let mut linked_pairs = Vec::new();
        for r in 0..rows {
            for c in 0..cols {
                let ch_low = raw_floors[f][r][c];
                let ch_up = raw_floors[f + 1][r][c];
                if (ch_low == 'S' && ch_up == 'S') || (ch_low == 'V' && ch_up == 'V') {
                    if let (Some(ca), Some(cb)) = (cell_grid[f][r][c], cell_grid[f + 1][r][c]) {
                        if !linked_pairs.contains(&(ca, cb)) {
                            linked_pairs.push((ca, cb));
                            let pid = portals.len();
                            let center_a = cells[ca].center();
                            let center_b = cells[cb].center();
                            portals.push(Portal {
                                id: pid,
                                cell_a: ca,
                                cell_b: cb,
                                floor_a: f,
                                floor_b: f + 1,
                                tile_rc: (r, c),
                                         pos_a: center_a,
                                         pos_b: center_b,
                                         attractor_a: center_a,
                                         attractor_b: center_b,
                                         width_m: 2.0,
                                         fire_resistance_s: 120.0,
                                         max_resistance_s: 120.0,
                                         is_closed_or_blocked: false,
                                         crossing_agents: Vec::new(),
                            });
                            cells[ca].portals.push(pid);
                            cells[cb].portals.push(pid);
                        }
                    }
                }
            }
        }
    }

    propagate_shortest_and_alternative_paths(&mut cells, &portals);

    let floors = (0..num_floors)
    .map(|i| FloorStatus {
        floor_index: i,
         is_disabled: false,
         collapsed_area_ratio: 0.0,
    })
    .collect();

    let corridor_flow = vec![vec![vec![f32::INFINITY; cols]; rows]; num_floors];
    let naive_flow = vec![vec![vec![f32::INFINITY; cols]; rows]; num_floors];
    let optimal_flow = vec![vec![vec![f32::INFINITY; cols]; rows]; num_floors];

    let tile_temp = vec![vec![vec![AMBIENT_TEMP_C; cols]; rows]; num_floors];
    let tile_smoke = vec![vec![vec![0.0_f32; cols]; rows]; num_floors];
    let tile_co = vec![vec![vec![0.0_f32; cols]; rows]; num_floors];
    let tile_co2 = vec![vec![vec![0.04_f32; cols]; rows]; num_floors];
    let tile_fuel = vec![vec![vec![1.0_f32; cols]; rows]; num_floors];
    let tile_fire_time = vec![vec![vec![0.0_f32; cols]; rows]; num_floors];

    let mut corridor_waypoints = vec![Vec::new(); num_floors];
    for cell in &cells {
        if cell.kind == CellKind::Corridor {
            let step = (cell.tiles.len() / 6).max(1);
            for (idx, &(tr, tc)) in cell.tiles.iter().enumerate() {
                if idx % step == 0 && walkable_grid[cell.floor][tr][tc] {
                    corridor_waypoints[cell.floor].push((
                        (tc as f32 + 0.5) * TILE_SIZE_M,
                                                         (tr as f32 + 0.5) * TILE_SIZE_M,
                    ));
                }
            }
        }
    }

    let mut building = LoadedBuilding {
        cells,
        portals,
        alarms,
        extinguishers,
        floors,
        grid_width_m: (cols as f32) * TILE_SIZE_M,
        grid_height_m: (rows as f32) * TILE_SIZE_M,
        rows,
        cols,
        raw_floors,
        cell_grid,
        portal_at_tile,
        walkable_grid,
        tile_temp,
        tile_smoke,
        tile_co,
        tile_co2,
        tile_fuel,
        tile_fire_time,
        corridor_flow,
        naive_flow,
        optimal_flow,
        corridor_waypoints,
    };
    building.recompute_maze_flow();

    Ok(building)
}

pub fn propagate_shortest_and_alternative_paths(cells: &mut [Cell], portals: &[Portal]) {
    let num_cells = cells.len();
    let mut best_dist = vec![f32::INFINITY; num_cells];
    let mut best_path: Vec<Vec<(usize, usize)>> = vec![Vec::new(); num_cells];
    let mut alt_paths: Vec<Vec<(f32, Vec<(usize, usize)>)>> = vec![Vec::new(); num_cells];

    let mut queue: VecDeque<(usize, f32, Vec<(usize, usize)>)> = VecDeque::new();

    for cell in cells.iter() {
        if cell.kind == CellKind::Exit {
            best_dist[cell.id] = 0.0;
            queue.push_back((cell.id, 0.0, Vec::new()));
        }
    }

    while let Some((curr_id, curr_dist, path_so_far)) = queue.pop_front() {
        let (cx, cy) = cells[curr_id].center();
        for &pid in &cells[curr_id].portals {
            let portal = &portals[pid];
            if portal.is_closed_or_blocked {
                continue;
            }
            let Some(neighbor_id) = portal.other_cell(curr_id) else {
                continue;
            };
            if path_so_far
                .iter()
                .any(|&(_, visited_cell)| visited_cell == neighbor_id)
                || cells[neighbor_id].kind == CellKind::Exit
                {
                    continue;
                }
                let (nx, ny) = cells[neighbor_id].center();
            let step_dist = (cx - nx).hypot(cy - ny).max(3.0);
            let new_dist = curr_dist + step_dist;
            let mut new_path = Vec::with_capacity(path_so_far.len() + 1);
            new_path.push((pid, curr_id));
            new_path.extend_from_slice(&path_so_far);

            if new_dist < best_dist[neighbor_id] {
                if best_dist[neighbor_id].is_finite() && !best_path[neighbor_id].is_empty() {
                    alt_paths[neighbor_id]
                    .push((best_dist[neighbor_id], best_path[neighbor_id].clone()));
                }
                best_dist[neighbor_id] = new_dist;
                best_path[neighbor_id] = new_path.clone();
                queue.push_back((neighbor_id, new_dist, new_path));
            } else if alt_paths[neighbor_id].len() < 3 {
                alt_paths[neighbor_id].push((new_dist, new_path.clone()));
                alt_paths[neighbor_id].sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
                queue.push_back((neighbor_id, new_dist, new_path));
            }
        }
    }

    for i in 0..num_cells {
        cells[i].routes = RouteInfo {
            shortest_path: best_path[i].clone(),
            shortest_distance: best_dist[i],
            alternative_paths: alt_paths[i].clone(),
        };
    }
}
