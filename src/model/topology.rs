use crate::config::BOUNDARY_LAYER_DOOR;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CellKind {
    Room,
    Corridor,
    Stairwell,
    Elevator { shaft_id: usize },
    Exit,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct WallSegment {
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
    pub nx: f32,
    pub ny: f32,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Obstacle {
    pub x: f32,
    pub y: f32,
    pub radius: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RouteInfo {
    pub shortest_path: Vec<(usize, usize)>,
    pub shortest_distance: f32,
    pub alternative_paths: Vec<(f32, Vec<(usize, usize)>)>,
}

#[derive(Clone, Debug)]
pub struct SpatialGrid {
    pub cell_size: f32,
    pub buckets: HashMap<(i32, i32), Vec<usize>>,
}

impl SpatialGrid {
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            buckets: HashMap::new(),
        }
    }

    pub fn clear(&mut self) {
        for vec in self.buckets.values_mut() {
            vec.clear();
        }
    }

    pub fn insert(&mut self, x: f32, y: f32, agent_id: usize) {
        let gx = (x / self.cell_size).floor() as i32;
        let gy = (y / self.cell_size).floor() as i32;
        self.buckets.entry((gx, gy)).or_default().push(agent_id);
    }

    pub fn query_nearby(&self, x: f32, y: f32, out: &mut Vec<usize>) {
        out.clear();
        let gx = (x / self.cell_size).floor() as i32;
        let gy = (y / self.cell_size).floor() as i32;
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(agents) = self.buckets.get(&(gx + dx, gy + dy)) {
                    out.extend_from_slice(agents);
                }
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct Cell {
    pub id: usize,
    pub floor: usize,
    pub name: String,
    pub kind: CellKind,
    pub min_x: f32,
    pub min_y: f32,
    pub max_x: f32,
    pub max_y: f32,
    /// Punto interior garantizado dentro de una baldosa caminable de la celda (nunca dentro de un muro en celdas en L o U)
    pub rep_point: (f32, f32),
    /// Lista exacta de baldosas (fila, columna) que componen esta celda
    pub tiles: Vec<(usize, usize)>,
    pub area_m2: f32,
    pub walls: Vec<WallSegment>,
    pub obstacles: Vec<Obstacle>,
    pub portals: Vec<usize>,
    pub temperature_c: f32,
    pub fire_time_s: f32,
    pub co_ppm: f32,
    pub co2_pct: f32,
    pub smoke_opacity: f32,
    pub fuel: f32,
    pub is_collapsed: bool,
    pub routes: RouteInfo,
    pub occupant_ids: Vec<usize>,
    pub spatial_grid: SpatialGrid,
}

impl Cell {
    pub fn center(&self) -> (f32, f32) {
        self.rep_point
    }

    pub fn density(&self) -> f32 {
        (self.occupant_ids.len() as f32) / self.area_m2.max(1.0)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Portal {
    pub id: usize,
    pub cell_a: usize,
    pub cell_b: usize,
    pub floor_a: usize,
    pub floor_b: usize,
    pub tile_rc: (usize, usize),
    pub pos_a: (f32, f32),
    pub pos_b: (f32, f32),
    pub attractor_a: (f32, f32),
    pub attractor_b: (f32, f32),
    pub width_m: f32,
    pub fire_resistance_s: f32,
    pub max_resistance_s: f32,
    pub is_closed_or_blocked: bool,
    pub crossing_agents: Vec<usize>,
}

impl Portal {
    pub fn effective_width(&self) -> f32 {
        (self.width_m - 2.0 * BOUNDARY_LAYER_DOOR).max(0.35)
    }

    pub fn other_cell(&self, current_cell: usize) -> Option<usize> {
        if self.cell_a == current_cell {
            Some(self.cell_b)
        } else if self.cell_b == current_cell {
            Some(self.cell_a)
        } else {
            None
        }
    }
}
