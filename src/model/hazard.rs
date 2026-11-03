use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AlarmStation {
    pub id: usize,
    pub floor: usize,
    pub cell_id: usize,
    pub x: f32,
    pub y: f32,
    pub is_triggered: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExtinguisherItem {
    pub id: usize,
    pub floor: usize,
    pub cell_id: usize,
    pub x: f32,
    pub y: f32,
    pub is_available: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FloorStatus {
    pub floor_index: usize,
    pub is_disabled: bool,
    pub collapsed_area_ratio: f32,
}
