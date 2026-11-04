use serde::{Deserialize, Serialize};

pub const SFPE_DENSITY_A: f32 = 0.266;
pub const SFPE_FREE_FLOW_DENSITY: f32 = 0.55;
pub const BOUNDARY_LAYER_DOOR: f32 = 0.15;
pub const BOUNDARY_LAYER_WALL: f32 = 0.20;
pub const ATTRACTOR_OFFSET_M: f32 = 0.70;
pub const ELEVATOR_MAX_CAPACITY: usize = 8;

pub const AMBIENT_TEMP_C: f32 = 22.0;
pub const FLASHOVER_TEMP_C: f32 = 500.0;
pub const COLLAPSE_TEMP_C: f32 = 850.0;
pub const EXTINGUISHER_MAX_TEMP_C: f32 = 250.0;
pub const EXTINGUISHER_COOLING_RATE_C_PER_S: f32 = 45.0;
pub const EXTINGUISHER_DURATION_S: f32 = 18.0;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SimulationConfig {
    pub population_size: usize,
    pub leadership_pct: f32,
    pub trained_pct: f32,
    // Interruptores modulares de comportamiento para experimentación en la UI
    pub enable_communication: bool,
    pub enable_social_bonds: bool,
    pub enable_wardens: bool,
    pub enable_warden_line_formation: bool, // Ponerse en fila detrás del brigadista (opuesto a su POV)
    pub enable_helping_fallen: bool,
    pub enable_panic_freeze: bool,
    pub initial_panic_pct: f32,
    pub panic_propagation_pct: f32,
    pub impatience_pct: f32,
    pub falling_prob_pct: f32,
    pub right_preference_pct: f32,
    pub avoidance_weight: f32,
    pub fire_growth_alpha: f32,
    pub smoke_yield_rate: f32,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            population_size: 180,
            leadership_pct: 0.35,
            trained_pct: 0.12,
            enable_communication: true,
            enable_social_bonds: true,
            enable_wardens: true,
            enable_warden_line_formation: true,
            enable_helping_fallen: true,
            enable_panic_freeze: true,
            initial_panic_pct: 0.15,
            panic_propagation_pct: 0.50,
            impatience_pct: 0.40,
            falling_prob_pct: 0.15,
            right_preference_pct: 0.80,
            avoidance_weight: 0.50,
            fire_growth_alpha: 0.0469,
            smoke_yield_rate: 1.0,
        }
    }
}
