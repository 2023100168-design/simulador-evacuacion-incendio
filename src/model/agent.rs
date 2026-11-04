use crate::config::{SFPE_DENSITY_A, SFPE_FREE_FLOW_DENSITY};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DemographicProfile {
    Adult,
    YoungAdult,
    Child,
    Elderly,
    Impaired,
}

impl DemographicProfile {
    pub fn name_es(&self) -> &'static str {
        match self {
            Self::Adult => "Adulto",
            Self::YoungAdult => "Joven Adulto",
            Self::Child => "Niño",
            Self::Elderly => "Adulto Mayor",
            Self::Impaired => "Movilidad Reducida",
        }
    }

    pub fn base_flat_speed(&self) -> f32 {
        match self {
            Self::Adult => 1.64,
            Self::YoungAdult => 1.61,
            Self::Child => 1.30,
            Self::Elderly => 1.12,
            Self::Impaired => 0.88,
        }
    }

    pub fn base_stair_speed(&self) -> f32 {
        match self {
            Self::Adult | Self::YoungAdult => 0.71,
            Self::Child => 0.52,
            Self::Elderly => 0.42,
            Self::Impaired => 0.28,
        }
    }

    pub fn cognitive_vulnerability(&self) -> f32 {
        match self {
            Self::Adult => 0.15,
            Self::YoungAdult => 0.18,
            Self::Child => 0.60,
            Self::Elderly => 0.45,
            Self::Impaired => 0.40,
        }
    }

    pub fn calculate_speed(&self, density: f32, on_stairs: bool, in_panic: bool) -> f32 {
        let d_clamped = density.clamp(0.0, 3.8);
        let speed = if on_stairs {
            let v0_esc = self.base_stair_speed();
            let bottleneck_factor = if d_clamped > 2.2 { 0.60 } else { 1.0 };
            (v0_esc * (1.0 - SFPE_DENSITY_A * d_clamped) * bottleneck_factor).max(0.14)
        } else {
            let v0 = self.base_flat_speed();
            if d_clamped <= SFPE_FREE_FLOW_DENSITY {
                v0
            } else {
                let k = v0 / 0.85;
                (k * (1.0 - SFPE_DENSITY_A * d_clamped)).max(0.18)
            }
        };

        if in_panic && !on_stairs {
            speed * 1.22
        } else {
            speed
        }
    }

    pub fn body_radius(&self) -> f32 {
        match self {
            Self::Child => 0.16,
            _ => 0.20,
        }
    }

    pub fn base_minute_volume_l_min(&self) -> f32 {
        match self {
            Self::Adult | Self::YoungAdult => 25.0,
            Self::Child => 18.0,
            Self::Elderly | Self::Impaired => 20.0,
        }
    }

    pub fn critical_fed_threshold(&self) -> f32 {
        match self {
            Self::Adult | Self::YoungAdult => 1.00,
            Self::Child | Self::Elderly | Self::Impaired => 0.30,
        }
    }

    pub fn can_use_extinguisher(&self) -> bool {
        matches!(self, Self::Adult | Self::YoungAdult)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MacesRole {
    TrainedLeader,
    UntrainedLeader,
    Follower,
}

impl MacesRole {
    pub fn name_es(&self) -> &'static str {
        match self {
            Self::TrainedLeader => "Brigadista Entrenado",
            Self::UntrainedLeader => "Líder Natural",
            Self::Follower => "Seguidor",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CellMemory {
    Unvisited,
    Visited,
    Finished,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortalCrossingState {
    NotCrossing,
    ApproachingA,
    CloseToB,
    LeavingB,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AgentActionState {
    Normal,
    Reacting { timer_s: f32 },
    TriggeringAlarm { station_id: usize },
    FetchingExtinguisher { ext_id: usize, fire_cell: usize },
    FightingFire { fire_cell: usize, remaining_charge_s: f32 },
    ExitingRoomToCorridor,
    SeekingFamilyInCorridor {
        patrol_waypoint_idx: usize,
        search_timer_s: f32,
    },
    PanickedFrozen { timer_s: f32 },
    PanickedWandering { timer_s: f32, angle: f32 },
    SweepingFloor { waypoint_idx: usize },
    EscortingToStairs { stair_cell_id: usize },
    FollowingWarden { warden_id: usize, slot_idx: u8 },
    HelpingFallen {
        target_agent_id: usize,
        timer_s: f32,
        attempt_timeout_s: f32,
    },
    Evacuating,
    Evacuated,
    Incapacitated,
}

impl AgentActionState {
    pub fn label_es(&self) -> &'static str {
        match self {
            Self::Normal => "Rutina Normal",
            Self::Reacting { .. } => "Reaccionando a Alerta",
            Self::TriggeringAlarm { .. } => "Activando Alarma",
            Self::FetchingExtinguisher { .. } => "Buscando Extintor",
            Self::FightingFire { .. } => "Combatiendo Fuego",
            Self::ExitingRoomToCorridor => "Meta 1: Saliendo al Pasillo",
            Self::SeekingFamilyInCorridor { .. } => "Meta 2: Buscando Familia",
            Self::PanickedFrozen { .. } => "En Shock (Paralizado)",
            Self::PanickedWandering { .. } => "Desorientado por Pánico",
            Self::SweepingFloor { .. } => "Brigadista: Buscando Rezagados",
            Self::EscortingToStairs { .. } => "Brigadista: Escoltando (Máx 2)",
            Self::FollowingWarden { .. } => "En Fila tras Brigadista",
            Self::HelpingFallen { .. } => "Levantando a Caído",
            Self::Evacuating => "Meta 3: Evacuando a Salida",
            Self::Evacuated => "Evacuado (A Salvo)",
            Self::Incapacitated => "Incapacitado (FED Crítico)",
        }
    }
}

#[derive(Clone, Debug)]
pub struct Agent {
    pub id: usize,
    pub profile: DemographicProfile,
    pub role: MacesRole,
    pub action_state: AgentActionState,
    pub crossing_state: PortalCrossingState,
    pub floor: usize,
    pub current_cell: usize,
    pub target_portal_id: Option<usize>,
    pub next_cell_id: Option<usize>,
    pub x: f32,
    pub y: f32,
    pub vx: f32,
    pub vy: f32,
    pub is_panic_prone: bool,
    pub panic_level: f32,
    pub impatience: f32,
    pub personal_space_xi: f32,
    pub prefers_right: bool,
    pub can_fall_easily: bool,
    pub is_fallen: bool,
    pub fallen_timer_s: f32,
    pub fall_immunity_s: f32,    // Evita que vuelva a caer inmediatamente al levantarse
    pub rescue_cooldown_s: f32,  // Evita que un rescatista quede atrapado en bucle ayudando caídos
    pub stuck_timer_s: f32,
    pub unstuck_waypoint: Option<(f32, f32, f32)>,
    pub stop_shaking_ticks: i32,
    pub fed_dose: f32,
    pub group_id: Option<usize>,
    pub group_leader_id: Option<usize>,
    pub social_bonds: Vec<usize>,
    pub family_reunited: bool,
    pub guided_by_warden: bool,
    pub escort_slots: Vec<usize>, // Máximo 2 personas escoltadas simultáneamente por el brigadista
    pub stair_lane: u8,
    pub stair_progress: f32,
    pub reaction_delay_assigned_s: f32,
    pub alert_start_time_s: Option<f32>,
    pub distance_traveled_m: f32,
    pub active_evac_time_s: f32,
    pub rescued_count: usize,
    pub helped_fallen_count: usize,
    pub swept_cells: Vec<usize>,
    pub mental_map: HashMap<usize, CellMemory>,
    pub dfs_stack: Vec<usize>,
}

impl Agent {
    pub fn avg_speed_m_s(&self) -> f32 {
        if self.active_evac_time_s > 0.1 {
            self.distance_traveled_m / self.active_evac_time_s
        } else {
            self.vx.hypot(self.vy)
        }
    }

    pub fn accumulate_fed(&mut self, dt_s: f32, temp_c: f32, co_ppm: f32, co2_pct: f32) {
        if matches!(
            self.action_state,
            AgentActionState::Evacuated | AgentActionState::Incapacitated
        ) {
            return;
        }

        let dt_min = dt_s / 60.0;
        let delta_fed_heat = if temp_c > 60.0 {
            let t_iconv_min = 5e7_f32 * temp_c.powf(-3.4);
            if t_iconv_min > 1e-5 {
                dt_min / t_iconv_min
            } else {
                1.0
            }
        } else {
            0.0
        };

        let panic_vent_mult = if self.panic_level >= 5.0 { 2.0 } else { 1.0 };
        let ve_l_min = self.profile.base_minute_volume_l_min() * panic_vent_mult;
        let f_co2 = (co2_pct.clamp(0.0, 15.0) / 5.0).exp();
        let delta_fed_smoke = ((co_ppm * dt_min) / 35_000.0) * (ve_l_min / 25.0) * f_co2;

        self.fed_dose += delta_fed_heat + delta_fed_smoke;

        if self.fed_dose >= self.profile.critical_fed_threshold() {
            self.action_state = AgentActionState::Incapacitated;
            self.is_fallen = true;
            self.vx = 0.0;
            self.vy = 0.0;
        }
    }
}
