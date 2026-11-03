pub mod hidac_physics;
pub mod maces_wayfinding;
pub mod psychology;
pub mod thermodynamics;

use crate::building_loader::{
    LoadedBuilding, propagate_shortest_and_alternative_paths, TILE_SIZE_M,
};
use crate::config::SimulationConfig;
use crate::model::agent::{
    Agent, AgentActionState, CellMemory, DemographicProfile, MacesRole, PortalCrossingState,
};
use crate::model::topology::CellKind;
use rand::Rng;
use std::collections::HashMap;

#[derive(Clone, Copy, Debug)]
pub struct TelemetryPoint {
    pub time_s: f32,
    pub evacuated_pct: f32,
    pub incapacitated_pct: f32,
    pub alerting_pct: f32,
}

pub struct SimulationEngine {
    pub building: LoadedBuilding,
    pub agents: Vec<Agent>,
    pub config: SimulationConfig,
    pub is_running: bool,
    pub global_alarm: bool,
    pub time_elapsed_s: f32,
    pub selected_agent_id: Option<usize>,
    pub telemetry_history: Vec<TelemetryPoint>,
    pub last_sample_time_s: f32,
    pub last_maze_update_s: f32,
    pub aset_time_s: Option<f32>,
    pub rset_time_s: Option<f32>,
}

impl SimulationEngine {
    pub fn new(mut building: LoadedBuilding, config: SimulationConfig) -> Self {
        let agents = spawn_population(&building, &config);
        hidac_physics::rebuild_spatial_index(&mut building, &agents);
        building.recompute_maze_flow();
        Self {
            building,
            agents,
            config,
            is_running: false,
            global_alarm: false,
            time_elapsed_s: 0.0,
            selected_agent_id: None,
            telemetry_history: vec![TelemetryPoint {
                time_s: 0.0,
                evacuated_pct: 0.0,
                incapacitated_pct: 0.0,
                alerting_pct: 0.0,
            }],
            last_sample_time_s: 0.0,
            last_maze_update_s: 0.0,
            aset_time_s: None,
            rset_time_s: None,
        }
    }

    pub fn reset(&mut self, fresh_building: LoadedBuilding) {
        self.building = fresh_building;
        self.global_alarm = false;
        self.agents = spawn_population(&self.building, &self.config);
        self.time_elapsed_s = 0.0;
        self.selected_agent_id = None;
        self.telemetry_history.clear();
        self.telemetry_history.push(TelemetryPoint {
            time_s: 0.0,
            evacuated_pct: 0.0,
            incapacitated_pct: 0.0,
            alerting_pct: 0.0,
        });
        self.last_sample_time_s = 0.0;
        self.last_maze_update_s = 0.0;
        self.aset_time_s = None;
        self.rset_time_s = None;
        hidac_physics::rebuild_spatial_index(&mut self.building, &self.agents);
        self.building.recompute_maze_flow();
    }

    /// Permite ciclar entre los Brigadistas con el botón de la interfaz
    pub fn select_next_warden(&mut self) {
        let wardens: Vec<usize> = self
        .agents
        .iter()
        .filter(|a| a.role == MacesRole::TrainedLeader)
        .map(|a| a.id)
        .collect();
        if wardens.is_empty() {
            return;
        }
        if let Some(curr) = self.selected_agent_id {
            if let Some(pos) = wardens.iter().position(|&id| id == curr) {
                self.selected_agent_id = Some(wardens[(pos + 1) % wardens.len()]);
                return;
            }
        }
        self.selected_agent_id = Some(wardens[0]);
    }

    pub fn format_inspector_header(&self) -> String {
        if let Some(aid) = self.selected_agent_id {
            if let Some(a) = self.agents.get(aid) {
                let grp = match (a.group_id, a.group_leader_id) {
                    (Some(g), Some(l)) if l == a.id => format!("Familia #{} (Jefe)", g),
                    (Some(g), Some(l)) => format!("Familia #{} (Líder #{})", g, l),
                    _ => "Sin grupo".to_string(),
                };
                return format!(
                    "AGENTE #{:03} — {} ({})\nEstado: {} | Piso {} | {}\nT. Resp: {:.2}s | Vel: {:.2} m/s | Dist: {:.1}m | Rescates: {}",
                               a.id,
                               a.profile.name_es(),
                               a.role.name_es(),
                               a.action_state.label_es(),
                               a.floor,
                               grp,
                               a.reaction_delay_assigned_s,
                               a.avg_speed_m_s(),
                               a.distance_traveled_m,
                               a.rescued_count + a.helped_fallen_count
                );
            }
        }

        let wardens: Vec<&Agent> = self
        .agents
        .iter()
        .filter(|a| a.role == MacesRole::TrainedLeader)
        .collect();
        let total_rescued: usize = wardens.iter().map(|w| w.rescued_count).sum();
        let total_helped: usize = self.agents.iter().map(|a| a.helped_fallen_count).sum();
        let fam_count = self
        .agents
        .iter()
        .filter_map(|a| a.group_id)
        .max()
        .unwrap_or(0);

        format!(
            "VISTA GLOBAL DE FLOTA (Clic en un agente o 'Sig. Brigadista')\nBrigadistas: {} | Grupos Familiares: {}\nGuiados por Brigada: {} | Caídos Levantados: {}",
                wardens.len(),
                fam_count,
                total_rescued,
                total_helped
        )
    }

    pub fn apply_preset(&mut self, preset_idx: i32, fresh_building: LoadedBuilding) -> String {
        match preset_idx {
            0 => {
                self.config.enable_communication = true;
                self.config.enable_social_bonds = true;
                self.config.trained_pct = 0.15;
                self.config.leadership_pct = 0.45;
                self.config.initial_panic_pct = 0.15;
                self.reset(fresh_building);
                if let Some(c) = self
                    .building
                    .cells
                    .iter_mut()
                    .find(|c| c.floor == 1 && c.kind == CellKind::Room)
                    {
                        c.temperature_c = 520.0;
                        c.smoke_opacity = 0.80;
                    }
                    self.building.recompute_maze_flow();
                "Preset 1: Laberinto Dinámico + Vínculos Familiares y Comunicación Activa".into()
            }
            1 => {
                self.config.enable_communication = false;
                self.config.enable_social_bonds = false;
                self.config.trained_pct = 0.0;
                self.config.leadership_pct = 0.45;
                self.config.initial_panic_pct = 0.25;
                self.reset(fresh_building);
                if let Some(c) = self
                    .building
                    .cells
                    .iter_mut()
                    .find(|c| c.floor == 1 && c.kind == CellKind::Room)
                    {
                        c.temperature_c = 520.0;
                        c.smoke_opacity = 0.80;
                    }
                    self.building.recompute_maze_flow();
                "Preset 2: Evacuación Individualista (Sin Vínculos ni Brigadistas)".into()
            }
            2 => {
                self.config.enable_communication = true;
                self.config.enable_social_bonds = true;
                self.config.leadership_pct = 0.50;
                self.config.trained_pct = 0.22;
                self.config.initial_panic_pct = 0.10;
                self.reset(fresh_building);
                if let Some(c) = self
                    .building
                    .cells
                    .iter_mut()
                    .find(|c| c.floor == 2 && c.kind == CellKind::Room)
                    {
                        c.temperature_c = 175.0;
                        c.smoke_opacity = 0.35;
                    }
                    self.building.recompute_maze_flow();
                "Preset 3: 10% Brigadistas + Extintor en Conato de Piso 2".into()
            }
            _ => {
                self.config.enable_communication = true;
                self.config.enable_social_bonds = true;
                self.config.leadership_pct = 0.20;
                self.config.trained_pct = 0.05;
                self.config.initial_panic_pct = 0.80;
                self.reset(fresh_building);
                for c in self
                    .building
                    .cells
                    .iter_mut()
                    .filter(|c| c.floor > 0 && c.kind == CellKind::Room)
                    .take(2)
                    {
                        c.temperature_c = 560.0;
                        c.smoke_opacity = 0.85;
                    }
                    self.building.recompute_maze_flow();
                "Preset 4: Pánico Alto (80%) y Múltiples Focos".into()
            }
        }
    }

    pub fn step(&mut self, dt: f32) {
        if !self.is_running {
            return;
        }
        self.time_elapsed_s += dt;

        thermodynamics::step_thermodynamics(
            &mut self.building,
            &mut self.agents,
            &self.config,
            &mut self.global_alarm,
            dt,
        );

        // Actualizar el mapa del laberinto cada 0.25s para desviar alrededor del fuego y balancear puertas
        if self.time_elapsed_s - self.last_maze_update_s >= 0.25 {
            self.last_maze_update_s = self.time_elapsed_s;
            let LoadedBuilding {
                ref mut cells,
                ref portals,
                ..
            } = self.building;
            propagate_shortest_and_alternative_paths(cells, portals);
            self.building.recompute_maze_flow();
        }

        psychology::step_psychology_and_physiology(
            &self.building,
            &mut self.agents,
            &self.config,
            self.global_alarm,
            self.time_elapsed_s,
            dt,
        );

        maces_wayfinding::step_maces_wayfinding(
            &mut self.building,
            &mut self.agents,
            &self.config,
            &mut self.global_alarm,
            dt,
        );

        hidac_physics::step_hidac_physics(
            &mut self.building,
            &mut self.agents,
            &self.config,
            self.global_alarm,
            dt,
        );

        let incap = self.count_incapacitated();
        let evac = self.count_evacuated();
        let total = self.agents.len().max(1);

        if incap > 0 && self.aset_time_s.is_none() {
            self.aset_time_s = Some(self.time_elapsed_s);
        }
        if evac + incap == total && evac > 0 && self.rset_time_s.is_none() {
            self.rset_time_s = Some(self.time_elapsed_s);
        }

        if self.time_elapsed_s - self.last_sample_time_s >= 0.4 {
            self.last_sample_time_s = self.time_elapsed_s;
            let alerting = self.count_alerted();
            self.telemetry_history.push(TelemetryPoint {
                time_s: self.time_elapsed_s,
                evacuated_pct: (evac as f32 / total as f32) * 100.0,
                                        incapacitated_pct: (incap as f32 / total as f32) * 100.0,
                                        alerting_pct: (alerting as f32 / total as f32) * 100.0,
            });
        }
    }

    pub fn count_evacuated(&self) -> usize {
        self.agents
        .iter()
        .filter(|a| a.action_state == AgentActionState::Evacuated)
        .count()
    }

    pub fn count_fallen(&self) -> usize {
        self.agents
        .iter()
        .filter(|a| a.is_fallen && a.action_state != AgentActionState::Incapacitated)
        .count()
    }

    pub fn count_incapacitated(&self) -> usize {
        self.agents
        .iter()
        .filter(|a| a.action_state == AgentActionState::Incapacitated)
        .count()
    }

    pub fn count_alerted(&self) -> usize {
        self.agents
        .iter()
        .filter(|a| {
            !matches!(
                a.action_state,
                AgentActionState::Normal
                | AgentActionState::Evacuated
                | AgentActionState::Incapacitated
            )
        })
        .count()
    }
}

pub fn spawn_population(building: &LoadedBuilding, config: &SimulationConfig) -> Vec<Agent> {
    let mut rng = rand::thread_rng();

    let spawnable_cells: Vec<(usize, f32)> = building
    .cells
    .iter()
    .filter(|c| matches!(c.kind, CellKind::Room | CellKind::Corridor))
    .map(|c| (c.id, c.area_m2))
    .collect();

    if spawnable_cells.is_empty() {
        return Vec::new();
    }

    let total_spawn_area: f32 = spawnable_cells.iter().map(|(_, a)| *a).sum();
    let mut agents = Vec::with_capacity(config.population_size);

    for id in 0..config.population_size {
        let mut pick = rng.gen_range(0.0..total_spawn_area);
        let mut cell_id = spawnable_cells[0].0;
        for &(cid, area) in &spawnable_cells {
            if pick <= area {
                cell_id = cid;
                break;
            }
            pick -= area;
        }

        let cell = &building.cells[cell_id];
        // Spawnear exactamente sobre una baldosa caminable válida de la celda (soporta L y U)
        let valid_tiles: Vec<(usize, usize)> = cell
        .tiles
        .iter()
        .copied()
        .filter(|&(r, c)| building.walkable_grid[cell.floor][r][c])
        .collect();
        let &(tr, tc) = if !valid_tiles.is_empty() {
            &valid_tiles[rng.gen_range(0..valid_tiles.len())]
        } else {
            &cell.tiles[0]
        };
        let x = (tc as f32 + rng.gen_range(0.28..0.72)) * TILE_SIZE_M;
        let y = (tr as f32 + rng.gen_range(0.28..0.72)) * TILE_SIZE_M;

        let roll: f32 = rng.gen_range(0.0..1.0);
        let profile = if roll < 0.45 {
            DemographicProfile::Adult
        } else if roll < 0.70 {
            DemographicProfile::YoungAdult
        } else if roll < 0.82 {
            DemographicProfile::Child
        } else if roll < 0.94 {
            DemographicProfile::Elderly
        } else {
            DemographicProfile::Impaired
        };

        let role = if rng.gen_range(0.0..1.0) < config.leadership_pct {
            if rng.gen_range(0.0..1.0) < config.trained_pct {
                MacesRole::TrainedLeader
            } else {
                MacesRole::UntrainedLeader
            }
        } else {
            MacesRole::Follower
        };

        let is_panic_prone = rng.gen_range(0.0..1.0) < config.initial_panic_pct;
        let xi = if rng.gen_bool(0.5) { 0.0 } else { 0.04 };

        let can_fall_easily = matches!(
            profile,
            DemographicProfile::Elderly | DemographicProfile::Impaired
        ) || rng.gen_range(0.0..1.0) < config.falling_prob_pct;

        let mut mental_map = HashMap::new();
        mental_map.insert(cell_id, CellMemory::Visited);

        let impatience = if rng.gen_range(0.0..1.0) < config.impatience_pct {
            0.8
        } else {
            0.2
        };

        let angle: f32 = rng.gen_range(0.0..std::f32::consts::TAU);

        agents.push(Agent {
            id,
            profile,
            role,
            action_state: AgentActionState::Normal,
            crossing_state: PortalCrossingState::NotCrossing,
            floor: cell.floor,
            current_cell: cell_id,
            target_portal_id: None,
            next_cell_id: None,
            x,
            y,
            vx: angle.cos() * 0.35,
                    vy: angle.sin() * 0.35,
                    is_panic_prone,
                    panic_level: rng.gen_range(0.0..1.5),
                    impatience,
                    personal_space_xi: xi,
                    prefers_right: rng.gen_range(0.0..1.0) < config.right_preference_pct,
                    can_fall_easily,
                    is_fallen: false,
                    fallen_timer_s: 0.0,
                    stuck_timer_s: 0.0,
                    unstuck_waypoint: None,
                    stop_shaking_ticks: -1,
                    fed_dose: 0.0,
                    group_id: None,
                    group_leader_id: None,
                    social_bonds: Vec::new(),
                    family_reunited: false, guided_by_warden: false,
                    stair_lane: (id % 2) as u8,
                    stair_progress: 0.0,
                    reaction_delay_assigned_s: 0.0,
                    alert_start_time_s: None,
                    distance_traveled_m: 0.0,
                    active_evac_time_s: 0.0,
                    rescued_count: 0,
                    helped_fallen_count: 0,
                    swept_cells: vec![cell_id],
                    mental_map,
                    dfs_stack: vec![cell_id],
        });
    }

    let mut next_group_id = 1usize;
    for f in 0..building.floors.len() {
        let floor_agents: Vec<usize> = agents
        .iter()
        .filter(|a| a.floor == f && a.role != MacesRole::TrainedLeader)
        .map(|a| a.id)
        .collect();

        let mut idx = 0;
        while idx < floor_agents.len() {
            let group_size = rng.gen_range(2..=4).min(floor_agents.len() - idx);
            if group_size >= 2 && rng.gen_bool(0.75) {
                let slice = &floor_agents[idx..(idx + group_size)];
                let leader_id = slice
                .iter()
                .copied()
                .find(|&aid| {
                    matches!(
                        agents[aid].profile,
                        DemographicProfile::Adult | DemographicProfile::YoungAdult
                    )
                })
                .unwrap_or(slice[0]);

                let gid = next_group_id;
                next_group_id += 1;

                for &a_id in slice {
                    agents[a_id].group_id = Some(gid);
                    agents[a_id].group_leader_id = Some(leader_id);
                    for &b_id in slice {
                        if a_id != b_id && agents[a_id].social_bonds.len() < 4 {
                            agents[a_id].social_bonds.push(b_id);
                        }
                    }
                }
            }
            idx += group_size.max(1);
        }
    }

    agents
}
