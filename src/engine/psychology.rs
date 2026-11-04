use crate::building_loader::LoadedBuilding;
use crate::config::{AMBIENT_TEMP_C, SimulationConfig};
use crate::model::agent::{Agent, AgentActionState, MacesRole};
use crate::model::topology::CellKind;
use rand::Rng;

pub fn step_psychology_and_physiology(
    building: &LoadedBuilding,
    agents: &mut [Agent],
    config: &SimulationConfig,
    global_alarm: bool,
    sim_time_s: f32,
    dt: f32,
) {
    let mut rng = rand::thread_rng();

    let mut active_panicked_in_cell = vec![0usize; building.cells.len()];
    let mut active_evac_in_cell = vec![0usize; building.cells.len()];

    for a in agents.iter() {
        let is_alerted = !matches!(
            a.action_state,
            AgentActionState::Normal
            | AgentActionState::Evacuated
            | AgentActionState::Incapacitated
        );
        if is_alerted {
            active_evac_in_cell[a.current_cell] += 1;
            if a.panic_level >= 5.0 {
                active_panicked_in_cell[a.current_cell] += 1;
            }
        }
    }

    for i in 0..agents.len() {
        if matches!(
            agents[i].action_state,
            AgentActionState::Evacuated | AgentActionState::Incapacitated
        ) {
            continue;
        }

        if agents[i].fall_immunity_s > 0.0 {
            agents[i].fall_immunity_s = (agents[i].fall_immunity_s - dt).max(0.0);
        }
        if agents[i].rescue_cooldown_s > 0.0 {
            agents[i].rescue_cooldown_s = (agents[i].rescue_cooldown_s - dt).max(0.0);
        }

        let cell_id = agents[i].current_cell;
        let cell = &building.cells[cell_id];
        let f = agents[i].floor;
        let (ar, ac) = building.tile_coords_at(agents[i].x, agents[i].y);

        // Exposición fisiológica exacta en la baldosa (1m x 1m) donde está parado el agente
        let local_temp = building.tile_temp[f][ar][ac];
        let local_co = building.tile_co[f][ar][ac];
        let local_co2 = building.tile_co2[f][ar][ac];
        let local_smoke = building.tile_smoke[f][ar][ac];

        agents[i].accumulate_fed(dt, local_temp, local_co, local_co2);
        if agents[i].action_state == AgentActionState::Incapacitated {
            continue;
        }

        // Detectar humo o calor en un radio visual de 3 baldosas dentro de su sala/pasillo
        let mut max_nearby_temp = local_temp;
        let mut max_nearby_smoke = local_smoke;
        for dr in -3..=3isize {
            for dc in -3..=3isize {
                let nr = ar as isize + dr;
                let nc = ac as isize + dc;
                if nr >= 0
                    && nr < building.rows as isize
                    && nc >= 0
                    && nc < building.cols as isize
                    {
                        let (ur, uc) = (nr as usize, nc as usize);
                        if building.cell_grid[f][ur][uc] == Some(cell_id) {
                            max_nearby_temp = max_nearby_temp.max(building.tile_temp[f][ur][uc]);
                            max_nearby_smoke = max_nearby_smoke.max(building.tile_smoke[f][ur][uc]);
                        }
                    }
            }
        }

        let sees_local_hazard = max_nearby_smoke > 0.08 || max_nearby_temp > AMBIENT_TEMP_C + 10.0;

        match agents[i].action_state {
            AgentActionState::Normal => {
                if global_alarm || sees_local_hazard || active_evac_in_cell[cell_id] > 0 {
                    let reaction_delay =
                    if agents[i].role == MacesRole::TrainedLeader && config.enable_wardens {
                        rng.gen_range(0.2..0.5)
                    } else {
                        rng.gen_range(0.7..2.4)
                        * (1.0 + agents[i].profile.cognitive_vulnerability())
                    };
                    agents[i].reaction_delay_assigned_s = reaction_delay;
                    agents[i].alert_start_time_s = Some(sim_time_s);
                    agents[i].action_state = AgentActionState::Reacting {
                        timer_s: reaction_delay,
                    };
                    if agents[i].is_panic_prone {
                        agents[i].panic_level = rng.gen_range(6.0..8.8);
                        agents[i].personal_space_xi = -0.05;
                    }
                }
            }
            AgentActionState::Reacting { ref mut timer_s } => {
                *timer_s -= dt;
                if *timer_s <= 0.0 {
                    if agents[i].role == MacesRole::TrainedLeader
                        && config.enable_wardens
                        && agents[i].floor > 0
                        {
                            agents[i].action_state =
                            AgentActionState::SweepingFloor { waypoint_idx: 0 };
                        } else if cell.kind == CellKind::Room {
                            agents[i].action_state = AgentActionState::ExitingRoomToCorridor;
                        } else {
                            agents[i].action_state = AgentActionState::Evacuating;
                        }
                }
            }
            AgentActionState::PanickedFrozen { ref mut timer_s } => {
                *timer_s -= dt;
                if !config.enable_panic_freeze || *timer_s <= 0.0 {
                    agents[i].action_state = if cell.kind == CellKind::Room {
                        AgentActionState::ExitingRoomToCorridor
                    } else {
                        AgentActionState::Evacuating
                    };
                }
            }
            AgentActionState::PanickedWandering {
                ref mut timer_s, ..
            } => {
                *timer_s -= dt;
                if !config.enable_panic_freeze || *timer_s <= 0.0 {
                    agents[i].action_state = if cell.kind == CellKind::Room {
                        AgentActionState::ExitingRoomToCorridor
                    } else {
                        AgentActionState::Evacuating
                    };
                }
            }
            _ => {}
        }

        if agents[i].action_state == AgentActionState::Normal {
            continue;
        }

        let density = cell.density();
        let mut panic_delta = 0.0_f32;
        if sees_local_hazard {
            panic_delta += 1.25 * dt;
        }
        if global_alarm {
            panic_delta += 0.18 * dt;
        }
        if (active_panicked_in_cell[cell_id] >= 2 || density > 2.6)
            && rng.gen_range(0.0..1.0) < config.panic_propagation_pct
            {
                panic_delta += 0.85 * dt;
            }
            if matches!(agents[i].action_state, AgentActionState::FollowingWarden { .. }) {
                panic_delta -= 1.8 * dt;
            }

            agents[i].panic_level = (agents[i].panic_level + panic_delta).clamp(0.0, 10.0);

        // Fallo cognitivo por pánico (solo si enable_panic_freeze está marcado en la UI)
        if config.enable_panic_freeze
            && (agents[i].role != MacesRole::TrainedLeader || !config.enable_wardens)
            && !agents[i].guided_by_warden
            && !matches!(
                agents[i].action_state,
                AgentActionState::FollowingWarden { .. }
                | AgentActionState::PanickedFrozen { .. }
                | AgentActionState::PanickedWandering { .. }
                | AgentActionState::HelpingFallen { .. }
            )
            && cell.kind != CellKind::Exit
            && cell.kind != CellKind::Stairwell
            && agents[i].panic_level >= 6.4
            {
                let vuln = agents[i].profile.cognitive_vulnerability();
                let breakdown_prob = vuln * ((agents[i].panic_level - 5.5) / 4.5) * 0.22 * dt;
                if rng.gen_range(0.0..1.0) < breakdown_prob {
                    if rng.gen_bool(0.45) {
                        agents[i].action_state = AgentActionState::PanickedFrozen {
                            timer_s: rng.gen_range(2.8..5.5),
                        };
                    } else {
                        agents[i].action_state = AgentActionState::PanickedWandering {
                            timer_s: rng.gen_range(2.2..4.2),
                            angle: rng.gen_range(0.0..std::f32::consts::TAU),
                        };
                    }
                    continue;
                }
            }

            // Altruismo: Ayudar a caídos (con Cooldown e Inmunidad post-caída para evitar bucles infinitos)
            let in_stair_or_exit = matches!(cell.kind, CellKind::Stairwell | CellKind::Exit);
            let room_safe_for_rescue = config.enable_helping_fallen
            && !in_stair_or_exit
            && cell.temperature_c < 135.0
            && cell.smoke_opacity < 0.45
            && agents[i].fed_dose < 0.35
            && density < 2.2;

            if let AgentActionState::HelpingFallen {
                target_agent_id,
                mut timer_s,
                mut attempt_timeout_s,
            } = agents[i].action_state
            {
                attempt_timeout_s -= dt;
                if !agents[target_agent_id].is_fallen
                    || agents[target_agent_id].action_state == AgentActionState::Incapacitated
                    || !room_safe_for_rescue
                    || attempt_timeout_s <= 0.0
                    {
                        agents[i].rescue_cooldown_s = 5.0; // No volver a detenerse inmediatamente si abortó
                        agents[i].action_state = AgentActionState::Evacuating;
                    } else {
                        let dist = (agents[i].x - agents[target_agent_id].x)
                        .hypot(agents[i].y - agents[target_agent_id].y);
                        if dist < 1.35 {
                            timer_s -= dt;
                            if timer_s <= 0.0 {
                                agents[target_agent_id].is_fallen = false;
                                agents[target_agent_id].fallen_timer_s = 0.0;
                                // Otorgar 6 segundos de inmunidad a caídas tanto al rescatado como al rescatista
                                agents[target_agent_id].fall_immunity_s = 6.0;
                                agents[i].fall_immunity_s = 6.0;
                                agents[i].rescue_cooldown_s = 6.5;
                                agents[i].helped_fallen_count += 1;
                                agents[i].action_state = AgentActionState::Evacuating;
                            } else {
                                agents[i].action_state = AgentActionState::HelpingFallen {
                                    target_agent_id,
                                    timer_s,
                                    attempt_timeout_s,
                                };
                            }
                        } else {
                            agents[i].action_state = AgentActionState::HelpingFallen {
                                target_agent_id,
                                timer_s,
                                attempt_timeout_s,
                            };
                        }
                    }
            } else if agents[i].action_state == AgentActionState::Evacuating
                && !agents[i].is_fallen
                && agents[i].rescue_cooldown_s <= 0.0
                && room_safe_for_rescue
                {
                    // Verificar que ningún otro agente esté ya ayudando a ese mismo caído (máximo 1 ayudante por caído)
                    let mut priority_fallen = None;
                    if config.enable_social_bonds {
                        for &bond_id in &agents[i].social_bonds {
                            if agents[bond_id].current_cell == cell_id
                                && agents[bond_id].is_fallen
                                && agents[bond_id].action_state != AgentActionState::Incapacitated
                                {
                                    let already_helped = agents.iter().any(|other| {
                                        matches!(
                                            other.action_state,
                                            AgentActionState::HelpingFallen { target_agent_id, .. }
                                            if target_agent_id == bond_id
                                        )
                                    });
                                    if !already_helped {
                                        priority_fallen = Some(bond_id);
                                        break;
                                    }
                                }
                        }
                    }

                    if priority_fallen.is_none()
                        && agents[i].profile.can_use_extinguisher()
                        && agents[i].panic_level < 5.2
                        && density < 1.6
                        {
                            for &occ_id in &cell.occupant_ids {
                                if occ_id != i
                                    && agents[occ_id].is_fallen
                                    && agents[occ_id].action_state != AgentActionState::Incapacitated
                                    && (agents[i].x - agents[occ_id].x).hypot(agents[i].y - agents[occ_id].y)
                                    < 1.9
                                    {
                                        let already_helped = agents.iter().any(|other| {
                                            matches!(
                                                other.action_state,
                                                AgentActionState::HelpingFallen { target_agent_id, .. }
                                                if target_agent_id == occ_id
                                            )
                                        });
                                        if !already_helped {
                                            priority_fallen = Some(occ_id);
                                            break;
                                        }
                                    }
                            }
                        }

                        if let Some(target_id) = priority_fallen {
                            agents[i].action_state = AgentActionState::HelpingFallen {
                                target_agent_id: target_id,
                                timer_s: 0.55,
                                attempt_timeout_s: 1.8,
                            };
                        }
                }
    }
}
