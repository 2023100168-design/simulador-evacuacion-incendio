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

        let cell_id = agents[i].current_cell;
        let cell = &building.cells[cell_id];

        agents[i].accumulate_fed(dt, cell.temperature_c, cell.co_ppm, cell.co2_pct);
        if agents[i].action_state == AgentActionState::Incapacitated {
            continue;
        }

        let sees_local_hazard = cell.smoke_opacity > 0.10
        || cell.temperature_c > AMBIENT_TEMP_C + 12.0
        || cell.portals.iter().any(|&pid| {
            if let Some(other_id) = building.portals[pid].other_cell(cell.id) {
                building.cells[other_id].temperature_c > 110.0
                || building.cells[other_id].smoke_opacity > 0.25
            } else {
                false
            }
        });

        match agents[i].action_state {
            AgentActionState::Normal => {
                if global_alarm || sees_local_hazard || active_evac_in_cell[cell_id] > 0 {
                    let reaction_delay = if agents[i].role == MacesRole::TrainedLeader {
                        rng.gen_range(0.2..0.5)
                    } else {
                        rng.gen_range(0.8..2.6) * (1.0 + agents[i].profile.cognitive_vulnerability())
                    };
                    agents[i].reaction_delay_assigned_s = reaction_delay;
                    agents[i].alert_start_time_s = Some(sim_time_s);
                    agents[i].action_state = AgentActionState::Reacting {
                        timer_s: reaction_delay,
                    };
                    if agents[i].is_panic_prone {
                        agents[i].panic_level = rng.gen_range(6.2..9.2);
                        agents[i].personal_space_xi = -0.05;
                    }
                }
            }
            AgentActionState::Reacting { ref mut timer_s } => {
                *timer_s -= dt;
                if *timer_s <= 0.0 {
                    // Al terminar de reaccionar: los brigadistas patrullan su piso; los civiles primero buscan salir al pasillo (Meta 1)
                    if agents[i].role == MacesRole::TrainedLeader && agents[i].floor > 0 {
                        agents[i].action_state = AgentActionState::SweepingFloor { waypoint_idx: 0 };
                    } else if cell.kind == CellKind::Room {
                        agents[i].action_state = AgentActionState::ExitingRoomToCorridor;
                    } else {
                        agents[i].action_state = AgentActionState::Evacuating;
                    }
                }
            }
            AgentActionState::PanickedFrozen { ref mut timer_s } => {
                *timer_s -= dt;
                if *timer_s <= 0.0 {
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
                if *timer_s <= 0.0 {
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

        // Propagación del pánico
        let density = cell.density();
        let mut panic_delta = 0.0_f32;
        if sees_local_hazard {
            panic_delta += 1.35 * dt;
        }
        if global_alarm {
            panic_delta += 0.22 * dt;
        }
        if (active_panicked_in_cell[cell_id] >= 2 || density > 2.6)
            && rng.gen_range(0.0..1.0) < config.panic_propagation_pct
            {
                panic_delta += 0.95 * dt;
            }
            if matches!(agents[i].action_state, AgentActionState::FollowingWarden { .. }) {
                panic_delta -= 1.5 * dt;
            }

            agents[i].panic_level = (agents[i].panic_level + panic_delta).clamp(0.0, 10.0);

        // FALLO COGNITIVO POR PÁNICO Y PERFIL DEMOGRÁFICO (Niños, Ancianos y Civiles en pánico se bloquean o deambulan)
        if agents[i].role != MacesRole::TrainedLeader
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
            && agents[i].panic_level >= 6.2
            {
                let vuln = agents[i].profile.cognitive_vulnerability();
                let breakdown_prob = vuln * ((agents[i].panic_level - 5.5) / 4.5) * 0.28 * dt;
                if rng.gen_range(0.0..1.0) < breakdown_prob {
                    if rng.gen_bool(0.45) {
                        // Se queda paralizado por pánico esperando que un brigadista o familiar lo guíe
                        agents[i].action_state = AgentActionState::PanickedFrozen {
                            timer_s: rng.gen_range(3.0..6.5),
                        };
                    } else {
                        // Camina desorientado sin pensar bien
                        agents[i].action_state = AgentActionState::PanickedWandering {
                            timer_s: rng.gen_range(2.5..5.0),
                            angle: rng.gen_range(0.0..std::f32::consts::TAU),
                        };
                    }
                    continue;
                }
            }

            // Altruismo: Ayudar a caídos
            let in_stair_or_exit = matches!(cell.kind, CellKind::Stairwell | CellKind::Exit);
            let room_safe_for_rescue = !in_stair_or_exit
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
                        agents[i].action_state = AgentActionState::Evacuating;
                    } else {
                        let dist = (agents[i].x - agents[target_agent_id].x)
                        .hypot(agents[i].y - agents[target_agent_id].y);
                        if dist < 1.25 {
                            timer_s -= dt;
                            if timer_s <= 0.0 {
                                agents[target_agent_id].is_fallen = false;
                                agents[target_agent_id].fallen_timer_s = 0.0;
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
                && room_safe_for_rescue
                {
                    let mut priority_fallen = None;
                    if config.enable_social_bonds {
                        for &bond_id in &agents[i].social_bonds {
                            if agents[bond_id].current_cell == cell_id
                                && agents[bond_id].is_fallen
                                && agents[bond_id].action_state != AgentActionState::Incapacitated
                                {
                                    priority_fallen = Some(bond_id);
                                    break;
                                }
                        }
                    }

                    if priority_fallen.is_none()
                        && agents[i].profile.can_use_extinguisher()
                        && agents[i].panic_level < 5.2
                        && density < 1.5
                        {
                            for &occ_id in &cell.occupant_ids {
                                if occ_id != i
                                    && agents[occ_id].is_fallen
                                    && agents[occ_id].action_state != AgentActionState::Incapacitated
                                    {
                                        if (agents[i].x - agents[occ_id].x).hypot(agents[i].y - agents[occ_id].y)
                                            < 2.0
                                            {
                                                priority_fallen = Some(occ_id);
                                                break;
                                            }
                                    }
                            }
                        }

                        if let Some(target_id) = priority_fallen {
                            agents[i].action_state = AgentActionState::HelpingFallen {
                                target_agent_id: target_id,
                                timer_s: 0.65,
                                attempt_timeout_s: 2.0,
                            };
                        }
                }
    }
}
