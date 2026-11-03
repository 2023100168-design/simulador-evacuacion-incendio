use crate::building_loader::LoadedBuilding;
use crate::config::{
    AMBIENT_TEMP_C, EXTINGUISHER_DURATION_S, EXTINGUISHER_MAX_TEMP_C, SimulationConfig,
};
use crate::model::agent::{Agent, AgentActionState, CellMemory, MacesRole};
use crate::model::topology::CellKind;

pub fn step_maces_wayfinding(
    building: &mut LoadedBuilding,
    agents: &mut [Agent],
    config: &SimulationConfig,
    global_alarm: &mut bool,
    _dt: f32,
) {
    if *global_alarm {
        for ag in agents.iter_mut() {
            if matches!(ag.action_state, AgentActionState::TriggeringAlarm { .. }) {
                ag.action_state = AgentActionState::Evacuating;
            }
        }
    }

    let num_agents = agents.len();
    for i in 0..num_agents {
        if matches!(
            agents[i].action_state,
            AgentActionState::Evacuated
            | AgentActionState::Incapacitated
            | AgentActionState::Normal
            | AgentActionState::Reacting { .. }
            | AgentActionState::HelpingFallen { .. }
        ) {
            continue;
        }

        let curr_cell = agents[i].current_cell;
        let curr_kind = building.cells[curr_cell].kind;
        let floor = agents[i].floor;

        agents[i]
        .mental_map
        .entry(curr_cell)
        .or_insert(CellMemory::Visited);

        if matches!(curr_kind, CellKind::Stairwell | CellKind::Exit) {
            agents[i].action_state = AgentActionState::Evacuating;
            continue;
        }

        // Detectar fuego cercano
        let mut detected_fire_cell = None;
        if building.cells[curr_cell].temperature_c > AMBIENT_TEMP_C + 20.0 {
            detected_fire_cell = Some(curr_cell);
        } else {
            for &pid in &building.cells[curr_cell].portals {
                if let Some(nc) = building.portals[pid].other_cell(curr_cell) {
                    if building.cells[nc].temperature_c > AMBIENT_TEMP_C + 20.0 {
                        detected_fire_cell = Some(nc);
                        break;
                    }
                }
            }
        }

        // A. Activar Alarma (Solo 1 agente)
        if let AgentActionState::TriggeringAlarm { station_id } = agents[i].action_state {
            let st = &mut building.alarms[station_id];
            if st.is_triggered || *global_alarm {
                agents[i].action_state = AgentActionState::Evacuating;
            } else if (agents[i].x - st.x).hypot(agents[i].y - st.y) < 1.5 {
                st.is_triggered = true;
                *global_alarm = true;
                agents[i].action_state = AgentActionState::Evacuating;
            }
            continue;
        }

        if detected_fire_cell.is_some() && !*global_alarm {
            let someone_going = agents
            .iter()
            .any(|a| matches!(a.action_state, AgentActionState::TriggeringAlarm { .. }));
            if !someone_going {
                if let Some(station) = building
                    .alarms
                    .iter_mut()
                    .find(|a| a.floor == floor && a.cell_id == curr_cell && !a.is_triggered)
                    {
                        let dist = (agents[i].x - station.x).hypot(agents[i].y - station.y);
                        if dist < 1.5 {
                            station.is_triggered = true;
                            *global_alarm = true;
                        } else if dist < 9.0 {
                            agents[i].action_state = AgentActionState::TriggeringAlarm {
                                station_id: station.id,
                            };
                            continue;
                        }
                    }
            }
        }

        // B. Extintor en Conatos (< 250 °C)
        if let Some(f_cell) = detected_fire_cell {
            let fire_temp = building.cells[f_cell].temperature_c;
            if fire_temp < EXTINGUISHER_MAX_TEMP_C
                && agents[i].profile.can_use_extinguisher()
                && agents[i].role != MacesRole::Follower
                && matches!(
                    agents[i].action_state,
                    AgentActionState::Evacuating | AgentActionState::SweepingFloor { .. }
                )
                {
                    if let Some(ext) = building.extinguishers.iter_mut().find(|e| {
                        e.floor == floor
                        && e.is_available
                        && (e.cell_id == curr_cell || e.cell_id == f_cell)
                    }) {
                        ext.is_available = false;
                        agents[i].action_state = AgentActionState::FetchingExtinguisher {
                            ext_id: ext.id,
                            fire_cell: f_cell,
                        };
                    }
                }
        }

        if let AgentActionState::FetchingExtinguisher { ext_id, fire_cell } = agents[i].action_state
        {
            let ext_pos = (
                building.extinguishers[ext_id].x,
                building.extinguishers[ext_id].y,
            );
            if (agents[i].x - ext_pos.0).hypot(agents[i].y - ext_pos.1) < 1.4 {
                agents[i].action_state = AgentActionState::FightingFire {
                    fire_cell,
                    remaining_charge_s: EXTINGUISHER_DURATION_S,
                };
            }
            continue;
        }

        if matches!(agents[i].action_state, AgentActionState::FightingFire { .. }) {
            continue;
        }

        // C. PROTOCOLO DE BRIGADISTAS: Patrullar piso -> Reclutar agentes en pánico -> Guiar a escalera -> Repetir
        if agents[i].role == MacesRole::TrainedLeader {
            let remaining_civilians_in_floor = agents.iter().any(|a| {
                a.id != i
                && a.floor == floor
                && a.role != MacesRole::TrainedLeader
                && !matches!(
                    a.action_state,
                    AgentActionState::Evacuated | AgentActionState::Incapacitated
                )
                && building.cells[a.current_cell].kind != CellKind::Stairwell
                && building.cells[a.current_cell].kind != CellKind::Exit
            });

            if !remaining_civilians_in_floor || agents[i].fed_dose > 0.35 || agents[i].active_evac_time_s > 42.0 {
                let mut follower_ids = Vec::new();
                for (idx, ag) in agents.iter().enumerate() {
                    if ag.action_state == (AgentActionState::FollowingWarden { warden_id: i }) {
                        follower_ids.push(idx);
                    }
                }
                for fid in follower_ids {
                    agents[fid].action_state = AgentActionState::Evacuating;
                    agents[fid].guided_by_warden = true;
                    agents[fid].family_reunited = true;
                }
                agents[i].action_state = AgentActionState::Evacuating;
                continue;
            }

            // Recolectar índices de agentes cercanos para reclutar sin violar las reglas de préstamo de Rust
            let (wx, wy) = (agents[i].x, agents[i].y);
            let mut recruits_to_update = Vec::new();
            let mut follower_count = 0usize;

            for (idx, ag) in agents.iter().enumerate() {
                if idx == i
                    || ag.floor != floor
                    || ag.role == MacesRole::TrainedLeader
                    || matches!(
                        ag.action_state,
                        AgentActionState::Evacuated | AgentActionState::Incapacitated
                    )
                    {
                        continue;
                    }

                    if ag.action_state == (AgentActionState::FollowingWarden { warden_id: i }) {
                        follower_count += 1;
                    } else if (ag.x - wx).hypot(ag.y - wy) < 3.8
                        && matches!(
                            ag.action_state,
                            AgentActionState::PanickedFrozen { .. }
                            | AgentActionState::PanickedWandering { .. }
                            | AgentActionState::ExitingRoomToCorridor
                            | AgentActionState::SeekingFamilyInCorridor { .. }
                        )
                        {
                            recruits_to_update.push(idx);
                        }
            }

            let newly_recruited = recruits_to_update.len();
            for r_idx in recruits_to_update {
                agents[r_idx].action_state = AgentActionState::FollowingWarden { warden_id: i };
                agents[r_idx].guided_by_warden = true;
                agents[r_idx].panic_level = (agents[r_idx].panic_level - 2.5).max(1.0);
                follower_count += 1;
            }
            agents[i].rescued_count += newly_recruited;

            match agents[i].action_state {
                AgentActionState::SweepingFloor { mut waypoint_idx } => {
                    if follower_count >= 3 {
                        if let Some(stair) = building
                            .cells
                            .iter()
                            .filter(|c| c.floor == floor && c.kind == CellKind::Stairwell && !c.is_collapsed)
                            .min_by_key(|c| c.occupant_ids.len())
                            {
                                agents[i].action_state = AgentActionState::EscortingToStairs {
                                    stair_cell_id: stair.id,
                                };
                                continue;
                            }
                    }

                    let wps = &building.corridor_waypoints[floor];
                    if !wps.is_empty() {
                        let target_wp = wps[waypoint_idx % wps.len()];
                        if (wx - target_wp.0).hypot(wy - target_wp.1) < 1.8 {
                            waypoint_idx = (waypoint_idx + 1) % wps.len();
                            agents[i].action_state =
                            AgentActionState::SweepingFloor { waypoint_idx };
                        }
                    }
                }
                AgentActionState::EscortingToStairs { stair_cell_id } => {
                    let (sx, sy) = building.cells[stair_cell_id].center();
                    if (wx - sx).hypot(wy - sy) < 3.2 || curr_cell == stair_cell_id {
                        let mut guided_ids = Vec::new();
                        for (idx, ag) in agents.iter().enumerate() {
                            if ag.action_state == (AgentActionState::FollowingWarden { warden_id: i }) {
                                guided_ids.push(idx);
                            }
                        }
                        for gid in guided_ids {
                            agents[gid].action_state = AgentActionState::Evacuating;
                            agents[gid].guided_by_warden = true;
                            agents[gid].family_reunited = true;
                        }
                        agents[i].action_state = AgentActionState::SweepingFloor { waypoint_idx: 0 };
                    }
                }
                _ => {
                    agents[i].action_state = AgentActionState::SweepingFloor { waypoint_idx: 0 };
                }
            }
            continue;
        }

        // Si el civil va siguiendo a un brigadista, verificar que el brigadista siga en el mismo piso
        if let AgentActionState::FollowingWarden { warden_id } = agents[i].action_state {
            if agents[warden_id].floor != floor
                || matches!(
                    agents[warden_id].action_state,
                    AgentActionState::Evacuated | AgentActionState::Incapacitated
                )
                {
                    agents[i].action_state = AgentActionState::Evacuating;
                    agents[i].guided_by_warden = true;
                }
                continue;
        }

        // D. TRANSICIÓN DE METAS DE LOS CIVILES (Meta 1: Salir al Pasillo -> Meta 2: Buscar Familia en Pasillo -> Meta 3: Escaleras)
        match agents[i].action_state {
            AgentActionState::ExitingRoomToCorridor => {
                // En cuanto pisa el pasillo (Corridor), completó la Meta 1
                if curr_kind == CellKind::Corridor {
                    if config.enable_social_bonds
                        && !agents[i].social_bonds.is_empty()
                        && !agents[i].family_reunited
                        {
                            agents[i].action_state = AgentActionState::SeekingFamilyInCorridor {
                                patrol_waypoint_idx: i % 5,
                            };
                        } else {
                            agents[i].action_state = AgentActionState::Evacuating;
                        }
                }
            }
            AgentActionState::SeekingFamilyInCorridor {
                mut patrol_waypoint_idx,
            } => {
                if !config.enable_social_bonds {
                    agents[i].family_reunited = true;
                    agents[i].action_state = AgentActionState::Evacuating;
                    continue;
                }

                // Verificar si todos los familiares vivos en este piso ya se encontraron (< 3.2m)
                let (ax, ay) = (agents[i].x, agents[i].y);
                let mut all_close = true;
                for &bid in &agents[i].social_bonds {
                    let b = &agents[bid];
                    if b.floor == floor
                        && !matches!(
                            b.action_state,
                            AgentActionState::Evacuated | AgentActionState::Incapacitated
                        )
                        {
                            if (b.x - ax).hypot(b.y - ay) > 3.2 {
                                all_close = false;
                                break;
                            }
                        }
                }

                // Si ya se reunieron todos (o si el humo/fuego en el pasillo es peligroso), pasan juntos a la Meta 3 (Escaleras)
                if all_close
                    || building.cells[curr_cell].temperature_c > 110.0
                    || building.cells[curr_cell].smoke_opacity > 0.35
                    {
                        agents[i].family_reunited = true;
                        agents[i].action_state = AgentActionState::Evacuating;
                        let bonds = agents[i].social_bonds.clone();
                        for bid in bonds {
                            if agents[bid].floor == floor {
                                agents[bid].family_reunited = true;
                                if matches!(
                                    agents[bid].action_state,
                                    AgentActionState::SeekingFamilyInCorridor { .. }
                                ) {
                                    agents[bid].action_state = AgentActionState::Evacuating;
                                }
                            }
                        }
                    } else {
                        // Si aún no ve a su familia, recorre todo el pasillo de waypoint en waypoint buscándolos
                        let wps = &building.corridor_waypoints[floor];
                        if !wps.is_empty() {
                            let wp = wps[patrol_waypoint_idx % wps.len()];
                            if (ax - wp.0).hypot(ay - wp.1) < 1.8 {
                                patrol_waypoint_idx = (patrol_waypoint_idx + 1) % wps.len();
                                agents[i].action_state = AgentActionState::SeekingFamilyInCorridor {
                                    patrol_waypoint_idx,
                                };
                            }
                        }
                    }
            }
            AgentActionState::Evacuating => {
                // Si el usuario activa el CheckBox de Vínculos Familiares en vivo y aún están en el pasillo sin reunirse
                if config.enable_social_bonds
                    && curr_kind == CellKind::Corridor
                    && !agents[i].family_reunited
                    && !agents[i].social_bonds.is_empty()
                    {
                        let missing = agents[i].social_bonds.iter().any(|&bid| {
                            agents[bid].floor == floor
                            && !matches!(
                                agents[bid].action_state,
                                AgentActionState::Evacuated | AgentActionState::Incapacitated
                            )
                            && (agents[bid].x - agents[i].x).hypot(agents[bid].y - agents[i].y) > 3.5
                        });
                        if missing {
                            agents[i].action_state = AgentActionState::SeekingFamilyInCorridor {
                                patrol_waypoint_idx: i % 5,
                            };
                        }
                    }
            }
            _ => {}
        }
    }
}
