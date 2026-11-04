use crate::building_loader::LoadedBuilding;
use crate::config::{
    AMBIENT_TEMP_C, EXTINGUISHER_DURATION_S, EXTINGUISHER_MAX_TEMP_C, SimulationConfig,
};
use crate::model::agent::{Agent, AgentActionState, CellMemory, MacesRole};
use crate::model::topology::CellKind;

const MAX_WARDEN_ESCORT: usize = 2;

pub fn step_maces_wayfinding(
    building: &mut LoadedBuilding,
    agents: &mut [Agent],
    config: &SimulationConfig,
    global_alarm: &mut bool,
    dt: f32,
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

        // C. PROTOCOLO SERIAL DE BRIGADISTAS: Máximo 2 personas a la vez -> Llevar a escalera -> Volver por más
        if agents[i].role == MacesRole::TrainedLeader && config.enable_wardens {
            // Limpiar slots de escolta si algún seguidor ya entró a la escalera o cambió de piso
            let current_slots = agents[i].escort_slots.clone();
            let mut valid_slots = Vec::new();
            for sid in current_slots {
                if agents[sid].floor == floor
                    && matches!(
                        agents[sid].action_state,
                        AgentActionState::FollowingWarden { warden_id, .. } if warden_id == i
                    )
                    && building.cells[agents[sid].current_cell].kind != CellKind::Stairwell
                    {
                        valid_slots.push(sid);
                    }
            }
            agents[i].escort_slots = valid_slots;

            let unguided_civilians_in_floor = agents.iter().any(|a| {
                a.id != i
                && a.floor == floor
                && a.role != MacesRole::TrainedLeader
                && !a.guided_by_warden
                && !matches!(
                    a.action_state,
                    AgentActionState::Evacuated
                    | AgentActionState::Incapacitated
                    | AgentActionState::FollowingWarden { .. }
                )
                && building.cells[a.current_cell].kind != CellKind::Stairwell
                && building.cells[a.current_cell].kind != CellKind::Exit
            });

            // Si ya no quedan civiles sin guiar en este piso o el brigadista corre peligro, evacúa
            if (!unguided_civilians_in_floor && agents[i].escort_slots.is_empty())
                || agents[i].fed_dose > 0.35
                || agents[i].active_evac_time_s > 48.0
                {
                    let slots_to_free = agents[i].escort_slots.clone();
                    for sid in slots_to_free {
                        agents[sid].action_state = AgentActionState::Evacuating;
                        agents[sid].guided_by_warden = true;
                        agents[sid].family_reunited = true;
                    }
                    agents[i].escort_slots.clear();
                    agents[i].action_state = AgentActionState::Evacuating;
                    continue;
                }

                let (wx, wy) = (agents[i].x, agents[i].y);

                match agents[i].action_state {
                    AgentActionState::SweepingFloor { mut waypoint_idx } => {
                        // Solo recluta si tiene espacio en sus 2 slots de escolta
                        if agents[i].escort_slots.len() < MAX_WARDEN_ESCORT {
                            let needed = MAX_WARDEN_ESCORT - agents[i].escort_slots.len();
                            let mut candidates = Vec::new();
                            for (idx, ag) in agents.iter().enumerate() {
                                if idx != i
                                    && ag.floor == floor
                                    && ag.role != MacesRole::TrainedLeader
                                    && !ag.guided_by_warden
                                    && !matches!(
                                        ag.action_state,
                                        AgentActionState::Evacuated
                                        | AgentActionState::Incapacitated
                                        | AgentActionState::FollowingWarden { .. }
                                    )
                                    {
                                        let d = (ag.x - wx).hypot(ag.y - wy);
                                        if d < 3.4 {
                                            candidates.push((idx, d));
                                        }
                                    }
                            }
                            candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

                            for (c_idx, _) in candidates.into_iter().take(needed) {
                                let slot_num = agents[i].escort_slots.len() as u8;
                                agents[i].escort_slots.push(c_idx);
                                agents[c_idx].action_state = AgentActionState::FollowingWarden {
                                    warden_id: i,
                                    slot_idx: slot_num,
                                };
                                agents[c_idx].guided_by_warden = true;
                                agents[c_idx].panic_level = (agents[c_idx].panic_level - 3.0).max(1.0);
                                agents[i].rescued_count += 1;
                            }
                        }

                        // En cuanto tiene al menos 1 persona (o llenó sus 2 cupos), va directo a dejarlos a la escalera sin llamar a más gente
                        if !agents[i].escort_slots.is_empty() {
                            if let Some(stair) = building
                                .cells
                                .iter()
                                .filter(|c| {
                                    c.floor == floor
                                    && c.kind == CellKind::Stairwell
                                    && !c.is_collapsed
                                })
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
                        // Al dejar a las 1 o 2 personas frente a la escalera (< 3.0m), les ordena bajar y regresa por el siguiente grupo
                        if (wx - sx).hypot(wy - sy) < 3.0
                            || curr_cell == stair_cell_id
                            || agents[i].escort_slots.is_empty()
                            {
                                let delivered = agents[i].escort_slots.clone();
                                for gid in delivered {
                                    agents[gid].action_state = AgentActionState::Evacuating;
                                    agents[gid].guided_by_warden = true;
                                    agents[gid].family_reunited = true;
                                }
                                agents[i].escort_slots.clear();
                                agents[i].action_state =
                                AgentActionState::SweepingFloor { waypoint_idx: 0 };
                            }
                    }
                    _ => {
                        agents[i].action_state = AgentActionState::SweepingFloor { waypoint_idx: 0 };
                    }
                }
                continue;
        }

        if let AgentActionState::FollowingWarden { warden_id, .. } = agents[i].action_state {
            if !config.enable_wardens
                || agents[warden_id].floor != floor
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

        // D. TRANSICIÓN DE METAS DE CIVILES (Sin formar círculos familiares)
        match agents[i].action_state {
            AgentActionState::ExitingRoomToCorridor => {
                if curr_kind == CellKind::Corridor {
                    if config.enable_social_bonds
                        && !agents[i].social_bonds.is_empty()
                        && !agents[i].family_reunited
                        {
                            // El grupo familiar elige un waypoint de reunión común según su group_id (evita que se persigan en círculo)
                            let wp_idx = agents[i].group_id.unwrap_or(i);
                            agents[i].action_state = AgentActionState::SeekingFamilyInCorridor {
                                patrol_waypoint_idx: wp_idx,
                                search_timer_s: 5.5,
                            };
                        } else {
                            agents[i].action_state = AgentActionState::Evacuating;
                        }
                }
            }
            AgentActionState::SeekingFamilyInCorridor {
                patrol_waypoint_idx,
                mut search_timer_s,
            } => {
                search_timer_s -= dt;
                if !config.enable_social_bonds || search_timer_s <= 0.0 {
                    agents[i].family_reunited = true;
                    agents[i].action_state = AgentActionState::Evacuating;
                    continue;
                }

                let (ax, ay) = (agents[i].x, agents[i].y);
                let lid = agents[i].group_leader_id.unwrap_or(i);
                let near_leader = agents[lid].floor != floor
                || (agents[lid].x - ax).hypot(agents[lid].y - ay) < 3.4;

                if near_leader
                    || building.cells[curr_cell].temperature_c > 105.0
                    || building.cells[curr_cell].smoke_opacity > 0.30
                    {
                        agents[i].family_reunited = true;
                        agents[i].action_state = AgentActionState::Evacuating;
                    } else {
                        agents[i].action_state = AgentActionState::SeekingFamilyInCorridor {
                            patrol_waypoint_idx,
                            search_timer_s,
                        };
                    }
            }
            _ => {}
        }
    }
}
