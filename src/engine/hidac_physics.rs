use crate::building_loader::{LoadedBuilding, TILE_SIZE_M};
use crate::config::SimulationConfig;
use crate::model::agent::{Agent, AgentActionState, CellMemory};
use crate::model::topology::CellKind;
use rand::Rng;

#[derive(Clone, Copy)]
struct AgentSnapshot {
    id: usize,
    floor: usize,
    cell_id: usize,
    x: f32,
    y: f32,
    vx: f32,
    vy: f32,
    radius: f32,
    is_fallen: bool,
    active: bool,
}

pub fn rebuild_spatial_index(building: &mut LoadedBuilding, agents: &[Agent]) {
    for cell in &mut building.cells {
        cell.occupant_ids.clear();
        cell.spatial_grid.clear();
    }

    for agent in agents {
        if agent.action_state == AgentActionState::Evacuated {
            continue;
        }
        if let Some(cell) = building.cells.get_mut(agent.current_cell) {
            cell.occupant_ids.push(agent.id);
            cell.spatial_grid.insert(agent.x, agent.y, agent.id);
        }
    }
}

pub fn step_hidac_physics(
    building: &mut LoadedBuilding,
    agents: &mut [Agent],
    config: &SimulationConfig,
    _global_alarm: bool,
    dt: f32,
) {
    rebuild_spatial_index(building, agents);

    let snapshots: Vec<AgentSnapshot> = agents
    .iter()
    .map(|a| AgentSnapshot {
        id: a.id,
         floor: a.floor,
         cell_id: a.current_cell,
         x: a.x,
         y: a.y,
         vx: a.vx,
         vy: a.vy,
         radius: a.profile.body_radius(),
         is_fallen: a.is_fallen,
         active: a.action_state != AgentActionState::Evacuated,
    })
    .collect();

    let mut rng = rand::thread_rng();
    let mut nearby_ids = Vec::with_capacity(24);

    for i in 0..agents.len() {
        if matches!(
            agents[i].action_state,
            AgentActionState::Evacuated | AgentActionState::Incapacitated
        ) {
            continue;
        }

        if agents[i].is_fallen {
            agents[i].fallen_timer_s -= dt;
            if agents[i].fallen_timer_s <= 0.0 {
                agents[i].is_fallen = false;
            } else {
                continue;
            }
        }

        // Actualizar la celda actual según la casilla del laberinto
        if let Some(real_cid) = building.cell_at(agents[i].floor, agents[i].x, agents[i].y) {
            agents[i].current_cell = real_cid;
            agents[i]
            .mental_map
            .entry(real_cid)
            .or_insert(CellMemory::Visited);
        }

        let curr_cell_id = agents[i].current_cell;
        let is_routine_mode = matches!(
            agents[i].action_state,
            AgentActionState::Normal | AgentActionState::Reacting { .. }
        );

        if !is_routine_mode {
            agents[i].active_evac_time_s += dt;
        }

        // 1. SALIDA INMEDIATA: si está en Planta Baja y pisa o roza una casilla 'X' (Exit), evacúa al instante
        if !is_routine_mode {
            if building.cells[curr_cell_id].kind == CellKind::Exit {
                agents[i].action_state = AgentActionState::Evacuated;
                continue;
            }
            let r_tile = (agents[i].y / TILE_SIZE_M).floor() as isize;
            let c_tile = (agents[i].x / TILE_SIZE_M).floor() as isize;
            let mut touched_exit = false;
            for (dr, dc) in [(0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)] {
                let nr = r_tile + dr;
                let nc = c_tile + dc;
                if nr >= 0
                    && nr < building.rows as isize
                    && nc >= 0
                    && nc < building.cols as isize
                    {
                        if let Some(cid) =
                            building.cell_grid[agents[i].floor][nr as usize][nc as usize]
                            {
                                if building.cells[cid].kind == CellKind::Exit {
                                    let tx = (nc as f32 + 0.5) * TILE_SIZE_M;
                                    let ty = (nr as f32 + 0.5) * TILE_SIZE_M;
                                    if (agents[i].x - tx).hypot(agents[i].y - ty) < 1.15 {
                                        touched_exit = true;
                                        break;
                                    }
                                }
                            }
                    }
            }
            if touched_exit {
                agents[i].action_state = AgentActionState::Evacuated;
                continue;
            }
        }

        // 2. DESCENSO POR ESCALERA EN 2 FILAS ORDENADAS (Pisos 1 y 2 -> Piso inferior)
        if !is_routine_mode
            && building.cells[curr_cell_id].kind == CellKind::Stairwell
            && agents[i].floor > 0
            {
                let stair_cell = &building.cells[curr_cell_id];
                let density = stair_cell.density();
                let stair_speed = agents[i].profile.calculate_speed(density, true, false);

                // Tramo de un piso al inferior (~3.8 metros)
                agents[i].stair_progress += (stair_speed * dt) / 3.8;
                agents[i].distance_traveled_m += stair_speed * dt;

                let p = agents[i].stair_progress.clamp(0.0, 1.0);
                let lane_offset = if agents[i].stair_lane == 0 { -0.35 } else { 0.35 };
                let (cx, _) = stair_cell.center();
                let target_x =
                (cx + lane_offset).clamp(stair_cell.min_x + 0.3, stair_cell.max_x - 0.3);
                let target_y = stair_cell.min_y
                + 0.4
                + p * (stair_cell.max_y - stair_cell.min_y - 0.8).max(0.5);

                agents[i].vx = (target_x - agents[i].x) / dt.max(1e-4);
                agents[i].vy = (target_y - agents[i].y) / dt.max(1e-4);
                agents[i].x = target_x;
                agents[i].y = target_y;

                if agents[i].stair_progress >= 1.0 {
                    agents[i].stair_progress = 0.0;
                    let down_cell = stair_cell
                    .portals
                    .iter()
                    .filter_map(|&pid| building.portals[pid].other_cell(curr_cell_id))
                    .find(|&nc| building.cells[nc].floor < agents[i].floor);

                    if let Some(next_c) = down_cell {
                        let next_floor = building.cells[next_c].floor;
                        agents[i].floor = next_floor;

                        if next_floor == 0 {
                            // Al llegar a Planta Baja (Piso 0), colocarlo directamente cruzando la puerta de la escalera hacia el pasillo
                            let corridor_portal = building.cells[next_c]
                            .portals
                            .iter()
                            .map(|&pid| &building.portals[pid])
                            .find(|p| p.floor_a == 0 && p.floor_b == 0);

                            if let Some(p_out) = corridor_portal {
                                let out_cell = p_out.other_cell(next_c).unwrap_or(next_c);
                                agents[i].current_cell = out_cell;
                                // Buscar una baldosa del pasillo pegada a la puerta F de la escalera
                                let (pr, pc) = p_out.tile_rc;
                                let mut spawn_pos = p_out.pos_a;
                                for (nr, nc) in [
                                    (pr.wrapping_sub(1), pc),
                                    (pr + 1, pc),
                                    (pr, pc.wrapping_sub(1)),
                                    (pr, pc + 1),
                                ] {
                                    if nr < building.rows
                                        && nc < building.cols
                                        && building.cell_grid[0][nr][nc] == Some(out_cell)
                                        {
                                            spawn_pos = (
                                                (nc as f32 + 0.5) * TILE_SIZE_M,
                                                         (nr as f32 + 0.5) * TILE_SIZE_M,
                                            );
                                            break;
                                        }
                                }
                                agents[i].x = spawn_pos.0 + rng.gen_range(-0.25..0.25);
                                agents[i].y = spawn_pos.1 + rng.gen_range(-0.25..0.25);
                            } else {
                                agents[i].current_cell = next_c;
                                let (nx, ny) = building.cells[next_c].center();
                                agents[i].x = nx + lane_offset;
                                agents[i].y = ny;
                            }
                        } else {
                            // Si bajó del Piso 2 al Piso 1, continúa descendiendo por la caja de escaleras del Piso 1
                            agents[i].current_cell = next_c;
                            let (nx, ny) = building.cells[next_c].center();
                            agents[i].x = nx + lane_offset;
                            agents[i].y = ny;
                        }
                    }
                }
                continue;
            }

            // 3. DIRECCIÓN DESEADA SEGÚN LA META ACTUAL DEL AGENTE
            let (mut goal_dir_x, mut goal_dir_y) = match agents[i].action_state {
                AgentActionState::Normal | AgentActionState::Reacting { .. } => {
                    let c = &building.cells[curr_cell_id];
                    let v_mag = agents[i].vx.hypot(agents[i].vy);
                    if v_mag < 0.08 || rng.gen_bool(0.03) {
                        let &(tr, tc) = c
                        .tiles
                        .get(rng.gen_range(0..c.tiles.len()))
                        .unwrap_or(&(0, 0));
                        let tx = (tc as f32 + 0.5) * TILE_SIZE_M;
                        let ty = (tr as f32 + 0.5) * TILE_SIZE_M;
                        let dx = tx - agents[i].x;
                        let dy = ty - agents[i].y;
                        let l = dx.hypot(dy).max(1e-4);
                        (dx / l, dy / l)
                    } else {
                        (agents[i].vx / v_mag, agents[i].vy / v_mag)
                    }
                }
                AgentActionState::PanickedFrozen { .. } => (0.0, 0.0),
                AgentActionState::PanickedWandering { angle, .. } => (angle.cos(), angle.sin()),
                AgentActionState::ExitingRoomToCorridor => building.sample_flow_direction(
                    &building.corridor_flow,
                    agents[i].floor,
                    agents[i].x,
                    agents[i].y,
                ),
                AgentActionState::SeekingFamilyInCorridor {
                    patrol_waypoint_idx,
                } => {
                    // Si el Jefe de Familia ya está en el pasillo, caminar por el laberinto hacia él; si es el Jefe, recorrer el pasillo buscando a los suyos
                    let lid = agents[i].group_leader_id.unwrap_or(i);
                    let target_pos = if lid != i
                    && snapshots[lid].active
                    && snapshots[lid].floor == agents[i].floor
                    && building.cells[snapshots[lid].cell_id].kind == CellKind::Corridor
                    {
                        (snapshots[lid].x, snapshots[lid].y)
                    } else if let Some(&bid) = agents[i].social_bonds.iter().find(|&&bid| {
                        snapshots[bid].active && snapshots[bid].floor == agents[i].floor
                    }) {
                        (snapshots[bid].x, snapshots[bid].y)
                    } else {
                        let wps = &building.corridor_waypoints[agents[i].floor];
                        if !wps.is_empty() {
                            wps[patrol_waypoint_idx % wps.len()]
                        } else {
                            (agents[i].x, agents[i].y)
                        }
                    };
                    building.direction_towards_point(
                        agents[i].floor,
                        agents[i].x,
                        agents[i].y,
                        target_pos.0,
                        target_pos.1,
                    )
                }
                AgentActionState::SweepingFloor { waypoint_idx } => {
                    // El brigadista va directo hacia el civil en pánico o perdido más cercano en su piso
                    let mut priority_person = None;
                    let mut best_d = f32::INFINITY;
                    for other in &snapshots {
                        if other.active
                            && other.floor == agents[i].floor
                            && other.id != i
                            && building.cells[other.cell_id].kind != CellKind::Stairwell
                            && building.cells[other.cell_id].kind != CellKind::Exit
                            {
                                let d = (other.x - agents[i].x).hypot(other.y - agents[i].y);
                                if d < best_d {
                                    best_d = d;
                                    priority_person = Some((other.x, other.y));
                                }
                            }
                    }
                    let target_pos = priority_person.unwrap_or_else(|| {
                        let wps = &building.corridor_waypoints[agents[i].floor];
                        if !wps.is_empty() {
                            wps[waypoint_idx % wps.len()]
                        } else {
                            building.cells[curr_cell_id].center()
                        }
                    });
                    building.direction_towards_point(
                        agents[i].floor,
                        agents[i].x,
                        agents[i].y,
                        target_pos.0,
                        target_pos.1,
                    )
                }
                AgentActionState::EscortingToStairs { stair_cell_id } => {
                    let (sx, sy) = building.cells[stair_cell_id].center();
                    building.direction_towards_point(agents[i].floor, agents[i].x, agents[i].y, sx, sy)
                }
                AgentActionState::FollowingWarden { warden_id } => building.direction_towards_point(
                    agents[i].floor,
                    agents[i].x,
                    agents[i].y,
                    snapshots[warden_id].x,
                    snapshots[warden_id].y,
                ),
                AgentActionState::TriggeringAlarm { station_id } => building.direction_towards_point(
                    agents[i].floor,
                    agents[i].x,
                    agents[i].y,
                    building.alarms[station_id].x,
                    building.alarms[station_id].y,
                ),
                AgentActionState::FetchingExtinguisher { ext_id, .. } => building
                .direction_towards_point(
                    agents[i].floor,
                    agents[i].x,
                    agents[i].y,
                    building.extinguishers[ext_id].x,
                    building.extinguishers[ext_id].y,
                ),
                AgentActionState::FightingFire { fire_cell, .. } => {
                    let (fx, fy) = building.cells[fire_cell].center();
                    building.direction_towards_point(agents[i].floor, agents[i].x, agents[i].y, fx, fy)
                }
                AgentActionState::HelpingFallen {
                    target_agent_id, ..
                } => {
                    let tx = snapshots[target_agent_id].x;
                    let ty = snapshots[target_agent_id].y;
                    let dx = tx - agents[i].x;
                    let dy = ty - agents[i].y;
                    let l = dx.hypot(dy).max(1e-4);
                    (dx / l, dy / l)
                }
                AgentActionState::Evacuating => {
                    // Los Brigadistas y quienes fueron guiados por ellos conocen la ruta óptima;
                    // los demás usan el flujo ingenuo (generando aglomeración realista en la escalera principal)
                    if agents[i].role == crate::model::agent::MacesRole::TrainedLeader
                        || agents[i].guided_by_warden
                        {
                            building.sample_flow_direction(
                                &building.optimal_flow,
                                agents[i].floor,
                                agents[i].x,
                                agents[i].y,
                            )
                        } else {
                            building.sample_flow_direction(
                                &building.naive_flow,
                                agents[i].floor,
                                agents[i].x,
                                agents[i].y,
                            )
                        }
                }
                _ => (0.0, 0.0),
            };

            // 4. Cohesión Familiar al caminar juntos hacia las escaleras
            let mut speed_social_mod = 1.0_f32;
            if config.enable_social_bonds
                && agents[i].action_state == AgentActionState::Evacuating
                && building.cells[curr_cell_id].kind == CellKind::Corridor
                {
                    if let Some(lid) = agents[i].group_leader_id {
                        if lid != i && snapshots[lid].active && snapshots[lid].floor == agents[i].floor {
                            let dl = (snapshots[lid].x - agents[i].x).hypot(snapshots[lid].y - agents[i].y);
                            if dl > 1.0 && dl < 5.5 {
                                let lx = (snapshots[lid].x - agents[i].x) / dl;
                                let ly = (snapshots[lid].y - agents[i].y) / dl;
                                goal_dir_x = goal_dir_x * 0.68 + lx * 0.32;
                                goal_dir_y = goal_dir_y * 0.68 + ly * 0.32;
                                let n = goal_dir_x.hypot(goal_dir_y).max(1e-4);
                                goal_dir_x /= n;
                                goal_dir_y /= n;
                            }
                        } else if lid == i {
                            // El Jefe de Familia adapta su velocidad al miembro más lento del grupo (ej. niño o abuelo)
                            for &bid in &agents[i].social_bonds {
                                if snapshots[bid].active && snapshots[bid].floor == agents[i].floor {
                                    let d = (snapshots[bid].x - agents[i].x)
                                    .hypot(snapshots[bid].y - agents[i].y);
                                    if d > 1.8 {
                                        speed_social_mod = 0.68;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                // 5. Interacción física entre agentes y cuellos de botella por pánico
                let cell_ref = &building.cells[curr_cell_id];
                cell_ref
                .spatial_grid
                .query_nearby(agents[i].x, agents[i].y, &mut nearby_ids);

                let ri = agents[i].profile.body_radius();
                let rescue_target = match agents[i].action_state {
                    AgentActionState::HelpingFallen {
                        target_agent_id, ..
                    } => Some(target_agent_id),
                    _ => None,
                };

                let mut push_x = 0.0_f32;
                let mut push_y = 0.0_f32;
                let mut tangent_x = 0.0_f32;
                let mut tangent_y = 0.0_f32;
                let mut crowd_neighbors = 0usize;

                for &other_id in &nearby_ids {
                    if other_id == agents[i].id || Some(other_id) == rescue_target {
                        continue;
                    }
                    let other = snapshots[other_id];
                    if !other.active || other.floor != agents[i].floor {
                        continue;
                    }

                    let wx = other.x - agents[i].x;
                    let wy = other.y - agents[i].y;
                    let dij = wx.hypot(wy).max(1e-4);
                    if dij < 1.1 {
                        crowd_neighbors += 1;
                    }
                    let min_sep = ri + other.radius + 0.02;

                    if !other.is_fallen && dij < min_sep {
                        let overlap = min_sep - dij;
                        let panic_push = if agents[i].panic_level >= 5.5 { 1.45 } else { 1.0 };
                        push_x += (-wx / dij) * overlap * panic_push;
                        push_y += (-wy / dij) * overlap * panic_push;
                    } else if dij < 1.25 && (goal_dir_x * wx + goal_dir_y * wy) > 0.0 {
                        let tx = goal_dir_y;
                        let ty = -goal_dir_x;
                        let w = ((1.25 - dij) / 1.25) * config.avoidance_weight * 0.45;
                        tangent_x += tx * w;
                        tangent_y += ty * w;
                    }
                }

                let mut steer_x = goal_dir_x + tangent_x;
                let mut steer_y = goal_dir_y + tangent_y;
                let steer_norm = steer_x.hypot(steer_y).max(1e-4);
                steer_x /= steer_norm;
                steer_y /= steer_norm;

                let cell_density = cell_ref.density();
                let in_panic = agents[i].panic_level >= 5.0 && !is_routine_mode;
                let mut base_speed = agents[i]
                .profile
                .calculate_speed(cell_density, false, in_panic)
                * speed_social_mod;

                // Efecto "Rápido es Lento" (Faster-is-Slower) en cuellos de botella con pánico alto (>= 4 vecinos pegados)
                if crowd_neighbors >= 4 && in_panic {
                    base_speed *= 0.45;
                }

                let speed = match agents[i].action_state {
                    AgentActionState::Normal => base_speed * 0.38,
                    AgentActionState::Reacting { .. } => base_speed * 0.20,
                    AgentActionState::PanickedFrozen { .. } => 0.0,
                    AgentActionState::PanickedWandering { .. } => base_speed * 0.55,
                    _ => base_speed,
                };

                let desired_vx = steer_x * speed;
                let desired_vy = steer_y * speed;
                let blend = 0.35_f32;
                agents[i].vx = agents[i].vx * (1.0 - blend) + desired_vx * blend;
                agents[i].vy = agents[i].vy * (1.0 - blend) + desired_vy * blend;

                let mut new_x = agents[i].x + agents[i].vx * dt + push_x * 0.38;
                let mut new_y = agents[i].y + agents[i].vy * dt + push_y * 0.38;

                building.resolve_maze_wall_collisions(agents[i].floor, &mut new_x, &mut new_y, ri * 0.85);

                let moved = (new_x - agents[i].x).hypot(new_y - agents[i].y);
                if !is_routine_mode
                    && !matches!(agents[i].action_state, AgentActionState::PanickedFrozen { .. })
                    {
                        agents[i].distance_traveled_m += moved;

                        if moved / dt.max(1e-4) < 0.16 {
                            agents[i].stuck_timer_s += dt;
                            if agents[i].stuck_timer_s > 0.45 {
                                let jitter_ang =
                                steer_y.atan2(steer_x) + if (i % 2) == 0 { 1.15 } else { -1.15 };
                                let mut jx = new_x + jitter_ang.cos() * speed * dt;
                                let mut jy = new_y + jitter_ang.sin() * speed * dt;
                                building.resolve_maze_wall_collisions(
                                    agents[i].floor,
                                    &mut jx,
                                    &mut jy,
                                    ri * 0.85,
                                );
                                new_x = jx;
                                new_y = jy;
                                agents[i].stuck_timer_s = 0.0;
                            }
                        } else {
                            agents[i].stuck_timer_s = 0.0;
                        }
                    }

                    agents[i].x = new_x;
                    agents[i].y = new_y;

                    // Caídas realistas por empujones cuando se aglomeran en puertas/escaleras con pánico
                    let push_mag = push_x.hypot(push_y);
                    if !is_routine_mode
                        && (agents[i].can_fall_easily || in_panic)
                        && push_mag > 0.14
                        && crowd_neighbors >= 3
                        && rng.gen_range(0..100) < (12 + (agents[i].panic_level as i32) * 3)
                        {
                            agents[i].is_fallen = true;
                            agents[i].fallen_timer_s = rng.gen_range(3.2..5.5);
                            agents[i].vx = 0.0;
                            agents[i].vy = 0.0;
                        }
    }
}
