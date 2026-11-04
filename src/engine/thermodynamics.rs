use crate::building_loader::{LoadedBuilding, propagate_shortest_and_alternative_paths};
use crate::config::{
    AMBIENT_TEMP_C, COLLAPSE_TEMP_C, EXTINGUISHER_COOLING_RATE_C_PER_S, FLASHOVER_TEMP_C,
    SimulationConfig,
};
use crate::model::agent::{Agent, AgentActionState};
use crate::model::topology::CellKind;

pub fn step_thermodynamics(
    building: &mut LoadedBuilding,
    agents: &mut [Agent],
    config: &SimulationConfig,
    global_alarm: &mut bool,
    dt: f32,
) {
    let num_floors = building.floors.len();
    let rows = building.rows;
    let cols = building.cols;

    // 1. Descarga de extintores sobre las baldosas en llamas cercanas al brigadista
    for agent in agents.iter_mut() {
        if let AgentActionState::FightingFire {
            fire_cell,
            ref mut remaining_charge_s,
        } = agent.action_state
        {
            if *remaining_charge_s > 0.0 && fire_cell < building.cells.len() {
                *remaining_charge_s -= dt;
                let f = agent.floor;
                let (ar, ac) = building.tile_coords_at(agent.x, agent.y);

                let mut any_active_fire = false;
                let mut hit_uncontrollable = false;

                for dr in -4..=4isize {
                    for dc in -4..=4isize {
                        let nr = ar as isize + dr;
                        let nc = ac as isize + dc;
                        if nr >= 0 && nr < rows as isize && nc >= 0 && nc < cols as isize {
                            let (ur, uc) = (nr as usize, nc as usize);
                            let t = building.tile_temp[f][ur][uc];
                            if t >= FLASHOVER_TEMP_C {
                                hit_uncontrollable = true;
                            } else if t > AMBIENT_TEMP_C + 2.0 {
                                building.tile_temp[f][ur][uc] = (t
                                - EXTINGUISHER_COOLING_RATE_C_PER_S * dt)
                                .max(AMBIENT_TEMP_C);
                                building.tile_smoke[f][ur][uc] =
                                (building.tile_smoke[f][ur][uc] - dt * 0.12).max(0.0);
                                building.tile_fire_time[f][ur][uc] =
                                (building.tile_fire_time[f][ur][uc] - dt * 2.0).max(0.0);
                                if building.tile_temp[f][ur][uc] > AMBIENT_TEMP_C + 5.0 {
                                    any_active_fire = true;
                                }
                            }
                        }
                    }
                }

                if hit_uncontrollable || !any_active_fire {
                    agent.action_state = AgentActionState::Evacuating;
                }
            } else {
                agent.action_state = AgentActionState::Evacuating;
            }
        }
    }

    // 2. Desgaste térmico de las puertas por el calor de las baldosas adyacentes
    for portal in &mut building.portals {
        if portal.floor_a != portal.floor_b {
            continue;
        }
        let f = portal.floor_a;
        let (pr, pc) = portal.tile_rc;
        let mut max_neighbor_t = building.tile_temp[f][pr][pc];
        for (dr, dc) in [(-1isize, 0isize), (1, 0), (0, -1), (0, 1)] {
            let nr = pr as isize + dr;
            let nc = pc as isize + dc;
            if nr >= 0 && nr < rows as isize && nc >= 0 && nc < cols as isize {
                max_neighbor_t =
                max_neighbor_t.max(building.tile_temp[f][nr as usize][nc as usize]);
            }
        }
        if max_neighbor_t > 110.0 && portal.fire_resistance_s > 0.0 {
            let wear = ((max_neighbor_t - 95.0) / 240.0).clamp(0.2, 4.5);
            portal.fire_resistance_s = (portal.fire_resistance_s - wear * dt).max(0.0);
        }
    }

    // 3. Ecuación de Reacción-Difusión en Grilla 2D (Propagación en Ondas Baldosa por Baldosa)
    let dirs_8: [(isize, isize, f32); 8] = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, 0.707),
        (-1, 1, 0.707),
        (1, -1, 0.707),
        (1, 1, 0.707),
    ];

    for f in 0..num_floors {
        let old_temp = building.tile_temp[f].clone();
        let old_smoke = building.tile_smoke[f].clone();
        let old_co = building.tile_co[f].clone();
        let old_co2 = building.tile_co2[f].clone();

        for r in 0..rows {
            for c in 0..cols {
                if !building.walkable_grid[f][r][c] {
                    continue;
                }
                let Some(cid) = building.cell_grid[f][r][c] else {
                    continue;
                };
                if building.cells[cid].kind == CellKind::Exit {
                    building.tile_temp[f][r][c] = AMBIENT_TEMP_C;
                    building.tile_smoke[f][r][c] = 0.0;
                    building.tile_co[f][r][c] = 0.0;
                    building.tile_co2[f][r][c] = 0.04;
                    continue;
                }

                let mut t = old_temp[r][c];
                let mut smk = old_smoke[r][c];
                let mut co = old_co[r][c];
                let mut co2 = old_co2[r][c];
                let fuel = building.tile_fuel[f][r][c];

                // A. Combustión local (si la baldosa ya superó el umbral de ignición de 130°C o fue encendida)
                if t > 130.0 && fuel > 0.0 {
                    building.tile_fire_time[f][r][c] += dt;
                    let ft = building.tile_fire_time[f][r][c];
                    let reaction_heat =
                    config.fire_growth_alpha * 420.0 * (1.0 + ft * 0.22) * fuel * dt;
                    t += reaction_heat;
                    building.tile_fuel[f][r][c] = (fuel - 0.004 * dt).max(0.0);

                    let excess = (t - AMBIENT_TEMP_C).max(0.0);
                    co += excess * 0.75 * config.smoke_yield_rate * dt;
                    co2 += (excess / 550.0) * 0.28 * config.smoke_yield_rate * dt;
                    smk += (excess / 380.0) * 0.14 * config.smoke_yield_rate * dt;
                } else {
                    // Enfriamiento convectivo lento hacia temperatura ambiente
                    t -= (t - AMBIENT_TEMP_C) * 0.012 * dt;
                }

                // B. Difusión Laplaciana en onda circular hacia las 8 baldosas vecinas
                let mut lap_t = 0.0_f32;
                let mut lap_smk = 0.0_f32;
                let mut lap_co = 0.0_f32;
                let mut lap_co2 = 0.0_f32;

                for (dr, dc, weight) in dirs_8 {
                    let nr = r as isize + dr;
                    let nc = c as isize + dc;
                    if nr < 0 || nr >= rows as isize || nc < 0 || nc >= cols as isize {
                        continue;
                    }
                    let (ur, uc) = (nr as usize, nc as usize);
                    if !building.walkable_grid[f][ur][uc] {
                        continue;
                    }

                    // Factor de permeabilidad si hay una puerta entre (r, c) y (ur, uc)
                    let mut perm_heat = 1.0_f32;
                    let mut perm_smoke = 1.0_f32;
                    if let Some(pid) =
                        building.portal_at_tile[f][r][c].or(building.portal_at_tile[f][ur][uc])
                        {
                            let p = &building.portals[pid];
                            if p.is_closed_or_blocked {
                                if p.fire_resistance_s > 0.0 {
                                    perm_heat = 0.02;
                                    perm_smoke = 0.04;
                                } else {
                                    perm_heat = 0.85;
                                    perm_smoke = 0.95;
                                }
                            } else if p.fire_resistance_s > 0.0 {
                                perm_heat = 0.18;
                                perm_smoke = 0.35;
                            }
                        }

                        lap_t += (old_temp[ur][uc] - old_temp[r][c]) * weight * perm_heat;
                        lap_smk += (old_smoke[ur][uc] - old_smoke[r][c]) * weight * perm_smoke;
                        lap_co += (old_co[ur][uc] - old_co[r][c]) * weight * perm_smoke;
                        lap_co2 += (old_co2[ur][uc] - old_co2[r][c]) * weight * perm_smoke;
                }

                // Velocidad de propagación de la onda térmica (0.55 m²/s) y de la onda de humo (1.35 m²/s)
                t += lap_t * 0.55 * dt;
                smk += lap_smk * 1.35 * dt;
                co += lap_co * 1.25 * dt;
                co2 += lap_co2 * 1.25 * dt;

                building.tile_temp[f][r][c] = t.clamp(AMBIENT_TEMP_C, 1100.0);
                building.tile_smoke[f][r][c] = smk.clamp(0.0, 1.0);
                building.tile_co[f][r][c] = co.clamp(0.0, 15_000.0);
                building.tile_co2[f][r][c] = co2.clamp(0.04, 12.0);
            }
        }
    }

    // 4. Efecto Chimenea Vertical: Difusión de humo y calor hacia los pisos superiores por escaleras y ascensores
    for p in &building.portals {
        if p.floor_a != p.floor_b && !p.is_closed_or_blocked {
            let (pr, pc) = p.tile_rc;
            let f_low = p.floor_a.min(p.floor_b);
            let f_high = p.floor_a.max(p.floor_b);
            let t_low = building.tile_temp[f_low][pr][pc];
            let s_low = building.tile_smoke[f_low][pr][pc];
            if t_low > AMBIENT_TEMP_C + 10.0 {
                building.tile_temp[f_high][pr][pc] =
                (building.tile_temp[f_high][pr][pc] + (t_low - AMBIENT_TEMP_C) * 0.12 * dt)
                .min(1100.0);
            }
            if s_low > 0.05 {
                building.tile_smoke[f_high][pr][pc] =
                (building.tile_smoke[f_high][pr][pc] + s_low * 0.25 * dt).min(1.0);
            }
        }
    }

    // 5. Sincronizar estadísticos agregados por Celda y verificar Colapso Estructural
    let mut topology_changed = false;
    for cell in &mut building.cells {
        if cell.kind == CellKind::Exit || cell.tiles.is_empty() {
            cell.temperature_c = AMBIENT_TEMP_C;
            cell.smoke_opacity = 0.0;
            continue;
        }
        let f = cell.floor;
        let mut sum_t = 0.0_f32;
        let mut max_t = AMBIENT_TEMP_C;
        let mut sum_s = 0.0_f32;
        let mut sum_co = 0.0_f32;
        let mut sum_co2 = 0.0_f32;
        let mut hot_tiles = 0usize;

        for &(tr, tc) in &cell.tiles {
            let tt = building.tile_temp[f][tr][tc];
            sum_t += tt;
            max_t = max_t.max(tt);
            sum_s += building.tile_smoke[f][tr][tc];
            sum_co += building.tile_co[f][tr][tc];
            sum_co2 += building.tile_co2[f][tr][tc];
            if tt >= COLLAPSE_TEMP_C {
                hot_tiles += 1;
            }
        }

        let n = cell.tiles.len() as f32;
        // La celda reporta la temperatura máxima de su frente de onda para la UI y el promedio para difusión macro
        cell.temperature_c = max_t * 0.65 + (sum_t / n) * 0.35;
        cell.smoke_opacity = sum_s / n;
        cell.co_ppm = sum_co / n;
        cell.co2_pct = sum_co2 / n;

        if matches!(cell.kind, CellKind::Elevator { .. })
            && (cell.temperature_c > 85.0 || cell.smoke_opacity > 0.25)
            {
                *global_alarm = true;
            }

            // Una celda colapsa si al menos el 40% de sus baldosas alcanzaron temperatura de colapso (850°C)
            if !cell.is_collapsed && hot_tiles * 5 >= cell.tiles.len() * 2 {
                cell.is_collapsed = true;
                topology_changed = true;
            }
    }

    if topology_changed {
        for portal in &mut building.portals {
            if building.cells[portal.cell_a].is_collapsed
                || building.cells[portal.cell_b].is_collapsed
                {
                    portal.is_closed_or_blocked = true;
                }
        }
        let LoadedBuilding {
            ref mut cells,
            ref portals,
            ..
        } = *building;
        propagate_shortest_and_alternative_paths(cells, portals);
    }
}
