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
    let num_cells = building.cells.len();
    let mut delta_temp = vec![0.0_f32; num_cells];
    let mut delta_co = vec![0.0_f32; num_cells];
    let mut delta_co2 = vec![0.0_f32; num_cells];
    let mut delta_smoke = vec![0.0_f32; num_cells];
    let mut topology_changed = false;

    // 1. Acción de agentes brigadistas descargando extintores sobre focos activos
    for agent in agents.iter_mut() {
        if let AgentActionState::FightingFire {
            fire_cell,
            ref mut remaining_charge_s,
        } = agent.action_state
        {
            if *remaining_charge_s > 0.0 && fire_cell < num_cells {
                *remaining_charge_s -= dt;
                let cell = &mut building.cells[fire_cell];

                // Un extintor portátil solo logra enfriar el fuego en fase pre-flashover (< 500 °C)
                if cell.temperature_c < FLASHOVER_TEMP_C {
                    cell.temperature_c = (cell.temperature_c
                    - EXTINGUISHER_COOLING_RATE_C_PER_S * dt)
                    .max(AMBIENT_TEMP_C);
                    cell.fire_time_s = (cell.fire_time_s - dt * 2.5).max(0.0);
                    cell.smoke_opacity = (cell.smoke_opacity - dt * 0.08).max(0.0);

                    if cell.temperature_c <= AMBIENT_TEMP_C + 2.0 {
                        cell.temperature_c = AMBIENT_TEMP_C;
                        cell.fire_time_s = 0.0;
                        agent.action_state = AgentActionState::Evacuating;
                    }
                } else {
                    // Ocurrió Flashover: el fuego supera la capacidad del extintor y el agente huye
                    agent.action_state = AgentActionState::Evacuating;
                }
            } else {
                agent.action_state = AgentActionState::Evacuating;
            }
        }
    }

    // 2. Crecimiento interno α·t² en celdas en llamas y generación de gases asfixiantes
    for i in 0..num_cells {
        let cell = &mut building.cells[i];
        if cell.is_collapsed || cell.kind == CellKind::Exit {
            continue;
        }

        if cell.temperature_c > AMBIENT_TEMP_C + 5.0 && cell.fuel > 0.0 {
            cell.fire_time_s += dt;

            // Tasa de crecimiento térmico proporcional a α·t y al combustible restante
            let growth_rate =
            (config.fire_growth_alpha * 380.0 * (1.0 + cell.fire_time_s * 0.18)) * cell.fuel;
            delta_temp[i] += growth_rate * dt;

            // Consumo gradual de combustible
            cell.fuel = (cell.fuel - 0.0035 * dt).max(0.0);

            // Generación de CO (ppm), CO₂ (%) y opacidad óptica de humo según temperatura
            let temp_excess = (cell.temperature_c - AMBIENT_TEMP_C).max(0.0);
            delta_co[i] += temp_excess * 0.55 * config.smoke_yield_rate * dt;
            delta_co2[i] += (temp_excess / 650.0) * 0.22 * config.smoke_yield_rate * dt;
            delta_smoke[i] += (temp_excess / 450.0) * 0.09 * config.smoke_yield_rate * dt;
        } else {
            // Disipación lenta si no hay fuego activo
            delta_temp[i] -= (cell.temperature_c - AMBIENT_TEMP_C) * 0.015 * dt;
        }
    }

    // 3. Difusión en el Grafo a través de los Portales y Degradación Térmica de Puertas (R_ij)
    for portal in &mut building.portals {
        let ca = portal.cell_a;
        let cb = portal.cell_b;
        let ta = building.cells[ca].temperature_c;
        let tb = building.cells[cb].temperature_c;
        let max_t = ta.max(tb);

        // Desgaste de la resistencia al fuego de la puerta cuando hay calor intenso (> 120 °C)
        if max_t > 120.0 && portal.fire_resistance_s > 0.0 {
            let wear_rate = ((max_t - 100.0) / 280.0).clamp(0.2, 4.0);
            portal.fire_resistance_s = (portal.fire_resistance_s - wear_rate * dt).max(0.0);
        }

        // Coeficiente de transferencia térmica (α_ij) y de humo (σ_ij)
        let (alpha_heat, sigma_smoke) = if portal.is_closed_or_blocked {
            if portal.fire_resistance_s > 0.0 {
                (0.015, 0.03) // Puerta cerrada intacta contiene el fuego y humo
            } else {
                (0.85, 0.95) // Puerta vencida por el fuego
            }
        } else if portal.fire_resistance_s > 0.0 {
            // Puerta cortafuegos o estándar aún resistiendo la propagación de llamas
            (0.06, 0.22)
        } else {
            // Puerta cuya resistencia R_ij llegó a 0: propagación libre
            (0.90, 1.00)
        };

        let we = portal.effective_width();
        let area_a = building.cells[ca].area_m2.max(1.0);
        let area_b = building.cells[cb].area_m2.max(1.0);

        // Intercambio Laplaciano de calor
        let heat_flux = alpha_heat * we * (tb - ta) * 0.18 * dt;
        delta_temp[ca] += heat_flux / area_a;
        delta_temp[cb] -= heat_flux / area_b;

        // Intercambio Laplaciano de gases y humo
        let co_flux = sigma_smoke * we * (building.cells[cb].co_ppm - building.cells[ca].co_ppm) * 0.35 * dt;
        delta_co[ca] += co_flux / area_a;
        delta_co[cb] -= co_flux / area_b;

        let co2_flux = sigma_smoke * we * (building.cells[cb].co2_pct - building.cells[ca].co2_pct) * 0.35 * dt;
        delta_co2[ca] += co2_flux / area_a;
        delta_co2[cb] -= co2_flux / area_b;

        let smoke_flux = sigma_smoke
        * we
        * (building.cells[cb].smoke_opacity - building.cells[ca].smoke_opacity)
        * 0.40
        * dt;
        delta_smoke[ca] += smoke_flux / area_a;
        delta_smoke[cb] -= smoke_flux / area_b;
    }

    // 4. Aplicar estados termodinámicos y verificar Colapso Estructural (>= 850 °C)
    for i in 0..num_cells {
        let cell = &mut building.cells[i];
        if cell.kind == CellKind::Exit {
            cell.temperature_c = AMBIENT_TEMP_C;
            cell.co_ppm = 0.0;
            cell.co2_pct = 0.04;
            cell.smoke_opacity = 0.0;
            continue;
        }

        cell.temperature_c = (cell.temperature_c + delta_temp[i]).clamp(AMBIENT_TEMP_C, 1100.0);
        cell.co_ppm = (cell.co_ppm + delta_co[i]).clamp(0.0, 15_000.0);
        cell.co2_pct = (cell.co2_pct + delta_co2[i]).clamp(0.04, 12.0);
        cell.smoke_opacity = (cell.smoke_opacity + delta_smoke[i]).clamp(0.0, 1.0);

        // Si un eje de ascensor se llena de humo o calor, dispara la alarma global y corta ascensores
        if matches!(cell.kind, CellKind::Elevator { .. })
            && (cell.temperature_c > 85.0 || cell.smoke_opacity > 0.25)
            {
                *global_alarm = true;
            }

            if !cell.is_collapsed && cell.temperature_c >= COLLAPSE_TEMP_C {
                cell.is_collapsed = true;
                topology_changed = true;
            }
    }

    // Si una celda colapsa, todos sus portales quedan clausurados (fallo en cascada)
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
        building.recompute_maze_flow();

    }

    // 5. Actualizar porcentaje de colapso por piso e inhabilitación de nivel (> 60% colapsado)
    for floor_stat in &mut building.floors {
        let f = floor_stat.floor_index;
        let mut total_area = 0.0_f32;
        let mut hazard_area = 0.0_f32;
        for c in building.cells.iter().filter(|c| c.floor == f && c.kind != CellKind::Exit) {
            total_area += c.area_m2;
            if c.is_collapsed || c.temperature_c >= FLASHOVER_TEMP_C {
                hazard_area += c.area_m2;
            }
        }
        floor_stat.collapsed_area_ratio = if total_area > 0.0 {
            hazard_area / total_area
        } else {
            0.0
        };
        if floor_stat.collapsed_area_ratio >= 0.60 {
            floor_stat.is_disabled = true;
        }
    }
}
