#![allow(dead_code)]

mod building_loader;
mod config;
mod engine;
mod model;
mod render;

use building_loader::{
    LoadedBuilding, load_building_from_ascii, propagate_shortest_and_alternative_paths,
};
use config::{AMBIENT_TEMP_C, FLASHOVER_TEMP_C, SimulationConfig};
use engine::SimulationEngine;
use model::agent::MacesRole;
use render::canvas_2d::{
    render_inspector_bars, render_simulation_frame, render_telemetry_chart, screen_to_agent,
    screen_to_cell, screen_to_portal,
};
use slint::{Timer, TimerMode};
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

slint::include_modules!();

const CANVAS_W: u32 = 960;
const CANVAS_H: u32 = 640;
const CHART_W: u32 = 325;
const CHART_H: u32 = 125;

fn main() {
    let building = load_building_from_ascii("assets/buildings/pabellon_universitario.txt")
    .expect("Fallo al construir el edificio");

    let config = SimulationConfig::default();
    let sim = SimulationEngine::new(building, config);

    let app = AppWindow::new().expect("No se pudo crear la ventana de Slint");
    app.set_total_agents(sim.agents.len() as i32);

    let sim_rc = Rc::new(RefCell::new(sim));

    let refresh_ui = {
        let app_weak = app.as_weak();
        let s_rc = sim_rc.clone();
        move || {
            if let Some(app) = app_weak.upgrade() {
                let s = s_rc.borrow();
                app.set_evacuated_agents(s.count_evacuated() as i32);
                app.set_alerted_agents(s.count_alerted() as i32);
                app.set_fallen_agents(s.count_fallen() as i32);
                app.set_incapacitated_agents(s.count_incapacitated() as i32);
                app.set_sim_time_text(format!("{:.1} s", s.time_elapsed_s).into());
                app.set_global_alarm(s.global_alarm);
                app.set_inspector_text(s.format_inspector_header().into());

                let sel_agent = s.selected_agent_id.and_then(|id| s.agents.get(id));
                app.set_inspector_bars_frame(render_inspector_bars(sel_agent, 310, 40));

                let aset_str = s
                .aset_time_s
                .map(|t| format!("{:.1}s", t))
                .unwrap_or_else(|| "Seg.".into());
                let rset_str = s
                .rset_time_s
                .map(|t| format!("{:.1}s", t))
                .unwrap_or_else(|| "--".into());
                app.set_aset_rset_text(format!("{} / {}", aset_str, rset_str).into());

                let img = render_simulation_frame(
                    &s.building,
                    &s.agents,
                    CANVAS_W,
                    CANVAS_H,
                    app.get_view_mode(),
                                                  app.get_show_graph(),
                                                  app.get_show_thermal(),
                                                  s.global_alarm, s.config.enable_social_bonds
                );
                app.set_canvas_frame(img);

                let chart_img = render_telemetry_chart(&s.telemetry_history, CHART_W, CHART_H);
                app.set_chart_frame(chart_img);
            }
        }
    };

    refresh_ui();

    let timer = Timer::default();
    {
        let s_rc = sim_rc.clone();
        let redraw = refresh_ui.clone();
        timer.start(TimerMode::Repeated, Duration::from_millis(16), move || {
            let mut s = s_rc.borrow_mut();
            if s.is_running {
                s.step(0.032);
                drop(s);
                redraw();
            }
        });
    }

    {
        let app_weak = app.as_weak();
        let s_rc = sim_rc.clone();
        app.on_toggle_running(move || {
            if let Some(app) = app_weak.upgrade() {
                let mut s = s_rc.borrow_mut();
                s.is_running = !s.is_running;
                app.set_is_running(s.is_running);
            }
        });
    }

    {
        let s_rc = sim_rc.clone();
        let redraw = refresh_ui.clone();
        app.on_next_warden(move || {
            let mut s = s_rc.borrow_mut();
            s.select_next_warden();
            drop(s);
            redraw();
        });
    }
    {
        let app_weak = app.as_weak();
        let s_rc = sim_rc.clone();
        let redraw = refresh_ui.clone();
        app.on_change_population(move || {
            if let Some(app) = app_weak.upgrade() {
                if let Ok(fresh) =
                    load_building_from_ascii("assets/buildings/pabellon_universitario.txt")
                    {
                        let mut s = s_rc.borrow_mut();
                        s.config.population_size = (app.get_population_slider().round() as usize).max(10);
                        s.reset(fresh);
                        app.set_total_agents(s.agents.len() as i32);
                        app.set_status_text(
                            format!("Población regenerada con {} agentes.", s.agents.len()).into(),
                        );
                        drop(s);
                        redraw();
                    }
            }
        });
    }

    {
        let app_weak = app.as_weak();
        let s_rc = sim_rc.clone();
        app.on_update_parameters(move || {
            if let Some(app) = app_weak.upgrade() {
                let mut s = s_rc.borrow_mut();
                let panic_pct = app.get_panic_slider() / 100.0;
                let trained_pct = app.get_trained_slider() / 100.0;
                s.config.initial_panic_pct = panic_pct;
                s.config.trained_pct = trained_pct;
                s.config.avoidance_weight = app.get_avoidance_slider();
                s.config.enable_communication = app.get_enable_comm();
                s.config.enable_social_bonds = app.get_enable_social();
                s.global_alarm = app.get_global_alarm();

                let total = s.agents.len();
                let panic_cut = ((total as f32) * panic_pct) as usize;
                let trained_cut = ((total as f32) * trained_pct) as usize;

                for (idx, agent) in s.agents.iter_mut().enumerate() {
                    agent.is_panic_prone = idx < panic_cut;
                    if agent.action_state != model::agent::AgentActionState::Normal {
                        if idx < panic_cut {
                            agent.panic_level = 7.5;
                            agent.personal_space_xi = -0.04;
                        } else {
                            agent.panic_level = 1.0;
                            agent.personal_space_xi = 0.04;
                        }
                    }
                    if idx < trained_cut {
                        agent.role = MacesRole::TrainedLeader;
                    }
                }
            }
        });
    }

    {
        let redraw = refresh_ui.clone();
        app.on_request_redraw(move || redraw());
    }

    {
        let app_weak = app.as_weak();
        let s_rc = sim_rc.clone();
        let redraw = refresh_ui.clone();
        app.on_load_preset(move |preset_idx| {
            if let Ok(fresh) = load_building_from_ascii("assets/buildings/pabellon_universitario.txt") {
                let mut s = s_rc.borrow_mut();
                let desc = s.apply_preset(preset_idx, fresh);
                if let Some(app) = app_weak.upgrade() {
                    app.set_enable_comm(s.config.enable_communication);
                    app.set_enable_social(s.config.enable_social_bonds);
                    app.set_panic_slider(s.config.initial_panic_pct * 100.0);
                    app.set_status_text(desc.into());
                }
                drop(s);
                redraw();
            }
        });
    }

    {
        let app_weak = app.as_weak();
        let s_rc = sim_rc.clone();
        let redraw = refresh_ui.clone();

        app.on_canvas_clicked(move |mx, my, w, h| {
            let Some(app) = app_weak.upgrade() else { return };
            let tool = app.get_active_tool();
            let view_mode = app.get_view_mode();
            let px = mx * (CANVAS_W as f32 / w.max(1.0));
            let py = my * (CANVAS_H as f32 / h.max(1.0));

            let mut s = s_rc.borrow_mut();

            // Si el usuario hace clic directamente sobre un agente con cualquier herramienta o con la herramienta 2, seleccionarlo en el Inspector
            if tool == 2 {
                if let Some(aid) = screen_to_agent(
                    &s.building,
                    &s.agents,
                    CANVAS_W as f32,
                    CANVAS_H as f32,
                    view_mode,
                    px,
                    py,
                ) {
                    s.selected_agent_id = Some(aid);
                    app.set_status_text(format!("Inspeccionando Agente #{:03}", aid).into());
                    drop(s);
                    redraw();
                    return;
                }
            }

            match tool {
                0 => {
                    if let Some(cid) =
                        screen_to_cell(&s.building, CANVAS_W as f32, CANVAS_H as f32, view_mode, px, py)
                        {
                            let cell = &mut s.building.cells[cid];
                            if cell.temperature_c < 100.0 {
                                cell.temperature_c = 165.0;
                                cell.fire_time_s = 2.0;
                                cell.smoke_opacity = 0.25;
                            } else if cell.temperature_c < FLASHOVER_TEMP_C {
                                cell.temperature_c = 550.0;
                                cell.fire_time_s = 25.0;
                                cell.smoke_opacity = 0.85;
                            } else {
                                cell.temperature_c = AMBIENT_TEMP_C;
                                cell.fire_time_s = 0.0;
                                cell.smoke_opacity = 0.0;
                            }
                            let temp_now = cell.temperature_c;
                            s.building.recompute_maze_flow();
                            app.set_status_text(
                                format!("Celda #{} -> {:.0} °C (Laberinto recalculado)", cid, temp_now)
                                    .into(),
                            );
                        }
                }
                1 => {
                    if let Some(pid) =
                        screen_to_portal(&s.building, CANVAS_W as f32, CANVAS_H as f32, view_mode, px, py)
                        {
                            s.building.portals[pid].is_closed_or_blocked =
                            !s.building.portals[pid].is_closed_or_blocked;
                            let blocked = s.building.portals[pid].is_closed_or_blocked;
                            let LoadedBuilding {
                                ref mut cells,
                                ref portals,
                                ..
                            } = s.building;
                            propagate_shortest_and_alternative_paths(cells, portals);
                            s.building.recompute_maze_flow();
                            app.set_status_text(
                                format!(
                                    "Puerta #{} {} -> El flujo del laberinto se redirigió a las otras puertas.",
                                    pid,
                                    if blocked { "CLAUSURADA" } else { "ABIERTA" }
                                )
                                    .into(),
                            );
                        }
                }
                _ => {
                    if let Some(cid) =
                        screen_to_cell(&s.building, CANVAS_W as f32, CANVAS_H as f32, view_mode, px, py)
                        {
                            let c = &s.building.cells[cid];
                            app.set_status_text(
                                format!(
                                    "[Piso {}] Celda #{} ({:?}, {:.0}m²) | T: {:.0}°C | Ocupantes: {}",
                                        c.floor,
                                        c.id,
                                        c.kind,
                                        c.area_m2,
                                        c.temperature_c,
                                        c.occupant_ids.len()
                                )
                                    .into(),
                            );
                        }
                }
            }
            drop(s);
            redraw();
        });
    }

    {
        let app_weak = app.as_weak();
        let s_rc = sim_rc.clone();
        let redraw = refresh_ui.clone();
        app.on_reset_building(move || {
            if let Ok(fresh) = load_building_from_ascii("assets/buildings/pabellon_universitario.txt") {
                let mut s = s_rc.borrow_mut();
                s.reset(fresh);
                if let Some(app) = app_weak.upgrade() {
                    app.set_status_text("Edificio reiniciado (22 °C). Rutina Normal.".into());
                }
                drop(s);
                redraw();
            }
        });
    }

    app.run().expect("Error en bucle de Slint");
}
