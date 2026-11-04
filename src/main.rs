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
    render_simulation_frame, render_telemetry_chart, screen_to_agent, screen_to_cell,
    screen_to_portal,
};
use slint::{Color as SlintColor, ModelRc, SharedString, Timer, TimerMode, VecModel};
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

slint::include_modules!();

const CANVAS_W: u32 = 960;
const CANVAS_H: u32 = 640;
const CHART_W: u32 = 340;
const CHART_H: u32 = 120;

fn update_floor_selector(app: &AppWindow, num_floors: usize) {
    let mut opts: Vec<SharedString> = Vec::with_capacity(num_floors + 1);
    opts.push(format!("Grilla: {} Pisos", num_floors).into());
    for f in 0..num_floors {
        opts.push(format!("Piso {}", f).into());
    }
    app.set_floor_options(ModelRc::new(VecModel::from(opts)));
}

fn main() {
    let building = load_building_from_ascii("assets/buildings/pabellon_universitario.txt")
    .expect("Fallo al construir el edificio");

    let config = SimulationConfig::default();
    let mut sim = SimulationEngine::new(building, config);
    sim.select_next_warden(); // Seleccionar el primer brigadista por defecto en el Inspector

    let app = AppWindow::new().expect("No se pudo crear la ventana de Slint");
    app.set_total_agents(sim.agents.len() as i32);
    update_floor_selector(&app, sim.building.floors.len());

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

                // Poblar el Inspector Visual con el Agente/Brigadista seleccionado
                let sel_id = s.selected_agent_id.unwrap_or(0);
                if let Some(a) = s.agents.get(sel_id) {
                    app.set_insp_title(
                        format!("AGENTE #{:03} — {}", a.id, a.role.name_es()).into(),
                    );
                    let fam_str = a
                    .group_id
                    .map(|g| format!("Familia #{}", g))
                    .unwrap_or_else(|| "Individual".into());
                    app.set_insp_badge_role(
                        format!("{} | {}", a.profile.name_es(), fam_str).into(),
                    );
                    app.set_insp_badge_state(
                        format!(
                            "{}  (Piso {} | T.Resp: {:.1}s)",
                                a.action_state.label_es(),
                                a.floor,
                                a.reaction_delay_assigned_s
                        )
                            .into(),
                    );

                    let accent = if a.role == MacesRole::TrainedLeader {
                        SlintColor::from_rgb_u8(148, 226, 213)
                    } else {
                        SlintColor::from_rgb_u8(137, 180, 250)
                    };
                    app.set_insp_accent(accent);

                    let health_r = (1.0
                    - a.fed_dose / a.profile.critical_fed_threshold().max(0.1))
                    .clamp(0.0, 1.0);
                    app.set_insp_health_ratio(health_r);
                    app.set_insp_health_text(format!("{:.0}%", health_r * 100.0).into());

                    let panic_r = (a.panic_level / 10.0).clamp(0.0, 1.0);
                    app.set_insp_panic_ratio(panic_r);
                    app.set_insp_panic_text(format!("{:.1}/10", a.panic_level).into());

                    let spd = a.avg_speed_m_s();
                    app.set_insp_speed_ratio((spd / 1.8).clamp(0.0, 1.0));
                    app.set_insp_speed_text(format!("{:.2} m/s", spd).into());

                    let tot_resc = a.rescued_count + a.helped_fallen_count;
                    app.set_insp_rescue_ratio((tot_resc as f32 / 10.0).clamp(0.0, 1.0));
                    app.set_insp_rescue_text(format!("{} pers.", tot_resc).into());
                }

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
                                                  s.global_alarm,
                                                  s.config.enable_social_bonds,
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
        let s_rc = sim_rc.clone();
        let redraw = refresh_ui.clone();
        app.on_next_agent(move || {
            let mut s = s_rc.borrow_mut();
            s.select_next_agent();
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
                        s.config.population_size =
                        (app.get_population_slider().round() as usize).max(10);
                        s.reset(fresh);
                        s.select_next_warden();
                        app.set_total_agents(s.agents.len() as i32);
                        update_floor_selector(&app, s.building.floors.len());
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
                s.config.enable_wardens = app.get_enable_wardens();
                s.config.enable_warden_line_formation = app.get_enable_warden_line();
                s.config.enable_social_bonds = app.get_enable_social();
                s.config.enable_helping_fallen = app.get_enable_helping();
                s.config.enable_panic_freeze = app.get_enable_panic_freeze();
                s.config.enable_communication = app.get_enable_comm();
                s.global_alarm = app.get_global_alarm();

                let total = s.agents.len();
                let panic_cut = ((total as f32) * panic_pct) as usize;
                let trained_cut = ((total as f32) * trained_pct) as usize;
                let wardens_enabled = s.config.enable_wardens;

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
                    if idx < trained_cut && wardens_enabled {
                        agent.role = MacesRole::TrainedLeader;
                    } else if agent.role == MacesRole::TrainedLeader && !wardens_enabled {
                        agent.role = MacesRole::UntrainedLeader;
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
            if let Ok(fresh) =
                load_building_from_ascii("assets/buildings/pabellon_universitario.txt")
                {
                    let mut s = s_rc.borrow_mut();
                    let desc = s.apply_preset(preset_idx, fresh);
                    s.select_next_warden();
                    if let Some(app) = app_weak.upgrade() {
                        app.set_enable_comm(s.config.enable_communication);
                        app.set_enable_social(s.config.enable_social_bonds);
                        app.set_enable_wardens(s.config.enable_wardens);
                        app.set_panic_slider(s.config.initial_panic_pct * 100.0);
                        update_floor_selector(&app, s.building.floors.len());
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

            if let Some(aid) = screen_to_agent(
                &s.building,
                &s.agents,
                CANVAS_W as f32,
                CANVAS_H as f32,
                view_mode,
                px,
                py,
            ) {
                if tool == 2 {
                    s.selected_agent_id = Some(aid);
                    app.set_status_text(format!("Inspeccionando Agente #{:03}", aid).into());
                    drop(s);
                    redraw();
                    return;
                }
            }

            match tool {
                0 => {
                    if let Some(cid) = screen_to_cell(
                        &s.building,
                        CANVAS_W as f32,
                        CANVAS_H as f32,
                        view_mode,
                        px,
                        py,
                    ) {
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
                            format!("Celda #{} -> {:.0} °C (Laberinto actualizado)", cid, temp_now)
                                .into(),
                        );
                    }
                }
                1 => {
                    if let Some(pid) = screen_to_portal(
                        &s.building,
                        CANVAS_W as f32,
                        CANVAS_H as f32,
                        view_mode,
                        px,
                        py,
                    ) {
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
                                "Puerta #{} {} -> Flujo del laberinto redirigido.",
                                pid,
                                if blocked { "CLAUSURADA" } else { "ABIERTA" }
                            )
                                .into(),
                        );
                    }
                }
                _ => {
                    if let Some(cid) = screen_to_cell(
                        &s.building,
                        CANVAS_W as f32,
                        CANVAS_H as f32,
                        view_mode,
                        px,
                        py,
                    ) {
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
            if let Ok(fresh) =
                load_building_from_ascii("assets/buildings/pabellon_universitario.txt")
                {
                    let mut s = s_rc.borrow_mut();
                    s.reset(fresh);
                    s.select_next_warden();
                    if let Some(app) = app_weak.upgrade() {
                        update_floor_selector(&app, s.building.floors.len());
                        app.set_status_text("Edificio reiniciado (22 °C). Rutina Normal.".into());
                    }
                    drop(s);
                    redraw();
                }
        });
    }

    app.run().expect("Error en bucle de Slint");
}
