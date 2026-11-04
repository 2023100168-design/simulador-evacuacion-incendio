#![cfg_attr(windows, windows_subsystem = "windows")]
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
    render_agent_pictogram, render_simulation_frame, render_telemetry_chart, screen_to_agent,
    screen_to_cell, screen_to_floor_tile, screen_to_portal,
};
use slint::{Color as SlintColor, Model, ModelRc, SharedString, Timer, TimerMode, VecModel};
use std::cell::RefCell;
use std::fs;
use std::rc::Rc;
use std::time::Duration;

slint::include_modules!();

const CANVAS_W: u32 = 980;
const CANVAS_H: u32 = 660;
const CHART_W: u32 = 320;
const CHART_H: u32 = 110;

fn scan_building_files() -> Vec<String> {
    let mut files = Vec::new();
    if let Ok(entries) = fs::read_dir("assets/buildings") {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) == Some("txt") {
                if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                    files.push(name.to_string());
                }
            }
        }
    }
    files.sort();
    if files.is_empty() {
        files.push("pabellon_universitario.txt".to_string());
    }
    files
}

fn update_floor_selector(app: &AppWindow, num_floors: usize) {
    let mut opts: Vec<SharedString> = Vec::with_capacity(num_floors + 1);
    opts.push(format!("Grilla: {} Pisos", num_floors).into());
    for f in 0..num_floors {
        opts.push(format!("Piso {}", f).into());
    }
    app.set_floor_options(ModelRc::new(VecModel::from(opts)));
}

fn main() {
    let available_files = scan_building_files();
    let initial_path = format!("assets/buildings/{}", available_files[0]);

    let building =
    load_building_from_ascii(&initial_path).expect("Fallo al construir el edificio inicial");

    let config = SimulationConfig::default();
    let mut sim = SimulationEngine::new(building, config);
    sim.select_next_warden();

    let app = AppWindow::new().expect("No se pudo crear la ventana de Slint");
    let slint_files: Vec<SharedString> = available_files.iter().map(|s| s.into()).collect();
    app.set_building_files(ModelRc::new(VecModel::from(slint_files)));
    app.set_total_agents(sim.agents.len() as i32);
    update_floor_selector(&app, sim.building.floors.len());

    let current_file_rc = Rc::new(RefCell::new(initial_path));
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
                        SlintColor::from_rgb_u8(20, 184, 166)
                    } else {
                        SlintColor::from_rgb_u8(59, 130, 246)
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
                app.set_pictogram_frame(render_agent_pictogram(s.agents.get(sel_id), 68, 92));

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
                                                  app.get_view_3d(),
                                                  app.get_camera_yaw(),
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

    // Selector e importador dinámico de archivos .txt en assets/buildings/
    {
        let app_weak = app.as_weak();
        app.on_refresh_building_list(move || {
            if let Some(app) = app_weak.upgrade() {
                let files = scan_building_files();
                let slint_files: Vec<SharedString> = files.iter().map(|s| s.into()).collect();
                app.set_building_files(ModelRc::new(VecModel::from(slint_files)));
                app.set_status_text(
                    format!("Carpeta assets/buildings/ escaneada ({} planos).", files.len()).into(),
                );
            }
        });
    }

    {
        let app_weak = app.as_weak();
        let s_rc = sim_rc.clone();
        let path_rc = current_file_rc.clone();
        let redraw = refresh_ui.clone();
        app.on_select_building_file(move |idx| {
            let Some(app) = app_weak.upgrade() else { return };
            let model = app.get_building_files();
            if let Some(fname) = model.row_data(idx as usize) {
                let full_path = format!("assets/buildings/{}", fname);
                match load_building_from_ascii(&full_path) {
                    Ok(fresh) => {
                        *path_rc.borrow_mut() = full_path;
                        let mut s = s_rc.borrow_mut();
                        s.is_running = false;
                        app.set_is_running(false);
                        app.set_view_mode(0);
                        s.reset(fresh);
                        s.select_next_warden();
                        update_floor_selector(&app, s.building.floors.len());
                        app.set_status_text(
                            format!(
                                "Plano '{}' cargado ({} pisos, {} celdas).",
                                    fname,
                                    s.building.floors.len(),
                                    s.building.cells.len()
                            )
                                .into(),
                        );
                        drop(s);
                        redraw();
                    }
                    Err(err) => {
                        app.set_status_text(format!("Error al cargar plano: {}", err).into());
                    }
                }
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
        let path_rc = current_file_rc.clone();
        let redraw = refresh_ui.clone();
        app.on_change_population(move || {
            if let Some(app) = app_weak.upgrade() {
                let current_path = path_rc.borrow().clone();
                if let Ok(fresh) = load_building_from_ascii(&current_path) {
                    let mut s = s_rc.borrow_mut();
                    s.config.population_size =
                    (app.get_population_slider().round() as usize).max(10);
                    s.reset(fresh);
                    s.select_next_warden();
                    app.set_total_agents(s.agents.len() as i32);
                    update_floor_selector(&app, s.building.floors.len());
                    app.set_status_text(
                        format!("Aforo actualizado a {} ocupantes.", s.agents.len()).into(),
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
        let path_rc = current_file_rc.clone();
        let redraw = refresh_ui.clone();
        app.on_load_preset(move |preset_idx| {
            let current_path = path_rc.borrow().clone();
            if let Ok(fresh) = load_building_from_ascii(&current_path) {
                let mut s = s_rc.borrow_mut();
                let desc = s.apply_preset(preset_idx, fresh);
                // Encender también las baldosas correspondientes al preset para la onda 2D
                for cid in 0..s.building.cells.len() {
                    let t = s.building.cells[cid].temperature_c;
                    if t > AMBIENT_TEMP_C + 10.0 {
                        let f = s.building.cells[cid].floor;
                        let (cx, cy) = s.building.cells[cid].center();
                        let (r, c) = s.building.tile_coords_at(cx, cy);
                        s.building.ignite_at_tile(f, r, c, t);
                    }
                }
                s.building.recompute_maze_flow();
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
            let is_3d = app.get_view_3d();
            let yaw = app.get_camera_yaw();
            let px = mx * (CANVAS_W as f32 / w.max(1.0));
            let py = my * (CANVAS_H as f32 / h.max(1.0));

            let mut s = s_rc.borrow_mut();

            if tool == 2 {
                if let Some(aid) = screen_to_agent(
                    &s.building,
                    &s.agents,
                    CANVAS_W as f32,
                    CANVAS_H as f32,
                    view_mode,
                    is_3d,
                    yaw,
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
                    // Inyección puntual de fuego en la baldosa exacta (f, r, c) clicada
                    if let Some((f, r, c)) = screen_to_floor_tile(
                        &s.building,
                        CANVAS_W as f32,
                        CANVAS_H as f32,
                        view_mode,
                        is_3d,
                        yaw,
                        px,
                        py,
                    ) {
                        let curr_t = s.building.tile_temp[f][r][c];
                        let new_t = if curr_t < 100.0 {
                            240.0 // Conato puntual
                        } else if curr_t < FLASHOVER_TEMP_C {
                            620.0 // Epicentro de Flashover en onda
                        } else {
                            AMBIENT_TEMP_C
                        };
                        s.building.ignite_at_tile(f, r, c, new_t);
                        s.building.recompute_maze_flow();
                        app.set_status_text(
                            format!(
                                "Piso {} Baldosa ({}, {}) -> {:.0} °C (Onda térmica propagándose por metro cuadrado)",
                                    f, r, c, new_t
                            )
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
                        is_3d,
                        yaw,
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
                                "Puerta #{} {} -> Rutas actualizadas.",
                                pid,
                                if blocked { "CLAUSURADA" } else { "ABIERTA" }
                            )
                                .into(),
                        );
                    }
                }
                _ => {
                    if let Some((f, r, c)) = screen_to_floor_tile(
                        &s.building,
                        CANVAS_W as f32,
                        CANVAS_H as f32,
                        view_mode,
                        is_3d,
                        yaw,
                        px,
                        py,
                    ) {
                        let cid = s.building.cell_grid[f][r][c].unwrap_or(0);
                        let t_tile = s.building.tile_temp[f][r][c];
                        let smk_tile = s.building.tile_smoke[f][r][c];
                        app.set_status_text(
                            format!(
                                "[Piso {} | Baldosa ({},{})] Celda #{} | T local: {:.0}°C | Humo local: {:.0}%",
                                    f,
                                    r,
                                    c,
                                    cid,
                                    t_tile,
                                    smk_tile * 100.0
                            )
                                .into(),
                        );
                    } else if let Some(cid) = screen_to_cell(
                        &s.building,
                        CANVAS_W as f32,
                        CANVAS_H as f32,
                        view_mode,
                        is_3d,
                        yaw,
                        px,
                        py,
                    ) {
                        let c = &s.building.cells[cid];
                        app.set_status_text(
                            format!(
                                "[Piso {}] Celda #{} ({:?}, {:.0}m²) | Ocupantes: {}",
                                    c.floor,
                                    c.id,
                                    c.kind,
                                    c.area_m2,
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
        let path_rc = current_file_rc.clone();
        let redraw = refresh_ui.clone();
        app.on_reset_building(move || {
            let current_path = path_rc.borrow().clone();
            if let Ok(fresh) = load_building_from_ascii(&current_path) {
                let mut s = s_rc.borrow_mut();
                s.reset(fresh);
                s.select_next_warden();
                if let Some(app) = app_weak.upgrade() {
                    update_floor_selector(&app, s.building.floors.len());
                    app.set_status_text("Simulación reiniciada (22 °C).".into());
                }
                drop(s);
                redraw();
            }
        });
    }

    app.run().expect("Error en bucle de Slint");
}
