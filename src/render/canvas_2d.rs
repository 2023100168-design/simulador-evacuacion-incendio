use crate::building_loader::LoadedBuilding;
use crate::config::{AMBIENT_TEMP_C, FLASHOVER_TEMP_C};
use crate::engine::TelemetryPoint;
use crate::model::agent::{Agent, AgentActionState, DemographicProfile, MacesRole};
use crate::model::topology::CellKind;
use slint::{Image, Rgba8Pixel, SharedPixelBuffer};
use tiny_skia::{Color, FillRule, LineCap, Paint, PathBuilder, PixmapMut, Rect, Stroke, Transform};

pub struct FloorViewport {
    pub floor_idx: usize,
    pub offset_x: f32,
    pub offset_y: f32,
    pub scale: f32,
}

pub fn compute_viewports(
    building: &LoadedBuilding,
    canvas_w: f32,
    canvas_h: f32,
    view_mode: i32,
) -> Vec<FloorViewport> {
    let num_floors = building.floors.len().max(1);
    let gw = building.grid_width_m.max(1.0);
    let gh = building.grid_height_m.max(1.0);
    let pad = 18.0;

    if view_mode > 0 && (view_mode as usize) <= num_floors {
        let f = (view_mode - 1) as usize;
        let scale_x = (canvas_w - 2.0 * pad) / gw;
        let scale_y = (canvas_h - 2.0 * pad) / gh;
        let scale = scale_x.min(scale_y).max(4.0);
        let offset_x = (canvas_w - gw * scale) * 0.5;
        let offset_y = (canvas_h - gh * scale) * 0.5;
        vec![FloorViewport {
            floor_idx: f,
            offset_x,
            offset_y,
            scale,
        }]
    } else {
        let col_w = canvas_w / (num_floors as f32);
        let scale_x = (col_w - 2.0 * pad) / gw;
        let scale_y = (canvas_h - 2.0 * pad) / gh;
        let scale = scale_x.min(scale_y).max(2.0);

        (0..num_floors)
        .map(|f| {
            let col_start = (f as f32) * col_w;
            let offset_x = col_start + (col_w - gw * scale) * 0.5;
            let offset_y = (canvas_h - gh * scale) * 0.5;
            FloorViewport {
                floor_idx: f,
                offset_x,
                offset_y,
                scale,
            }
        })
        .collect()
    }
}

pub fn screen_to_cell(
    building: &LoadedBuilding,
    canvas_w: f32,
    canvas_h: f32,
    view_mode: i32,
    px: f32,
    py: f32,
) -> Option<usize> {
    let viewports = compute_viewports(building, canvas_w, canvas_h, view_mode);
    for vp in viewports {
        let mx = (px - vp.offset_x) / vp.scale;
        let my = (py - vp.offset_y) / vp.scale;
        if let Some(cid) = building.cell_at(vp.floor_idx, mx, my) {
            return Some(cid);
        }
    }
    None
}

pub fn screen_to_portal(
    building: &LoadedBuilding,
    canvas_w: f32,
    canvas_h: f32,
    view_mode: i32,
    px: f32,
    py: f32,
) -> Option<usize> {
    let viewports = compute_viewports(building, canvas_w, canvas_h, view_mode);
    for vp in viewports {
        let mx = (px - vp.offset_x) / vp.scale;
        let my = (py - vp.offset_y) / vp.scale;
        if mx >= 0.0 && mx <= building.grid_width_m && my >= 0.0 && my <= building.grid_height_m {
            let mut best: Option<(usize, f32)> = None;
            for portal in &building.portals {
                if portal.floor_a == vp.floor_idx && portal.floor_b == vp.floor_idx {
                    let dist = (mx - portal.pos_a.0).hypot(my - portal.pos_a.1);
                    if dist < 1.2 && best.map_or(true, |(_, d)| dist < d) {
                        best = Some((portal.id, dist));
                    }
                }
            }
            if let Some((pid, _)) = best {
                return Some(pid);
            }
        }
    }
    None
}
/// Permite seleccionar cualquier agente o brigadista haciendo clic sobre él en el mapa
pub fn screen_to_agent(
    building: &LoadedBuilding,
    agents: &[Agent],
    canvas_w: f32,
    canvas_h: f32,
    view_mode: i32,
    px: f32,
    py: f32,
) -> Option<usize> {
    let viewports = compute_viewports(building, canvas_w, canvas_h, view_mode);
    for vp in viewports {
        let mx = (px - vp.offset_x) / vp.scale;
        let my = (py - vp.offset_y) / vp.scale;
        let mut best: Option<(usize, f32)> = None;
        for a in agents.iter().filter(|a| {
            a.floor == vp.floor_idx && a.action_state != AgentActionState::Evacuated
        }) {
            let d = (mx - a.x).hypot(my - a.y);
            if d < 1.1 && best.map_or(true, |(_, bd)| d < bd) {
                best = Some((a.id, d));
            }
        }
        if let Some((aid, _)) = best {
            return Some(aid);
        }
    }
    None
}

pub fn render_simulation_frame(
    building: &LoadedBuilding,
    agents: &[Agent],
    width: u32,
    height: u32,
    view_mode: i32,
    show_graph: bool,
    show_thermal: bool,
    global_alarm: bool,
    show_social_bonds: bool,
) -> Image {
    let mut pixel_buffer = SharedPixelBuffer::<Rgba8Pixel>::new(width, height);
    let mut pixmap =
    PixmapMut::from_bytes(pixel_buffer.make_mut_bytes(), width, height).expect("Pixmap inválido");

    pixmap.fill(Color::from_rgba8(17, 17, 27, 255));

    let viewports = compute_viewports(building, width as f32, height as f32, view_mode);

    for vp in &viewports {
        let to_sx = |mx: f32| vp.offset_x + mx * vp.scale;
        let to_sy = |my: f32| vp.offset_y + my * vp.scale;

        // 1. Sombra exterior y placa base del piso
        let fw = building.grid_width_m * vp.scale;
        let fh = building.grid_height_m * vp.scale;
        if let Some(shadow_rect) = Rect::from_xywh(vp.offset_x + 4.0, vp.offset_y + 4.0, fw, fh) {
            let mut sp = Paint::default();
            sp.set_color_rgba8(10, 10, 18, 200);
            pixmap.fill_rect(shadow_rect, &sp, Transform::identity(), None);
        }
        if let Some(bg_rect) = Rect::from_xywh(vp.offset_x, vp.offset_y, fw, fh) {
            let mut bg_paint = Paint::default();
            bg_paint.set_color_rgba8(24, 24, 37, 255);
            pixmap.fill_rect(bg_rect, &bg_paint, Transform::identity(), None);
        }

        // 2. Celdas poligonales (L, U, T) pintadas baldosa por baldosa
        for cell in building.cells.iter().filter(|c| c.floor == vp.floor_idx) {
            let mut fill_paint = Paint::default();
            match cell.kind {
                CellKind::Room => fill_paint.set_color_rgba8(30, 30, 46, 255),
                CellKind::Corridor => fill_paint.set_color_rgba8(43, 45, 62, 255),
                CellKind::Stairwell => fill_paint.set_color_rgba8(55, 62, 92, 255),
                CellKind::Elevator { .. } => fill_paint.set_color_rgba8(65, 67, 88, 255),
                CellKind::Exit => fill_paint.set_color_rgba8(27, 67, 50, 255),
            }

            let ratio_heat = ((cell.temperature_c - AMBIENT_TEMP_C)
            / (FLASHOVER_TEMP_C - AMBIENT_TEMP_C))
            .clamp(0.0, 1.0);

            for &(tr, tc) in &cell.tiles {
                let tx = to_sx(tc as f32);
                let ty = to_sy(tr as f32);
                let tw = vp.scale + 0.6;
                if let Some(t_rect) = Rect::from_xywh(tx, ty, tw, tw) {
                    pixmap.fill_rect(t_rect, &fill_paint, Transform::identity(), None);

                    if show_thermal && cell.temperature_c > AMBIENT_TEMP_C + 2.0 {
                        let mut heat_paint = Paint::default();
                        if cell.temperature_c >= FLASHOVER_TEMP_C {
                            heat_paint.set_color_rgba8(243, 139, 168, 205);
                        } else {
                            let alpha = (65.0 + ratio_heat * 140.0) as u8;
                            heat_paint.set_color_rgba8(250, 179, 135, alpha);
                        }
                        pixmap.fill_rect(t_rect, &heat_paint, Transform::identity(), None);
                    }

                    if show_thermal && cell.smoke_opacity > 0.02 {
                        let mut smoke_paint = Paint::default();
                        let alpha = (cell.smoke_opacity.clamp(0.0, 1.0) * 160.0) as u8;
                        smoke_paint.set_color_rgba8(166, 173, 200, alpha);
                        pixmap.fill_rect(t_rect, &smoke_paint, Transform::identity(), None);
                    }
                }
            }

            let rx = to_sx(cell.min_x);
            let ry = to_sy(cell.min_y);
            let rw = (cell.max_x - cell.min_x) * vp.scale;
            let rh = (cell.max_y - cell.min_y) * vp.scale;

            if cell.kind == CellKind::Stairwell {
                let steps = 5;
                for s in 1..steps {
                    let sy = ry + (rh / steps as f32) * (s as f32);
                    draw_line(&mut pixmap, rx + 2.0, sy, rx + rw - 2.0, sy, (137, 180, 250, 170), 1.4);
                }
            }

            for w in &cell.walls {
                draw_line(
                    &mut pixmap,
                    to_sx(w.x1),
                          to_sy(w.y1),
                          to_sx(w.x2),
                          to_sy(w.y2),
                          (186, 194, 222, 255),
                          2.0,
                );
            }

            for obs in &cell.obstacles {
                let ox = to_sx(obs.x);
                let oy = to_sy(obs.y);
                let orad = (obs.radius * vp.scale).max(3.5);
                draw_circle(&mut pixmap, ox, oy, orad, (88, 91, 112, 255));
                draw_circle_outline(&mut pixmap, ox, oy, orad, (205, 214, 244, 220), 1.2);
            }
        }

        // 3. Estaciones de Alarma y Extintores
        for alarm in building.alarms.iter().filter(|a| a.floor == vp.floor_idx) {
            let ax = to_sx(alarm.x);
            let ay = to_sy(alarm.y);
            let active = alarm.is_triggered || global_alarm;
            let color = if active {
                (243, 139, 168, 255)
            } else {
                (249, 226, 175, 255)
            };
            draw_circle(&mut pixmap, ax, ay, (0.34 * vp.scale).max(4.0), color);
            if active {
                draw_circle_outline(&mut pixmap, ax, ay, (0.85 * vp.scale).max(8.0), (243, 139, 168, 190), 1.6);
                draw_circle_outline(&mut pixmap, ax, ay, (1.35 * vp.scale).max(12.0), (243, 139, 168, 100), 1.2);
            }
        }

        for ext in building
            .extinguishers
            .iter()
            .filter(|e| e.floor == vp.floor_idx && e.is_available)
            {
                let ex = to_sx(ext.x);
                let ey = to_sy(ext.y);
                let er = (0.32 * vp.scale).max(3.8);
                draw_circle(&mut pixmap, ex, ey, er, (148, 226, 213, 255));
                draw_circle_outline(&mut pixmap, ex, ey, er + 1.5, (137, 220, 235, 180), 1.2);
            }

            // 4. Portales (Puertas y su desgaste térmico R_ij)
            for portal in building
                .portals
                .iter()
                .filter(|p| p.floor_a == vp.floor_idx && p.floor_b == vp.floor_idx)
                {
                    let px = to_sx(portal.pos_a.0);
                    let py = to_sy(portal.pos_a.1);
                    let half_w = (portal.width_m * 0.45 * vp.scale).max(4.5);

                    let is_horizontal_wall = (portal.attractor_a.1 - portal.attractor_b.1).abs() > 0.1;
                    let (x1, y1, x2, y2) = if is_horizontal_wall {
                        (px - half_w, py, px + half_w, py)
                    } else {
                        (px, py - half_w, px, py + half_w)
                    };

                    // Hueco oscuro del umbral de la puerta
                    draw_line(&mut pixmap, x1, y1, x2, y2, (24, 24, 37, 255), 5.0);

                    let hp_ratio = if portal.max_resistance_s > 0.0 {
                        (portal.fire_resistance_s / portal.max_resistance_s).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };

                    let color = if portal.is_closed_or_blocked {
                        (243, 139, 168, 255)
                    } else if portal.max_resistance_s >= 100.0 {
                        (250, 179, 135, (110.0 + hp_ratio * 145.0) as u8)
                    } else {
                        (166, 227, 161, (110.0 + hp_ratio * 145.0) as u8)
                    };

                    draw_line(&mut pixmap, x1, y1, x2, y2, color, 3.6);
                }

                // 5. Superposición del Grafo y Atractores Dobles
                if show_graph {
                    for portal in building
                        .portals
                        .iter()
                        .filter(|p| p.floor_a == vp.floor_idx && p.floor_b == vp.floor_idx)
                        {
                            let c_a = building.cells[portal.cell_a].center();
                            let c_b = building.cells[portal.cell_b].center();
                            let line_col = if portal.is_closed_or_blocked {
                                (243, 139, 168, 95)
                            } else {
                                (203, 166, 247, 125)
                            };
                            draw_line(
                                &mut pixmap,
                                to_sx(c_a.0),
                                      to_sy(c_a.1),
                                      to_sx(c_b.0),
                                      to_sy(c_b.1),
                                      line_col,
                                      1.5,
                            );

                            draw_circle(
                                &mut pixmap,
                                to_sx(portal.attractor_a.0),
                                        to_sy(portal.attractor_a.1),
                                        2.5,
                                        (137, 220, 235, 220),
                            );
                            draw_circle(
                                &mut pixmap,
                                to_sx(portal.attractor_b.0),
                                        to_sy(portal.attractor_b.1),
                                        2.5,
                                        (137, 220, 235, 220),
                            );
                        }

                        for cell in building.cells.iter().filter(|c| c.floor == vp.floor_idx) {
                            let (cx, cy) = cell.center();
                            draw_circle(&mut pixmap, to_sx(cx), to_sy(cy), 4.0, (203, 166, 247, 240));

                            if let Some(&(first_pid, _)) = cell.routes.shortest_path.first() {
                                let target_portal = &building.portals[first_pid];
                                let target_pos = if target_portal.cell_a == cell.id {
                                    target_portal.pos_a
                                } else {
                                    target_portal.pos_b
                                };
                                draw_line(
                                    &mut pixmap,
                                    to_sx(cx),
                                          to_sy(cy),
                                          to_sx(target_pos.0),
                                          to_sy(target_pos.1),
                                          (166, 227, 161, 195),
                                          2.0,
                                );
                            }
                        }
                }

                // 6. Capa de Agentes con Vínculos Sociales, Brigadistas y Vector POV
                for agent in agents
                    .iter()
                    .filter(|a| a.floor == vp.floor_idx && a.action_state != AgentActionState::Evacuated)
                    {
                        let ax = to_sx(agent.x);
                        let ay = to_sy(agent.y);
                        let r = (agent.profile.body_radius() * vp.scale).max(3.4);

                        if agent.is_fallen {
                            if let Some(body_rect) =
                                Rect::from_xywh(ax - r * 1.45, ay - r * 0.75, r * 2.9, r * 1.5)
                                {
                                    let mut fallen_paint = Paint::default();
                                    if agent.action_state == AgentActionState::Incapacitated {
                                        fallen_paint.set_color_rgba8(108, 112, 134, 255);
                                    } else {
                                        fallen_paint.set_color_rgba8(250, 179, 135, 255);
                                    }
                                    pixmap.fill_rect(body_rect, &fallen_paint, Transform::identity(), None);
                                }
                                continue;
                        }

                        // Enlace visual cuando busca a un familiar/amigo (SeekingBonded) o sigue a un brigadista (FollowingWarden)
                        // Enlaces Lavanda de Vínculos Familiares (visibles cuando el CheckBox está activo)
                        if show_social_bonds && agent.action_state != AgentActionState::Normal {
                            if let Some(lid) = agent.group_leader_id {
                                if lid != agent.id {
                                    let leader = &agents[lid];
                                    if leader.floor == vp.floor_idx
                                        && leader.action_state != AgentActionState::Evacuated
                                        {
                                            let alpha = if matches!(
                                                agent.action_state,
                                                AgentActionState::SeekingFamilyInCorridor { .. }
                                            ) {
                                                210
                                            } else {
                                                115
                                            };
                                            draw_line(
                                                &mut pixmap,
                                                ax,
                                                ay,
                                                to_sx(leader.x),
                                                      to_sy(leader.y),
                                                      (203, 166, 247, alpha),
                                                      1.6,
                                            );
                                        }
                                }
                            }
                        }

                        // Enlace Celeste cuando sigue directamente a un Brigadista
                        if let AgentActionState::FollowingWarden { warden_id } = agent.action_state {
                            let w = &agents[warden_id];
                            if w.floor == vp.floor_idx {
                                draw_line(
                                    &mut pixmap,
                                    ax,
                                    ay,
                                    to_sx(w.x),
                                          to_sy(w.y),
                                          (137, 220, 235, 220),
                                          2.0,
                                );
                            }
                        }

                        // Anillo Magenta pulsante cuando el agente entró en Shock o Desorientación por Pánico
                        if matches!(
                            agent.action_state,
                            AgentActionState::PanickedFrozen { .. }
                            | AgentActionState::PanickedWandering { .. }
                        ) {
                            draw_circle_outline(&mut pixmap, ax, ay, r + 3.0, (243, 139, 168, 240), 1.8);
                        }

                        // Vector de dirección de mirada (POV)
                        let v_mag = agent.vx.hypot(agent.vy);
                        if v_mag > 0.04 {
                            let look_x = ax + (agent.vx / v_mag) * (r * 1.75);
                            let look_y = ay + (agent.vy / v_mag) * (r * 1.75);
                            draw_line(
                                &mut pixmap,
                                ax,
                                ay,
                                look_x,
                                look_y,
                                (205, 214, 244, 210),
                                      1.5,
                            );
                        }

                        let body_color = if agent.role == MacesRole::TrainedLeader {
                            (148, 226, 213, 255) // Cian/Teal brillante: Brigadista (Warden)
                        } else if matches!(
                            agent.action_state,
                            AgentActionState::FetchingExtinguisher { .. }
                            | AgentActionState::FightingFire { .. }
                            | AgentActionState::HelpingFallen { .. }
                        ) {
                            (166, 227, 161, 255) // Verde: Rescatista / Extintor
                        } else {
                            match agent.profile {
                                DemographicProfile::Adult | DemographicProfile::YoungAdult => {
                                    (137, 180, 250, 255)
                                }
                                DemographicProfile::Child => (249, 226, 175, 255),
                                DemographicProfile::Elderly | DemographicProfile::Impaired => {
                                    (203, 166, 247, 255)
                                }
                            }
                        };

                        draw_circle(&mut pixmap, ax, ay, r, body_color);

                        // Halo distintivo para los Brigadistas (TrainedLeader)
                        if agent.role == MacesRole::TrainedLeader {
                            draw_circle_outline(&mut pixmap, ax, ay, r + 1.6, (255, 255, 255, 235), 1.3);
                        }

                        if agent.panic_level >= 5.0 && agent.action_state != AgentActionState::Normal {
                            draw_circle(&mut pixmap, ax, ay, (r * 0.55).max(1.8), (243, 139, 168, 255));
                        }

                        if matches!(agent.action_state, AgentActionState::Reacting { .. }) {
                            draw_circle_outline(&mut pixmap, ax, ay, r + 2.6, (249, 226, 175, 240), 1.6);
                        }

                        if let AgentActionState::HelpingFallen {
                            target_agent_id, ..
                        } = agent.action_state
                        {
                            let target = &agents[target_agent_id];
                            draw_line(
                                &mut pixmap,
                                ax,
                                ay,
                                to_sx(target.x),
                                      to_sy(target.y),
                                      (166, 227, 161, 240),
                                      2.4,
                            );
                            draw_circle_outline(
                                &mut pixmap,
                                to_sx(target.x),
                                                to_sy(target.y),
                                                r + 3.5,
                                                (166, 227, 161, 220),
                                                1.6,
                            );
                        }

                        if let AgentActionState::FightingFire { fire_cell, .. } = agent.action_state {
                            let (fx, fy) = building.cells[fire_cell].center();
                            draw_line(
                                &mut pixmap,
                                ax,
                                ay,
                                to_sx(fx),
                                      to_sy(fy),
                                      (148, 226, 213, 235),
                                      3.2,
                            );
                        }
                    }
    }

    Image::from_rgba8(pixel_buffer)
}

/// Panel Gráfico de Telemetría Avanzada: Curvas Suavizadas + Áreas + Barras Laterales de Distribución
pub fn render_telemetry_chart(history: &[TelemetryPoint], width: u32, height: u32) -> Image {
    let mut pixel_buffer = SharedPixelBuffer::<Rgba8Pixel>::new(width, height);
    let mut pixmap =
    PixmapMut::from_bytes(pixel_buffer.make_mut_bytes(), width, height).expect("Pixmap inválido");

    pixmap.fill(Color::from_rgba8(17, 17, 27, 255));

    let pad_l = 12.0_f32;
    let bar_zone_w = 34.0_f32; // Zona derecha para barras verticales en vivo
    let pad_r = 12.0_f32 + bar_zone_w;
    let pad_t = 12.0_f32;
    let pad_b = 12.0_f32;
    let w = (width as f32 - pad_l - pad_r).max(20.0);
    let h = (height as f32 - pad_t - pad_b).max(20.0);
    let base_y = pad_t + h;

    // Marco interior del área de ploteo
    draw_line(&mut pixmap, pad_l, pad_t, pad_l, base_y, (88, 91, 112, 220), 1.4);
    draw_line(&mut pixmap, pad_l, base_y, pad_l + w, base_y, (88, 91, 112, 220), 1.4);

    // Líneas guía horizontales (25%, 50%, 75%, 100%)
    for level in [0.25_f32, 0.50, 0.75, 1.0] {
        let y = pad_t + h * (1.0 - level);
        draw_line(&mut pixmap, pad_l, y, pad_l + w, y, (49, 50, 68, 190), 1.0);
    }
    for v_step in [0.20_f32, 0.40, 0.60, 0.80, 1.0] {
        let x = pad_l + w * v_step;
        draw_line(&mut pixmap, x, pad_t, x, base_y, (49, 50, 68, 110), 1.0);
    }

    // Barras verticales de estado instantáneo a la derecha del gráfico
    if let Some(last) = history.last() {
        let bx_start = pad_l + w + 8.0;
        let bar_w = 7.0_f32;
        let bars = [
            (last.evacuated_pct, (166, 227, 161, 240)),
            (last.alerting_pct, (137, 180, 250, 240)),
            (last.incapacitated_pct, (243, 139, 168, 240)),
        ];
        for (idx, (pct, col)) in bars.iter().enumerate() {
            let bx = bx_start + (idx as f32) * (bar_w + 3.5);
            if let Some(bg_r) = Rect::from_xywh(bx, pad_t, bar_w, h) {
                let mut p = Paint::default();
                p.set_color_rgba8(30, 30, 46, 255);
                pixmap.fill_rect(bg_r, &p, Transform::identity(), None);
            }
            let bh = (h * (pct / 100.0).clamp(0.0, 1.0)).max(1.5);
            if let Some(fg_r) = Rect::from_xywh(bx, base_y - bh, bar_w, bh) {
                let mut p = Paint::default();
                p.set_color_rgba8(col.0, col.1, col.2, col.3);
                pixmap.fill_rect(fg_r, &p, Transform::identity(), None);
            }
        }
    }

    if history.len() < 2 {
        return Image::from_rgba8(pixel_buffer);
    }

    let max_t = history.last().map(|p| p.time_s).unwrap_or(20.0).max(12.0);

    // 1. Área sombreada Azul (% En Alerta)
    let mut pb_alert = PathBuilder::new();
    pb_alert.move_to(pad_l, base_y);
    for pt in history {
        let x = pad_l + (pt.time_s / max_t) * w;
        let y = pad_t + h * (1.0 - (pt.alerting_pct / 100.0).clamp(0.0, 1.0));
        pb_alert.line_to(x, y);
    }
    if let Some(last) = history.last() {
        pb_alert.line_to(pad_l + (last.time_s / max_t) * w, base_y);
    }
    pb_alert.close();
    if let Some(path) = pb_alert.finish() {
        let mut paint = Paint::default();
        paint.set_color_rgba8(137, 180, 250, 32);
        pixmap.fill_path(&path, &paint, FillRule::Winding, Transform::identity(), None);
    }

    // 2. Área sombreada Verde (% Evacuados)
    let mut pb_evac = PathBuilder::new();
    pb_evac.move_to(pad_l, base_y);
    for pt in history {
        let x = pad_l + (pt.time_s / max_t) * w;
        let y = pad_t + h * (1.0 - (pt.evacuated_pct / 100.0).clamp(0.0, 1.0));
        pb_evac.line_to(x, y);
    }
    if let Some(last) = history.last() {
        pb_evac.line_to(pad_l + (last.time_s / max_t) * w, base_y);
    }
    pb_evac.close();
    if let Some(path) = pb_evac.finish() {
        let mut paint = Paint::default();
        paint.set_color_rgba8(166, 227, 161, 60);
        pixmap.fill_path(&path, &paint, FillRule::Winding, Transform::identity(), None);
    }

    // 3. Área sombreada Roja (% Víctimas FED)
    let mut pb_inc = PathBuilder::new();
    pb_inc.move_to(pad_l, base_y);
    for pt in history {
        let x = pad_l + (pt.time_s / max_t) * w;
        let y = pad_t + h * (1.0 - (pt.incapacitated_pct / 100.0).clamp(0.0, 1.0));
        pb_inc.line_to(x, y);
    }
    if let Some(last) = history.last() {
        pb_inc.line_to(pad_l + (last.time_s / max_t) * w, base_y);
    }
    pb_inc.close();
    if let Some(path) = pb_inc.finish() {
        let mut paint = Paint::default();
        paint.set_color_rgba8(243, 139, 168, 65);
        pixmap.fill_path(&path, &paint, FillRule::Winding, Transform::identity(), None);
    }

    // 4. Trazado de curvas y nodos de telemetría
    let mut aset_drawn = false;
    for w_pair in history.windows(2) {
        let p0 = w_pair[0];
        let p1 = w_pair[1];
        let x0 = pad_l + (p0.time_s / max_t) * w;
        let x1 = pad_l + (p1.time_s / max_t) * w;

        let y0_alert = pad_t + h * (1.0 - (p0.alerting_pct / 100.0).clamp(0.0, 1.0));
        let y1_alert = pad_t + h * (1.0 - (p1.alerting_pct / 100.0).clamp(0.0, 1.0));
        draw_line(&mut pixmap, x0, y0_alert, x1, y1_alert, (137, 180, 250, 215), 1.8);

        let y0_evac = pad_t + h * (1.0 - (p0.evacuated_pct / 100.0).clamp(0.0, 1.0));
        let y1_evac = pad_t + h * (1.0 - (p1.evacuated_pct / 100.0).clamp(0.0, 1.0));
        draw_line(&mut pixmap, x0, y0_evac, x1, y1_evac, (166, 227, 161, 255), 2.8);

        let y0_inc = pad_t + h * (1.0 - (p0.incapacitated_pct / 100.0).clamp(0.0, 1.0));
        let y1_inc = pad_t + h * (1.0 - (p1.incapacitated_pct / 100.0).clamp(0.0, 1.0));
        draw_line(&mut pixmap, x0, y0_inc, x1, y1_inc, (243, 139, 168, 255), 2.4);

        if !aset_drawn && p1.incapacitated_pct > 0.0 {
            aset_drawn = true;
            draw_line(&mut pixmap, x1, pad_t, x1, base_y, (249, 226, 175, 225), 1.8);
            draw_circle(&mut pixmap, x1, y1_inc, 3.2, (249, 226, 175, 255));
        }
    }

    // Puntos brillantes en el extremo actual de cada serie
    if let Some(last) = history.last() {
        let lx = pad_l + (last.time_s / max_t) * w;
        let ly_evac = pad_t + h * (1.0 - (last.evacuated_pct / 100.0).clamp(0.0, 1.0));
        let ly_alert = pad_t + h * (1.0 - (last.alerting_pct / 100.0).clamp(0.0, 1.0));
        let ly_inc = pad_t + h * (1.0 - (last.incapacitated_pct / 100.0).clamp(0.0, 1.0));
        draw_circle(&mut pixmap, lx, ly_alert, 3.0, (137, 180, 250, 255));
        draw_circle(&mut pixmap, lx, ly_evac, 3.6, (166, 227, 161, 255));
        draw_circle(&mut pixmap, lx, ly_inc, 3.2, (243, 139, 168, 255));
    }

    Image::from_rgba8(pixel_buffer)
}

fn draw_line(
    pixmap: &mut PixmapMut,
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
    rgba: (u8, u8, u8, u8),
             width: f32,
) {
    let mut pb = PathBuilder::new();
    pb.move_to(x1, y1);
    pb.line_to(x2, y2);
    if let Some(path) = pb.finish() {
        let mut paint = Paint::default();
        paint.set_color_rgba8(rgba.0, rgba.1, rgba.2, rgba.3);
        paint.anti_alias = true;
        let stroke = Stroke {
            width,
            line_cap: LineCap::Round,
            ..Default::default()
        };
        pixmap.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
    }
}

fn draw_circle(
    pixmap: &mut PixmapMut,
    cx: f32,
    cy: f32,
    radius: f32,
    rgba: (u8, u8, u8, u8),
) {
    let mut pb = PathBuilder::new();
    pb.push_circle(cx, cy, radius);
    if let Some(path) = pb.finish() {
        let mut paint = Paint::default();
        paint.set_color_rgba8(rgba.0, rgba.1, rgba.2, rgba.3);
        paint.anti_alias = true;
        pixmap.fill_path(&path, &paint, FillRule::Winding, Transform::identity(), None);
    }
}

fn draw_circle_outline(
    pixmap: &mut PixmapMut,
    cx: f32,
    cy: f32,
    radius: f32,
    rgba: (u8, u8, u8, u8),
                       width: f32,
) {
    let mut pb = PathBuilder::new();
    pb.push_circle(cx, cy, radius);
    if let Some(path) = pb.finish() {
        let mut paint = Paint::default();
        paint.set_color_rgba8(rgba.0, rgba.1, rgba.2, rgba.3);
        paint.anti_alias = true;
        let stroke = Stroke {
            width,
            ..Default::default()
        };
        pixmap.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
    }
}
/// Dibuja las barras de progreso visuales (Salud/FED, Pánico, Velocidad y Eficiencia de Rescate) para el Inspector
pub fn render_inspector_bars(
    selected_agent: Option<&Agent>,
    width: u32,
    height: u32,
) -> Image {
    let mut pixel_buffer = SharedPixelBuffer::<Rgba8Pixel>::new(width, height);
    let mut pixmap =
    PixmapMut::from_bytes(pixel_buffer.make_mut_bytes(), width, height).expect("Pixmap inválido");

    pixmap.fill(Color::from_rgba8(17, 17, 27, 255));

    let (fed_ratio, panic_ratio, speed_ratio, rescue_ratio) = if let Some(a) = selected_agent {
        let fed_r = (a.fed_dose / a.profile.critical_fed_threshold().max(0.1)).clamp(0.0, 1.0);
        let pan_r = (a.panic_level / 10.0).clamp(0.0, 1.0);
        let spd_r = (a.avg_speed_m_s() / 2.0).clamp(0.0, 1.0);
        let res_r = ((a.rescued_count + a.helped_fallen_count) as f32 / 8.0).clamp(0.0, 1.0);
        (1.0 - fed_r, pan_r, spd_r, res_r)
    } else {
        (1.0, 0.2, 0.65, 0.4)
    };

    let bars = [
        (fed_ratio, (166, 227, 161, 255)),   // Barra 1 (Verde): Integridad Fisiológica (100% - FED)
        (panic_ratio, (243, 139, 168, 255)), // Barra 2 (Roja): Nivel de Pánico
        (speed_ratio, (137, 180, 250, 255)), // Barra 3 (Azul): Velocidad Media (0 - 2.0 m/s)
        (rescue_ratio, (249, 226, 175, 255)),// Barra 4 (Oro): Impacto Altruista / Brigada
    ];

    let pad_x = 8.0_f32;
    let bar_w = (width as f32 - 2.0 * pad_x).max(20.0);
    let bar_h = 7.0_f32;
    let gap = 11.0_f32;

    for (idx, (ratio, col)) in bars.iter().enumerate() {
        let y = 6.0 + (idx as f32) * gap;
        if let Some(bg) = Rect::from_xywh(pad_x, y, bar_w, bar_h) {
            let mut p = Paint::default();
            p.set_color_rgba8(49, 50, 68, 255);
            pixmap.fill_rect(bg, &p, Transform::identity(), None);
        }
        let fill_w = (bar_w * ratio).max(2.0);
        if let Some(fg) = Rect::from_xywh(pad_x, y, fill_w, bar_h) {
            let mut p = Paint::default();
            p.set_color_rgba8(col.0, col.1, col.2, col.3);
            pixmap.fill_rect(fg, &p, Transform::identity(), None);
        }
    }

    Image::from_rgba8(pixel_buffer)
}
