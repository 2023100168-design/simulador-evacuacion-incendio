use crate::building_loader::{LoadedBuilding, TILE_SIZE_M};
use crate::config::{AMBIENT_TEMP_C, FLASHOVER_TEMP_C};
use crate::engine::TelemetryPoint;
use crate::model::agent::{Agent, AgentActionState, DemographicProfile, MacesRole};
use crate::model::topology::CellKind;
use slint::{Image, Rgba8Pixel, SharedPixelBuffer};
use tiny_skia::{
    Color, FillRule, LineCap, LineJoin, Paint, PathBuilder, PixmapMut, Rect, Stroke, Transform,
};

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
    let pad = 22.0;

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
        let cols = if num_floors <= 2 {
            num_floors
        } else if num_floors <= 4 {
            2
        } else {
            3
        };
        let rows = (num_floors + cols - 1) / cols;

        let cell_w = canvas_w / (cols as f32);
        let cell_h = canvas_h / (rows as f32);
        let scale_x = (cell_w - 2.0 * pad) / gw;
        let scale_y = (cell_h - 2.0 * pad) / gh;
        let scale = scale_x.min(scale_y).max(2.0);

        (0..num_floors)
        .map(|f| {
            let c_idx = f % cols;
            let r_idx = f / cols;
            let col_start = (c_idx as f32) * cell_w;
            let row_start = (r_idx as f32) * cell_h;
            let offset_x = col_start + (cell_w - gw * scale) * 0.5;
            let offset_y = row_start + (cell_h - gh * scale) * 0.5;
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

pub fn screen_to_floor_tile(
    building: &LoadedBuilding,
    canvas_w: f32,
    canvas_h: f32,
    view_mode: i32,
    px: f32,
    py: f32,
) -> Option<(usize, usize, usize)> {
    let viewports = compute_viewports(building, canvas_w, canvas_h, view_mode);
    for vp in viewports {
        let mx = (px - vp.offset_x) / vp.scale;
        let my = (py - vp.offset_y) / vp.scale;
        if mx >= 0.0 && mx < building.grid_width_m && my >= 0.0 && my < building.grid_height_m {
            let (r, c) = building.tile_coords_at(mx, my);
            if building.cell_grid[vp.floor_idx][r][c].is_some() {
                return Some((vp.floor_idx, r, c));
            }
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
                    if dist < 1.3 && best.map_or(true, |(_, d)| dist < d) {
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

    // Fondo técnico oscuro profundo (#11111b)
    pixmap.fill(Color::from_rgba8(17, 17, 27, 255));

    let viewports = compute_viewports(building, width as f32, height as f32, view_mode);

    for vp in &viewports {
        let to_sx = |mx: f32| vp.offset_x + mx * vp.scale;
        let to_sy = |my: f32| vp.offset_y + my * vp.scale;
        let fw = building.grid_width_m * vp.scale;
        let fh = building.grid_height_m * vp.scale;

        // 1. Sombra proyectada del edificio y losa base
        if let Some(sh_rect) = Rect::from_xywh(vp.offset_x + 6.0, vp.offset_y + 6.0, fw, fh) {
            let mut sp = Paint::default();
            sp.set_color_rgba8(7, 7, 13, 210);
            pixmap.fill_rect(sh_rect, &sp, Transform::identity(), None);
        }
        if let Some(bg_rect) = Rect::from_xywh(vp.offset_x, vp.offset_y, fw, fh) {
            let mut bp = Paint::default();
            bp.set_color_rgba8(30, 32, 48, 255);
            pixmap.fill_rect(bg_rect, &bp, Transform::identity(), None);
        }

        // 2. Acabado de Pisos por Tipo de Ambiente (Estilo CAD / Plano Arquitectónico)
        for cell in building.cells.iter().filter(|c| c.floor == vp.floor_idx) {
            let mut fill_paint = Paint::default();
            match cell.kind {
                CellKind::Room => fill_paint.set_color_rgba8(36, 39, 58, 255),     // Losa de oficina clara
                CellKind::Corridor => fill_paint.set_color_rgba8(28, 30, 44, 255), // Pasillo circulación
                CellKind::Stairwell => fill_paint.set_color_rgba8(42, 46, 68, 255),
                CellKind::Elevator { .. } => fill_paint.set_color_rgba8(32, 34, 48, 255),
                CellKind::Exit => fill_paint.set_color_rgba8(29, 78, 58, 255),     // Zona segura esmeralda
            }

            for &(tr, tc) in &cell.tiles {
                let tx = to_sx(tc as f32);
                let ty = to_sy(tr as f32);
                let tw = vp.scale + 0.5;
                if let Some(t_rect) = Rect::from_xywh(tx, ty, tw, tw) {
                    pixmap.fill_rect(t_rect, &fill_paint, Transform::identity(), None);
                }
                // Junta sutil de baldosa arquitectónica
                if vp.scale >= 9.0 {
                    draw_line(&mut pixmap, tx, ty, tx + vp.scale, ty, (49, 52, 74, 70), 0.6);
                    draw_line(&mut pixmap, tx, ty, tx, ty + vp.scale, (49, 52, 74, 70), 0.6);
                }
            }

            // Franjas de área segura en Salidas (Exit)
            if cell.kind == CellKind::Exit {
                let rx = to_sx(cell.min_x);
                let ry = to_sy(cell.min_y);
                let rw = (cell.max_x - cell.min_x) * vp.scale;
                let rh = (cell.max_y - cell.min_y) * vp.scale;
                if let Some(exit_border) = Rect::from_xywh(rx + 2.0, ry + 2.0, (rw - 4.0).max(2.0), (rh - 4.0).max(2.0)) {
                    let mut ep = Paint::default();
                    ep.set_color_rgba8(166, 227, 161, 45);
                    pixmap.fill_rect(exit_border, &ep, Transform::identity(), None);
                }
            }

            // Escaleras en Peldaño Completo (Full Treads de borde a borde, sin descanso que estorbe a las puertas)
            if cell.kind == CellKind::Stairwell {
                let rx = to_sx(cell.min_x);
                let ry = to_sy(cell.min_y);
                let rw = (cell.max_x - cell.min_x) * vp.scale;
                let rh = (cell.max_y - cell.min_y) * vp.scale;

                let num_treads = 9;
                for s in 1..num_treads {
                    let sy = ry + (rh * (s as f32 / num_treads as f32));
                    draw_line(
                        &mut pixmap,
                        rx + 1.5,
                        sy,
                        rx + rw - 1.5,
                        sy,
                        (148, 163, 184, 175),
                              1.1,
                    );
                }
                // Pasamanos central continuo de extremo a extremo
                let mid_x = rx + rw * 0.5;
                draw_line(
                    &mut pixmap,
                    mid_x - 1.0,
                    ry + 2.0,
                    mid_x - 1.0,
                    ry + rh - 2.0,
                    (203, 213, 225, 200),
                          1.2,
                );
                draw_line(
                    &mut pixmap,
                    mid_x + 1.0,
                    ry + 2.0,
                    mid_x + 1.0,
                    ry + rh - 2.0,
                    (203, 213, 225, 200),
                          1.2,
                );
            }
        }

        // 3. Renderizado de Ondas de Fuego y Humo Baldosa por Baldosa (1m²)
        if show_thermal {
            let f = vp.floor_idx;
            for r in 0..building.rows {
                for c in 0..building.cols {
                    if !building.walkable_grid[f][r][c] {
                        continue;
                    }
                    let t = building.tile_temp[f][r][c];
                    let smk = building.tile_smoke[f][r][c];
                    let tx = to_sx(c as f32);
                    let ty = to_sy(r as f32);
                    let tw = vp.scale + 0.6;

                    if t > AMBIENT_TEMP_C + 2.0 {
                        let ratio =
                        ((t - AMBIENT_TEMP_C) / (FLASHOVER_TEMP_C - AMBIENT_TEMP_C)).clamp(0.0, 1.0);
                        if let Some(t_rect) = Rect::from_xywh(tx, ty, tw, tw) {
                            let mut hp = Paint::default();
                            if ratio > 0.70 {
                                hp.set_color_rgba8(220, 38, 38, (130.0 + ratio * 95.0) as u8);
                            } else if ratio > 0.35 {
                                hp.set_color_rgba8(234, 88, 12, (105.0 + ratio * 95.0) as u8);
                            } else {
                                hp.set_color_rgba8(234, 179, 8, (60.0 + ratio * 110.0) as u8);
                            }
                            pixmap.fill_rect(t_rect, &hp, Transform::identity(), None);
                        }

                        // Si la baldosa está en ignición activa (> 220°C), dibujar núcleo ígneo sobre esa baldosa
                        if t > 220.0 {
                            let cx = tx + vp.scale * 0.5;
                            let cy = ty + vp.scale * 0.5;
                            draw_circle(&mut pixmap, cx, cy, vp.scale * 0.65, (239, 68, 68, 95));
                            draw_flame_icon(&mut pixmap, cx, cy, vp.scale * (0.35 + ratio * 0.25));
                        }
                    }

                    if smk > 0.02 {
                        let smoke_a = (smk.clamp(0.0, 1.0) * 155.0) as u8;
                        if let Some(t_rect) = Rect::from_xywh(tx, ty, tw, tw) {
                            let mut sp = Paint::default();
                            sp.set_color_rgba8(71, 85, 105, smoke_a);
                            pixmap.fill_rect(t_rect, &sp, Transform::identity(), None);
                        }
                    }
                }
            }
        }

        // 4. Muros Sólidos Rellenos (Estilo CAD con espesor real '#' y bisel claro)
        if vp.floor_idx < building.raw_floors.len() {
            let raw = &building.raw_floors[vp.floor_idx];
            for r in 0..building.rows {
                for c in 0..building.cols {
                    if raw[r][c] == '#' {
                        let wx = to_sx(c as f32);
                        let wy = to_sy(r as f32);
                        let ww = vp.scale + 0.6;
                        if let Some(w_rect) = Rect::from_xywh(wx, wy, ww, ww) {
                            let mut wp = Paint::default();
                            // Relleno de hormigón estructural oscuro (#454a64)
                            wp.set_color_rgba8(62, 68, 92, 255);
                            pixmap.fill_rect(w_rect, &wp, Transform::identity(), None);
                        }
                    }
                }
            }
        }

        // Contorno nítido de aristas de muro y columnas estructurales ('O')
        for cell in building.cells.iter().filter(|c| c.floor == vp.floor_idx) {
            for w in &cell.walls {
                draw_line(
                    &mut pixmap,
                    to_sx(w.x1),
                          to_sy(w.y1),
                          to_sx(w.x2),
                          to_sy(w.y2),
                          (148, 158, 190, 235),
                          1.5,
                );
            }
            for obs in &cell.obstacles {
                let ox = to_sx(obs.x);
                let oy = to_sy(obs.y);
                let s = (0.44 * vp.scale).max(3.5);
                if let Some(col_r) = Rect::from_xywh(ox - s, oy - s, s * 2.0, s * 2.0) {
                    let mut cp = Paint::default();
                    cp.set_color_rgba8(54, 59, 80, 255);
                    pixmap.fill_rect(col_r, &cp, Transform::identity(), None);
                }
                draw_line(&mut pixmap, ox - s, oy - s, ox + s, oy - s, (166, 173, 200, 230), 1.3);
                draw_line(&mut pixmap, ox - s, oy - s, ox - s, oy + s, (166, 173, 200, 230), 1.3);
                draw_line(&mut pixmap, ox + s, oy - s, ox + s, oy + s, (108, 112, 134, 230), 1.3);
                draw_line(&mut pixmap, ox - s, oy + s, ox + s, oy + s, (108, 112, 134, 230), 1.3);
            }
        }

        // 5. Puertas con Hoja Batiente y Arco de Barrido CAD de 90°
        for portal in building
            .portals
            .iter()
            .filter(|p| p.floor_a == vp.floor_idx && p.floor_b == vp.floor_idx)
            {
                let px = to_sx(portal.pos_a.0);
                let py = to_sy(portal.pos_a.1);
                let door_r = (0.92 * vp.scale).max(5.0);
                let is_horizontal_wall = (portal.attractor_a.1 - portal.attractor_b.1).abs() > 0.1;

                // Limpiar el vano de la puerta
                if let Some(vano) = Rect::from_xywh(
                    px - vp.scale * 0.48,
                    py - vp.scale * 0.48,
                    vp.scale * 0.96,
                    vp.scale * 0.96,
                ) {
                    let mut vp_paint = Paint::default();
                    vp_paint.set_color_rgba8(32, 35, 52, 255);
                    pixmap.fill_rect(vano, &vp_paint, Transform::identity(), None);
                }

                let door_col = if portal.is_closed_or_blocked {
                    (243, 139, 168, 255) // Clausurada (Rojo)
                } else if portal.max_resistance_s >= 100.0 {
                    (250, 179, 135, 255) // Cortafuegos 120s (Ámbar)
                } else {
                    (166, 227, 161, 255) // Estándar (Verde)
                };

                if portal.is_closed_or_blocked {
                    // Puerta cerrada bloqueando el vano con una X
                    if is_horizontal_wall {
                        draw_line(&mut pixmap, px - door_r * 0.5, py, px + door_r * 0.5, py, door_col, 3.2);
                    } else {
                        draw_line(&mut pixmap, px, py - door_r * 0.5, px, py + door_r * 0.5, door_col, 3.2);
                    }
                } else {
                    // Dibujo arquitectónico de puerta abierta: bisagra + hoja + cuarto de círculo (arco de 90°)
                    let (hinge_x, hinge_y, leaf_x, leaf_y, arc_end_x, arc_end_y) = if is_horizontal_wall {
                        let hx = px - door_r * 0.5;
                        let hy = py;
                        (hx, hy, hx, hy + door_r, hx + door_r, hy)
                    } else {
                        let hx = px;
                        let hy = py - door_r * 0.5;
                        (hx, hy, hx + door_r, hy, hx, hy + door_r)
                    };

                    // Hoja de la puerta abierta
                    draw_line(&mut pixmap, hinge_x, hinge_y, leaf_x, leaf_y, door_col, 2.2);
                    // Arco de barrido de 90° (curva cuadrática de Bézier desde la punta de la hoja hasta el marco)
                    let mut pb = PathBuilder::new();
                    pb.move_to(leaf_x, leaf_y);
                    let ctrl_x = if is_horizontal_wall { hinge_x + door_r } else { hinge_x + door_r };
                    let ctrl_y = if is_horizontal_wall { hinge_y + door_r } else { hinge_y + door_r };
                    pb.quad_to(ctrl_x, ctrl_y, arc_end_x, arc_end_y);
                    if let Some(arc_path) = pb.finish() {
                        let mut ap = Paint::default();
                        ap.set_color_rgba8(door_col.0, door_col.1, door_col.2, 130);
                        ap.anti_alias = true;
                        let st = Stroke {
                            width: 1.1,
                            ..Default::default()
                        };
                        pixmap.stroke_path(&arc_path, &ap, &st, Transform::identity(), None);
                    }
                }
            }

            // 6. Equipamiento: Estaciones de Alarma (Caja Roja/Amarilla) y Extintores (Cilindro Rojo/Cian)
            for alarm in building.alarms.iter().filter(|a| a.floor == vp.floor_idx) {
                let ax = to_sx(alarm.x);
                let ay = to_sy(alarm.y);
                let active = alarm.is_triggered || global_alarm;
                let s = (0.28 * vp.scale).max(3.5);
                if let Some(ar) = Rect::from_xywh(ax - s, ay - s, s * 2.0, s * 2.0) {
                    let mut p = Paint::default();
                    if active {
                        p.set_color_rgba8(243, 139, 168, 255);
                    } else {
                        p.set_color_rgba8(249, 226, 175, 255);
                    }
                    pixmap.fill_rect(ar, &p, Transform::identity(), None);
                }
                if active {
                    draw_circle_outline(&mut pixmap, ax, ay, s * 2.4, (243, 139, 168, 180), 1.4);
                }
            }

            for ext in building
                .extinguishers
                .iter()
                .filter(|e| e.floor == vp.floor_idx && e.is_available)
                {
                    let ex = to_sx(ext.x);
                    let ey = to_sy(ext.y);
                    let er = (0.26 * vp.scale).max(3.2);
                    draw_circle(&mut pixmap, ex, ey, er, (230, 69, 83, 255));
                    draw_circle_outline(&mut pixmap, ex, ey, er + 1.2, (148, 226, 213, 230), 1.3);
                }

                // 7. Grafo superpuesto opcional
                if show_graph {
                    for portal in building
                        .portals
                        .iter()
                        .filter(|p| p.floor_a == vp.floor_idx && p.floor_b == vp.floor_idx)
                        {
                            let c_a = building.cells[portal.cell_a].center();
                            let c_b = building.cells[portal.cell_b].center();
                            draw_line(
                                &mut pixmap,
                                to_sx(c_a.0),
                                      to_sy(c_a.1),
                                      to_sx(c_b.0),
                                      to_sy(c_b.1),
                                      (203, 166, 247, 110),
                                      1.3,
                            );
                        }
                }

                // 8. CAPA DE AGENTES CON SILUETA ANATÓMICA CENITAL (Hombros, Cabeza, Brazos y Posturas)
                for agent in agents
                    .iter()
                    .filter(|a| a.floor == vp.floor_idx && a.action_state != AgentActionState::Evacuated)
                    {
                        let ax = to_sx(agent.x);
                        let ay = to_sy(agent.y);
                        let r = (agent.profile.body_radius() * vp.scale).max(3.8);

                        // Enlaces Familiares (Lavanda) y de Brigada (Celeste)
                        if show_social_bonds && agent.action_state != AgentActionState::Normal && !agent.is_fallen {
                            if let Some(lid) = agent.group_leader_id {
                                if lid != agent.id {
                                    let leader = &agents[lid];
                                    if leader.floor == vp.floor_idx
                                        && leader.action_state != AgentActionState::Evacuated
                                        {
                                            draw_line(
                                                &mut pixmap,
                                                ax,
                                                ay,
                                                to_sx(leader.x),
                                                      to_sy(leader.y),
                                                      (203, 166, 247, 130),
                                                      1.3,
                                            );
                                        }
                                }
                            }
                        }

                        if let AgentActionState::FollowingWarden { warden_id, .. } = agent.action_state {
                            let w = &agents[warden_id];
                            if w.floor == vp.floor_idx {
                                draw_line(
                                    &mut pixmap,
                                    ax,
                                    ay,
                                    to_sx(w.x),
                                          to_sy(w.y),
                                          (137, 220, 235, 200),
                                          1.8,
                                );
                            }
                        }

                        draw_top_down_human(&mut pixmap, agent, ax, ay, r);

                        // Enlace verde cuando está levantando a un compañero caído
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
                                      2.2,
                            );
                        }

                        // Chorro químico cian en abanico cuando usa un extintor
                        if let AgentActionState::FightingFire { fire_cell, .. } = agent.action_state {
                            let (fx, fy) = building.cells[fire_cell].center();
                            draw_line(&mut pixmap, ax, ay, to_sx(fx), to_sy(fy), (148, 226, 213, 220), 3.5);
                            draw_circle(&mut pixmap, to_sx(fx), to_sy(fy), r * 1.4, (148, 226, 213, 140));
                        }
                    }
    }

    Image::from_rgba8(pixel_buffer)
}

/// Dibuja la silueta cenital (Top-Down) de una persona diferenciando si camina, corre en pánico, está en shock, caída o fallecida
fn draw_top_down_human(pixmap: &mut PixmapMut, agent: &Agent, cx: f32, cy: f32, r: f32) {
    // 1. CASO VÍCTIMA INCAPACITADA / MUERTA POR FED (Cuerpo inerte gris + Cruz roja oscura)
    if agent.action_state == AgentActionState::Incapacitated {
        // Sombra en el suelo
        draw_capsule(pixmap, cx - r * 1.1, cy - r * 0.4, cx + r * 1.1, cy + r * 0.4, r * 0.75, (24, 24, 37, 220));
        // Torso tendido
        draw_capsule(pixmap, cx - r * 0.9, cy - r * 0.3, cx + r * 0.6, cy + r * 0.2, r * 0.62, (88, 91, 112, 255));
        // Cabeza inerte
        draw_circle(pixmap, cx + r * 0.95, cy + r * 0.3, r * 0.52, (127, 132, 156, 255));
        // Señal X roja de baja por toxicidad/calor
        let s = r * 0.55;
        draw_line(pixmap, cx - s, cy - s, cx + s, cy + s, (243, 139, 168, 245), 1.8);
        draw_line(pixmap, cx + s, cy - s, cx - s, cy + s, (243, 139, 168, 245), 1.8);
        return;
    }

    // 2. CASO AGENTE CAÍDO AL PISO POR EMPUJÓN (Silueta tendida color ámbar intentando levantarse)
    if agent.is_fallen {
        draw_capsule(pixmap, cx - r * 1.0, cy, cx + r * 0.7, cy, r * 0.68, (250, 179, 135, 245));
        // Brazo extendido apoyándose en el suelo
        draw_line(pixmap, cx + r * 0.3, cy, cx + r * 1.1, cy - r * 0.85, (250, 179, 135, 255), r * 0.38);
        // Cabeza
        draw_circle(pixmap, cx + r * 0.85, cy + r * 0.1, r * 0.55, (249, 226, 175, 255));
        draw_circle_outline(pixmap, cx, cy, r * 1.45, (250, 179, 135, 170), 1.2);
        return;
    }

    // Dirección hacia donde mira el agente (unitaria) y vector perpendicular de sus hombros
    let v_mag = agent.vx.hypot(agent.vy);
    let (fx, fy) = if v_mag > 0.04 {
        (agent.vx / v_mag, agent.vy / v_mag)
    } else {
        (0.0, -1.0)
    };
    let (rx, ry) = (-fy, fx); // Vector derecha (hombros)

    // Color de vestimenta según rol y perfil demográfico
    let (torso_col, head_col) = if agent.role == MacesRole::TrainedLeader {
        ((20, 184, 166, 255), (240, 253, 250, 255)) // Brigadista: Chaleco Teal + Casco Blanco
    } else if matches!(
        agent.action_state,
        AgentActionState::FetchingExtinguisher { .. }
        | AgentActionState::FightingFire { .. }
        | AgentActionState::HelpingFallen { .. }
    ) {
        ((166, 227, 161, 255), (235, 245, 235, 255)) // Rescatista activo: Verde esmeralda
    } else {
        match agent.profile {
            DemographicProfile::Adult | DemographicProfile::YoungAdult => {
                ((59, 130, 246, 255), (219, 234, 254, 255)) // Azul corporativo
            }
            DemographicProfile::Child => ((245, 158, 11, 255), (254, 243, 199, 255)), // Ámbar/Amarillo
            DemographicProfile::Elderly | DemographicProfile::Impaired => {
                ((168, 85, 247, 255), (243, 232, 255, 255)) // Púrpura/Lavanda
            }
        }
    };

    let sh_half = r * 0.88;
    let sh_lx = cx - rx * sh_half;
    let sh_ly = cy - ry * sh_half;
    let sh_rx = cx + rx * sh_half;
    let sh_ry = cy + ry * sh_half;

    // Sombra suave bajo los hombros
    draw_capsule(
        pixmap,
        sh_lx + 1.2,
        sh_ly + 1.2,
        sh_rx + 1.2,
        sh_ry + 1.2,
        r * 0.62,
        (10, 10, 18, 160),
    );

    // 3. POSTURA DE BRAZOS SEGÚN ESTADO PSICOLÓGICO
    let in_high_panic = agent.panic_level >= 5.5 && agent.action_state != AgentActionState::Normal;
    if matches!(
        agent.action_state,
        AgentActionState::PanickedFrozen { .. } | AgentActionState::PanickedWandering { .. }
    ) {
        // En Shock: Manos sujetándose la cabeza + doble halo magenta de bloqueo
        draw_line(
            pixmap,
            sh_lx,
            sh_ly,
            cx + fx * r * 0.55 - rx * r * 0.4,
            cy + fy * r * 0.55 - ry * r * 0.4,
            torso_col,
            r * 0.42,
        );
        draw_line(
            pixmap,
            sh_rx,
            sh_ry,
            cx + fx * r * 0.55 + rx * r * 0.4,
            cy + fy * r * 0.55 + ry * r * 0.4,
            torso_col,
            r * 0.42,
        );
        draw_circle_outline(pixmap, cx, cy, r * 1.55, (243, 139, 168, 235), 1.6);
    } else if in_high_panic {
        // Desesperado / Pánico activo: Brazos extendidos hacia adelante pidiendo paso
        let reach = r * 1.25;
        draw_line(
            pixmap,
            sh_lx,
            sh_ly,
            sh_lx + fx * reach,
            sh_ly + fy * reach,
            (243, 139, 168, 255),
                  r * 0.38,
        );
        draw_line(
            pixmap,
            sh_rx,
            sh_ry,
            sh_rx + fx * reach,
            sh_ry + fy * reach,
            (243, 139, 168, 255),
                  r * 0.38,
        );
    } else if v_mag > 0.15 {
        // Caminando / Corriendo normal: balanceo sutil de brazos
        let swing = r * 0.65;
        draw_line(
            pixmap,
            sh_lx,
            sh_ly,
            sh_lx + fx * swing,
            sh_ly + fy * swing,
            torso_col,
            r * 0.36,
        );
        draw_line(
            pixmap,
            sh_rx,
            sh_ry,
            sh_rx - fx * swing * 0.5,
            sh_ry - fy * swing * 0.5,
            torso_col,
            r * 0.36,
        );
    }

    // 4. TORSO / HOMBROS (Cápsula transversal a la dirección de marcha)
    draw_capsule(pixmap, sh_lx, sh_ly, sh_rx, sh_ry, r * 0.58, torso_col);

    // Si es Brigadista, dibujar franja reflectante blanca sobre el chaleco
    if agent.role == MacesRole::TrainedLeader {
        draw_line(pixmap, sh_lx, sh_ly, sh_rx, sh_ry, (255, 255, 255, 210), r * 0.22);
    }

    // 5. CABEZA (Ligeramente adelantada en la dirección de visión)
    let head_x = cx + fx * (r * 0.22);
    let head_y = cy + fy * (r * 0.22);
    let head_r = r * 0.54;
    draw_circle(pixmap, head_x, head_y, head_r, head_col);

    // Si está en pánico, núcleo rojizo en la cabeza
    if in_high_panic {
        draw_circle(pixmap, head_x, head_y, head_r * 0.58, (239, 68, 68, 255));
    }

    // Aro amarillo de percepción cuando acaba de detectar el peligro (Reacting)
    if matches!(agent.action_state, AgentActionState::Reacting { .. }) {
        draw_circle_outline(pixmap, cx, cy, r * 1.45, (250, 204, 21, 240), 1.5);
    }
}

/// Dibuja un ícono vectorial de llama de dos tonos en el foco del incendio
fn draw_flame_icon(pixmap: &mut PixmapMut, cx: f32, cy: f32, size: f32) {
    let s = size.max(4.0);
    let mut pb = PathBuilder::new();
    pb.move_to(cx, cy - s * 1.15);
    pb.quad_to(cx + s * 0.85, cy - s * 0.2, cx + s * 0.55, cy + s * 0.65);
    pb.quad_to(cx, cy + s * 1.0, cx - s * 0.55, cy + s * 0.65);
    pb.quad_to(cx - s * 0.85, cy - s * 0.2, cx, cy - s * 1.15);
    pb.close();
    if let Some(path) = pb.finish() {
        let mut p = Paint::default();
        p.set_color_rgba8(255, 111, 0, 240);
        p.anti_alias = true;
        pixmap.fill_path(&path, &p, FillRule::Winding, Transform::identity(), None);
    }
    draw_circle(pixmap, cx, cy + s * 0.25, s * 0.36, (255, 245, 157, 250));
}

fn draw_capsule(
    pixmap: &mut PixmapMut,
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
    radius: f32,
    rgba: (u8, u8, u8, u8),
) {
    let mut pb = PathBuilder::new();
    pb.move_to(x1, y1);
    pb.line_to(x2, y2);
    if let Some(path) = pb.finish() {
        let mut paint = Paint::default();
        paint.set_color_rgba8(rgba.0, rgba.1, rgba.2, rgba.3);
        paint.anti_alias = true;
        let stroke = Stroke {
            width: radius * 2.0,
            line_cap: LineCap::Round,
            line_join: LineJoin::Round,
            ..Default::default()
        };
        pixmap.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
    }
}

pub fn render_telemetry_chart(history: &[TelemetryPoint], width: u32, height: u32) -> Image {
    let mut pixel_buffer = SharedPixelBuffer::<Rgba8Pixel>::new(width, height);
    let mut pixmap =
    PixmapMut::from_bytes(pixel_buffer.make_mut_bytes(), width, height).expect("Pixmap inválido");

    pixmap.fill(Color::from_rgba8(13, 15, 23, 255));

    let pad_l = 10.0_f32;
    let bar_zone_w = 32.0_f32;
    let pad_r = 10.0_f32 + bar_zone_w;
    let pad_t = 10.0_f32;
    let pad_b = 10.0_f32;
    let w = (width as f32 - pad_l - pad_r).max(20.0);
    let h = (height as f32 - pad_t - pad_b).max(20.0);
    let base_y = pad_t + h;

    draw_line(&mut pixmap, pad_l, pad_t, pad_l, base_y, (71, 85, 105, 220), 1.3);
    draw_line(&mut pixmap, pad_l, base_y, pad_l + w, base_y, (71, 85, 105, 220), 1.3);

    for level in [0.25_f32, 0.50, 0.75, 1.0] {
        let y = pad_t + h * (1.0 - level);
        draw_line(&mut pixmap, pad_l, y, pad_l + w, y, (30, 41, 59, 200), 1.0);
    }
    for v_step in [0.20_f32, 0.40, 0.60, 0.80, 1.0] {
        let x = pad_l + w * v_step;
        draw_line(&mut pixmap, x, pad_t, x, base_y, (30, 41, 59, 130), 1.0);
    }

    if let Some(last) = history.last() {
        let bx_start = pad_l + w + 7.0;
        let bar_w = 6.5_f32;
        let bars = [
            (last.evacuated_pct, (52, 211, 153, 245)),
            (last.alerting_pct, (96, 165, 250, 245)),
            (last.incapacitated_pct, (248, 113, 113, 245)),
        ];
        for (idx, (pct, col)) in bars.iter().enumerate() {
            let bx = bx_start + (idx as f32) * (bar_w + 3.2);
            if let Some(bg_r) = Rect::from_xywh(bx, pad_t, bar_w, h) {
                let mut p = Paint::default();
                p.set_color_rgba8(24, 27, 40, 255);
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
        paint.set_color_rgba8(52, 211, 153, 50);
        pixmap.fill_path(&path, &paint, FillRule::Winding, Transform::identity(), None);
    }

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
        paint.set_color_rgba8(248, 113, 113, 60);
        pixmap.fill_path(&path, &paint, FillRule::Winding, Transform::identity(), None);
    }

    let mut aset_drawn = false;
    for w_pair in history.windows(2) {
        let p0 = w_pair[0];
        let p1 = w_pair[1];
        let x0 = pad_l + (p0.time_s / max_t) * w;
        let x1 = pad_l + (p1.time_s / max_t) * w;

        let y0_alert = pad_t + h * (1.0 - (p0.alerting_pct / 100.0).clamp(0.0, 1.0));
        let y1_alert = pad_t + h * (1.0 - (p1.alerting_pct / 100.0).clamp(0.0, 1.0));
        draw_line(&mut pixmap, x0, y0_alert, x1, y1_alert, (96, 165, 250, 210), 1.8);

        let y0_evac = pad_t + h * (1.0 - (p0.evacuated_pct / 100.0).clamp(0.0, 1.0));
        let y1_evac = pad_t + h * (1.0 - (p1.evacuated_pct / 100.0).clamp(0.0, 1.0));
        draw_line(&mut pixmap, x0, y0_evac, x1, y1_evac, (52, 211, 153, 255), 2.5);

        let y0_inc = pad_t + h * (1.0 - (p0.incapacitated_pct / 100.0).clamp(0.0, 1.0));
        let y1_inc = pad_t + h * (1.0 - (p1.incapacitated_pct / 100.0).clamp(0.0, 1.0));
        draw_line(&mut pixmap, x0, y0_inc, x1, y1_inc, (248, 113, 113, 255), 2.2);

        if !aset_drawn && p1.incapacitated_pct > 0.0 {
            aset_drawn = true;
            draw_line(&mut pixmap, x1, pad_t, x1, base_y, (250, 204, 21, 210), 1.5);
        }
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
