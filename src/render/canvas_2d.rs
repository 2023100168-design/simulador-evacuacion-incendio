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

#[derive(Clone, Copy)]
struct IsoCamera {
    cx: f32,
    cy: f32,
    scale: f32,
    cos_a: f32,
    sin_a: f32,
    tilt: f32,
    floor_gap_px: f32,
    center_w_x: f32,
    center_w_y: f32,
    base_floor: f32,
}

impl IsoCamera {
    fn new(
        building: &LoadedBuilding,
        canvas_w: f32,
        canvas_h: f32,
        view_mode: i32,
        yaw_deg: f32,
    ) -> Self {
        let num_floors = building.floors.len().max(1);
        let gw = building.grid_width_m.max(1.0);
        let gh = building.grid_height_m.max(1.0);
        let rad = yaw_deg.to_radians();

        let single_floor = view_mode > 0 && (view_mode as usize) <= num_floors;
        let scale = if single_floor {
            (canvas_w / (gw + gh)).min(canvas_h / ((gw + gh) * 0.58)) * 0.92
        } else {
            (canvas_w / (gw + gh)).min(canvas_h / ((gw + gh) * 0.55 + (num_floors as f32) * 5.2))
            * 0.88
        }
        .max(4.0);

        let floor_gap_px = if single_floor { 0.0 } else { scale * 4.6 };
        let base_floor = if single_floor {
            (view_mode - 1) as f32
        } else {
            (num_floors as f32 - 1.0) * 0.5
        };

        Self {
            cx: canvas_w * 0.5,
            cy: canvas_h * 0.54,
            scale,
            cos_a: rad.cos(),
            sin_a: rad.sin(),
            tilt: 0.52,
            floor_gap_px,
            center_w_x: gw * 0.5,
            center_w_y: gh * 0.5,
            base_floor,
        }
    }

    #[inline]
    fn project(&self, wx: f32, wy: f32, wz_m: f32, floor_idx: usize) -> (f32, f32) {
        let dx = wx - self.center_w_x;
        let dy = wy - self.center_w_y;
        let rx = dx * self.cos_a - dy * self.sin_a;
        let ry = dx * self.sin_a + dy * self.cos_a;

        let floor_offset = (floor_idx as f32 - self.base_floor) * self.floor_gap_px;
        let sx = self.cx + rx * self.scale;
        let sy = self.cy + ry * self.scale * self.tilt - wz_m * self.scale - floor_offset;
        (sx, sy)
    }

    #[inline]
    fn unproject_to_floor(&self, sx: f32, sy: f32, floor_idx: usize) -> (f32, f32) {
        let floor_offset = (floor_idx as f32 - self.base_floor) * self.floor_gap_px;
        let rx = (sx - self.cx) / self.scale;
        let ry = (sy - self.cy + floor_offset) / (self.scale * self.tilt);

        let dx = rx * self.cos_a + ry * self.sin_a;
        let dy = -rx * self.sin_a + ry * self.cos_a;
        (dx + self.center_w_x, dy + self.center_w_y)
    }
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

pub fn screen_to_floor_tile(
    building: &LoadedBuilding,
    canvas_w: f32,
    canvas_h: f32,
    view_mode: i32,
    is_3d: bool,
    yaw_deg: f32,
    px: f32,
    py: f32,
) -> Option<(usize, usize, usize)> {
    if is_3d {
        let cam = IsoCamera::new(building, canvas_w, canvas_h, view_mode, yaw_deg);
        let num_floors = building.floors.len();
        let floor_range: Vec<usize> = if view_mode > 0 && (view_mode as usize) <= num_floors {
            vec![(view_mode - 1) as usize]
        } else {
            (0..num_floors).rev().collect()
        };
        for f in floor_range {
            let (mx, my) = cam.unproject_to_floor(px, py, f);
            if mx >= 0.0 && mx < building.grid_width_m && my >= 0.0 && my < building.grid_height_m {
                let (r, c) = building.tile_coords_at(mx, my);
                if building.cell_grid[f][r][c].is_some() {
                    return Some((f, r, c));
                }
            }
        }
        return None;
    }

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

pub fn screen_to_cell(
    building: &LoadedBuilding,
    canvas_w: f32,
    canvas_h: f32,
    view_mode: i32,
    is_3d: bool,
    yaw_deg: f32,
    px: f32,
    py: f32,
) -> Option<usize> {
    if let Some((f, r, c)) =
        screen_to_floor_tile(building, canvas_w, canvas_h, view_mode, is_3d, yaw_deg, px, py)
        {
            return building.cell_grid[f][r][c];
        }
        None
}

pub fn screen_to_portal(
    building: &LoadedBuilding,
    canvas_w: f32,
    canvas_h: f32,
    view_mode: i32,
    is_3d: bool,
    yaw_deg: f32,
    px: f32,
    py: f32,
) -> Option<usize> {
    if is_3d {
        let cam = IsoCamera::new(building, canvas_w, canvas_h, view_mode, yaw_deg);
        let num_floors = building.floors.len();
        let floor_range: Vec<usize> = if view_mode > 0 && (view_mode as usize) <= num_floors {
            vec![(view_mode - 1) as usize]
        } else {
            (0..num_floors).rev().collect()
        };
        for f in floor_range {
            let (mx, my) = cam.unproject_to_floor(px, py, f);
            let mut best: Option<(usize, f32)> = None;
            for portal in &building.portals {
                if portal.floor_a == f && portal.floor_b == f {
                    let dist = (mx - portal.pos_a.0).hypot(my - portal.pos_a.1);
                    if dist < 1.4 && best.map_or(true, |(_, d)| dist < d) {
                        best = Some((portal.id, dist));
                    }
                }
            }
            if let Some((pid, _)) = best {
                return Some(pid);
            }
        }
        return None;
    }

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
    is_3d: bool,
    yaw_deg: f32,
    px: f32,
    py: f32,
) -> Option<usize> {
    if is_3d {
        let cam = IsoCamera::new(building, canvas_w, canvas_h, view_mode, yaw_deg);
        let num_floors = building.floors.len();
        let floor_range: Vec<usize> = if view_mode > 0 && (view_mode as usize) <= num_floors {
            vec![(view_mode - 1) as usize]
        } else {
            (0..num_floors).rev().collect()
        };
        for f in floor_range {
            let (mx, my) = cam.unproject_to_floor(px, py, f);
            let mut best: Option<(usize, f32)> = None;
            for a in agents
                .iter()
                .filter(|a| a.floor == f && a.action_state != AgentActionState::Evacuated)
                {
                    let d = (mx - a.x).hypot(my - a.y);
                    if d < 1.2 && best.map_or(true, |(_, bd)| d < bd) {
                        best = Some((a.id, d));
                    }
                }
                if let Some((aid, _)) = best {
                    return Some(aid);
                }
        }
        return None;
    }

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
    is_3d: bool,
    yaw_deg: f32,
) -> Image {
    if is_3d {
        return render_isometric_25d_frame(
            building,
            agents,
            width,
            height,
            view_mode,
            show_thermal,
            global_alarm,
            show_social_bonds,
            yaw_deg,
        );
    }

    let mut pixel_buffer = SharedPixelBuffer::<Rgba8Pixel>::new(width, height);
    let mut pixmap =
    PixmapMut::from_bytes(pixel_buffer.make_mut_bytes(), width, height).expect("Pixmap inválido");

    pixmap.fill(Color::from_rgba8(14, 15, 20, 255));

    let viewports = compute_viewports(building, width as f32, height as f32, view_mode);

    for vp in &viewports {
        let to_sx = |mx: f32| vp.offset_x + mx * vp.scale;
        let to_sy = |my: f32| vp.offset_y + my * vp.scale;
        let fw = building.grid_width_m * vp.scale;
        let fh = building.grid_height_m * vp.scale;

        if let Some(sh_rect) = Rect::from_xywh(vp.offset_x + 6.0, vp.offset_y + 6.0, fw, fh) {
            let mut sp = Paint::default();
            sp.set_color_rgba8(6, 6, 10, 210);
            pixmap.fill_rect(sh_rect, &sp, Transform::identity(), None);
        }
        if let Some(bg_rect) = Rect::from_xywh(vp.offset_x, vp.offset_y, fw, fh) {
            let mut bp = Paint::default();
            bp.set_color_rgba8(30, 32, 48, 255);
            pixmap.fill_rect(bg_rect, &bp, Transform::identity(), None);
        }

        for cell in building.cells.iter().filter(|c| c.floor == vp.floor_idx) {
            let mut fill_paint = Paint::default();
            match cell.kind {
                CellKind::Room => fill_paint.set_color_rgba8(36, 39, 58, 255),
                CellKind::Corridor => fill_paint.set_color_rgba8(28, 30, 44, 255),
                CellKind::Stairwell => fill_paint.set_color_rgba8(42, 46, 68, 255),
                CellKind::Elevator { .. } => fill_paint.set_color_rgba8(32, 34, 48, 255),
                CellKind::Exit => fill_paint.set_color_rgba8(29, 78, 58, 255),
            }

            for &(tr, tc) in &cell.tiles {
                let tx = to_sx(tc as f32);
                let ty = to_sy(tr as f32);
                let tw = vp.scale + 0.5;
                if let Some(t_rect) = Rect::from_xywh(tx, ty, tw, tw) {
                    pixmap.fill_rect(t_rect, &fill_paint, Transform::identity(), None);
                }
                if vp.scale >= 9.0 {
                    draw_line(&mut pixmap, tx, ty, tx + vp.scale, ty, (49, 52, 74, 70), 0.6);
                    draw_line(&mut pixmap, tx, ty, tx, ty + vp.scale, (49, 52, 74, 70), 0.6);
                }
            }

            if cell.kind == CellKind::Exit {
                let rx = to_sx(cell.min_x);
                let ry = to_sy(cell.min_y);
                let rw = (cell.max_x - cell.min_x) * vp.scale;
                let rh = (cell.max_y - cell.min_y) * vp.scale;
                if let Some(exit_border) =
                    Rect::from_xywh(rx + 2.0, ry + 2.0, (rw - 4.0).max(2.0), (rh - 4.0).max(2.0))
                    {
                        let mut ep = Paint::default();
                        ep.set_color_rgba8(166, 227, 161, 45);
                        pixmap.fill_rect(exit_border, &ep, Transform::identity(), None);
                    }
            }

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
                        let ratio = ((t - AMBIENT_TEMP_C) / (FLASHOVER_TEMP_C - AMBIENT_TEMP_C))
                        .clamp(0.0, 1.0);
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
                            wp.set_color_rgba8(62, 68, 92, 255);
                            pixmap.fill_rect(w_rect, &wp, Transform::identity(), None);
                        }
                    }
                }
            }
        }

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

        for portal in building
            .portals
            .iter()
            .filter(|p| p.floor_a == vp.floor_idx && p.floor_b == vp.floor_idx)
            {
                let px = to_sx(portal.pos_a.0);
                let py = to_sy(portal.pos_a.1);
                let door_r = (0.92 * vp.scale).max(5.0);
                let is_horizontal_wall = (portal.attractor_a.1 - portal.attractor_b.1).abs() > 0.1;

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
                    (243, 139, 168, 255)
                } else if portal.max_resistance_s >= 100.0 {
                    (250, 179, 135, 255)
                } else {
                    (166, 227, 161, 255)
                };

                if portal.is_closed_or_blocked {
                    if is_horizontal_wall {
                        draw_line(
                            &mut pixmap,
                            px - door_r * 0.5,
                            py,
                            px + door_r * 0.5,
                            py,
                            door_col,
                            3.2,
                        );
                    } else {
                        draw_line(
                            &mut pixmap,
                            px,
                            py - door_r * 0.5,
                            px,
                            py + door_r * 0.5,
                            door_col,
                            3.2,
                        );
                    }
                } else {
                    let (hinge_x, hinge_y, leaf_x, leaf_y, arc_end_x, arc_end_y) = if is_horizontal_wall
                    {
                        let hx = px - door_r * 0.5;
                        let hy = py;
                        (hx, hy, hx, hy + door_r, hx + door_r, hy)
                    } else {
                        let hx = px;
                        let hy = py - door_r * 0.5;
                        (hx, hy, hx + door_r, hy, hx, hy + door_r)
                    };

                    draw_line(&mut pixmap, hinge_x, hinge_y, leaf_x, leaf_y, door_col, 2.2);
                    let mut pb = PathBuilder::new();
                    pb.move_to(leaf_x, leaf_y);
                    let ctrl_x = hinge_x + door_r;
                    let ctrl_y = hinge_y + door_r;
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

                for agent in agents
                    .iter()
                    .filter(|a| a.floor == vp.floor_idx && a.action_state != AgentActionState::Evacuated)
                    {
                        let ax = to_sx(agent.x);
                        let ay = to_sy(agent.y);
                        let r = (agent.profile.body_radius() * vp.scale).max(3.8);

                        if show_social_bonds && agent.action_state != AgentActionState::Normal && !agent.is_fallen
                        {
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

                        if let AgentActionState::FightingFire { fire_cell, .. } = agent.action_state {
                            let (fx, fy) = building.cells[fire_cell].center();
                            draw_line(&mut pixmap, ax, ay, to_sx(fx), to_sy(fy), (148, 226, 213, 220), 3.5);
                            draw_circle(&mut pixmap, to_sx(fx), to_sy(fy), r * 1.4, (148, 226, 213, 140));
                        }
                    }
    }

    Image::from_rgba8(pixel_buffer)
}

/// Motor de Renderizado Axonométrico 2.5D con Extrusión Volumétrica y Ordenamiento de Profundidad (Painter's Algorithm)
fn render_isometric_25d_frame(
    building: &LoadedBuilding,
    agents: &[Agent],
    width: u32,
    height: u32,
    view_mode: i32,
    show_thermal: bool,
    _global_alarm: bool,
    show_social_bonds: bool,
    yaw_deg: f32,
) -> Image {
    let mut pixel_buffer = SharedPixelBuffer::<Rgba8Pixel>::new(width, height);
    let mut pixmap =
    PixmapMut::from_bytes(pixel_buffer.make_mut_bytes(), width, height).expect("Pixmap inválido");

    pixmap.fill(Color::from_rgba8(10, 11, 16, 255));

    let cam = IsoCamera::new(building, width as f32, height as f32, view_mode, yaw_deg);
    let num_floors = building.floors.len();
    let floors_to_draw: Vec<usize> = if view_mode > 0 && (view_mode as usize) <= num_floors {
        vec![(view_mode - 1) as usize]
    } else {
        (0..num_floors).collect()
    };

    // Ordenar las baldosas de atrás hacia adelante según su profundidad en pantalla (y proyectada)
    let mut sorted_tiles: Vec<(usize, usize, f32)> =
    Vec::with_capacity(building.rows * building.cols);
    for r in 0..building.rows {
        for c in 0..building.cols {
            let rx = (c as f32 + 0.5 - cam.center_w_x) * cam.sin_a
            + (r as f32 + 0.5 - cam.center_w_y) * cam.cos_a;
            sorted_tiles.push((r, c, rx));
        }
    }
    sorted_tiles.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

    for &f in &floors_to_draw {
        let gw = building.grid_width_m;
        let gh = building.grid_height_m;

        // 1. Placa base de hormigón del piso en 3D con espesor de losa
        let p00 = cam.project(0.0, 0.0, 0.0, f);
        let p10 = cam.project(gw, 0.0, 0.0, f);
        let p11 = cam.project(gw, gh, 0.0, f);
        let p01 = cam.project(0.0, gh, 0.0, f);
        let b00 = cam.project(0.0, 0.0, -0.35, f);
        let b10 = cam.project(gw, 0.0, -0.35, f);
        let b11 = cam.project(gw, gh, -0.35, f);
        let b01 = cam.project(0.0, gh, -0.35, f);

        // Canto grueso de la losa de entrepiso
        fill_quad(&mut pixmap, [p01, p11, b11, b01], (20, 22, 32, 245));
        fill_quad(&mut pixmap, [p10, p11, b11, b10], (16, 18, 26, 245));
        fill_quad(&mut pixmap, [p00, p10, p11, p01], (26, 29, 42, 235));

        // 2. Conectores verticales de cajas de escaleras hacia el piso superior
        if f + 1 < num_floors && floors_to_draw.len() > 1 {
            for cell in building
                .cells
                .iter()
                .filter(|c| c.floor == f && c.kind == CellKind::Stairwell)
                {
                    let (cx, cy) = cell.center();
                    let s_low = cam.project(cx, cy, 0.2, f);
                    let s_high = cam.project(cx, cy, 0.0, f + 1);
                    draw_line(&mut pixmap, s_low.0, s_low.1, s_high.0, s_high.1, (56, 189, 248, 110), 2.0);
                }
        }

        // 3. Pintar todas las baldosas de suelo y ondas térmicas primero
        for &(r, c, _) in &sorted_tiles {
            let x0 = c as f32;
            let y0 = r as f32;
            let x1 = x0 + 1.0;
            let y1 = y0 + 1.0;

            let Some(cid) = building.cell_grid[f][r][c] else {
                continue;
            };
            let kind = building.cells[cid].kind;
            let base_col = match kind {
                CellKind::Room => (38, 41, 60, 255),
                CellKind::Corridor => (29, 32, 46, 255),
                CellKind::Stairwell => (48, 56, 86, 255),
                CellKind::Elevator { .. } => (36, 39, 54, 255),
                CellKind::Exit => (26, 82, 58, 255),
            };

            let q0 = cam.project(x0, y0, 0.0, f);
            let q1 = cam.project(x1, y0, 0.0, f);
            let q2 = cam.project(x1, y1, 0.0, f);
            let q3 = cam.project(x0, y1, 0.0, f);
            fill_quad(&mut pixmap, [q0, q1, q2, q3], base_col);

            // Borde sutil de baldosa 3D
            draw_line(&mut pixmap, q0.0, q0.1, q1.0, q1.1, (48, 52, 74, 85), 0.6);
            draw_line(&mut pixmap, q0.0, q0.1, q3.0, q3.1, (48, 52, 74, 85), 0.6);

            // Peldaños 3D si es escalera
            if kind == CellKind::Stairwell {
                let m1 = cam.project(x0, y0 + 0.5, 0.12, f);
                let m2 = cam.project(x1, y0 + 0.5, 0.12, f);
                draw_line(&mut pixmap, m1.0, m1.1, m2.0, m2.1, (148, 163, 184, 180), 1.1);
            }

            if show_thermal && building.walkable_grid[f][r][c] {
                let t = building.tile_temp[f][r][c];
                let smk = building.tile_smoke[f][r][c];
                if t > AMBIENT_TEMP_C + 2.0 {
                    let ratio = ((t - AMBIENT_TEMP_C) / (FLASHOVER_TEMP_C - AMBIENT_TEMP_C))
                    .clamp(0.0, 1.0);
                    let h_col = if ratio > 0.70 {
                        (220, 38, 38, (135.0 + ratio * 95.0) as u8)
                    } else if ratio > 0.35 {
                        (234, 88, 12, (110.0 + ratio * 95.0) as u8)
                    } else {
                        (234, 179, 8, (70.0 + ratio * 105.0) as u8)
                    };
                    fill_quad(&mut pixmap, [q0, q1, q2, q3], h_col);
                }
                if smk > 0.03 {
                    let sa = (smk.clamp(0.0, 1.0) * 125.0) as u8;
                    fill_quad(&mut pixmap, [q0, q1, q2, q3], (71, 85, 105, sa));
                }
            }
        }

        // 4. Pintar Muros Extruidos en 3D, Columnas, Fuego Volumétrico y Agentes ordenados por profundidad (Back-to-Front)
        for &(r, c, _) in &sorted_tiles {
            let ch = building.raw_floors[f][r][c];
            let x0 = c as f32;
            let y0 = r as f32;

            // Muros a media altura (corte arquitectónico 1.35m) o puertas cerradas
            if ch == '#' || ch == 'O' {
                let wall_h = if ch == 'O' { 1.55 } else { 1.15 };
                let top_col = if ch == 'O' {
                    (94, 104, 138, 245)
                } else {
                    (82, 90, 120, 230)
                };
                let side_a = (54, 60, 82, 230);
                let side_b = (40, 45, 64, 230);
                draw_iso_box(&mut pixmap, &cam, f, x0, y0, 1.0, 1.0, wall_h, top_col, side_a, side_b);
            } else if let Some(pid) = building.portal_at_tile[f][r][c] {
                let portal = &building.portals[pid];
                if portal.is_closed_or_blocked {
                    draw_iso_box(
                        &mut pixmap,
                        &cam,
                        f,
                        x0 + 0.15,
                        y0 + 0.15,
                        0.7,
                        0.7,
                        1.05,
                        (239, 68, 68, 220),
                                 (185, 28, 28, 220),
                                 (153, 27, 27, 220),
                    );
                } else {
                    // Marco verde/ámbar de puerta abierta en el piso
                    let d_col = if portal.max_resistance_s >= 100.0 {
                        (250, 179, 135, 210)
                    } else {
                        (52, 211, 153, 210)
                    };
                    let p_c = cam.project(x0 + 0.5, y0 + 0.5, 0.05, f);
                    draw_circle(&mut pixmap, p_c.0, p_c.1, cam.scale * 0.28, d_col);
                }
            }

            // Pluma vertical de fuego en 3D sobre la baldosa en ignición
            if show_thermal && building.walkable_grid[f][r][c] {
                let t = building.tile_temp[f][r][c];
                if t > 200.0 {
                    let ratio = ((t - AMBIENT_TEMP_C) / (FLASHOVER_TEMP_C - AMBIENT_TEMP_C))
                    .clamp(0.0, 1.0);
                    let f_pos = cam.project(x0 + 0.5, y0 + 0.5, 0.45 * ratio, f);
                    draw_flame_icon(&mut pixmap, f_pos.0, f_pos.1, cam.scale * (0.42 + ratio * 0.35));
                }
            }

            // Dibujar los agentes que están parados en esta baldosa (r, c) como peones 3D con cabeza y brazos
            for agent in agents.iter().filter(|a| {
                a.floor == f
                && a.action_state != AgentActionState::Evacuated
                && (a.y / TILE_SIZE_M).floor() as usize == r
                && (a.x / TILE_SIZE_M).floor() as usize == c
            }) {
                let foot = cam.project(agent.x, agent.y, 0.0, f);
                let head = cam.project(agent.x, agent.y, 0.85, f);
                let r_px = (agent.profile.body_radius() * cam.scale).max(3.2);

                if show_social_bonds
                    && agent.action_state != AgentActionState::Normal
                    && !agent.is_fallen
                    {
                        if let Some(lid) = agent.group_leader_id {
                            if lid != agent.id && agents[lid].floor == f {
                                let l_pos = cam.project(agents[lid].x, agents[lid].y, 0.4, f);
                                draw_line(
                                    &mut pixmap,
                                    foot.0,
                                    foot.1 - r_px,
                                    l_pos.0,
                                    l_pos.1,
                                    (192, 132, 252, 150),
                                          1.3,
                                );
                            }
                        }
                    }

                    // Sombra elíptica en la losa
                    draw_circle(&mut pixmap, foot.0, foot.1, r_px * 0.95, (5, 5, 10, 175));

                    if agent.is_fallen || agent.action_state == AgentActionState::Incapacitated {
                        let col = if agent.action_state == AgentActionState::Incapacitated {
                            (100, 116, 139, 255)
                        } else {
                            (251, 146, 60, 255)
                        };
                        draw_capsule(
                            &mut pixmap,
                            foot.0 - r_px * 1.1,
                            foot.1,
                            foot.0 + r_px * 1.1,
                            foot.1,
                            r_px * 0.65,
                            col,
                        );
                    } else {
                        let (body_col, head_col) = if agent.role == MacesRole::TrainedLeader {
                            ((20, 184, 166, 255), (255, 255, 255, 255))
                        } else {
                            match agent.profile {
                                DemographicProfile::Adult | DemographicProfile::YoungAdult => {
                                    ((59, 130, 246, 255), (219, 234, 254, 255))
                                }
                                DemographicProfile::Child => {
                                    ((245, 158, 11, 255), (254, 243, 199, 255))
                                }
                                DemographicProfile::Elderly | DemographicProfile::Impaired => {
                                    ((168, 85, 247, 255), (243, 232, 255, 255))
                                }
                            }
                        };

                        // Cilindro vertical del torso 3D + esfera de la cabeza
                        draw_capsule(
                            &mut pixmap,
                            foot.0,
                            foot.1,
                            head.0,
                            head.1 + r_px * 0.6,
                            r_px * 0.72,
                            body_col,
                        );
                        draw_circle(&mut pixmap, head.0, head.1, r_px * 0.68, head_col);

                        if agent.panic_level >= 5.5 && agent.action_state != AgentActionState::Normal {
                            draw_circle_outline(
                                &mut pixmap,
                                head.0,
                                head.1,
                                r_px * 1.25,
                                (239, 68, 68, 235),
                                                1.5,
                            );
                        }
                    }
            }
        }
    }

    Image::from_rgba8(pixel_buffer)
}

#[allow(clippy::too_many_arguments)]
fn draw_iso_box(
    pixmap: &mut PixmapMut,
    cam: &IsoCamera,
    floor: usize,
    x: f32,
    y: f32,
    w: f32,
    h: f32,
    z_height: f32,
    top_col: (u8, u8, u8, u8),
                side_a_col: (u8, u8, u8, u8),
                side_b_col: (u8, u8, u8, u8),
) {
    let b0 = cam.project(x, y, 0.0, floor);
    let b1 = cam.project(x + w, y, 0.0, floor);
    let b2 = cam.project(x + w, y + h, 0.0, floor);
    let b3 = cam.project(x, y + h, 0.0, floor);

    let t0 = cam.project(x, y, z_height, floor);
    let t1 = cam.project(x + w, y, z_height, floor);
    let t2 = cam.project(x + w, y + h, z_height, floor);
    let t3 = cam.project(x, y + h, z_height, floor);

    // Caras laterales visibles y tapa superior
    fill_quad(pixmap, [b3, b2, t2, t3], side_a_col);
    fill_quad(pixmap, [b1, b2, t2, t1], side_b_col);
    fill_quad(pixmap, [b0, b3, t3, t0], side_b_col);
    fill_quad(pixmap, [t0, t1, t2, t3], top_col);

    // Aristas superiores nítidas
    draw_line(pixmap, t0.0, t0.1, t1.0, t1.1, (148, 163, 184, 160), 0.8);
    draw_line(pixmap, t1.0, t1.1, t2.0, t2.1, (148, 163, 184, 160), 0.8);
    draw_line(pixmap, t2.0, t2.1, t3.0, t3.1, (148, 163, 184, 160), 0.8);
    draw_line(pixmap, t3.0, t3.1, t0.0, t0.1, (148, 163, 184, 160), 0.8);
}

fn fill_quad(pixmap: &mut PixmapMut, pts: [(f32, f32); 4], rgba: (u8, u8, u8, u8)) {
    let mut pb = PathBuilder::new();
    pb.move_to(pts[0].0, pts[0].1);
    pb.line_to(pts[1].0, pts[1].1);
    pb.line_to(pts[2].0, pts[2].1);
    pb.line_to(pts[3].0, pts[3].1);
    pb.close();
    if let Some(path) = pb.finish() {
        let mut p = Paint::default();
        p.set_color_rgba8(rgba.0, rgba.1, rgba.2, rgba.3);
        p.anti_alias = true;
        pixmap.fill_path(&path, &p, FillRule::Winding, Transform::identity(), None);
    }
}

/// Dibuja un Pictograma Estilo Señalética ISO (tipo figura de baño/emergencia) que cambia según el perfil y estado del agente seleccionado
pub fn render_agent_pictogram(selected_agent: Option<&Agent>, width: u32, height: u32) -> Image {
    let mut pixel_buffer = SharedPixelBuffer::<Rgba8Pixel>::new(width, height);
    let mut pixmap =
    PixmapMut::from_bytes(pixel_buffer.make_mut_bytes(), width, height).expect("Pixmap inválido");

    pixmap.fill(Color::from_rgba8(20, 20, 25, 255));

    let Some(agent) = selected_agent else {
        return Image::from_rgba8(pixel_buffer);
    };

    let cx = width as f32 * 0.5;
    let cy = height as f32 * 0.5;

    let fig_col = if agent.action_state == AgentActionState::Incapacitated {
        (113, 113, 122, 255)
    } else if agent.is_fallen {
        (249, 115, 22, 255)
    } else if agent.panic_level >= 5.5 && agent.action_state != AgentActionState::Normal {
        (239, 68, 68, 255)
    } else if agent.role == MacesRole::TrainedLeader {
        (20, 184, 166, 255)
    } else {
        match agent.profile {
            DemographicProfile::Adult | DemographicProfile::YoungAdult => (59, 130, 246, 255),
            DemographicProfile::Child => (245, 158, 11, 255),
            DemographicProfile::Elderly | DemographicProfile::Impaired => (168, 85, 247, 255),
        }
    };

    // 1. Si está Caído o Incapacitado: Pictograma horizontal tendido en el suelo
    if agent.is_fallen || agent.action_state == AgentActionState::Incapacitated {
        // Línea de suelo
        draw_line(&mut pixmap, 10.0, cy + 18.0, width as f32 - 10.0, cy + 18.0, (63, 63, 70, 255), 2.0);
        // Cabeza
        draw_circle(&mut pixmap, cx - 20.0, cy + 6.0, 7.0, fig_col);
        // Torso horizontal
        draw_capsule(&mut pixmap, cx - 10.0, cy + 9.0, cx + 10.0, cy + 11.0, 5.0, fig_col);
        // Piernas
        draw_capsule(&mut pixmap, cx + 10.0, cy + 11.0, cx + 26.0, cy + 12.0, 3.5, fig_col);
        if agent.action_state == AgentActionState::Incapacitated {
            // Señal de cruz médica arriba
            draw_line(&mut pixmap, cx - 6.0, cy - 16.0, cx + 6.0, cy - 4.0, (239, 68, 68, 255), 2.5);
            draw_line(&mut pixmap, cx + 6.0, cy - 16.0, cx - 6.0, cy - 4.0, (239, 68, 68, 255), 2.5);
        } else {
            // Brazo apoyándose para levantarse
            draw_capsule(&mut pixmap, cx - 8.0, cy + 9.0, cx - 12.0, cy + 16.0, 2.8, fig_col);
        }
        return Image::from_rgba8(pixel_buffer);
    }

    // 2. Caso Perfil Movilidad Reducida (Pictograma ISO de Silla de Ruedas)
    if agent.profile == DemographicProfile::Impaired {
        draw_circle(&mut pixmap, cx + 2.0, cy - 22.0, 6.5, fig_col); // Cabeza
        draw_capsule(&mut pixmap, cx, cy - 12.0, cx, cy + 2.0, 4.2, fig_col); // Espalda
        draw_capsule(&mut pixmap, cx, cy + 2.0, cx + 12.0, cy + 2.0, 3.5, fig_col); // Muslos
        draw_capsule(&mut pixmap, cx + 12.0, cy + 2.0, cx + 14.0, cy + 16.0, 3.0, fig_col); // Piernas
        draw_circle_outline(&mut pixmap, cx - 3.0, cy + 10.0, 11.5, (228, 228, 231, 255), 2.6); // Rueda
        return Image::from_rgba8(pixel_buffer);
    }

    // Escala vertical según edad (el Niño es más bajo)
    let s = if agent.profile == DemographicProfile::Child {
        0.76_f32
    } else {
        1.0_f32
    };
    let head_y = cy - 24.0 * s;

    // Cabeza estilo pictograma de baño/señalética (separada del torso)
    draw_circle(&mut pixmap, cx, head_y, 7.2 * s, fig_col);

    // Si es Brigadista, visera/casco blanco encima de la cabeza
    if agent.role == MacesRole::TrainedLeader {
        draw_capsule(
            &mut pixmap,
            cx - 6.5,
            head_y - 4.0,
            cx + 6.5,
            head_y - 4.0,
            2.6,
            (255, 255, 255, 255),
        );
    }

    let in_shock = matches!(
        agent.action_state,
        AgentActionState::PanickedFrozen { .. } | AgentActionState::PanickedWandering { .. }
    );
    let is_running = agent.action_state != AgentActionState::Normal && agent.avg_speed_m_s() > 0.3;

    // Torso estilo señalética
    let shoulder_y = cy - 11.0 * s;
    let hip_y = cy + 6.0 * s;
    draw_capsule(&mut pixmap, cx, shoulder_y, cx, hip_y, 5.5 * s, fig_col);

    // Brazos según estado
    if in_shock {
        // Manos arriba hacia la cabeza (¡Pánico / Shock!)
        draw_capsule(&mut pixmap, cx - 6.0 * s, shoulder_y, cx - 11.0 * s, head_y, 2.8 * s, fig_col);
        draw_capsule(&mut pixmap, cx + 6.0 * s, shoulder_y, cx + 11.0 * s, head_y, 2.8 * s, fig_col);
    } else if is_running {
        // Postura dinámica de carrera (como el ícono de Salida de Emergencia ISO 7010)
        draw_capsule(
            &mut pixmap,
            cx - 5.0 * s,
            shoulder_y,
            cx - 14.0 * s,
            shoulder_y + 6.0 * s,
            2.8 * s,
            fig_col,
        );
        draw_capsule(
            &mut pixmap,
            cx + 5.0 * s,
            shoulder_y,
            cx + 14.0 * s,
            shoulder_y - 4.0 * s,
            2.8 * s,
            fig_col,
        );
    } else {
        // Brazos relajados a los costados (estilo señal de baño)
        draw_capsule(
            &mut pixmap,
            cx - 6.5 * s,
            shoulder_y + 1.0,
            cx - 10.0 * s,
            hip_y + 2.0,
            2.8 * s,
            fig_col,
        );
        draw_capsule(
            &mut pixmap,
            cx + 6.5 * s,
            shoulder_y + 1.0,
            cx + 10.0 * s,
            hip_y + 2.0,
            2.8 * s,
            fig_col,
        );
    }

    // Piernas
    let foot_y = cy + 25.0 * s;
    if is_running {
        draw_capsule(&mut pixmap, cx - 2.5 * s, hip_y, cx - 11.0 * s, foot_y, 3.2 * s, fig_col);
        draw_capsule(&mut pixmap, cx + 2.5 * s, hip_y, cx + 11.0 * s, foot_y - 3.0 * s, 3.2 * s, fig_col);
    } else {
        draw_capsule(&mut pixmap, cx - 3.2 * s, hip_y, cx - 3.8 * s, foot_y, 3.2 * s, fig_col);
        draw_capsule(&mut pixmap, cx + 3.2 * s, hip_y, cx + 3.8 * s, foot_y, 3.2 * s, fig_col);
    }

    // Si es Adulto Mayor (Elderly), dibujar su bastón de apoyo en la mano derecha
    if agent.profile == DemographicProfile::Elderly {
        let cane_x = cx + 12.5;
        draw_line(
            &mut pixmap,
            cane_x,
            hip_y,
            cane_x + 2.0,
            foot_y + 1.0,
            (212, 212, 216, 255),
                  2.2,
        );
        draw_line(
            &mut pixmap,
            cane_x - 3.0,
            hip_y,
            cane_x + 1.0,
            hip_y,
            (212, 212, 216, 255),
                  2.2,
        );
    }

    Image::from_rgba8(pixel_buffer)
}

fn draw_top_down_human(pixmap: &mut PixmapMut, agent: &Agent, cx: f32, cy: f32, r: f32) {
    if agent.action_state == AgentActionState::Incapacitated {
        draw_capsule(
            pixmap,
            cx - r * 1.1,
            cy - r * 0.4,
            cx + r * 1.1,
            cy + r * 0.4,
            r * 0.75,
            (24, 24, 37, 220),
        );
        draw_capsule(
            pixmap,
            cx - r * 0.9,
            cy - r * 0.3,
            cx + r * 0.6,
            cy + r * 0.2,
            r * 0.62,
            (88, 91, 112, 255),
        );
        draw_circle(pixmap, cx + r * 0.95, cy + r * 0.3, r * 0.52, (127, 132, 156, 255));
        let s = r * 0.55;
        draw_line(pixmap, cx - s, cy - s, cx + s, cy + s, (243, 139, 168, 245), 1.8);
        draw_line(pixmap, cx + s, cy - s, cx - s, cy + s, (243, 139, 168, 245), 1.8);
        return;
    }

    if agent.is_fallen {
        draw_capsule(
            pixmap,
            cx - r * 1.0,
            cy,
            cx + r * 0.7,
            cy,
            r * 0.68,
            (250, 179, 135, 245),
        );
        draw_line(
            pixmap,
            cx + r * 0.3,
            cy,
            cx + r * 1.1,
            cy - r * 0.85,
            (250, 179, 135, 255),
                  r * 0.38,
        );
        draw_circle(pixmap, cx + r * 0.85, cy + r * 0.1, r * 0.55, (249, 226, 175, 255));
        draw_circle_outline(pixmap, cx, cy, r * 1.45, (250, 179, 135, 170), 1.2);
        return;
    }

    let v_mag = agent.vx.hypot(agent.vy);
    let (fx, fy) = if v_mag > 0.04 {
        (agent.vx / v_mag, agent.vy / v_mag)
    } else {
        (0.0, -1.0)
    };
    let (rx, ry) = (-fy, fx);

    let (torso_col, head_col) = if agent.role == MacesRole::TrainedLeader {
        ((20, 184, 166, 255), (240, 253, 250, 255))
    } else if matches!(
        agent.action_state,
        AgentActionState::FetchingExtinguisher { .. }
        | AgentActionState::FightingFire { .. }
        | AgentActionState::HelpingFallen { .. }
    ) {
        ((166, 227, 161, 255), (235, 245, 235, 255))
    } else {
        match agent.profile {
            DemographicProfile::Adult | DemographicProfile::YoungAdult => {
                ((59, 130, 246, 255), (219, 234, 254, 255))
            }
            DemographicProfile::Child => ((245, 158, 11, 255), (254, 243, 199, 255)),
            DemographicProfile::Elderly | DemographicProfile::Impaired => {
                ((168, 85, 247, 255), (243, 232, 255, 255))
            }
        }
    };

    let sh_half = r * 0.88;
    let sh_lx = cx - rx * sh_half;
    let sh_ly = cy - ry * sh_half;
    let sh_rx = cx + rx * sh_half;
    let sh_ry = cy + ry * sh_half;

    draw_capsule(
        pixmap,
        sh_lx + 1.2,
        sh_ly + 1.2,
        sh_rx + 1.2,
        sh_ry + 1.2,
        r * 0.62,
        (10, 10, 18, 160),
    );

    let in_high_panic = agent.panic_level >= 5.5 && agent.action_state != AgentActionState::Normal;
    if matches!(
        agent.action_state,
        AgentActionState::PanickedFrozen { .. } | AgentActionState::PanickedWandering { .. }
    ) {
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

    draw_capsule(pixmap, sh_lx, sh_ly, sh_rx, sh_ry, r * 0.58, torso_col);

    if agent.role == MacesRole::TrainedLeader {
        draw_line(pixmap, sh_lx, sh_ly, sh_rx, sh_ry, (255, 255, 255, 210), r * 0.22);
    }

    let head_x = cx + fx * (r * 0.22);
    let head_y = cy + fy * (r * 0.22);
    let head_r = r * 0.54;
    draw_circle(pixmap, head_x, head_y, head_r, head_col);

    if in_high_panic {
        draw_circle(pixmap, head_x, head_y, head_r * 0.58, (239, 68, 68, 255));
    }

    if matches!(agent.action_state, AgentActionState::Reacting { .. }) {
        draw_circle_outline(pixmap, cx, cy, r * 1.45, (250, 204, 21, 240), 1.5);
    }
}

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
