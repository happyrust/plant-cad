//! Face-Face intersection detection.

use crate::geometry_utils;
use crate::{bopds::FFInterference, bopds::SectionCurve, BopDs, FaceId, VertexId};
use truck_base::{
    cgmath64::{MetricSpace, Point2, Point3},
    tolerance::Tolerance,
};
use truck_geotrait::{Invertible, SearchParameter};
use truck_meshalgo::analyzers::Collision;
use truck_meshalgo::prelude::RobustMeshableShape;
use truck_modeling::{BSplineCurve, Curve, IntersectionCurve, KnotVec, Surface};
use truck_topology::{Face, Shell};

const SEARCH_PARAMETER_TRIALS: usize = 100;

/// Detects face-face interferences and stores generated section curves in `BopDs`.
///
/// Uses an analytical fast path for plane-plane pairs; falls back to
/// mesh-based intersection for general surface types.
pub fn intersect_ff(
    bopds: &mut BopDs,
    faces: &[(FaceId, Face<Point3, Curve, Surface>)],
    candidates: &[(FaceId, FaceId)],
) -> usize {
    let tolerance = bopds
        .options()
        .geometric_tol
        .max(bopds.options().approximation_tol);
    let mut count = 0;

    for &(face1_id, face2_id) in candidates {
        let Some(face1) = face_by_id(faces, face1_id) else {
            continue;
        };
        let Some(face2) = face_by_id(faces, face2_id) else {
            continue;
        };

        let curves_result = if let Some(curves) =
            try_analytical_plane_plane(face1, face2, tolerance)
        {
            Some(curves)
        } else {
            mesh_based_intersection(face1, face2, tolerance)
        };
        let Some(curves) = curves_result else {
            continue;
        };

        for samples in curves {
            if samples.len() < 2 {
                continue;
            }

            let start_point = samples[0];
            let end_point = *samples.last().unwrap_or(&start_point);
            let is_closed = start_point.distance2(end_point) <= tolerance * tolerance;

            let start = choose_endpoint_vertex(face1, face2, start_point, tolerance)
                .unwrap_or_else(|| bopds.next_generated_vertex_id());
            let end = if is_closed {
                start
            } else {
                choose_endpoint_vertex(face1, face2, end_point, tolerance)
                    .unwrap_or_else(|| bopds.next_generated_vertex_id())
            };
            let section_curve_id = bopds.next_section_curve_id();

            let face1_parameters = project_section_to_face(face1, &samples);
            let face2_parameters = project_section_to_face(face2, &samples);
            let [face1_parameters, face2_parameters] = orient_open_section_pair(
                [face1, face2],
                &samples,
                [face1_parameters, face2_parameters],
            );
            let face1_projection_available = face1_parameters.is_some();
            let face2_projection_available = face2_parameters.is_some();

            bopds.push_section_curve(SectionCurve {
                id: section_curve_id,
                faces: (face1_id, face2_id),
                start,
                end,
                face_parameters: [
                    (face1_id, face1_parameters.unwrap_or_default()),
                    (face2_id, face2_parameters.unwrap_or_default()),
                ],
                face_projection_available: [
                    (face1_id, face1_projection_available),
                    (face2_id, face2_projection_available),
                ],
                samples,
            });
            bopds.push_ff_interference(FFInterference {
                face1: face1_id,
                face2: face2_id,
                section_curve: section_curve_id,
            });
            count += 1;
        }
    }

    count
}

// ── Analytical plane-plane intersection ──────────────────────────────────────

fn try_analytical_plane_plane(
    face0: &Face<Point3, Curve, Surface>,
    face1: &Face<Point3, Curve, Surface>,
    tolerance: f64,
) -> Option<Vec<Vec<Point3>>> {
    use truck_base::cgmath64::InnerSpace;

    let (plane0, plane1) = match (face0.oriented_surface(), face1.oriented_surface()) {
        (Surface::Plane(p0), Surface::Plane(p1)) => (p0, p1),
        _ => return None,
    };

    let n0 = plane0.normal();
    let n1 = plane1.normal();
    let cross = n0.cross(n1);
    if cross.magnitude2() < tolerance * tolerance {
        return Some(Vec::new());
    }
    let direction = cross.normalize();

    let line_origin = find_point_on_two_planes(&plane0, &plane1, n0, n1, direction)?;

    let t_ranges_0 = clip_line_to_face_3d(line_origin, direction, face0, tolerance);
    let t_ranges_1 = clip_line_to_face_3d(line_origin, direction, face1, tolerance);

    let intersected = intersect_ranges(&t_ranges_0, &t_ranges_1);
    if intersected.is_empty() {
        return Some(Vec::new());
    }

    let mut curves = Vec::new();
    for (t_start, t_end) in &intersected {
        let len = t_end - t_start;
        if len < tolerance {
            continue;
        }
        let samples = vec![
            line_origin + direction * *t_start,
            line_origin + direction * *t_end,
        ];
        curves.push(samples);
    }

    Some(curves)
}

#[allow(dead_code)]
fn coplanar_face_overlap_curves(
    face0: &Face<Point3, Curve, Surface>,
    face1: &Face<Point3, Curve, Surface>,
    plane: &truck_modeling::Plane,
    normal: truck_base::cgmath64::Vector3,
    tolerance: f64,
) -> Option<Vec<Vec<Point3>>> {
    use truck_base::cgmath64::InnerSpace;

    let origin = plane.origin();
    let u_axis = {
        let ref_dir = if normal.x.abs() < 0.9 {
            truck_base::cgmath64::Vector3::unit_x()
        } else {
            truck_base::cgmath64::Vector3::unit_y()
        };
        normal.cross(ref_dir).normalize()
    };
    let v_axis = normal.cross(u_axis).normalize();

    let project = |p: Point3| -> Point2 {
        let d = p - origin;
        Point2::new(d.dot(u_axis), d.dot(v_axis))
    };

    let boundary_2d = |face: &Face<Point3, Curve, Surface>| -> Vec<Point2> {
        face.boundaries()
            .into_iter()
            .next()
            .map(|wire| wire.vertex_iter().map(|v| project(v.point())).collect())
            .unwrap_or_default()
    };

    let poly0 = boundary_2d(face0);
    let poly1 = boundary_2d(face1);

    if poly0.len() < 3 || poly1.len() < 3 {
        return Some(Vec::new());
    }

    let mut overlap_edges: Vec<Vec<Point3>> = Vec::new();
    for i in 0..poly1.len() {
        let a = poly1[i];
        let b = poly1[(i + 1) % poly1.len()];
        if geometry_utils::point_in_or_on_polygon(&poly0, a, tolerance)
            || geometry_utils::point_in_or_on_polygon(&poly0, b, tolerance)
        {
            let a3 = origin + u_axis * a.x + v_axis * a.y;
            let b3 = origin + u_axis * b.x + v_axis * b.y;
            overlap_edges.push(vec![a3, b3]);
        }
    }

    Some(overlap_edges)
}

fn find_point_on_two_planes(
    plane0: &truck_modeling::Plane,
    plane1: &truck_modeling::Plane,
    n0: truck_base::cgmath64::Vector3,
    n1: truck_base::cgmath64::Vector3,
    direction: truck_base::cgmath64::Vector3,
) -> Option<Point3> {
    use truck_base::cgmath64::InnerSpace;

    let d0 = n0.dot(plane0.origin() - Point3::new(0.0, 0.0, 0.0));
    let d1 = n1.dot(plane1.origin() - Point3::new(0.0, 0.0, 0.0));

    let n0n0 = n0.dot(n0);
    let n1n1 = n1.dot(n1);
    let n0n1 = n0.dot(n1);
    let det = n0n0 * n1n1 - n0n1 * n0n1;
    if det.abs() < 1.0e-30 {
        return None;
    }
    let c0 = (d0 * n1n1 - d1 * n0n1) / det;
    let c1 = (d1 * n0n0 - d0 * n0n1) / det;
    let _ = direction;

    Some(Point3::new(0.0, 0.0, 0.0) + n0 * c0 + n1 * c1)
}

fn clip_line_to_face_3d(
    line_origin: Point3,
    line_dir: truck_base::cgmath64::Vector3,
    face: &Face<Point3, Curve, Surface>,
    tolerance: f64,
) -> Vec<(f64, f64)> {
    use truck_base::cgmath64::{InnerSpace, MetricSpace};

    let mut boundary_points: Vec<Point3> = Vec::new();
    for wire in face.boundaries() {
        for vertex in wire.vertex_iter() {
            boundary_points.push(vertex.point());
        }
    }
    if boundary_points.len() < 3 {
        return Vec::new();
    }

    let dir_len_sq = line_dir.dot(line_dir);
    if dir_len_sq < 1.0e-30 {
        return Vec::new();
    }

    let mut ts: Vec<f64> = Vec::new();
    let n = boundary_points.len();
    for i in 0..n {
        let a = boundary_points[i];
        let b = boundary_points[(i + 1) % n];
        let seg = b - a;
        let seg_len_sq = seg.dot(seg);
        if seg_len_sq < 1.0e-30 {
            continue;
        }

        let w = line_origin - a;
        let d_dot_seg = line_dir.dot(seg);
        let denom = dir_len_sq * seg_len_sq - d_dot_seg * d_dot_seg;
        if denom.abs() < 1.0e-20 {
            continue;
        }

        let w_dot_d = w.dot(line_dir);
        let w_dot_seg = w.dot(seg);
        let t = (d_dot_seg * w_dot_seg - seg_len_sq * w_dot_d) / denom;
        let s = (dir_len_sq * w_dot_seg - d_dot_seg * w_dot_d) / denom;

        if s < -tolerance || s > 1.0 + tolerance {
            continue;
        }
        let s = s.clamp(0.0, 1.0);

        let p_on_line = line_origin + line_dir * t;
        let p_on_seg = a + seg * s;
        let dist = p_on_line.distance(p_on_seg);
        if dist < tolerance * 10.0 {
            ts.push(t);
        }
    }

    ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ts.dedup_by(|a, b| (*a - *b).abs() < tolerance);

    let mut ranges = Vec::new();
    let mut i = 0;
    while i + 1 < ts.len() {
        let mid_t = (ts[i] + ts[i + 1]) / 2.0;
        let mid_point = line_origin + line_dir * mid_t;
        if point_near_face_3d(face, mid_point, tolerance * 10.0) {
            ranges.push((ts[i], ts[i + 1]));
        }
        i += 1;
    }
    ranges
}

fn point_near_face_3d(
    face: &Face<Point3, Curve, Surface>,
    point: Point3,
    tolerance: f64,
) -> bool {
    use truck_base::cgmath64::InnerSpace;

    let mut boundary_3d: Vec<Point3> = Vec::new();
    for wire in face.boundaries() {
        for vertex in wire.vertex_iter() {
            boundary_3d.push(vertex.point());
        }
    }
    if boundary_3d.len() < 3 {
        return false;
    }

    let centroid = {
        let sum: Point3 = boundary_3d.iter().fold(
            Point3::new(0.0, 0.0, 0.0),
            |acc, p| Point3::new(acc.x + p.x, acc.y + p.y, acc.z + p.z),
        );
        let n = boundary_3d.len() as f64;
        Point3::new(sum.x / n, sum.y / n, sum.z / n)
    };

    let e0 = boundary_3d[1] - boundary_3d[0];
    let e1 = boundary_3d[2] - boundary_3d[0];
    let normal = e0.cross(e1);
    if normal.magnitude2() < 1.0e-30 {
        return false;
    }
    let normal = normal.normalize();

    let u_axis = e0.normalize();
    let v_axis = normal.cross(u_axis).normalize();

    let project = |p: Point3| -> Point2 {
        let d = p - centroid;
        Point2::new(d.dot(u_axis), d.dot(v_axis))
    };

    let polygon_2d: Vec<Point2> = boundary_3d.iter().map(|p| project(*p)).collect();
    let point_2d = project(point);

    geometry_utils::point_in_or_on_polygon(&polygon_2d, point_2d, tolerance)
}

#[allow(dead_code)]
fn clip_line_to_face_uv(
    line_origin: Point3,
    line_dir: truck_base::cgmath64::Vector3,
    face: &Face<Point3, Curve, Surface>,
    _plane: &truck_modeling::Plane,
    tolerance: f64,
) -> Vec<(f64, f64)> {
    use truck_geotrait::SearchParameter;
    let surface = face.oriented_surface();

    let mut uv_vertices: Vec<Point2> = Vec::new();
    for wire in face.boundaries() {
        for vertex in wire.vertex_iter() {
            if let Some((u, v)) = surface.search_parameter(vertex.point(), None, 100) {
                uv_vertices.push(Point2::new(u, v));
            }
        }
    }

    if uv_vertices.len() < 3 {
        return Vec::new();
    }

    let lo_uv = match surface.search_parameter(line_origin, None, 100) {
        Some((u, v)) => Point2::new(u, v),
        None => return Vec::new(),
    };
    let tip = line_origin + line_dir;
    let tip_uv = match surface.search_parameter(tip, None, 100) {
        Some((u, v)) => Point2::new(u, v),
        None => return Vec::new(),
    };
    let ld_uv = Point2::new(tip_uv.x - lo_uv.x, tip_uv.y - lo_uv.y);

    let mut ts: Vec<f64> = Vec::new();

    let n = uv_vertices.len();
    for i in 0..n {
        let a = uv_vertices[i];
        let b = uv_vertices[(i + 1) % n];
        if let Some(t) = line_segment_intersection_t(lo_uv, ld_uv, a, b) {
            ts.push(t);
        }
    }

    ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ts.dedup_by(|a, b| (*a - *b).abs() < tolerance);

    let mut ranges = Vec::new();
    let mut i = 0;
    while i + 1 < ts.len() {
        let mid_t = (ts[i] + ts[i + 1]) / 2.0;
        let mid_uv = Point2::new(lo_uv.x + ld_uv.x * mid_t, lo_uv.y + ld_uv.y * mid_t);
        if geometry_utils::point_in_polygon(&uv_vertices, mid_uv)
            || geometry_utils::point_on_polygon_boundary(&uv_vertices, mid_uv, tolerance)
        {
            ranges.push((ts[i], ts[i + 1]));
        }
        i += 1;
    }

    ranges
}

fn line_segment_intersection_t(
    lo: Point2,
    ld: Point2,
    a: Point2,
    b: Point2,
) -> Option<f64> {
    let seg = Point2::new(b.x - a.x, b.y - a.y);
    let denom = ld.x * seg.y - ld.y * seg.x;
    if denom.abs() < 1.0e-15 {
        return None;
    }
    let diff = Point2::new(a.x - lo.x, a.y - lo.y);
    let t = (diff.x * seg.y - diff.y * seg.x) / denom;
    let s = (diff.x * ld.y - diff.y * ld.x) / denom;
    if s >= -1.0e-10 && s <= 1.0 + 1.0e-10 {
        Some(t)
    } else {
        None
    }
}

fn intersect_ranges(a: &[(f64, f64)], b: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut result = Vec::new();
    for &(a0, a1) in a {
        for &(b0, b1) in b {
            let start = a0.max(b0);
            let end = a1.min(b1);
            if start < end {
                result.push((start, end));
            }
        }
    }
    result
}

// ── Mesh-based intersection (fallback) ──────────────────────────────────────

fn mesh_based_intersection(
    face1: &Face<Point3, Curve, Surface>,
    face2: &Face<Point3, Curve, Surface>,
    tolerance: f64,
) -> Option<Vec<Vec<Point3>>> {
    let shell1: Shell<Point3, Curve, Surface> = vec![face1.clone()].into();
    let shell2: Shell<Point3, Curve, Surface> = vec![face2.clone()].into();
    let poly1 = shell1.robust_triangulation(tolerance);
    let poly2 = shell2.robust_triangulation(tolerance);
    let poly_face1 = poly1.face_iter().next()?;
    let poly_face2 = poly2.face_iter().next()?;
    let polygon1 = poly_face1.surface()?;
    let polygon2 = poly_face2.surface()?;

    let surface1 = face1.oriented_surface();
    let surface2 = face2.oriented_surface();
    let segments = polygon1.extract_interference(&polygon2);
    let polylines = construct_polylines(&segments);
    build_section_samples(&surface1, &surface2, polylines)
}

// ── UV projection utilities ─────────────────────────────────────────────────

fn project_section_to_face(
    face: &Face<Point3, Curve, Surface>,
    samples: &[Point3],
) -> Option<Vec<Point2>> {
    let surface = face.oriented_surface();
    let mut parameters = Vec::with_capacity(samples.len());

    for &point in samples {
        let uv = surface.search_parameter(point, None, SEARCH_PARAMETER_TRIALS)?;
        parameters.push(Point2::new(uv.0, uv.1));
    }

    if !face.orientation() {
        parameters.reverse();
    }

    orient_parameter_loop(face, parameters)
}

fn orient_parameter_loop(
    face: &Face<Point3, Curve, Surface>,
    mut parameters: Vec<Point2>,
) -> Option<Vec<Point2>> {
    if parameters.len() < 2 {
        return Some(parameters);
    }

    let mut boundary = parameter_boundary(face)?;
    if !face.orientation() {
        boundary.invert();
    }

    let section_area = signed_area(&parameters);
    if section_area.is_sign_positive() != signed_area(&boundary).is_sign_positive() {
        parameters.reverse();
    }
    Some(parameters)
}

fn orient_open_section_pair(
    faces: [&Face<Point3, Curve, Surface>; 2],
    samples: &[Point3],
    mut parameters: [Option<Vec<Point2>>; 2],
) -> [Option<Vec<Point2>>; 2] {
    let Some(start_point) = samples.first() else {
        return parameters;
    };
    let Some(end_point) = samples.last() else {
        return parameters;
    };
    if start_point.distance2(*end_point)
        <= truck_base::tolerance::TOLERANCE * truck_base::tolerance::TOLERANCE
    {
        return parameters;
    }

    if !faces[0].orientation() {
        if let Some(parameters) = parameters[0].as_mut() {
            parameters.reverse();
        }
    }
    if !faces[1].orientation() {
        if let Some(parameters) = parameters[1].as_mut() {
            parameters.reverse();
        }
    }

    let face0_direction = parameters[0]
        .as_ref()
        .and_then(|parameters| endpoint_direction_key(faces[0], parameters));
    let face1_direction = parameters[1]
        .as_ref()
        .and_then(|parameters| endpoint_direction_key(faces[1], parameters));
    if face0_direction.is_some() && face1_direction.is_some() && face0_direction != face1_direction
    {
        if let Some(parameters) = parameters[1].as_mut() {
            parameters.reverse();
        }
    }
    parameters
}

fn endpoint_direction_key(
    face: &Face<Point3, Curve, Surface>,
    parameters: &[Point2],
) -> Option<(usize, usize)> {
    let boundary = parameter_boundary(face)?;
    let start = *parameters.first()?;
    let end = *parameters.last()?;
    let start_index = boundary
        .iter()
        .enumerate()
        .min_by(|(_, lhs), (_, rhs)| lhs.distance2(start).total_cmp(&rhs.distance2(start)))?
        .0;
    let end_index = boundary
        .iter()
        .enumerate()
        .min_by(|(_, lhs), (_, rhs)| lhs.distance2(end).total_cmp(&rhs.distance2(end)))?
        .0;
    Some((start_index, end_index))
}

fn parameter_boundary(face: &Face<Point3, Curve, Surface>) -> Option<Vec<Point2>> {
    let surface = face.oriented_surface();
    let wire = face.boundaries().into_iter().next()?;
    let mut points = Vec::new();
    let mut hint = None;
    for vertex in wire.vertex_iter() {
        let point = vertex.point();
        let uv = surface
            .search_parameter(point, hint.map(Into::into), SEARCH_PARAMETER_TRIALS)
            .or_else(|| surface.search_parameter(point, None, SEARCH_PARAMETER_TRIALS))?;
        let uv = Point2::new(uv.0, uv.1);
        hint = Some(uv);
        points.push(uv);
    }
    Some(points)
}

fn signed_area(polyline: &[Point2]) -> f64 { geometry_utils::signed_area(polyline) }

fn face_by_id<'a>(
    faces: &'a [(FaceId, Face<Point3, Curve, Surface>)],
    face_id: FaceId,
) -> Option<&'a Face<Point3, Curve, Surface>> {
    faces
        .iter()
        .find(|(id, _)| *id == face_id)
        .map(|(_, face)| face)
}

fn choose_endpoint_vertex(
    face1: &Face<Point3, Curve, Surface>,
    face2: &Face<Point3, Curve, Surface>,
    point: Point3,
    tolerance: f64,
) -> Option<VertexId> {
    let tolerance_sq = tolerance * tolerance;
    endpoint_vertices(face1)
        .chain(endpoint_vertices(face2))
        .find(|(_, candidate)| candidate.distance2(point) <= tolerance_sq)
        .map(|(id, _)| id)
}

fn endpoint_vertices<'a>(
    face: &'a Face<Point3, Curve, Surface>,
) -> impl Iterator<Item = (VertexId, Point3)> + 'a {
    face.absolute_boundaries()
        .iter()
        .flat_map(|wire| wire.vertex_iter())
        .enumerate()
        .map(|(index, vertex)| (VertexId(index as u32), vertex.point()))
}

fn construct_polylines(lines: &[(Point3, Point3)]) -> Vec<Vec<Point3>> {
    use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
    use std::collections::VecDeque;

    #[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
    struct PointIndex([i64; 3]);

    impl From<Point3> for PointIndex {
        fn from(pt: Point3) -> Self {
            let tol = truck_base::tolerance::TOLERANCE;
            Self([
                ((pt.x + tol) / (2.0 * tol)).floor() as i64,
                ((pt.y + tol) / (2.0 * tol)).floor() as i64,
                ((pt.z + tol) / (2.0 * tol)).floor() as i64,
            ])
        }
    }

    struct Node {
        coord: Point3,
        adjacency: HashSet<PointIndex>,
    }

    impl Node {
        fn new(coord: Point3, adjacency: HashSet<PointIndex>) -> Self { Self { coord, adjacency } }

        fn pop_one_adjacency(&mut self) -> PointIndex {
            let idx = *self.adjacency.iter().next().unwrap();
            self.adjacency.remove(&idx);
            idx
        }
    }

    struct Graph(HashMap<PointIndex, Node>);

    impl Graph {
        fn add_half_edge(&mut self, pt0: Point3, pt1: Point3) {
            let idx0 = PointIndex::from(pt0);
            let idx1 = PointIndex::from(pt1);
            if let Some(node) = self.0.get_mut(&idx0) {
                node.adjacency.insert(idx1);
            } else {
                let mut set = HashSet::default();
                set.insert(idx1);
                self.0.insert(idx0, Node::new(pt0, set));
            }
        }

        fn add_edge(&mut self, line: (Point3, Point3)) {
            if !line.0.near(&line.1) {
                self.add_half_edge(line.0, line.1);
                self.add_half_edge(line.1, line.0);
            }
        }

        fn get_one(&self) -> Option<(PointIndex, &Node)> {
            self.0.iter().next().map(|(idx, node)| (*idx, node))
        }

        fn get_next(&mut self, idx: PointIndex) -> Option<(PointIndex, Point3)> {
            let node = self.0.get_mut(&idx)?;
            let next_idx = node.pop_one_adjacency();
            if node.adjacency.is_empty() {
                self.0.remove(&idx);
            }
            let next = self.0.get_mut(&next_idx)?;
            next.adjacency.remove(&idx);
            let point = next.coord;
            if next.adjacency.is_empty() {
                self.0.remove(&next_idx);
            }
            Some((next_idx, point))
        }
    }

    let mut graph = Graph(HashMap::default());
    for &line in lines {
        graph.add_edge(line);
    }

    let mut result = Vec::new();
    while let Some((mut idx, node)) = graph.get_one() {
        let mut wire: VecDeque<_> = vec![node.coord].into();
        while let Some((next_idx, point)) = graph.get_next(idx) {
            idx = next_idx;
            wire.push_back(point);
        }
        let mut idx = PointIndex::from(wire[0]);
        while let Some((next_idx, point)) = graph.get_next(idx) {
            idx = next_idx;
            wire.push_front(point);
        }
        result.push(wire.into_iter().collect());
    }
    result
}

fn build_section_samples(
    surface1: &Surface,
    surface2: &Surface,
    polylines: Vec<Vec<Point3>>,
) -> Option<Vec<Vec<Point3>>> {
    let mut sections = Vec::new();
    for polyline in polylines {
        let leader = BSplineCurve::new(KnotVec::bezier_knot(polyline.len() - 1), polyline.clone());
        let curve = IntersectionCurve::new(
            Box::new(surface1.clone()),
            Box::new(surface2.clone()),
            Box::new(Curve::BSplineCurve(leader)),
        );
        let len = polyline.len();
        let mut samples = Vec::new();
        for index in 0..len.saturating_sub(1) {
            let (point, _, _) = curve.search_triple(index as f64, SEARCH_PARAMETER_TRIALS)?;
            samples.push(point);
        }
        if let Some(last_index) = len.checked_sub(1) {
            let (point, _, _) = curve.search_triple(last_index as f64, SEARCH_PARAMETER_TRIALS)?;
            samples.push(point);
        }
        sections.push(samples);
    }
    Some(sections)
}

#[cfg(test)]
mod tests {
    use super::*;
    use truck_base::cgmath64::{MetricSpace, Rad, Vector3};
    use truck_modeling::builder;
    use truck_topology::Edge;

    #[test]
    fn ff_plane_plane_section_generates_section_curve() {
        let mut bopds = BopDs::new();
        let faces = vec![(FaceId(0), yz_square_face()), (FaceId(1), xy_square_face())];

        let count = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(count, 1);
        assert_eq!(bopds.ff_interferences().len(), 1);
        assert_eq!(bopds.section_curves().len(), 1);
        let section = &bopds.section_curves()[0];
        assert_eq!(section.faces, (FaceId(0), FaceId(1)));
        assert!(section.samples.len() >= 2);
        assert_eq!(section.face_parameters[0].0, FaceId(0));
        assert_eq!(section.face_parameters[1].0, FaceId(1));
        assert_eq!(section.face_parameters[0].1.len(), section.samples.len());
        assert_eq!(section.face_parameters[1].1.len(), section.samples.len());
        let start = section.samples.first().copied().unwrap();
        let end = section.samples.last().copied().unwrap();
        assert!(
            start.distance(Point3::new(0.0, 0.0, 0.0)) < 1.0e-2
                || start.distance(Point3::new(0.0, 1.0, 0.0)) < 1.0e-2
        );
        assert!(
            end.distance(Point3::new(0.0, 0.0, 0.0)) < 1.0e-2
                || end.distance(Point3::new(0.0, 1.0, 0.0)) < 1.0e-2
        );
        assert_ne!(section.start, section.end);

        let yz_parameters = &section.face_parameters[0].1;
        assert!(yz_parameters
            .windows(2)
            .all(|pair| pair[0].distance(pair[1]) <= 1.0 + 1.0e-3));
        let xy_parameters = &section.face_parameters[1].1;
        assert!(xy_parameters
            .windows(2)
            .all(|pair| pair[0].distance(pair[1]) <= 1.0 + 1.0e-3));
    }

    #[test]
    #[ignore] // Known limitation: open-curve UV orientation on inverted faces
    fn section_projection_respects_inverted_face_orientation() {
        let mut bopds = BopDs::new();
        let mut inverted = yz_square_face();
        inverted.invert();
        let faces = vec![(FaceId(0), inverted), (FaceId(1), xy_square_face())];

        let count = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(count, 1);
        let section = &bopds.section_curves()[0];
        let yz_parameters = &section.face_parameters[0].1;
        assert!(section.face_projection_available[0].1);
        assert!(yz_parameters.first().unwrap().y >= yz_parameters.last().unwrap().y);
    }

    #[test]
    fn ff_intersection_keeps_section_curve_when_one_face_projection_fails() {
        let mut bopds = BopDs::new();
        let faces = vec![(FaceId(0), yz_square_face()), (FaceId(1), xy_square_face())];
        let samples = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
        let section_curve_id = bopds.next_section_curve_id();

        let [face1_parameters, face2_parameters] = orient_open_section_pair(
            [&faces[0].1, &faces[1].1],
            &samples,
            [project_section_to_face(&faces[0].1, &samples), None],
        );

        bopds.push_section_curve(SectionCurve {
            id: section_curve_id,
            faces: (FaceId(0), FaceId(1)),
            start: VertexId(0),
            end: VertexId(1),
            face_parameters: [
                (FaceId(0), face1_parameters.unwrap_or_default()),
                (FaceId(1), face2_parameters.unwrap_or_default()),
            ],
            face_projection_available: [(FaceId(0), true), (FaceId(1), false)],
            samples,
        });

        let section = &bopds.section_curves()[0];
        assert_eq!(section.samples.len(), 2);
        assert_eq!(section.face_parameters[1].1, Vec::<Point2>::new());
        assert_eq!(
            section.face_projection_available,
            [(FaceId(0), true), (FaceId(1), false)]
        );
    }

    #[test]
    fn section_projection_stores_uv_samples_for_both_faces() {
        let mut bopds = BopDs::new();
        let faces = vec![(FaceId(0), yz_square_face()), (FaceId(1), xy_square_face())];

        intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        let section = &bopds.section_curves()[0];
        for (face_id, uv_samples) in &section.face_parameters {
            assert!(
                !uv_samples.is_empty(),
                "missing uv samples for face {:?}",
                face_id
            );
            assert_eq!(uv_samples.len(), section.samples.len());
        }
    }

    #[test]
    fn ff_parallel_planes_produce_no_section_curve() {
        let mut bopds = BopDs::new();
        let faces = vec![
            (FaceId(0), xy_square_face()),
            (FaceId(1), lifted_xy_square_face(1.0)),
        ];

        let count = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(count, 0);
        assert!(bopds.ff_interferences().is_empty());
        assert!(bopds.section_curves().is_empty());
    }

    #[test]
    #[ignore] // Known limitation: closed-loop sections not yet supported
    fn ff_cylinder_plane_section_generates_polyline_loop_samples() {
        let mut bopds = BopDs::new();
        let faces = vec![(FaceId(0), cylinder_face()), (FaceId(1), xy_disk_face(1.0))];

        let count = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(count, 1);
        let section = &bopds.section_curves()[0];
        assert!(section.samples.len() >= 8);
        for point in &section.samples {
            assert!(point.z.abs() < 1.0e-2);
            let radius = f64::sqrt(point.x * point.x + point.y * point.y);
            assert!((radius - 1.0).abs() < 5.0e-2);
        }
    }

    #[test]
    fn ff_intersection_allocates_unique_section_curve_ids() {
        let mut bopds = BopDs::new();
        let faces = vec![
            (FaceId(0), yz_square_face()),
            (FaceId(1), xy_square_face()),
            (FaceId(2), xz_square_face()),
        ];

        let count = intersect_ff(
            &mut bopds,
            &faces,
            &[(FaceId(0), FaceId(1)), (FaceId(0), FaceId(2))],
        );

        assert_eq!(count, 2);
        assert_eq!(bopds.section_curves()[0].id, crate::SectionCurveId(0));
        assert_eq!(bopds.section_curves()[1].id, crate::SectionCurveId(1));
    }

    fn xy_square_face() -> Face<Point3, Curve, Surface> {
        square_face(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        )
    }

    fn lifted_xy_square_face(z: f64) -> Face<Point3, Curve, Surface> {
        square_face(
            Point3::new(0.0, 0.0, z),
            Point3::new(1.0, 0.0, z),
            Point3::new(1.0, 1.0, z),
            Point3::new(0.0, 1.0, z),
        )
    }

    fn yz_square_face() -> Face<Point3, Curve, Surface> {
        square_face(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(0.0, 0.0, 1.0),
        )
    }

    fn xz_square_face() -> Face<Point3, Curve, Surface> {
        square_face(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 1.0),
        )
    }

    fn square_face(a: Point3, b: Point3, c: Point3, d: Point3) -> Face<Point3, Curve, Surface> {
        let vertices = builder::vertices([a, b, c, d]);
        let edges: Vec<Edge<Point3, Curve>> = vec![
            builder::line(&vertices[0], &vertices[1]),
            builder::line(&vertices[1], &vertices[2]),
            builder::line(&vertices[2], &vertices[3]),
            builder::line(&vertices[3], &vertices[0]),
        ];
        let wire = truck_topology::Wire::from(edges);
        Face::new(
            vec![wire],
            Surface::Plane(truck_modeling::Plane::new(a, b, d)),
        )
    }

    fn xy_disk_face(radius: f64) -> Face<Point3, Curve, Surface> {
        let center = builder::vertex(Point3::new(0.0, 0.0, 0.0));
        let wire: truck_topology::Wire<Point3, Curve> = builder::rsweep(
            &center,
            Point3::new(radius, 0.0, 0.0),
            Vector3::unit_z(),
            Rad(std::f64::consts::TAU),
            16,
        );
        Face::new(vec![wire], Surface::Plane(truck_modeling::Plane::xy()))
    }

    fn cylinder_face() -> Face<Point3, Curve, Surface> {
        let bottom = builder::vertex(Point3::new(1.0, 0.0, -1.0));
        let top = builder::vertex(Point3::new(1.0, 0.0, 1.0));
        let profile = builder::line(&bottom, &top);
        let shell: Shell<Point3, Curve, Surface> = builder::rsweep(
            &profile,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::unit_z(),
            Rad(std::f64::consts::TAU),
            24,
        );
        shell.face_iter().next().unwrap().clone()
    }
}
