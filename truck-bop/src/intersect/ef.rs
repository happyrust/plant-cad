//! Edge-Face intersection detection.

use crate::{
    bopds::{CommonBlock, EFInterference, Pave},
    BopDs, EdgeId, FaceId, VertexId,
};
use truck_base::cgmath64::{EuclideanSpace, InnerSpace, MetricSpace, Point2, Point3};
use truck_geotrait::{
    algo::surface, BoundedCurve, Invertible, ParametricCurve, ParametricSurface,
    ParametricSurface3D, SearchNearestParameter,
};
use truck_topology::{Edge, Face};

const SEARCH_PARAMETER_TRIALS: usize = 100;
const SURFACE_SAMPLES: usize = 64;
const ENDPOINT_PARAMETER_EPS: f64 = 1.0e-9;

/// Detects edge-face intersections and stores them in `BopDs`.
pub fn intersect_ef<C, S>(
    bopds: &mut BopDs,
    edges: &[(EdgeId, Edge<Point3, C>)],
    faces: &[(FaceId, Face<Point3, C, S>)],
    candidates: &[(EdgeId, FaceId)],
) -> usize
where
    C: Clone
        + BoundedCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + Invertible
        + ParametricCurve,
    S: Clone
        + Invertible
        + ParametricSurface<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + ParametricSurface3D
        + SearchNearestParameter<truck_geotrait::D2, Point = Point3>,
{
    let tolerance = bopds.options().geometric_tol;
    let tolerance_sq = tolerance * tolerance;

    let mut count = 0;
    for &(edge_id, face_id) in candidates {
        let Some(edge) = edge_by_id(edges, edge_id) else {
            continue;
        };
        let Some(face) = face_by_id(faces, face_id) else {
            continue;
        };

        let curve = edge.oriented_curve();
        let surface_geometry = face.oriented_surface();
        let surface = &surface_geometry;
        let (start, end) = curve.range_tuple();
        let sampled_hits = detect_sampled_hits(&curve, surface, face, start, end, tolerance);

        if (end - start).abs() <= tolerance {
            let point = curve.subs(start);
            let Some(uv) = project_point(surface, point, face) else {
                continue;
            };
            if surface.subs(uv.0, uv.1).distance2(point) <= tolerance_sq
                && point_projects_inside_face(face, uv, tolerance)
                && push_unique_intersection(bopds, edge_id, face_id, start, uv, tolerance)
            {
                count += 1;
            }
            continue;
        }
        let mut local_count = 0;

        for (parameter, uv) in sampled_hits {
            if push_unique_intersection(
                bopds,
                edge_id,
                face_id,
                edge_parameter_hint(parameter, start, end),
                uv,
                tolerance,
            ) {
                count += 1;
                local_count += 1;
            }
        }

        if local_count == 0 {
            let fallback_hits =
                detect_fallback_hits(&curve, surface, face, start, end, tolerance, tolerance_sq);
            for (parameter, uv) in fallback_hits {
                if push_unique_intersection(
                    bopds,
                    edge_id,
                    face_id,
                    edge_parameter_hint(parameter, start, end),
                    uv,
                    tolerance,
                ) {
                    count += 1;
                }
            }
        }
    }

    count
}

fn detect_sampled_hits<C, S>(
    curve: &C,
    surface: &S,
    face: &Face<Point3, C, S>,
    start: f64,
    end: f64,
    tolerance: f64,
) -> Vec<(f64, (f64, f64))>
where
    C: ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>,
    S: ParametricSurface<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + ParametricSurface3D
        + SearchNearestParameter<truck_geotrait::D2, Point = Point3>,
{
    let tolerance_sq = tolerance * tolerance;
    let samples = sample_parameters((start, end), SURFACE_SAMPLES);
    let projections: Vec<(f64, Option<((f64, f64), f64, bool)>)> = samples
        .into_iter()
        .map(|parameter| {
            let point = curve.subs(parameter);
            let projection = project_point(surface, point, face).map(|uv| {
                let distance_sq = surface.subs(uv.0, uv.1).distance2(point);
                let inside = point_projects_inside_face(face, uv, tolerance);
                (uv, distance_sq, inside)
            });
            (parameter, projection)
        })
        .collect();

    let mut hits = Vec::new();

    for (parameter, projection) in &projections {
        let Some((uv, distance_sq, inside)) = projection else {
            continue;
        };
        if *distance_sq <= tolerance_sq && *inside {
            push_unique_sampled_hit(&mut hits, *parameter, *uv, tolerance);
        }
    }

    for window in projections.windows(3) {
        let [(_t0, p0), (t1, p1), (_t2, p2)] = window else {
            continue;
        };
        let (
            Some((_uv0, dist0, inside0)),
            Some((uv1, dist1, inside1)),
            Some((_uv2, dist2, inside2)),
        ) = (p0, p1, p2)
        else {
            continue;
        };

        let hit_by_valley = *inside1 && *dist1 <= *dist0 && *dist1 <= *dist2;
        let hit_by_crossing = (*inside0 || *inside1 || *inside2)
            && ((*dist0 > tolerance_sq && *dist1 <= tolerance_sq)
                || (*dist1 <= tolerance_sq && *dist2 > tolerance_sq)
                || (*dist0 > tolerance_sq
                    && *dist2 > tolerance_sq
                    && *dist1 <= (*dist0).min(*dist2)));

        if !hit_by_valley && !hit_by_crossing {
            continue;
        }

        let hint = *uv1;
        let Some((uv, parameter)) = surface::search_intersection_parameter(
            surface,
            hint,
            curve,
            *t1,
            SEARCH_PARAMETER_TRIALS,
        ) else {
            continue;
        };

        if parameter < start - tolerance || parameter > end + tolerance {
            continue;
        }

        let point = curve.subs(parameter);
        if surface.subs(uv.0, uv.1).distance2(point) > tolerance_sq {
            continue;
        }

        if !point_projects_inside_face(face, uv, tolerance) {
            continue;
        }

        push_unique_sampled_hit(&mut hits, parameter, uv, tolerance);
    }

    hits
}

fn detect_fallback_hits<C, S>(
    curve: &C,
    surface: &S,
    face: &Face<Point3, C, S>,
    start: f64,
    end: f64,
    tolerance: f64,
    tolerance_sq: f64,
) -> Vec<(f64, (f64, f64))>
where
    C: ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>,
    S: ParametricSurface<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + ParametricSurface3D
        + SearchNearestParameter<truck_geotrait::D2, Point = Point3>,
{
    let mut hits = Vec::new();
    for &parameter in &[start, end] {
        let point = curve.subs(parameter);
        let Some(uv) = project_point(surface, point, face) else {
            continue;
        };
        if surface.subs(uv.0, uv.1).distance2(point) > tolerance_sq {
            continue;
        }
        if !point_projects_inside_face(face, uv, tolerance) {
            continue;
        }
        push_unique_sampled_hit(&mut hits, parameter, uv, tolerance);
    }
    hits
}

fn push_unique_sampled_hit(
    hits: &mut Vec<(f64, (f64, f64))>,
    parameter: f64,
    surface_parameters: (f64, f64),
    tolerance: f64,
) {
    if hits.iter().any(|(existing_parameter, existing_uv)| {
        (*existing_parameter - parameter).abs() <= tolerance
            && (existing_uv.0 - surface_parameters.0).abs() <= tolerance
            && (existing_uv.1 - surface_parameters.1).abs() <= tolerance
    }) {
        return;
    }
    hits.push((parameter, surface_parameters));
}

fn edge_by_id<C>(edges: &[(EdgeId, Edge<Point3, C>)], edge_id: EdgeId) -> Option<&Edge<Point3, C>> {
    edges
        .iter()
        .find(|(id, _)| *id == edge_id)
        .map(|(_, edge)| edge)
}

fn face_by_id<C, S>(
    faces: &[(FaceId, Face<Point3, C, S>)],
    face_id: FaceId,
) -> Option<&Face<Point3, C, S>> {
    faces
        .iter()
        .find(|(id, _)| *id == face_id)
        .map(|(_, face)| face)
}

fn sample_parameters(range: (f64, f64), division: usize) -> Vec<f64> {
    let (start, end) = range;
    let step = (end - start) / division as f64;
    (0..=division)
        .map(|index| start + step * index as f64)
        .collect()
}


fn project_point<C, S>(
    surface: &S,
    point: Point3,
    face: &Face<Point3, C, S>,
) -> Option<(f64, f64)>
where
    S: SearchNearestParameter<truck_geotrait::D2, Point = Point3>
        + ParametricSurface<Point = Point3>,
    C: Clone,
{
    surface.search_nearest_parameter(point, SPHint::from_face(face), SEARCH_PARAMETER_TRIALS)
}

fn edge_parameter_hint(parameter: f64, start: f64, end: f64) -> f64 {
    if (parameter - start).abs() <= ENDPOINT_PARAMETER_EPS {
        start
    } else if (parameter - end).abs() <= ENDPOINT_PARAMETER_EPS {
        end
    } else {
        parameter
    }
}

fn push_unique_intersection(
    bopds: &mut BopDs,
    edge_id: EdgeId,
    face_id: FaceId,
    parameter: f64,
    surface_parameters: (f64, f64),
    tolerance: f64,
) -> bool {
    if bopds.ef_interferences().iter().any(|existing| {
        existing.edge == edge_id
            && existing.face == face_id
            && (existing.parameter - parameter).abs() <= tolerance
            && (existing.surface_parameters.0 - surface_parameters.0).abs() <= tolerance
            && (existing.surface_parameters.1 - surface_parameters.1).abs() <= tolerance
    }) {
        return false;
    }

    let Some((pave_block_id, vertex_id)) = register_edge_face_fact(
        bopds,
        edge_id,
        face_id,
        parameter,
        surface_parameters,
        tolerance,
    ) else {
        return false;
    };
    bopds.push_ef_interference(EFInterference {
        edge: edge_id,
        face: face_id,
        parameter,
        surface_parameters,
    });
    bind_face_info_from_edge_face_fact(
        bopds,
        face_id,
        parameter,
        surface_parameters,
        pave_block_id,
        vertex_id,
    );
    true
}

fn register_edge_face_fact(
    bopds: &mut BopDs,
    edge_id: EdgeId,
    face_id: FaceId,
    parameter: f64,
    surface_parameters: (f64, f64),
    tolerance: f64,
) -> Option<(crate::PaveBlockId, VertexId)> {
    ensure_edge_endpoint_paves(bopds, edge_id, tolerance);
    let vertex_id = existing_vertex_for_parameter(bopds, edge_id, parameter, tolerance)
        .unwrap_or_else(|| bopds.next_generated_vertex_id());
    let pave = Pave::new(edge_id, vertex_id, parameter, tolerance).ok()?;
    let appended = if parameter_hits_endpoint(bopds, edge_id, parameter, tolerance) {
        false
    } else {
        bopds.append_ext_pave(edge_id, pave)
    };
    if appended {
        bopds.split_pave_blocks_for_edge(edge_id);
    } else {
        bopds.push_pave(pave);
        bopds.rebuild_pave_blocks_from_paves(edge_id);
    }

    let pave_block_id = locate_pave_block_for_parameter(bopds, edge_id, parameter, tolerance)?;
    register_overlap_common_block(bopds, edge_id, pave_block_id, face_id, surface_parameters);
    Some((pave_block_id, vertex_id))
}

fn ensure_edge_endpoint_paves(
    bopds: &mut BopDs,
    edge_id: EdgeId,
    tolerance: f64,
) {
    let edge_paves = bopds.paves_for_edge(edge_id);
    if edge_paves.len() >= 2 {
        return;
    }

    let start_vertex = bopds.next_generated_vertex_id();
    let end_vertex = bopds.next_generated_vertex_id();
    if edge_paves.is_empty() || !edge_paves.iter().any(|pave| (pave.parameter - 0.0).abs() <= tolerance) {
        if let Ok(start) = Pave::new(edge_id, start_vertex, 0.0, tolerance) {
            bopds.push_pave(start);
        }
    }
    if edge_paves.is_empty() || !edge_paves.iter().any(|pave| (pave.parameter - 1.0).abs() <= tolerance) {
        if let Ok(end) = Pave::new(edge_id, end_vertex, 1.0, tolerance) {
            bopds.push_pave(end);
        }
    }
}

fn existing_vertex_for_parameter(
    bopds: &BopDs,
    edge_id: EdgeId,
    parameter: f64,
    tolerance: f64,
) -> Option<VertexId> {
    bopds
        .paves_for_edge(edge_id)
        .into_iter()
        .find(|pave| (pave.parameter - parameter).abs() <= tolerance.max(pave.tolerance))
        .map(|pave| pave.vertex)
}

fn parameter_hits_endpoint(
    bopds: &BopDs,
    edge_id: EdgeId,
    parameter: f64,
    tolerance: f64,
) -> bool {
    let paves = bopds.paves_for_edge(edge_id);
    let Some(first) = paves.first() else {
        return false;
    };
    let Some(last) = paves.last() else {
        return false;
    };
    (parameter - first.parameter).abs() <= tolerance.max(first.tolerance)
        || (parameter - last.parameter).abs() <= tolerance.max(last.tolerance)
}

fn locate_pave_block_for_parameter(
    bopds: &BopDs,
    edge_id: EdgeId,
    parameter: f64,
    tolerance: f64,
) -> Option<crate::PaveBlockId> {
    let matching_blocks: Vec<(usize, &crate::PaveBlock)> = bopds
        .pave_blocks()
        .iter()
        .enumerate()
        .filter(|(_, block)| {
            block.original_edge == edge_id
                && parameter + tolerance >= block.param_range.0
                && parameter - tolerance <= block.param_range.1
        })
        .collect();

    if let Some((index, _)) = matching_blocks
        .iter()
        .find(|(_, block)| block.param_range.1 - block.param_range.0 > tolerance)
    {
        return Some(crate::PaveBlockId(*index as u32));
    }

    matching_blocks
        .first()
        .map(|(index, _)| crate::PaveBlockId(*index as u32))
}

fn register_overlap_common_block(
    bopds: &mut BopDs,
    edge_id: EdgeId,
    pave_block_id: crate::PaveBlockId,
    face_id: FaceId,
    surface_parameters: (f64, f64),
) {
    if !surface_parameters_on_boundary(surface_parameters) {
        return;
    }

    if let Some(common_block_id) = bopds.common_block_for_pave_block(pave_block_id) {
        if let Some(common_block) = bopds.common_block(common_block_id).cloned() {
            let mut updated = common_block;
            updated.push_face(face_id);
            let _ = bopds.update_common_block(common_block_id, updated);
        }
        return;
    }

    bopds.push_common_block(CommonBlock::new(
        vec![pave_block_id],
        vec![face_id],
        Some(edge_id),
    ));
}

fn bind_face_info_from_edge_face_fact(
    bopds: &mut BopDs,
    face_id: FaceId,
    parameter: f64,
    surface_parameters: (f64, f64),
    pave_block_id: crate::PaveBlockId,
    vertex_id: VertexId,
) {
    let common_block_paves = bopds
        .common_block_for_pave_block(pave_block_id)
        .and_then(|common_block_id| bopds.common_block(common_block_id))
        .map(|common_block| common_block.pave_blocks.clone());
    let is_boundary = surface_parameters_on_boundary(surface_parameters);
    if let Some(common_block_paves) = common_block_paves {
        bopds.push_face_on_vertex(face_id, vertex_id);
        bopds.push_face_sc_vertex(face_id, vertex_id);
        bopds.push_face_on_pave_block(face_id, pave_block_id);
        bopds.push_face_sc_pave_block(face_id, pave_block_id);
        for block in common_block_paves {
            bopds.push_face_sc_pave_block(face_id, block);
        }
        return;
    }

    if parameter.is_finite() {
        if is_boundary {
            bopds.push_face_on_vertex(face_id, vertex_id);
            bopds.push_face_on_pave_block(face_id, pave_block_id);
        } else {
            bopds.push_face_in_vertex(face_id, vertex_id);
            bopds.push_face_in_pave_block(face_id, pave_block_id);
        }
    }
}

fn surface_parameters_on_boundary(surface_parameters: (f64, f64)) -> bool {
    let (u, v) = surface_parameters;
    let tolerance = 1.0e-9;
    u.abs() <= tolerance
        || v.abs() <= tolerance
        || (u - 1.0).abs() <= tolerance
        || (v - 1.0).abs() <= tolerance
}

fn point_projects_inside_face<C, S>(
    face: &Face<Point3, C, S>,
    parameters: (f64, f64),
    tolerance: f64,
) -> bool
where
    C: Clone,
{
    let uv = Point2::new(parameters.0, parameters.1);
    let mut boundaries = face.boundaries().into_iter();
    let Some(outer) = boundaries.next() else {
        return false;
    };

    point_on_or_inside_wire(&outer, uv, tolerance)
        && boundaries.all(|hole| !point_strictly_inside_wire(&hole, uv, tolerance))
}

fn point_on_or_inside_wire<C>(
    wire: &truck_topology::Wire<Point3, C>,
    point: Point2,
    tolerance: f64,
) -> bool
where
    C: Clone,
{
    let polygon: Vec<Point2> = wire
        .vertex_iter()
        .map(|vertex| Point2::new(vertex.point().x, vertex.point().y))
        .collect();
    point_in_polygon(&polygon, point, tolerance)
}

fn point_strictly_inside_wire<C>(
    wire: &truck_topology::Wire<Point3, C>,
    point: Point2,
    tolerance: f64,
) -> bool
where
    C: Clone,
{
    let polygon: Vec<Point2> = wire
        .vertex_iter()
        .map(|vertex| Point2::new(vertex.point().x, vertex.point().y))
        .collect();
    point_strictly_in_polygon(&polygon, point, tolerance)
}

fn point_in_polygon(polygon: &[Point2], point: Point2, tolerance: f64) -> bool {
    if polygon.len() < 3 {
        return false;
    }

    if polygon
        .windows(2)
        .any(|edge| point_on_segment(point, edge[0], edge[1], tolerance))
        || point_on_segment(
            point,
            *polygon.last().expect("polygon has at least 3 vertices"),
            polygon[0],
            tolerance,
        )
    {
        return true;
    }

    let mut inside = false;
    let mut prev = *polygon.last().expect("polygon has at least 3 vertices");
    for &curr in polygon {
        let intersects = ((curr.y > point.y) != (prev.y > point.y))
            && (point.x
                < (prev.x - curr.x) * (point.y - curr.y)
                    / ((prev.y - curr.y).abs().max(f64::EPSILON))
                    + curr.x);
        if intersects {
            inside = !inside;
        }
        prev = curr;
    }

    inside
}

fn point_strictly_in_polygon(polygon: &[Point2], point: Point2, tolerance: f64) -> bool {
    point_in_polygon(polygon, point, tolerance)
        && !point_on_polygon_boundary(polygon, point, tolerance)
}

fn point_on_polygon_boundary(polygon: &[Point2], point: Point2, tolerance: f64) -> bool {
    polygon
        .windows(2)
        .any(|edge| point_on_segment(point, edge[0], edge[1], tolerance))
        || point_on_segment(
            point,
            *polygon.last().expect("polygon has at least 3 vertices"),
            polygon[0],
            tolerance,
        )
}

fn point_on_segment(point: Point2, start: Point2, end: Point2, tolerance: f64) -> bool {
    let segment = end - start;
    let to_point = point - start;
    let length_sq = segment.magnitude2();
    if length_sq <= f64::EPSILON {
        return point.distance2(start) <= tolerance * tolerance;
    }

    let cross = segment.x * to_point.y - segment.y * to_point.x;
    if cross.abs() > tolerance {
        return false;
    }

    let dot = to_point.dot(segment);
    dot >= -tolerance && dot <= length_sq + tolerance
}

struct SPHint;

impl SPHint {
    fn from_face<C, S>(face: &Face<Point3, C, S>) -> Option<(f64, f64)> {
        let boundary = face.boundaries().into_iter().next()?;
        let mut count = 0usize;
        let mut sum_u = 0.0;
        let mut sum_v = 0.0;
        for vertex in boundary.vertex_iter() {
            let point = vertex.point();
            sum_u += point.x;
            sum_v += point.y;
            count += 1;
        }
        (count > 0).then_some((sum_u / count as f64, sum_v / count as f64))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::BopOptions;
    use truck_modeling::builder;
    use truck_topology::Edge;

    #[test]
    fn ef_line_plane_intersection_detects_crossing() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let edge = line_edge(Point3::new(0.25, 0.25, -1.0), Point3::new(0.25, 0.25, 1.0));
        let face = unit_square_face();

        let count = intersect_ef(
            &mut bopds,
            &[(EdgeId(0), edge)],
            &[(FaceId(0), face)],
            &[(EdgeId(0), FaceId(0))],
        );

        assert_eq!(count, 1);
        assert_eq!(bopds.ef_interferences().len(), 1);
        let interference = bopds.ef_interferences()[0];
        assert_eq!(interference.edge, EdgeId(0));
        assert_eq!(interference.face, FaceId(0));
        assert!((interference.parameter - 0.5).abs() < 1.0e-9);
    }

    #[test]
    fn ef_intersection_rejects_parallel_edge() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let edge = line_edge(Point3::new(0.0, 0.25, 1.0), Point3::new(1.0, 0.25, 1.0));

        let count = intersect_ef(
            &mut bopds,
            &[(EdgeId(0), edge)],
            &[(FaceId(0), unit_square_face())],
            &[(EdgeId(0), FaceId(0))],
        );

        assert_eq!(count, 0);
        assert!(bopds.ef_interferences().is_empty());
    }

    #[test]
    fn ef_intersection_rejects_points_outside_face_bounds() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let edge = line_edge(Point3::new(1.5, 0.25, -1.0), Point3::new(1.5, 0.25, 1.0));

        let count = intersect_ef(
            &mut bopds,
            &[(EdgeId(0), edge)],
            &[(FaceId(0), unit_square_face())],
            &[(EdgeId(0), FaceId(0))],
        );

        assert_eq!(count, 0);
        assert!(bopds.ef_interferences().is_empty());
    }

    #[test]
    fn ef_intersection_handles_multiple_curve_crossings() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let edge_a = line_edge(Point3::new(0.25, 0.25, -1.0), Point3::new(0.25, 0.25, 1.0));
        let edge_b = line_edge(Point3::new(0.75, 0.75, -1.0), Point3::new(0.75, 0.75, 1.0));

        let count_a = intersect_ef(
            &mut bopds,
            &[(EdgeId(0), edge_a)],
            &[(FaceId(0), unit_square_face())],
            &[(EdgeId(0), FaceId(0))],
        );
        let count_b = intersect_ef(
            &mut bopds,
            &[(EdgeId(1), edge_b)],
            &[(FaceId(0), unit_square_face())],
            &[(EdgeId(1), FaceId(0))],
        );

        assert_eq!(count_a + count_b, 2);
        assert_eq!(bopds.ef_interferences().len(), 2);
        assert!(bopds
            .ef_interferences()
            .iter()
            .any(|interference| interference.edge == EdgeId(0)));
        assert!(bopds
            .ef_interferences()
            .iter()
            .any(|interference| interference.edge == EdgeId(1)));
    }

    #[test]
    fn ef_intersection_handles_tangent_edge_once() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let edge = line_edge(Point3::new(0.5, 0.5, 0.0), Point3::new(0.5, 0.5, 1.0));

        let count = intersect_ef(
            &mut bopds,
            &[(EdgeId(0), edge)],
            &[(FaceId(0), unit_square_face())],
            &[(EdgeId(0), FaceId(0))],
        );

        assert_eq!(count, 1);
        assert_eq!(bopds.ef_interferences().len(), 1);
        assert!((bopds.ef_interferences()[0].parameter - 0.0).abs() < 1.0e-9);
    }

    #[test]
    fn ef_intersection_promotes_interior_point_hit_into_face_info_and_split_blocks() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let edge = line_edge(Point3::new(0.25, 0.25, -1.0), Point3::new(0.25, 0.25, 1.0));
        bopds.rebuild_paves_for_edges(&[(EdgeId(10), edge.clone())]);

        let count = intersect_ef(
            &mut bopds,
            &[(EdgeId(10), edge)],
            &[(FaceId(20), unit_square_face())],
            &[(EdgeId(10), FaceId(20))],
        );

        assert_eq!(count, 1);
        assert_eq!(bopds.pave_blocks_for_edge(EdgeId(10)).len(), 2);
        let info = bopds.face_info(FaceId(20)).expect("face info should be stored");
        assert_eq!(info.on_vertices, Vec::<VertexId>::new());
        assert_eq!(info.sc_vertices, Vec::<VertexId>::new());
        assert_eq!(info.on_pave_blocks, Vec::<crate::PaveBlockId>::new());
        assert_eq!(info.sc_pave_blocks, Vec::<crate::PaveBlockId>::new());
        assert_eq!(info.in_vertices.len(), 1);
        assert_eq!(info.in_pave_blocks.len(), 1);
    }

    #[test]
    fn ef_intersection_promotes_boundary_overlap_into_on_and_sc_face_info() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let edge = line_edge(Point3::new(0.0, 0.5, 0.0), Point3::new(0.0, 0.5, 1.0));
        bopds.rebuild_paves_for_edges(&[(EdgeId(11), edge.clone())]);

        let count = intersect_ef(
            &mut bopds,
            &[(EdgeId(11), edge)],
            &[(FaceId(21), unit_square_face())],
            &[(EdgeId(11), FaceId(21))],
        );

        assert_eq!(count, 1);
        let info = bopds.face_info(FaceId(21)).expect("face info should be stored");
        assert_eq!(info.in_vertices, Vec::<VertexId>::new());
        assert_eq!(info.in_pave_blocks, Vec::<crate::PaveBlockId>::new());
        assert_eq!(info.on_vertices.len(), 1);
        assert_eq!(info.sc_vertices.len(), 1);
        assert_eq!(info.on_pave_blocks.len(), 1);
        assert_eq!(info.sc_pave_blocks.len(), 1);
        let pave_block_id = info.on_pave_blocks[0];
        let common_block_id = bopds
            .common_block_for_pave_block(pave_block_id)
            .expect("boundary hit should bind a common block");
        let common_block = bopds
            .common_block(common_block_id)
            .expect("common block should exist");
        assert_eq!(common_block.faces, vec![FaceId(21)]);
        assert_eq!(common_block.pave_blocks, vec![pave_block_id]);
    }

    fn line_edge(start: Point3, end: Point3) -> Edge<Point3, truck_modeling::Curve> {
        let vertices = builder::vertices([start, end]);
        builder::line(&vertices[0], &vertices[1])
    }

    fn unit_square_face() -> Face<Point3, truck_modeling::Curve, truck_modeling::Surface> {
        let vertices = builder::vertices([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ]);
        let edges: Vec<Edge<Point3, truck_modeling::Curve>> = vec![
            builder::line(&vertices[0], &vertices[1]),
            builder::line(&vertices[1], &vertices[2]),
            builder::line(&vertices[2], &vertices[3]),
            builder::line(&vertices[3], &vertices[0]),
        ];
        let wire = truck_topology::Wire::from(edges);
        Face::new(
            vec![wire],
            truck_modeling::Surface::Plane(truck_modeling::Plane::new(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            )),
        )
    }
}
