//! Edge-Edge intersection detection.

use crate::{
    bopds::{CommonBlock, EEInterference, EEInterferenceKind, Pave},
    BopDs, BopError, BopOptions, EdgeId,
};
use truck_base::cgmath64::{EuclideanSpace, InnerSpace, MetricSpace, Point3};
use truck_geotrait::{algo::curve, BoundedCurve, Invertible, ParametricCurve};
use truck_topology::Edge;

const PRESEARCH_DIVISION: usize = 16;
const SEARCH_PARAMETER_TRIALS: usize = 100;
const SAMPLE_DIVISION: usize = 63;

/// Detects intersection between two edges and returns parameter pairs on both curves.
pub fn intersect_ee<C>(
    edge_a_id: EdgeId,
    edge_b_id: EdgeId,
    edge_a: &Edge<Point3, C>,
    edge_b: &Edge<Point3, C>,
    options: &BopOptions,
) -> Result<Vec<(EdgeId, EdgeId, f64, f64)>, BopError>
where
    C: Clone
        + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + BoundedCurve
        + Invertible,
{
    let curve_a = edge_a.oriented_curve();
    let curve_b = edge_b.oriented_curve();
    let range_a = curve_a.range_tuple();
    let range_b = curve_b.range_tuple();
    let tolerance = options.geometric_tol;
    let tolerance_sq = tolerance * tolerance;

    let mut intersections = Vec::new();

    if let Some((t_a, t_b)) =
        refine_intersection(&curve_a, &curve_b, range_a, range_b, tolerance_sq)
    {
        push_unique_intersection(
            &mut intersections,
            edge_a_id,
            edge_b_id,
            t_a,
            t_b,
            tolerance,
        );
        return Ok(intersections);
    }

    for t_a in interior_sample_parameters(range_a, SAMPLE_DIVISION) {
        let point = curve_a.subs(t_a);
        let hint_b = curve::presearch(&curve_b, point, range_b, PRESEARCH_DIVISION);
        let Some(t_b) = curve::search_parameter(&curve_b, point, hint_b, SEARCH_PARAMETER_TRIALS)
        else {
            continue;
        };
        if !parameter_in_range(t_b, range_b, tolerance) {
            continue;
        }
        if curve_a.subs(t_a).distance2(curve_b.subs(t_b)) > tolerance_sq {
            continue;
        }
        push_unique_intersection(
            &mut intersections,
            edge_a_id,
            edge_b_id,
            t_a,
            t_b,
            tolerance,
        );
    }

    Ok(intersections)
}

/// Detects edge-edge interferences and stores shared-vertex or shared-overlap facts in `BopDs`.
pub fn intersect_ee_into_bopds<C>(
    bopds: &mut BopDs,
    edges: &[(EdgeId, Edge<Point3, C>)],
    candidates: &[(EdgeId, EdgeId)],
) -> usize
where
    C: Clone
        + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + BoundedCurve
        + Invertible,
{
    let tolerance = bopds.options().geometric_tol;
    let parametric_tol = bopds.options().parametric_tol;
    let mut count = 0;

    for &(edge_a_id, edge_b_id) in candidates {
        let Some(edge_a) = edge_by_id(edges, edge_a_id) else {
            continue;
        };
        let Some(edge_b) = edge_by_id(edges, edge_b_id) else {
            continue;
        };

        let overlaps = match line_overlap_range(edge_a, edge_b, tolerance) {
            Some(overlap) => vec![overlap],
            None => detect_overlap_ranges(edge_a, edge_b, tolerance),
        };
        if !overlaps.is_empty() {
            for overlap in overlaps {
                if register_overlap_fact(
                    bopds,
                    edge_a_id,
                    edge_b_id,
                    overlap,
                    (edge_parameter_range(edge_a), edge_parameter_range(edge_b)),
                    parametric_tol,
                ) {
                    count += 1;
                }
            }
            continue;
        }

        let Ok(intersections) = intersect_ee(edge_a_id, edge_b_id, edge_a, edge_b, bopds.options())
        else {
            continue;
        };
        for (_, _, t_a, t_b) in intersections {
            let vertex_id = bopds.next_generated_vertex_id();
            let pave_a = Pave::new(edge_a_id, vertex_id, t_a, parametric_tol).unwrap();
            let pave_b = Pave::new(edge_b_id, vertex_id, t_b, parametric_tol).unwrap();
            append_or_push_split_pave(bopds, edge_a_id, pave_a, edge_parameter_range(edge_a));
            append_or_push_split_pave(bopds, edge_b_id, pave_b, edge_parameter_range(edge_b));
            bopds.split_pave_blocks_for_edge(edge_a_id);
            bopds.split_pave_blocks_for_edge(edge_b_id);
            bopds.push_ee_interference(EEInterference {
                edge1: edge_a_id,
                edge2: edge_b_id,
                t_a,
                t_b,
                kind: EEInterferenceKind::VertexHit,
            });
            count += 1;
        }
    }

    count
}

fn refine_intersection<C>(
    curve_a: &C,
    curve_b: &C,
    range_a: (f64, f64),
    range_b: (f64, f64),
    tolerance_sq: f64,
) -> Option<(f64, f64)>
where
    C: ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>,
{
    let hint = curve::presearch_closest_point::<Point3, _, _>(
        curve_a,
        curve_b,
        (range_a, range_b),
        PRESEARCH_DIVISION,
    );
    let (t_a, t_b) = curve::search_intersection_parameter::<Point3, _, _>(
        curve_a,
        curve_b,
        hint,
        SEARCH_PARAMETER_TRIALS,
    )?;
    (curve_a.subs(t_a).distance2(curve_b.subs(t_b)) <= tolerance_sq).then_some((t_a, t_b))
}

fn interior_sample_parameters(range: (f64, f64), division: usize) -> Vec<f64> {
    let (start, end) = range;
    let step = (end - start) / division as f64;
    (1..division)
        .map(|index| start + step * index as f64)
        .collect()
}

fn parameter_in_range(parameter: f64, range: (f64, f64), tolerance: f64) -> bool {
    parameter >= range.0 - tolerance && parameter <= range.1 + tolerance
}

fn push_unique_intersection(
    intersections: &mut Vec<(EdgeId, EdgeId, f64, f64)>,
    edge_a_id: EdgeId,
    edge_b_id: EdgeId,
    t_a: f64,
    t_b: f64,
    tolerance: f64,
) {
    if intersections.iter().any(|&(_, _, existing_a, existing_b)| {
        (existing_a - t_a).abs() <= tolerance && (existing_b - t_b).abs() <= tolerance
    }) {
        return;
    }
    intersections.push((edge_a_id, edge_b_id, t_a, t_b));
}

fn append_or_push_split_pave(
    bopds: &mut BopDs,
    edge_id: EdgeId,
    pave: Pave,
    edge_range: (f64, f64),
) {
    let near_endpoint = (pave.parameter - edge_range.0).abs() <= pave.tolerance
        || (pave.parameter - edge_range.1).abs() <= pave.tolerance;
    let inserted = if near_endpoint {
        false
    } else {
        bopds.append_ext_pave(edge_id, pave)
    };
    if !inserted {
        bopds.push_pave(pave);
        bopds.rebuild_pave_blocks_from_paves(edge_id);
    }
}

fn register_overlap_fact(
    bopds: &mut BopDs,
    edge_a_id: EdgeId,
    edge_b_id: EdgeId,
    overlap: OverlapRange,
    edge_ranges: ((f64, f64), (f64, f64)),
    parametric_tol: f64,
) -> bool {
    let pave_a_start = overlap_pave(bopds, edge_a_id, overlap.t_a.0, edge_ranges.0, parametric_tol);
    let pave_a_end = overlap_pave(bopds, edge_a_id, overlap.t_a.1, edge_ranges.0, parametric_tol);
    let pave_b_start = overlap_pave(bopds, edge_b_id, overlap.t_b.0, edge_ranges.1, parametric_tol);
    let pave_b_end = overlap_pave(bopds, edge_b_id, overlap.t_b.1, edge_ranges.1, parametric_tol);

    for pave in [pave_a_start, pave_a_end] {
        append_or_push_split_pave(bopds, edge_a_id, pave, edge_ranges.0);
    }
    for pave in [pave_b_start, pave_b_end] {
        append_or_push_split_pave(bopds, edge_b_id, pave, edge_ranges.1);
    }

    bopds.split_pave_blocks_for_edge(edge_a_id);
    bopds.split_pave_blocks_for_edge(edge_b_id);

    let Some(block_a) = bopds.find_pave_block_by_range(
        edge_a_id,
        overlap.t_a.0,
        overlap.t_a.1,
        parametric_tol,
    ) else {
        return false;
    };
    let Some(block_b) = bopds.find_pave_block_by_range(
        edge_b_id,
        overlap.t_b.0,
        overlap.t_b.1,
        parametric_tol,
    ) else {
        return false;
    };

    let block_ids = [block_a, block_b];
    let maybe_existing = block_ids
        .into_iter()
        .find_map(|block_id| bopds.common_block_for_pave_block(block_id));
    if let Some(common_block_id) = maybe_existing {
        if let Some(existing) = bopds.common_block(common_block_id).cloned() {
            let mut updated = existing;
            updated.push_pave_block(block_a);
            updated.push_pave_block(block_b);
            let _ = bopds.update_common_block(common_block_id, updated);
        }
    } else {
        bopds.push_common_block(CommonBlock::new(
            vec![block_a, block_b],
            vec![],
            Some(edge_a_id),
        ));
    }

    bopds.push_ee_interference(EEInterference {
        edge1: edge_a_id,
        edge2: edge_b_id,
        t_a: overlap.t_a.0,
        t_b: overlap.t_b.0,
        kind: EEInterferenceKind::OverlapHit,
    });
    bopds.push_ee_interference(EEInterference {
        edge1: edge_a_id,
        edge2: edge_b_id,
        t_a: overlap.t_a.1,
        t_b: overlap.t_b.1,
        kind: EEInterferenceKind::OverlapHit,
    });
    true
}

fn overlap_pave(
    bopds: &mut BopDs,
    edge_id: EdgeId,
    parameter: f64,
    edge_range: (f64, f64),
    parametric_tol: f64,
) -> Pave {
    let vertex = existing_vertex_for_parameter(bopds, edge_id, parameter, parametric_tol)
        .unwrap_or_else(|| bopds.next_generated_vertex_id());
    let clamped = if (parameter - edge_range.0).abs() <= parametric_tol {
        edge_range.0
    } else if (parameter - edge_range.1).abs() <= parametric_tol {
        edge_range.1
    } else {
        parameter
    };
    Pave::new(edge_id, vertex, clamped, parametric_tol)
        .expect("parametric tolerance is validated when BopOptions is created")
}

fn existing_vertex_for_parameter(
    bopds: &BopDs,
    edge_id: EdgeId,
    parameter: f64,
    tolerance: f64,
) -> Option<crate::VertexId> {
    bopds
        .paves_for_edge(edge_id)
        .into_iter()
        .find(|pave| (pave.parameter - parameter).abs() <= tolerance.max(pave.tolerance))
        .map(|pave| pave.vertex)
}

fn edge_parameter_range<C>(edge: &Edge<Point3, C>) -> (f64, f64)
where
    C: Clone
        + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + BoundedCurve
        + Invertible,
{
    edge.oriented_curve().range_tuple()
}


fn edge_by_id<C>(edges: &[(EdgeId, Edge<Point3, C>)], edge_id: EdgeId) -> Option<&Edge<Point3, C>> {
    edges
        .iter()
        .find(|(id, _)| *id == edge_id)
        .map(|(_, edge)| edge)
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct OverlapRange {
    t_a: (f64, f64),
    t_b: (f64, f64),
}

fn detect_overlap_ranges<C>(
    edge_a: &Edge<Point3, C>,
    edge_b: &Edge<Point3, C>,
    tolerance: f64,
) -> Vec<OverlapRange>
where
    C: Clone
        + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + BoundedCurve
        + Invertible,
{
    let curve_a = edge_a.oriented_curve();
    let curve_b = edge_b.oriented_curve();
    let range_a = curve_a.range_tuple();
    let range_b = curve_b.range_tuple();

    if !edge_segments_are_colinear(&curve_a, &curve_b, range_a, range_b, tolerance) {
        return Vec::new();
    }

    let start_a = curve_a.subs(range_a.0);
    let end_a = curve_a.subs(range_a.1);
    let start_b = curve_b.subs(range_b.0);
    let end_b = curve_b.subs(range_b.1);

    let point_on_b = |point: Point3| -> Option<f64> {
        let hint = curve::presearch(&curve_b, point, range_b, PRESEARCH_DIVISION);
        let parameter = curve::search_parameter(&curve_b, point, hint, SEARCH_PARAMETER_TRIALS)?;
        (curve_b.subs(parameter).distance(point) <= tolerance).then_some(parameter)
    };

    let point_on_a = |point: Point3| -> Option<f64> {
        let hint = curve::presearch(&curve_a, point, range_a, PRESEARCH_DIVISION);
        let parameter = curve::search_parameter(&curve_a, point, hint, SEARCH_PARAMETER_TRIALS)?;
        (curve_a.subs(parameter).distance(point) <= tolerance).then_some(parameter)
    };

    let mut hits = Vec::new();
    if let Some(t_b) = point_on_b(start_a) {
        hits.push((range_a.0, t_b));
    }
    if let Some(t_b) = point_on_b(end_a) {
        hits.push((range_a.1, t_b));
    }
    if let Some(t_a) = point_on_a(start_b) {
        hits.push((t_a, range_b.0));
    }
    if let Some(t_a) = point_on_a(end_b) {
        hits.push((t_a, range_b.1));
    }

    hits.sort_by(|lhs, rhs| lhs.0.total_cmp(&rhs.0).then(lhs.1.total_cmp(&rhs.1)));
    hits.dedup_by(|lhs, rhs| {
        (lhs.0 - rhs.0).abs() <= tolerance && (lhs.1 - rhs.1).abs() <= tolerance
    });

    if hits.len() < 2 {
        return Vec::new();
    }

    let first = hits.first().copied().unwrap();
    let last = hits.last().copied().unwrap();
    if (first.0 - last.0).abs() <= tolerance || (first.1 - last.1).abs() <= tolerance {
        return Vec::new();
    }

    vec![OverlapRange {
        t_a: if first.0 <= last.0 {
            (first.0, last.0)
        } else {
            (last.0, first.0)
        },
        t_b: if first.1 <= last.1 {
            (first.1, last.1)
        } else {
            (last.1, first.1)
        },
    }]
}

fn line_overlap_range<C>(
    edge_a: &Edge<Point3, C>,
    edge_b: &Edge<Point3, C>,
    tolerance: f64,
) -> Option<OverlapRange>
where
    C: Clone
        + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + BoundedCurve
        + Invertible,
{
    let curve_a = edge_a.oriented_curve();
    let curve_b = edge_b.oriented_curve();
    let range_a = curve_a.range_tuple();
    let range_b = curve_b.range_tuple();
    let start_a = curve_a.subs(range_a.0);
    let end_a = curve_a.subs(range_a.1);
    let start_b = curve_b.subs(range_b.0);
    let end_b = curve_b.subs(range_b.1);

    if !points_are_colinear(start_a, end_a, start_b, tolerance)
        || !points_are_colinear(start_a, end_a, end_b, tolerance)
    {
        return None;
    }

    let axis = dominant_axis(end_a - start_a)?;
    let (a0, a1) = ordered_pair(select_axis(start_a, axis), select_axis(end_a, axis));
    let (b0, b1) = ordered_pair(select_axis(start_b, axis), select_axis(end_b, axis));
    let overlap_start = a0.max(b0);
    let overlap_end = a1.min(b1);
    if overlap_end - overlap_start <= tolerance {
        return None;
    }

    let t_a = point_to_parameter(overlap_start, a0, a1, range_a);
    let t_b = point_to_parameter(overlap_start, b0, b1, range_b);
    let end_t_a = point_to_parameter(overlap_end, a0, a1, range_a);
    let end_t_b = point_to_parameter(overlap_end, b0, b1, range_b);

    Some(OverlapRange {
        t_a: ordered_pair(t_a, end_t_a),
        t_b: ordered_pair(t_b, end_t_b),
    })
}

fn points_are_colinear(a: Point3, b: Point3, c: Point3, tolerance: f64) -> bool {
    let tolerance_sq = tolerance * tolerance;
    (b - a).cross(c - a).magnitude2() <= tolerance_sq
}

fn dominant_axis(vector: <Point3 as EuclideanSpace>::Diff) -> Option<usize> {
    let components = [vector.x.abs(), vector.y.abs(), vector.z.abs()];
    components
        .iter()
        .enumerate()
        .max_by(|lhs, rhs| lhs.1.total_cmp(rhs.1))
        .and_then(|(index, value)| if *value > 0.0 { Some(index) } else { None })
}

fn select_axis(point: Point3, axis: usize) -> f64 {
    match axis {
        0 => point.x,
        1 => point.y,
        _ => point.z,
    }
}

fn ordered_pair(lhs: f64, rhs: f64) -> (f64, f64) {
    if lhs <= rhs {
        (lhs, rhs)
    } else {
        (rhs, lhs)
    }
}

fn point_to_parameter(value: f64, range_start: f64, range_end: f64, parameter_range: (f64, f64)) -> f64 {
    let span = range_end - range_start;
    if span.abs() <= f64::EPSILON {
        parameter_range.0
    } else {
        parameter_range.0 + (value - range_start) / span * (parameter_range.1 - parameter_range.0)
    }
}

fn edge_segments_are_colinear<C>(
    curve_a: &C,
    curve_b: &C,
    range_a: (f64, f64),
    range_b: (f64, f64),
    tolerance: f64,
) -> bool
where
    C: ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>,
{
    let start_a = curve_a.subs(range_a.0);
    let end_a = curve_a.subs(range_a.1);
    let start_b = curve_b.subs(range_b.0);
    let end_b = curve_b.subs(range_b.1);

    let dir_a = end_a - start_a;
    let dir_b = end_b - start_b;
    let tolerance_sq = tolerance * tolerance;

    dir_a.cross(dir_b).magnitude2() <= tolerance_sq
        && dir_a.cross(start_b - start_a).magnitude2() <= tolerance_sq
        && dir_a.cross(end_b - start_a).magnitude2() <= tolerance_sq
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{bopds::EEInterferenceKind, BopDs};
    use truck_base::cgmath64::Point3;
    use truck_modeling::{builder, Curve};

    #[test]
    fn ee_line_line_intersection_detects_perpendicular_segments() {
        let opts = BopOptions::default();
        let edge_a = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let edge_b = line_edge(Point3::new(0.5, -0.5, 0.0), Point3::new(0.5, 0.5, 0.0));

        let result = intersect_ee(EdgeId(0), EdgeId(1), &edge_a, &edge_b, &opts).unwrap();

        assert_eq!(result.len(), 1);
        let (_, _, t_a, t_b) = result[0];
        assert!((t_a - 0.5).abs() < 1.0e-9);
        assert!((t_b - 0.5).abs() < 1.0e-9);
    }

    #[test]
    fn ee_parallel_no_intersection_for_parallel_segments() {
        let opts = BopOptions::default();
        let edge_a = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let edge_b = line_edge(Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0));

        let result = intersect_ee(EdgeId(0), EdgeId(1), &edge_a, &edge_b, &opts).unwrap();

        assert!(result.is_empty());
    }

    #[test]
    fn ee_parallel_no_intersection_for_separated_colinear_segments() {
        let opts = BopOptions::default();
        let edge_a = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let edge_b = line_edge(Point3::new(1.5, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));

        let result = intersect_ee(EdgeId(0), EdgeId(1), &edge_a, &edge_b, &opts).unwrap();

        assert!(result.is_empty());
    }

    #[test]
    fn ee_skew_lines_in_3d_produce_no_intersection() {
        let opts = BopOptions::default();
        let edge_a = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let edge_b = line_edge(Point3::new(0.5, -0.5, 1.0), Point3::new(0.5, 0.5, 1.0));

        let result = intersect_ee(EdgeId(0), EdgeId(1), &edge_a, &edge_b, &opts).unwrap();

        assert!(result.is_empty());
    }

    #[test]
    fn ee_intersection_parameters_stay_within_curve_bounds() {
        let opts = BopOptions::default();
        let edge_a = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let edge_b = line_edge(Point3::new(0.5, -0.5, 0.0), Point3::new(0.5, 0.5, 0.0));

        let result = intersect_ee(EdgeId(0), EdgeId(1), &edge_a, &edge_b, &opts).unwrap();
        let (_, _, t_a, t_b) = result[0];
        let range_a = edge_a.oriented_curve().range_tuple();
        let range_b = edge_b.oriented_curve().range_tuple();

        assert!(parameter_in_range(t_a, range_a, 1.0e-12));
        assert!(parameter_in_range(t_b, range_b, 1.0e-12));
    }

    #[test]
    fn ee_intersection_into_bopds_splits_both_edges_for_perpendicular_hit() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-6,
            parametric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let edge_a = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let edge_b = line_edge(Point3::new(0.5, -0.5, 0.0), Point3::new(0.5, 0.5, 0.0));
        bopds.rebuild_paves_for_edges(&[(EdgeId(10), edge_a.clone()), (EdgeId(11), edge_b.clone())]);

        let count = intersect_ee_into_bopds(
            &mut bopds,
            &[(EdgeId(10), edge_a), (EdgeId(11), edge_b)],
            &[(EdgeId(10), EdgeId(11))],
        );

        assert_eq!(count, 1);
        assert_eq!(bopds.ee_interferences().len(), 1);
        assert_eq!(bopds.ee_interferences()[0].kind, EEInterferenceKind::VertexHit);
        assert_eq!(
            bopds.pave_blocks_for_edge(EdgeId(10))
                .iter()
                .map(|block| block.param_range)
                .collect::<Vec<_>>(),
            vec![(0.0, 0.5), (0.5, 1.0)]
        );
        assert_eq!(
            bopds.pave_blocks_for_edge(EdgeId(11))
                .iter()
                .map(|block| block.param_range)
                .collect::<Vec<_>>(),
            vec![(0.0, 0.5), (0.5, 1.0)]
        );
    }

    #[test]
    fn ee_intersection_into_bopds_reuses_endpoint_touch_without_duplicate_split() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-6,
            parametric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let edge_a = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let edge_b = line_edge(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0));
        bopds.rebuild_paves_for_edges(&[(EdgeId(12), edge_a.clone()), (EdgeId(13), edge_b.clone())]);

        let count = intersect_ee_into_bopds(
            &mut bopds,
            &[(EdgeId(12), edge_a), (EdgeId(13), edge_b)],
            &[(EdgeId(12), EdgeId(13))],
        );

        assert_eq!(count, 1);
        assert_eq!(bopds.ee_interferences()[0].kind, EEInterferenceKind::VertexHit);
        assert_eq!(bopds.pave_blocks_for_edge(EdgeId(12)).len(), 1);
        assert_eq!(bopds.pave_blocks_for_edge(EdgeId(13)).len(), 1);
    }

    #[test]
    fn ee_intersection_into_bopds_promotes_colinear_overlap_to_common_block_ready_facts() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-6,
            parametric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let edge_a = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let edge_b = line_edge(Point3::new(0.25, 0.0, 0.0), Point3::new(0.75, 0.0, 0.0));
        bopds.rebuild_paves_for_edges(&[(EdgeId(20), edge_a.clone()), (EdgeId(21), edge_b.clone())]);

        let count = intersect_ee_into_bopds(
            &mut bopds,
            &[(EdgeId(20), edge_a), (EdgeId(21), edge_b)],
            &[(EdgeId(20), EdgeId(21))],
        );

        assert_eq!(count, 1);
        assert_eq!(bopds.ee_interferences().len(), 2);
        assert!(bopds
            .ee_interferences()
            .iter()
            .all(|item| item.kind == EEInterferenceKind::OverlapHit));
        assert_eq!(bopds.common_blocks().len(), 1);
        let common_block = &bopds.common_blocks()[0];
        assert_eq!(common_block.pave_blocks.len(), 2);
        assert_ne!(common_block.pave_blocks[0], common_block.pave_blocks[1]);
        assert!(common_block
            .pave_blocks
            .iter()
            .all(|id| id.0 != 0));
        assert_eq!(
            bopds.pave_blocks_for_edge(EdgeId(20))
                .iter()
                .map(|block| block.param_range)
                .collect::<Vec<_>>(),
            vec![(0.0, 0.25), (0.25, 0.75), (0.75, 1.0)]
        );
        assert_eq!(
            bopds.pave_blocks_for_edge(EdgeId(21))
                .iter()
                .map(|block| block.param_range)
                .collect::<Vec<_>>(),
            vec![(0.0, 1.0)]
        );
    }

    fn line_edge(start: Point3, end: Point3) -> Edge<Point3, Curve> {
        let vertices = builder::vertices([start, end]);
        builder::line(&vertices[0], &vertices[1])
    }
}
