//! Edge-Edge intersection detection.

use crate::{BopError, BopOptions, EdgeId};
use truck_base::cgmath64::{EuclideanSpace, MetricSpace, Point3};
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
    C: Clone + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff> + BoundedCurve + Invertible,
{
    let curve_a = edge_a.oriented_curve();
    let curve_b = edge_b.oriented_curve();
    let range_a = curve_a.range_tuple();
    let range_b = curve_b.range_tuple();
    let tolerance = options.geometric_tol;
    let tolerance_sq = tolerance * tolerance;

    let mut intersections = Vec::new();

    if let Some((t_a, t_b)) = refine_intersection(&curve_a, &curve_b, range_a, range_b, tolerance_sq) {
        push_unique_intersection(&mut intersections, edge_a_id, edge_b_id, t_a, t_b, tolerance);
        return Ok(intersections);
    }

    for t_a in interior_sample_parameters(range_a, SAMPLE_DIVISION) {
        let point = curve_a.subs(t_a);
        let hint_b = curve::presearch(&curve_b, point, range_b, PRESEARCH_DIVISION);
        let Some(t_b) = curve::search_parameter(&curve_b, point, hint_b, SEARCH_PARAMETER_TRIALS) else {
            continue;
        };
        if !parameter_in_range(t_b, range_b, tolerance) {
            continue;
        }
        if curve_a.subs(t_a).distance2(curve_b.subs(t_b)) > tolerance_sq {
            continue;
        }
        push_unique_intersection(&mut intersections, edge_a_id, edge_b_id, t_a, t_b, tolerance);
    }

    Ok(intersections)
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
    let hint = curve::presearch_closest_point::<Point3, _, _>(curve_a, curve_b, (range_a, range_b), PRESEARCH_DIVISION);
    let (t_a, t_b) = curve::search_intersection_parameter::<Point3, _, _>(curve_a, curve_b, hint, SEARCH_PARAMETER_TRIALS)?;
    (curve_a.subs(t_a).distance2(curve_b.subs(t_b)) <= tolerance_sq).then_some((t_a, t_b))
}

fn interior_sample_parameters(range: (f64, f64), division: usize) -> Vec<f64> {
    let (start, end) = range;
    let step = (end - start) / division as f64;
    (1..division).map(|index| start + step * index as f64).collect()
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

#[cfg(test)]
mod tests {
    use super::*;
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

    fn line_edge(start: Point3, end: Point3) -> Edge<Point3, Curve> {
        let vertices = builder::vertices([start, end]);
        builder::line(&vertices[0], &vertices[1])
    }

}
