//! Common geometry utility functions shared across intersection and trimming modules.

use truck_base::cgmath64::{InnerSpace, MetricSpace, Point2};

/// Pure winding-number point-in-polygon test.
///
/// Returns `true` when the point is strictly inside the polygon (not on the boundary).
/// Uses the ray-casting / winding algorithm with safe division.
pub fn point_in_polygon(polygon: &[Point2], point: Point2) -> bool {
    if polygon.len() < 3 {
        return false;
    }

    let mut inside = false;
    let mut prev = *polygon.last().unwrap();
    for &curr in polygon {
        let intersects = (curr.y > point.y) != (prev.y > point.y)
            && point.x
                < (prev.x - curr.x) * (point.y - curr.y)
                    / (prev.y - curr.y).abs().max(f64::EPSILON)
                    + curr.x;
        if intersects {
            inside = !inside;
        }
        prev = curr;
    }

    inside
}

/// Check whether a point lies on any edge of the polygon within `tolerance`.
pub fn point_on_polygon_boundary(polygon: &[Point2], point: Point2, tolerance: f64) -> bool {
    if polygon.len() < 2 {
        return false;
    }

    polygon
        .windows(2)
        .any(|edge| point_on_segment(edge[0], edge[1], point, tolerance))
        || point_on_segment(
            *polygon.last().unwrap(),
            polygon[0],
            point,
            tolerance,
        )
}

/// Check whether a point is on or inside a polygon (boundary counts as inside).
pub fn point_in_or_on_polygon(polygon: &[Point2], point: Point2, tolerance: f64) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    point_on_polygon_boundary(polygon, point, tolerance) || point_in_polygon(polygon, point)
}

/// Check whether a point lies on the segment from `start` to `end` within `tolerance`.
pub fn point_on_segment(start: Point2, end: Point2, point: Point2, tolerance: f64) -> bool {
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

/// Signed area of a polygon (positive = counter-clockwise, negative = clockwise).
pub fn signed_area(polygon: &[Point2]) -> f64 {
    if polygon.len() < 3 {
        return 0.0;
    }

    let mut area = 0.0;
    let mut prev = *polygon.last().unwrap();
    for &curr in polygon {
        area += (curr.x + prev.x) * (curr.y - prev.y);
        prev = curr;
    }
    area / 2.0
}

/// Strip the closing vertex from a closed polygon (if first == last).
pub fn open_polygon_vertices(polygon: &[Point2]) -> &[Point2] {
    if polygon.len() >= 2 && polygon.first() == polygon.last() {
        &polygon[..polygon.len() - 1]
    } else {
        polygon
    }
}

/// Compute the centroid of a polygon using the shoelace formula.
///
/// Returns `None` if the polygon has fewer than 3 open vertices or near-zero area.
pub fn polygon_centroid(polygon: &[Point2]) -> Option<Point2> {
    let vertices = open_polygon_vertices(polygon);
    if vertices.len() < 3 {
        return None;
    }

    let mut area2 = 0.0;
    let mut centroid_x = 0.0;
    let mut centroid_y = 0.0;
    for window in vertices.windows(2) {
        let start = window[0];
        let end = window[1];
        let cross = start.x * end.y - end.x * start.y;
        area2 += cross;
        centroid_x += (start.x + end.x) * cross;
        centroid_y += (start.y + end.y) * cross;
    }

    if area2.abs() <= 1.0e-9 {
        return None;
    }

    Some(Point2::new(
        centroid_x / (3.0 * area2),
        centroid_y / (3.0 * area2),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_square() -> Vec<Point2> {
        vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
        ]
    }

    #[test]
    fn point_inside_unit_square() {
        assert!(point_in_polygon(&unit_square(), Point2::new(0.5, 0.5)));
    }

    #[test]
    fn point_outside_unit_square() {
        assert!(!point_in_polygon(&unit_square(), Point2::new(2.0, 0.5)));
    }

    #[test]
    fn point_on_boundary_detected_by_in_or_on() {
        assert!(point_in_or_on_polygon(
            &unit_square(),
            Point2::new(0.5, 0.0),
            1.0e-9
        ));
    }

    #[test]
    fn point_on_boundary_detected() {
        assert!(point_on_polygon_boundary(
            &unit_square(),
            Point2::new(0.5, 0.0),
            1.0e-9
        ));
    }

    #[test]
    fn point_in_or_on_includes_boundary() {
        assert!(point_in_or_on_polygon(
            &unit_square(),
            Point2::new(0.5, 0.0),
            1.0e-9
        ));
        assert!(point_in_or_on_polygon(
            &unit_square(),
            Point2::new(0.5, 0.5),
            1.0e-9
        ));
    }

    #[test]
    fn signed_area_of_ccw_square_is_positive() {
        let closed = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
            Point2::new(0.0, 0.0),
        ];
        let area = signed_area(&closed);
        assert!((area - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn open_polygon_strips_closing_vertex() {
        let closed = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 0.0),
        ];
        assert_eq!(open_polygon_vertices(&closed).len(), 3);
    }

    #[test]
    fn polygon_centroid_of_unit_square() {
        let centroid = polygon_centroid(&unit_square()).unwrap();
        assert!((centroid.x - 0.5).abs() < 1.0e-9);
        assert!((centroid.y - 0.5).abs() < 1.0e-9);
    }

    #[test]
    fn segment_contains_midpoint() {
        assert!(point_on_segment(
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(0.5, 0.0),
            1.0e-9
        ));
    }

    #[test]
    fn segment_rejects_distant_point() {
        assert!(!point_on_segment(
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(0.5, 1.0),
            1.0e-9
        ));
    }
}
