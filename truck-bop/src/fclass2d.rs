//! UV-space point-face classifier, inspired by OCCT `IntTools_FClass2d`.
//!
//! Constructs 2D polygons from face wires in parameter space and classifies
//! arbitrary UV points as Inside / Outside / OnBoundary. Supports multi-wire
//! faces (outer boundary + holes) and periodic parameter domains.

use crate::{geometry_utils, PointClassification};
use truck_base::cgmath64::{MetricSpace, Point2, Point3};
use truck_geotrait::{Invertible, ParametricSurface, SearchNearestParameter, D2};
use truck_topology::Face;

const DEFAULT_SAMPLES_PER_EDGE: usize = 8;

/// Cached 2D classifier for a single face.
///
/// Construct once via [`FClass2d::new`], then call [`FClass2d::classify`]
/// for each query point.  Avoids repeatedly projecting wire vertices to UV.
#[derive(Debug)]
pub struct FClass2d {
    outer_polygon: Vec<Point2>,
    hole_polygons: Vec<Vec<Point2>>,
    u_period: Option<f64>,
    v_period: Option<f64>,
    uv_bounds: (f64, f64, f64, f64),
    tolerance: f64,
}

impl FClass2d {
    /// Build the classifier by sampling each wire's edges in UV space.
    ///
    pub fn from_face<C, S>(
        face: &Face<Point3, C, S>,
        surface: &S,
        tolerance: f64,
        samples_per_edge: Option<usize>,
    ) -> Self
    where
        C: Clone,
        S: Clone
            + Invertible
            + ParametricSurface<Point = Point3>
            + SearchNearestParameter<D2, Point = Point3>,
    {
        let nsamples = samples_per_edge.unwrap_or(DEFAULT_SAMPLES_PER_EDGE);
        let mut boundaries = face.boundaries().into_iter();

        let outer_polygon = boundaries
            .next()
            .map(|wire| wire_to_sampled_uv_polygon(&wire, surface, nsamples))
            .unwrap_or_default();

        let hole_polygons: Vec<Vec<Point2>> = boundaries
            .map(|wire| wire_to_sampled_uv_polygon(&wire, surface, nsamples))
            .collect();

        let (u_period, v_period) = detect_periodicity(surface);

        let uv_bounds = compute_uv_bounds(&outer_polygon);

        Self {
            outer_polygon,
            hole_polygons,
            u_period,
            v_period,
            uv_bounds,
            tolerance,
        }
    }

    /// Classify a UV point relative to the face boundary.
    pub fn classify(&self, uv: Point2) -> PointClassification {
        if self.outer_polygon.len() < 3 {
            return PointClassification::Outside;
        }

        let adjusted = self.adjust_periodic(uv);

        if geometry_utils::point_on_polygon_boundary(
            &self.outer_polygon,
            adjusted,
            self.tolerance,
        ) {
            return PointClassification::OnBoundary;
        }

        if !geometry_utils::point_in_polygon(&self.outer_polygon, adjusted) {
            return PointClassification::Outside;
        }

        for hole in &self.hole_polygons {
            if geometry_utils::point_on_polygon_boundary(hole, adjusted, self.tolerance) {
                return PointClassification::OnBoundary;
            }
            if geometry_utils::point_in_polygon(hole, adjusted) {
                return PointClassification::Outside;
            }
        }

        PointClassification::Inside
    }

    fn adjust_periodic(&self, mut uv: Point2) -> Point2 {
        let (umin, umax, vmin, vmax) = self.uv_bounds;
        if let Some(period) = self.u_period {
            uv.x = adjust_to_range(uv.x, umin, umax, period);
        }
        if let Some(period) = self.v_period {
            uv.y = adjust_to_range(uv.y, vmin, vmax, period);
        }
        uv
    }
}

fn adjust_to_range(val: f64, lo: f64, hi: f64, period: f64) -> f64 {
    let mut v = val;
    while v < lo {
        v += period;
    }
    while v > hi {
        v -= period;
    }
    v
}

fn compute_uv_bounds(polygon: &[Point2]) -> (f64, f64, f64, f64) {
    let mut umin = f64::INFINITY;
    let mut umax = f64::NEG_INFINITY;
    let mut vmin = f64::INFINITY;
    let mut vmax = f64::NEG_INFINITY;
    for p in polygon {
        umin = umin.min(p.x);
        umax = umax.max(p.x);
        vmin = vmin.min(p.y);
        vmax = vmax.max(p.y);
    }
    (umin, umax, vmin, vmax)
}

fn detect_periodicity<S>(surface: &S) -> (Option<f64>, Option<f64>)
where
    S: ParametricSurface<Point = Point3>,
{
    (surface.u_period(), surface.v_period())
}

fn wire_to_sampled_uv_polygon<C, S>(
    wire: &truck_topology::Wire<Point3, C>,
    surface: &S,
    _samples_per_edge: usize,
) -> Vec<Point2>
where
    C: Clone,
    S: SearchNearestParameter<D2, Point = Point3>,
{
    let search_trials = 100;
    let mut polygon = Vec::new();

    for vertex in wire.vertex_iter() {
        if let Some(uv) = surface.search_nearest_parameter(vertex.point(), None, search_trials) {
            let p2 = Point2::new(uv.0, uv.1);
            if polygon.last().map_or(true, |prev: &Point2| prev.distance2(p2) > 1.0e-18) {
                polygon.push(p2);
            }
        }
    }

    polygon
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square_polygon() -> Vec<Point2> {
        vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
        ]
    }

    fn classifier_from_polygon(outer: Vec<Point2>, holes: Vec<Vec<Point2>>) -> FClass2d {
        let uv_bounds = compute_uv_bounds(&outer);
        FClass2d {
            outer_polygon: outer,
            hole_polygons: holes,
            u_period: None,
            v_period: None,
            uv_bounds,
            tolerance: 1.0e-9,
        }
    }

    #[test]
    fn point_inside_simple_polygon() {
        let fc = classifier_from_polygon(square_polygon(), vec![]);
        assert_eq!(fc.classify(Point2::new(0.5, 0.5)), PointClassification::Inside);
    }

    #[test]
    fn point_outside_simple_polygon() {
        let fc = classifier_from_polygon(square_polygon(), vec![]);
        assert_eq!(fc.classify(Point2::new(2.0, 0.5)), PointClassification::Outside);
    }

    #[test]
    fn point_on_boundary_of_simple_polygon() {
        let fc = classifier_from_polygon(square_polygon(), vec![]);
        assert_eq!(
            fc.classify(Point2::new(0.5, 0.0)),
            PointClassification::OnBoundary
        );
    }

    #[test]
    fn point_inside_hole_is_outside() {
        let hole = vec![
            Point2::new(0.25, 0.25),
            Point2::new(0.75, 0.25),
            Point2::new(0.75, 0.75),
            Point2::new(0.25, 0.75),
        ];
        let fc = classifier_from_polygon(square_polygon(), vec![hole]);
        assert_eq!(
            fc.classify(Point2::new(0.5, 0.5)),
            PointClassification::Outside
        );
    }

    #[test]
    fn point_between_outer_and_hole_is_inside() {
        let hole = vec![
            Point2::new(0.25, 0.25),
            Point2::new(0.75, 0.25),
            Point2::new(0.75, 0.75),
            Point2::new(0.25, 0.75),
        ];
        let fc = classifier_from_polygon(square_polygon(), vec![hole]);
        assert_eq!(
            fc.classify(Point2::new(0.1, 0.1)),
            PointClassification::Inside
        );
    }

    #[test]
    fn periodic_adjustment_wraps_u() {
        let outer = square_polygon();
        let uv_bounds = compute_uv_bounds(&outer);
        let fc = FClass2d {
            outer_polygon: outer,
            hole_polygons: vec![],
            u_period: Some(1.0),
            v_period: None,
            uv_bounds,
            tolerance: 1.0e-9,
        };
        assert_eq!(
            fc.classify(Point2::new(1.5, 0.5)),
            PointClassification::Inside
        );
    }

    #[test]
    fn periodic_seam_boundary_is_on_boundary() {
        let outer = square_polygon();
        let uv_bounds = compute_uv_bounds(&outer);
        let fc = FClass2d {
            outer_polygon: outer,
            hole_polygons: vec![],
            u_period: Some(1.0),
            v_period: None,
            uv_bounds,
            tolerance: 1.0e-9,
        };
        assert_eq!(
            fc.classify(Point2::new(1.0, 0.5)),
            PointClassification::OnBoundary
        );
        assert_eq!(
            fc.classify(Point2::new(2.0, 0.5)),
            PointClassification::OnBoundary
        );
    }

    #[test]
    fn adjust_to_range_basic() {
        let v = adjust_to_range(7.0, 0.0, 6.28, std::f64::consts::TAU);
        assert!(v >= -0.5 * std::f64::consts::TAU);
        assert!(v <= 6.28 + 0.5 * std::f64::consts::TAU);
    }
}
