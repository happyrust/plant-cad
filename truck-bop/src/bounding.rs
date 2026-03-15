//! Bounding-box providers for topology entities.

use crate::BopOptions;
use truck_base::{
    bounding_box::BoundingBox,
    cgmath64::{Point3, Vector3},
};
use truck_geotrait::{BoundedCurve, Invertible};
use truck_topology::{Edge, Face, Vertex};

const SAMPLE_COUNT: usize = 16;

/// Provides an axis-aligned bounding box for a topological entity.
pub trait BoundingProvider {
    /// Returns a bounding box expanded by the geometric tolerance.
    fn bounding_box(&self, options: &BopOptions) -> BoundingBox<Point3>;
}

impl BoundingProvider for Vertex<Point3> {
    fn bounding_box(&self, options: &BopOptions) -> BoundingBox<Point3> {
        expand_point_bbox(self.point(), options.geometric_tol)
    }
}

impl<C> BoundingProvider for Edge<Point3, C>
where
    C: Clone + BoundedCurve<Point = Point3> + Invertible,
{
    fn bounding_box(&self, options: &BopOptions) -> BoundingBox<Point3> {
        let curve = self.oriented_curve();
        let mut bbox = sample_curve_bbox(&curve);
        bbox += curve_endpoints_bbox(&curve);
        expand_bbox(bbox, options.geometric_tol)
    }
}

impl<C, S> BoundingProvider for Face<Point3, C, S>
where
    C: Clone + BoundedCurve<Point = Point3> + Invertible,
{
    fn bounding_box(&self, options: &BopOptions) -> BoundingBox<Point3> {
        let mut bbox = BoundingBox::new();
        for edge in self.boundaries().into_iter().flatten() {
            let curve = edge.oriented_curve();
            bbox += sample_curve_bbox(&curve);
            bbox += curve_endpoints_bbox(&curve);
        }
        expand_bbox(bbox, options.geometric_tol)
    }
}

fn expand_point_bbox(point: Point3, tolerance: f64) -> BoundingBox<Point3> {
    BoundingBox::from_iter([
        point - Vector3::new(tolerance, tolerance, tolerance),
        point + Vector3::new(tolerance, tolerance, tolerance),
    ])
}

fn expand_bbox(bbox: BoundingBox<Point3>, tolerance: f64) -> BoundingBox<Point3> {
    if bbox.is_empty() {
        return bbox;
    }

    let offset = Vector3::new(tolerance, tolerance, tolerance);
    BoundingBox::from_iter([bbox.min() - offset, bbox.max() + offset])
}

fn curve_endpoints_bbox<C>(curve: &C) -> BoundingBox<Point3>
where
    C: BoundedCurve<Point = Point3>,
{
    BoundingBox::from_iter([curve.front(), curve.back()])
}

fn sample_curve_bbox<C>(curve: &C) -> BoundingBox<Point3>
where
    C: BoundedCurve<Point = Point3>,
{
    let (t0, t1) = curve.range_tuple();
    sample_1d_bbox(|i| {
        let t = interpolate(t0, t1, i, SAMPLE_COUNT);
        curve.subs(t)
    })
}

fn sample_1d_bbox<F>(mut sampler: F) -> BoundingBox<Point3>
where
    F: FnMut(usize) -> Point3,
{
    let mut bbox = BoundingBox::new();
    for i in 0..=SAMPLE_COUNT {
        bbox.push(sampler(i));
    }
    bbox
}

fn interpolate(start: f64, end: f64, step: usize, divisions: usize) -> f64 {
    if divisions == 0 {
        return start;
    }
    let ratio = step as f64 / divisions as f64;
    start + (end - start) * ratio
}

#[cfg(test)]
mod tests {
    use super::*;
    use truck_geotrait::ParametricCurve;
    use truck_modeling::builder;
    use truck_topology::{Edge, Face};

    #[test]
    fn bounding_provider_vertex_expands_point_by_tolerance() {
        let options = BopOptions { geometric_tol: 0.25, ..BopOptions::default() };
        let vertex = Vertex::new(Point3::new(1.0, -2.0, 3.0));

        let bbox = vertex.bounding_box(&options);

        assert!(bbox.contains(vertex.point()));
        assert_eq!(bbox.min(), Point3::new(0.75, -2.25, 2.75));
        assert_eq!(bbox.max(), Point3::new(1.25, -1.75, 3.25));
        assert!(bbox.size() <= 2.0 * options.geometric_tol);
    }

    #[test]
    fn bounding_provider_edge_contains_sampled_curve_points_and_expansion() {
        let options = BopOptions { geometric_tol: 0.1, ..BopOptions::default() };
        let vertices = builder::vertices([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ]);
        let edge: Edge<Point3, truck_modeling::Curve> =
            builder::bezier(&vertices[0], &vertices[2], vec![vertices[1].point()]);

        let bbox = edge.bounding_box(&options);
        let curve: truck_modeling::Curve = edge.oriented_curve();
        let raw_bbox = sample_curve_bbox(&curve) + curve_endpoints_bbox(&curve);

        for i in 0..=SAMPLE_COUNT {
            let t = interpolate(0.0, 1.0, i, SAMPLE_COUNT);
            assert!(bbox.contains(curve.subs(t)));
        }
        assert_eq!(bbox.min(), raw_bbox.min() - Vector3::new(0.1, 0.1, 0.1));
        assert_eq!(bbox.max(), raw_bbox.max() + Vector3::new(0.1, 0.1, 0.1));
    }

    #[test]
    fn bounding_provider_face_contains_boundary_curve_samples_and_expansion() {
        let options = BopOptions { geometric_tol: 0.2, ..BopOptions::default() };
        let vertices = builder::vertices([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.5),
            Point3::new(0.0, 1.0, 1.0),
        ]);
        let edge0: Edge<Point3, truck_modeling::Curve> = builder::line(&vertices[0], &vertices[1]);
        let edge1: Edge<Point3, truck_modeling::Curve> = builder::line(&vertices[1], &vertices[2]);
        let edge2: Edge<Point3, truck_modeling::Curve> = builder::line(&vertices[2], &vertices[0]);
        let face: Face<Point3, truck_modeling::Curve, ()> =
            Face::new(vec![truck_topology::Wire::from(vec![edge0.clone(), edge1.clone(), edge2.clone()])], ());

        let bbox = face.bounding_box(&options);
        let mut raw_bbox = BoundingBox::new();
        for edge in face.boundary_iters().into_iter().flatten() {
            let curve: truck_modeling::Curve = edge.oriented_curve();
            raw_bbox += sample_curve_bbox(&curve);
            raw_bbox += curve_endpoints_bbox(&curve);
            for i in 0..=SAMPLE_COUNT {
                let t = interpolate(0.0, 1.0, i, SAMPLE_COUNT);
                assert!(bbox.contains(curve.subs(t)));
            }
        }
        assert_eq!(bbox.min(), raw_bbox.min() - Vector3::new(0.2, 0.2, 0.2));
        assert_eq!(bbox.max(), raw_bbox.max() + Vector3::new(0.2, 0.2, 0.2));
    }
}
