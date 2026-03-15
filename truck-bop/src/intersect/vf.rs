//! Vertex-Face intersection detection.

use crate::{bopds::VFInterference, BopDs, FaceId, VertexId};
use truck_base::cgmath64::{EuclideanSpace, InnerSpace, MetricSpace, Point2, Point3};
use truck_geotrait::{D2, Invertible, ParametricSurface, SearchNearestParameter};
use truck_topology::{Face, Vertex};

const SEARCH_PARAMETER_TRIALS: usize = 100;

/// Detects vertex-face interferences and stores them in `BopDs`.
pub fn intersect_vf<C, S>(
    bopds: &mut BopDs,
    vertices: &[(VertexId, Vertex<Point3>)],
    faces: &[(FaceId, Face<Point3, C, S>)],
    candidates: &[(VertexId, FaceId)],
) -> usize
where
    C: Clone,
    S: Clone + Invertible + ParametricSurface<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + SearchNearestParameter<D2, Point = Point3>,
{
    let tolerance = bopds.options().geometric_tol;
    let tolerance_sq = tolerance * tolerance;

    let mut count = 0;
    for &(vertex_id, face_id) in candidates {
        let Some(point) = vertex_point(vertices, vertex_id) else {
            continue;
        };
        let Some(face) = face_by_id(faces, face_id) else {
            continue;
        };

        let surface = face.oriented_surface();
        let Some(parameters) = surface.search_nearest_parameter(point, SPHint::from_face(face), SEARCH_PARAMETER_TRIALS) else {
            continue;
        };

        let projection = surface.subs(parameters.0, parameters.1);
        if projection.distance2(point) > tolerance_sq {
            continue;
        }

        if !point_projects_inside_face(face, parameters, tolerance) {
            continue;
        }

        bopds.push_vf_interference(VFInterference {
            vertex: vertex_id,
            face: face_id,
            parameters,
        });
        count += 1;
    }

    count
}

fn vertex_point(vertices: &[(VertexId, Vertex<Point3>)], vertex_id: VertexId) -> Option<Point3> {
    vertices
        .iter()
        .find(|(id, _)| *id == vertex_id)
        .map(|(_, vertex)| vertex.point())
}

fn face_by_id<C, S>(faces: &[(FaceId, Face<Point3, C, S>)], face_id: FaceId) -> Option<&Face<Point3, C, S>> {
    faces.iter().find(|(id, _)| *id == face_id).map(|(_, face)| face)
}

fn point_projects_inside_face<C, S>(face: &Face<Point3, C, S>, parameters: (f64, f64), tolerance: f64) -> bool
where
    C: Clone,
{
    let uv = Point2::new(parameters.0, parameters.1);
    face.boundaries().into_iter().all(|wire| point_on_or_inside_wire(&wire, uv, tolerance))
}

fn point_on_or_inside_wire<C>(wire: &truck_topology::Wire<Point3, C>, point: Point2, tolerance: f64) -> bool
where
    C: Clone,
{
    let polygon: Vec<Point2> = wire.vertex_iter().map(|vertex| Point2::new(vertex.point().x, vertex.point().y)).collect();
    point_in_polygon(&polygon, point, tolerance)
}

fn point_in_polygon(polygon: &[Point2], point: Point2, tolerance: f64) -> bool {
    if polygon.len() < 3 {
        return false;
    }

    if polygon.windows(2).any(|edge| point_on_segment(point, edge[0], edge[1], tolerance))
        || point_on_segment(point, *polygon.last().unwrap(), polygon[0], tolerance)
    {
        return true;
    }

    let mut inside = false;
    let mut prev = *polygon.last().unwrap();
    for &curr in polygon {
        let intersects = ((curr.y > point.y) != (prev.y > point.y))
            && (point.x
                < (prev.x - curr.x) * (point.y - curr.y) / ((prev.y - curr.y).abs().max(f64::EPSILON))
                    + curr.x);
        if intersects {
            inside = !inside;
        }
        prev = curr;
    }

    inside
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
    fn vf_intersection_detects_vertex_on_surface() {
        let mut bopds = BopDs::with_options(BopOptions { geometric_tol: 1.0e-3, ..BopOptions::default() });
        let vertex = Vertex::new(Point3::new(0.25, 0.25, 0.0));
        let face = unit_square_face();

        let count = intersect_vf(
            &mut bopds,
            &[(VertexId(0), vertex)],
            &[(FaceId(10), face)],
            &[(VertexId(0), FaceId(10))],
        );

        assert_eq!(count, 1);
        assert_eq!(bopds.vf_interferences().len(), 1);
        assert_eq!(bopds.vf_interferences()[0].vertex, VertexId(0));
        assert_eq!(bopds.vf_interferences()[0].face, FaceId(10));
        assert!((bopds.vf_interferences()[0].parameters.0 - 0.25).abs() < 1.0e-9);
        assert!((bopds.vf_interferences()[0].parameters.1 - 0.25).abs() < 1.0e-9);
    }

    #[test]
    fn vf_intersection_detects_vertex_within_tolerance_of_surface() {
        let mut bopds = BopDs::with_options(BopOptions { geometric_tol: 0.01, ..BopOptions::default() });
        let vertex = Vertex::new(Point3::new(0.3, 0.4, 0.005));

        let count = intersect_vf(
            &mut bopds,
            &[(VertexId(0), vertex)],
            &[(FaceId(10), unit_square_face())],
            &[(VertexId(0), FaceId(10))],
        );

        assert_eq!(count, 1);
        assert_eq!(bopds.vf_interferences().len(), 1);
    }

    #[test]
    fn vf_intersection_ignores_vertex_far_from_surface() {
        let mut bopds = BopDs::with_options(BopOptions { geometric_tol: 0.01, ..BopOptions::default() });
        let vertex = Vertex::new(Point3::new(0.3, 0.4, 0.05));

        let count = intersect_vf(
            &mut bopds,
            &[(VertexId(0), vertex)],
            &[(FaceId(10), unit_square_face())],
            &[(VertexId(0), FaceId(10))],
        );

        assert_eq!(count, 0);
        assert!(bopds.vf_interferences().is_empty());
    }

    #[test]
    fn vf_intersection_rejects_projection_outside_face_bounds() {
        let mut bopds = BopDs::with_options(BopOptions { geometric_tol: 0.01, ..BopOptions::default() });
        let vertex = Vertex::new(Point3::new(1.2, 0.25, 0.0));

        let count = intersect_vf(
            &mut bopds,
            &[(VertexId(0), vertex)],
            &[(FaceId(10), unit_square_face())],
            &[(VertexId(0), FaceId(10))],
        );

        assert_eq!(count, 0);
        assert!(bopds.vf_interferences().is_empty());
    }

    #[test]
    fn vf_intersection_accepts_vertex_on_face_boundary() {
        let mut bopds = BopDs::with_options(BopOptions { geometric_tol: 1.0e-3, ..BopOptions::default() });
        let vertex = Vertex::new(Point3::new(1.0, 0.5, 0.0));

        let count = intersect_vf(
            &mut bopds,
            &[(VertexId(0), vertex)],
            &[(FaceId(10), unit_square_face())],
            &[(VertexId(0), FaceId(10))],
        );

        assert_eq!(count, 1);
        assert_eq!(bopds.vf_interferences().len(), 1);
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
        Face::new(vec![wire], truck_modeling::Surface::Plane(truck_modeling::Plane::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        )))
    }
}
