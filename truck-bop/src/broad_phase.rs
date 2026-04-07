//! Broad-phase candidate generation based on bounding-box overlap.

use crate::bopds::shape_info::ShapeKind;
use crate::{BopDs, BopOptions, BoundingProvider, EdgeId, FaceBoundingSurface, FaceId, VertexId};
use truck_base::cgmath64::Point3;
use truck_geotrait::{BoundedCurve, Invertible};
use truck_topology::{Edge, Face, Vertex};

/// Broad-phase output grouped by entity pair type.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct CandidatePairs {
    /// Vertex-vertex candidate pairs.
    pub vv: Vec<(VertexId, VertexId)>,
    /// Vertex-edge candidate pairs.
    pub ve: Vec<(VertexId, EdgeId)>,
    /// Vertex-face candidate pairs.
    pub vf: Vec<(VertexId, FaceId)>,
    /// Edge-edge candidate pairs.
    pub ee: Vec<(EdgeId, EdgeId)>,
    /// Edge-face candidate pairs.
    pub ef: Vec<(EdgeId, FaceId)>,
    /// Face-face candidate pairs.
    pub ff: Vec<(FaceId, FaceId)>,
}

/// Generates naive O(n^2) broad-phase pairs using bounding-box overlap.
pub fn generate_candidate_pairs<C, S>(
    vertices: &[(VertexId, Vertex<Point3>)],
    edges: &[(EdgeId, Edge<Point3, C>)],
    faces: &[(FaceId, Face<Point3, C, S>)],
    options: &BopOptions,
) -> CandidatePairs
where
    C: Clone + BoundedCurve<Point = Point3> + Invertible,
    S: FaceBoundingSurface,
{
    let vertex_bboxes = vertices
        .iter()
        .map(|(id, vertex)| (*id, vertex.bounding_box(options)))
        .collect::<Vec<_>>();
    let edge_bboxes = edges
        .iter()
        .map(|(id, edge)| (*id, edge.bounding_box(options)))
        .collect::<Vec<_>>();
    let face_bboxes = faces
        .iter()
        .map(|(id, face)| (*id, face.bounding_box(options)))
        .collect::<Vec<_>>();

    collect_candidate_pairs(&vertex_bboxes, &edge_bboxes, &face_bboxes)
}

/// Generates broad-phase pairs from the source entities registered in `BopDs`.
pub fn generate_candidate_pairs_from_bopds<C, S>(
    bopds: &BopDs,
    vertices: &[(VertexId, Vertex<Point3>)],
    edges: &[(EdgeId, Edge<Point3, C>)],
    faces: &[(FaceId, Face<Point3, C, S>)],
) -> CandidatePairs
where
    C: Clone + BoundedCurve<Point = Point3> + Invertible,
    S: FaceBoundingSurface,
{
    let options = bopds.options();
    let vertex_bboxes = vertices
        .iter()
        .filter(|(id, _)| {
            bopds
                .vertex_shape_info(*id)
                .is_some_and(is_registered_vertex)
        })
        .map(|(id, vertex)| (*id, vertex.bounding_box(options)))
        .collect::<Vec<_>>();
    let edge_bboxes = edges
        .iter()
        .filter(|(id, _)| bopds.edge_shape_info(*id).is_some_and(is_registered_edge))
        .map(|(id, edge)| (*id, edge.bounding_box(options)))
        .collect::<Vec<_>>();
    let face_bboxes = faces
        .iter()
        .filter(|(id, _)| bopds.face_shape_info(*id).is_some_and(is_registered_face))
        .map(|(id, face)| (*id, face.bounding_box(options)))
        .collect::<Vec<_>>();

    collect_candidate_pairs(&vertex_bboxes, &edge_bboxes, &face_bboxes)
}

fn collect_candidate_pairs(
    vertex_bboxes: &[(VertexId, truck_base::bounding_box::BoundingBox<Point3>)],
    edge_bboxes: &[(EdgeId, truck_base::bounding_box::BoundingBox<Point3>)],
    face_bboxes: &[(FaceId, truck_base::bounding_box::BoundingBox<Point3>)],
) -> CandidatePairs {
    let mut candidates = CandidatePairs::default();

    for i in 0..vertex_bboxes.len() {
        for j in (i + 1)..vertex_bboxes.len() {
            let (lhs_id, lhs_bbox) = vertex_bboxes[i];
            let (rhs_id, rhs_bbox) = vertex_bboxes[j];
            if bbox_overlaps(lhs_bbox, rhs_bbox) {
                candidates.vv.push((lhs_id, rhs_id));
            }
        }
    }

    for &(vertex_id, vertex_bbox) in vertex_bboxes {
        for &(edge_id, edge_bbox) in edge_bboxes {
            if bbox_overlaps(vertex_bbox, edge_bbox) {
                candidates.ve.push((vertex_id, edge_id));
            }
        }
        for &(face_id, face_bbox) in face_bboxes {
            if bbox_overlaps(vertex_bbox, face_bbox) {
                candidates.vf.push((vertex_id, face_id));
            }
        }
    }

    for i in 0..edge_bboxes.len() {
        for j in (i + 1)..edge_bboxes.len() {
            let (lhs_id, lhs_bbox) = edge_bboxes[i];
            let (rhs_id, rhs_bbox) = edge_bboxes[j];
            if bbox_overlaps(lhs_bbox, rhs_bbox) {
                candidates.ee.push((lhs_id, rhs_id));
            }
        }
        let (edge_id, edge_bbox) = edge_bboxes[i];
        for &(face_id, face_bbox) in face_bboxes {
            if bbox_overlaps(edge_bbox, face_bbox) {
                candidates.ef.push((edge_id, face_id));
            }
        }
    }

    for i in 0..face_bboxes.len() {
        for j in (i + 1)..face_bboxes.len() {
            let (lhs_id, lhs_bbox) = face_bboxes[i];
            let (rhs_id, rhs_bbox) = face_bboxes[j];
            if bbox_overlaps(lhs_bbox, rhs_bbox) {
                candidates.ff.push((lhs_id, rhs_id));
            }
        }
    }

    candidates
}

fn is_registered_vertex(info: &crate::bopds::shape_info::ShapeInfo) -> bool {
    info.is_source && info.kind == ShapeKind::Vertex
}

fn is_registered_edge(info: &crate::bopds::shape_info::ShapeInfo) -> bool {
    info.is_source && info.kind == ShapeKind::Edge
}

fn is_registered_face(info: &crate::bopds::shape_info::ShapeInfo) -> bool {
    info.is_source && info.kind == ShapeKind::Face
}

fn bbox_overlaps(
    lhs: truck_base::bounding_box::BoundingBox<Point3>,
    rhs: truck_base::bounding_box::BoundingBox<Point3>,
) -> bool {
    !(lhs ^ rhs).is_empty()
}

#[cfg(test)]
mod tests {
    use super::*;
    use truck_modeling::builder;
    use truck_topology::{Edge, Face, Vertex, Wire};

    #[test]
    fn broad_phase_collects_all_overlapping_typed_pairs() {
        let options = BopOptions {
            geometric_tol: 0.05,
            ..BopOptions::default()
        };
        let vertex_a = Vertex::new(Point3::new(0.0, 0.0, 0.0));
        let vertex_b = Vertex::new(Point3::new(0.03, 0.0, 0.0));
        let edge_vertices =
            builder::vertices([Point3::new(-0.5, 0.0, 0.0), Point3::new(0.5, 0.0, 0.0)]);
        let edge: Edge<Point3, truck_modeling::Curve> =
            builder::line(&edge_vertices[0], &edge_vertices[1]);
        let face = triangle_face(
            Point3::new(-0.5, -0.5, 0.0),
            Point3::new(0.5, -0.5, 0.0),
            Point3::new(0.0, 0.5, 0.0),
        );

        let candidates = generate_candidate_pairs(
            &[
                (VertexId(0), vertex_a.clone()),
                (VertexId(1), vertex_b.clone()),
            ],
            &[(EdgeId(10), edge.clone())],
            &[(FaceId(20), face.clone())],
            &options,
        );

        assert_eq!(candidates.vv, vec![(VertexId(0), VertexId(1))]);
        assert_eq!(
            candidates.ve,
            vec![(VertexId(0), EdgeId(10)), (VertexId(1), EdgeId(10))]
        );
        assert_eq!(
            candidates.vf,
            vec![(VertexId(0), FaceId(20)), (VertexId(1), FaceId(20))]
        );
        assert_eq!(candidates.ee, Vec::<(EdgeId, EdgeId)>::new());
        assert_eq!(candidates.ef, vec![(EdgeId(10), FaceId(20))]);
        assert_eq!(candidates.ff, Vec::<(FaceId, FaceId)>::new());
    }

    #[test]
    fn broad_phase_filters_non_overlapping_pairs() {
        let options = BopOptions {
            geometric_tol: 0.01,
            ..BopOptions::default()
        };
        let vertex = Vertex::new(Point3::new(0.0, 0.0, 0.0));
        let edge_vertices =
            builder::vertices([Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)]);
        let edge: Edge<Point3, truck_modeling::Curve> =
            builder::line(&edge_vertices[0], &edge_vertices[1]);
        let face = triangle_face(
            Point3::new(4.0, 4.0, 0.0),
            Point3::new(5.0, 4.0, 0.0),
            Point3::new(4.0, 5.0, 0.0),
        );

        let candidates = generate_candidate_pairs(
            &[(VertexId(0), vertex)],
            &[(EdgeId(10), edge)],
            &[(FaceId(20), face)],
            &options,
        );

        assert!(candidates.vv.is_empty());
        assert!(candidates.ve.is_empty());
        assert!(candidates.vf.is_empty());
        assert!(candidates.ee.is_empty());
        assert!(candidates.ef.is_empty());
        assert!(candidates.ff.is_empty());
    }

    #[test]
    fn broad_phase_excludes_self_pairs_even_when_bbox_overlaps_itself() {
        let options = BopOptions {
            geometric_tol: 0.05,
            ..BopOptions::default()
        };
        let vertex = Vertex::new(Point3::new(0.0, 0.0, 0.0));
        let edge_vertices =
            builder::vertices([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);
        let edge: Edge<Point3, truck_modeling::Curve> =
            builder::line(&edge_vertices[0], &edge_vertices[1]);
        let face = triangle_face(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );

        let candidates = generate_candidate_pairs(
            &[(VertexId(0), vertex)],
            &[(EdgeId(10), edge)],
            &[(FaceId(20), face)],
            &options,
        );

        assert!(candidates.vv.is_empty());
        assert!(candidates.ee.is_empty());
        assert!(candidates.ff.is_empty());
        assert_eq!(candidates.ve.len(), 1);
        assert_eq!(candidates.vf.len(), 1);
        assert_eq!(candidates.ef.len(), 1);
    }

    #[test]
    fn broad_phase_bopds_reads_registered_shapes_and_preserves_overlap_filtering() {
        let options = BopOptions {
            geometric_tol: 0.05,
            ..BopOptions::default()
        };
        let mut bopds = BopDs::with_options(options);
        let vertex_a_id = bopds.register_vertex_source(0);
        let vertex_b_id = bopds.register_vertex_source(1);
        let edge_id = bopds.register_edge_source(0);
        let face_id = bopds.register_face_source(1);

        let vertex_a = Vertex::new(Point3::new(0.0, 0.0, 0.0));
        let vertex_b = Vertex::new(Point3::new(0.03, 0.0, 0.0));
        let edge_vertices =
            builder::vertices([Point3::new(-0.5, 0.0, 0.0), Point3::new(0.5, 0.0, 0.0)]);
        let edge: Edge<Point3, truck_modeling::Curve> =
            builder::line(&edge_vertices[0], &edge_vertices[1]);
        let face = triangle_face(
            Point3::new(-0.5, -0.5, 0.0),
            Point3::new(0.5, -0.5, 0.0),
            Point3::new(0.0, 0.5, 0.0),
        );

        let far_edge_vertices = builder::vertices([
            Point3::new(100.0, 100.0, 100.0),
            Point3::new(101.0, 100.0, 100.0),
        ]);
        let far_edge: Edge<Point3, truck_modeling::Curve> =
            builder::line(&far_edge_vertices[0], &far_edge_vertices[1]);

        let candidates = generate_candidate_pairs_from_bopds(
            &bopds,
            &[
                (vertex_a_id, vertex_a.clone()),
                (vertex_b_id, vertex_b.clone()),
                (VertexId(99), Vertex::new(Point3::new(100.0, 100.0, 100.0))),
            ],
            &[(edge_id, edge.clone()), (EdgeId(98), far_edge)],
            &[
                (face_id, face.clone()),
                (
                    FaceId(97),
                    triangle_face(
                        Point3::new(100.0, 100.0, 100.0),
                        Point3::new(101.0, 100.0, 100.0),
                        Point3::new(100.0, 101.0, 100.0),
                    ),
                ),
            ],
        );

        assert_eq!(candidates.vv, vec![(vertex_a_id, vertex_b_id)]);
        assert_eq!(
            candidates.ve,
            vec![(vertex_a_id, edge_id), (vertex_b_id, edge_id)]
        );
        assert_eq!(
            candidates.vf,
            vec![(vertex_a_id, face_id), (vertex_b_id, face_id)]
        );
        assert!(candidates.ee.is_empty());
        assert_eq!(candidates.ef, vec![(edge_id, face_id)]);
        assert!(candidates.ff.is_empty());
    }

    fn triangle_face(a: Point3, b: Point3, c: Point3) -> Face<Point3, truck_modeling::Curve, ()> {
        let vertices = builder::vertices([a, b, c]);
        let edge0: Edge<Point3, truck_modeling::Curve> = builder::line(&vertices[0], &vertices[1]);
        let edge1: Edge<Point3, truck_modeling::Curve> = builder::line(&vertices[1], &vertices[2]);
        let edge2: Edge<Point3, truck_modeling::Curve> = builder::line(&vertices[2], &vertices[0]);
        Face::new(vec![Wire::from(vec![edge0, edge1, edge2])], ())
    }
}
