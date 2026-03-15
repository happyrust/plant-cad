//! Face trimming loop construction.

use crate::{bopds::{SectionCurve, TrimmingEdge, TrimmingLoop}, BopDs, FaceId};
use truck_base::cgmath64::{MetricSpace, Point2, Point3};
use truck_geotrait::{D2, Invertible, SearchParameter};
use truck_topology::Face;

const SEARCH_PARAMETER_TRIALS: usize = 100;

/// Builds trimming loops for the provided faces from original boundaries and projected section edges.
pub fn build_trimming_loops<C, S>(
    bopds: &mut BopDs,
    faces: &[(FaceId, Face<Point3, C, S>)],
) -> usize
where
    C: Clone,
    S: Clone + Invertible + SearchParameter<D2, Point = Point3>,
{
    let tolerance = bopds.options().parametric_tol.max(bopds.options().geometric_tol);
    let section_curves = bopds.section_curves().to_vec();
    let mut built = 0;

    for &(face_id, ref face) in faces {
        let loops = build_loops_for_face(face_id, face, &section_curves, tolerance);
        built += loops.len();
        for trimming_loop in loops {
            bopds.push_trimming_loop(trimming_loop);
        }
    }

    built
}

fn build_loops_for_face<C, S>(
    face_id: FaceId,
    face: &Face<Point3, C, S>,
    section_curves: &[SectionCurve],
    tolerance: f64,
) -> Vec<TrimmingLoop>
where
    C: Clone,
    S: Clone + Invertible + SearchParameter<D2, Point = Point3>,
{
    let mut loops = boundary_loops(face_id, face, tolerance);

    for section_curve in section_curves {
        let Some(parameters) = section_curve
            .face_parameters
            .iter()
            .find(|(candidate_face_id, _)| *candidate_face_id == face_id)
            .map(|(_, parameters)| parameters.clone())
        else {
            continue;
        };

        let closed = close_polyline(parameters, tolerance);
        if closed.len() < 4 {
            continue;
        }

        loops.push(loop_from_polyline(
            face_id,
            vec![TrimmingEdge {
                section_curve: Some(section_curve.id),
                uv_points: closed.clone(),
            }],
            closed,
            tolerance,
        ));
    }

    classify_loops(&mut loops);
    loops
}

fn boundary_loops<C, S>(face_id: FaceId, face: &Face<Point3, C, S>, tolerance: f64) -> Vec<TrimmingLoop>
where
    C: Clone,
    S: Clone + Invertible + SearchParameter<D2, Point = Point3>,
{
    let surface = face.oriented_surface();
    let mut result = Vec::new();

    for wire in face.boundaries() {
        let mut hint = None;
        let mut uv_points = Vec::new();

        for vertex in wire.vertex_iter() {
            let point = vertex.point();
            let uv = surface
                .search_parameter(point, hint.map(Into::into), SEARCH_PARAMETER_TRIALS)
                .or_else(|| surface.search_parameter(point, None, SEARCH_PARAMETER_TRIALS));
            let Some(uv) = uv else {
                uv_points.clear();
                break;
            };
            let uv = Point2::new(uv.0, uv.1);
            hint = Some(uv);
            uv_points.push(uv);
        }

        if uv_points.len() < 3 {
            continue;
        }

        let closed = close_polyline(uv_points, tolerance);
        let edges = closed
            .windows(2)
            .map(|segment| TrimmingEdge {
                section_curve: None,
                uv_points: vec![segment[0], segment[1]],
            })
            .collect();
        result.push(loop_from_polyline(face_id, edges, closed, tolerance));
    }

    result
}

fn loop_from_polyline(
    face_id: FaceId,
    edges: Vec<TrimmingEdge>,
    uv_points: Vec<Point2>,
    tolerance: f64,
) -> TrimmingLoop {
    let mut closed = close_polyline(uv_points, tolerance);
    let area = signed_area(&closed);
    if area.abs() <= tolerance {
        closed = dedup_consecutive_points(closed, tolerance);
    }

    TrimmingLoop {
        face: face_id,
        edges,
        signed_area: signed_area(&closed),
        uv_points: closed,
        is_outer: false,
    }
}

fn classify_loops(loops: &mut [TrimmingLoop]) {
    if loops.is_empty() {
        return;
    }

    let outer_index = loops
        .iter()
        .enumerate()
        .max_by(|(_, lhs), (_, rhs)| lhs.signed_area.abs().total_cmp(&rhs.signed_area.abs()))
        .map(|(index, _)| index)
        .unwrap();

    let outer_sign = loops[outer_index].signed_area.signum();
    for (index, trimming_loop) in loops.iter_mut().enumerate() {
        trimming_loop.is_outer = index == outer_index;
        if outer_sign != 0.0 && trimming_loop.signed_area.signum() == outer_sign {
            trimming_loop.uv_points.reverse();
            for edge in &mut trimming_loop.edges {
                edge.uv_points.reverse();
            }
            trimming_loop.signed_area = -trimming_loop.signed_area;
        }
    }
}

fn close_polyline(mut uv_points: Vec<Point2>, tolerance: f64) -> Vec<Point2> {
    uv_points = dedup_consecutive_points(uv_points, tolerance);
    if uv_points.len() >= 2 && uv_points.first().unwrap().distance2(*uv_points.last().unwrap()) > tolerance * tolerance {
        uv_points.push(*uv_points.first().unwrap());
    }
    uv_points
}

fn dedup_consecutive_points(uv_points: Vec<Point2>, tolerance: f64) -> Vec<Point2> {
    let mut deduped = Vec::with_capacity(uv_points.len());
    for point in uv_points {
        if deduped
            .last()
            .is_some_and(|previous: &Point2| previous.distance2(point) <= tolerance * tolerance)
        {
            continue;
        }
        deduped.push(point);
    }
    deduped
}

fn signed_area(polyline: &[Point2]) -> f64 {
    if polyline.len() < 3 {
        return 0.0;
    }

    let mut area = 0.0;
    let mut prev = *polyline.last().unwrap();
    for &curr in polyline {
        area += (curr.x + prev.x) * (curr.y - prev.y);
        prev = curr;
    }
    area / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{bopds::SectionCurve, BopOptions, VertexId};
    use truck_modeling::{builder, Curve, Surface};
    use truck_topology::Wire;

    #[test]
    fn trimming_loop_construction_uses_face_boundary_as_outer_loop() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face = unit_square_face();

        let count = build_trimming_loops(&mut bopds, &[(FaceId(0), face)]);

        assert_eq!(count, 1);
        let loops = bopds.trimming_loops();
        assert_eq!(loops.len(), 1);
        assert!(loops[0].is_outer);
        assert_eq!(loops[0].face, FaceId(0));
        assert_eq!(loops[0].uv_points.first(), loops[0].uv_points.last());
        assert_eq!(loops[0].edges.len(), 4);
    }

    #[test]
    fn trimming_loop_construction_adds_closed_section_loop_as_hole() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let section_curve_id = bopds.next_section_curve_id();
        bopds.push_section_curve(SectionCurve {
            id: section_curve_id,
            faces: (FaceId(0), FaceId(1)),
            start: VertexId(10),
            end: VertexId(10),
            samples: vec![
                Point3::new(0.25, 0.25, 0.0),
                Point3::new(0.75, 0.25, 0.0),
                Point3::new(0.75, 0.75, 0.0),
                Point3::new(0.25, 0.75, 0.0),
            ],
            face_parameters: [
                (
                    FaceId(0),
                    vec![
                        Point2::new(0.25, 0.25),
                        Point2::new(0.75, 0.25),
                        Point2::new(0.75, 0.75),
                        Point2::new(0.25, 0.75),
                    ],
                ),
                (
                    FaceId(1),
                    vec![
                        Point2::new(0.25, 0.25),
                        Point2::new(0.75, 0.25),
                        Point2::new(0.75, 0.75),
                        Point2::new(0.25, 0.75),
                    ],
                ),
            ],
        });

        let count = build_trimming_loops(&mut bopds, &[(FaceId(0), unit_square_face())]);

        assert_eq!(count, 2);
        let loops = bopds.trimming_loops();
        assert_eq!(loops.len(), 2);
        assert_eq!(loops.iter().filter(|loop_| loop_.is_outer).count(), 1);
        let hole = loops.iter().find(|loop_| !loop_.is_outer).unwrap();
        assert_eq!(hole.uv_points.first(), hole.uv_points.last());
        assert_eq!(hole.edges.len(), 1);
        assert_eq!(hole.edges[0].section_curve, Some(section_curve_id));
    }

    #[test]
    fn trimming_loop_construction_marks_existing_inner_boundary_as_hole() {
        let mut bopds = BopDs::with_options(BopOptions::default());

        let count = build_trimming_loops(&mut bopds, &[(FaceId(0), square_face_with_square_hole())]);

        assert_eq!(count, 2);
        let loops = bopds.trimming_loops();
        assert_eq!(loops.iter().filter(|loop_| loop_.is_outer).count(), 1);
        assert_eq!(loops.iter().filter(|loop_| !loop_.is_outer).count(), 1);
        assert!(loops.iter().all(|loop_| loop_.uv_points.first() == loop_.uv_points.last()));
        let outer = loops.iter().find(|loop_| loop_.is_outer).unwrap();
        let inner = loops.iter().find(|loop_| !loop_.is_outer).unwrap();
        assert!(outer.signed_area.abs() > inner.signed_area.abs());
        assert!(outer.edges.iter().all(|edge| edge.section_curve.is_none()));
        assert!(inner.edges.iter().all(|edge| edge.section_curve.is_none()));
    }

    fn unit_square_face() -> Face<Point3, Curve, Surface> {
        square_face(
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            None,
        )
    }

    fn square_face_with_square_hole() -> Face<Point3, Curve, Surface> {
        square_face(
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            Some([
                Point3::new(0.25, 0.25, 0.0),
                Point3::new(0.75, 0.25, 0.0),
                Point3::new(0.75, 0.75, 0.0),
                Point3::new(0.25, 0.75, 0.0),
            ]),
        )
    }

    fn square_face(
        outer: [Point3; 4],
        hole: Option<[Point3; 4]>,
    ) -> Face<Point3, Curve, Surface> {
        let outer_vertices = builder::vertices(outer);
        let outer_wire = Wire::from(vec![
            builder::line(&outer_vertices[0], &outer_vertices[1]),
            builder::line(&outer_vertices[1], &outer_vertices[2]),
            builder::line(&outer_vertices[2], &outer_vertices[3]),
            builder::line(&outer_vertices[3], &outer_vertices[0]),
        ]);

        let mut boundaries = vec![outer_wire];
        if let Some(hole) = hole {
            let hole_vertices = builder::vertices(hole);
            let mut hole_wire = Wire::from(vec![
                builder::line(&hole_vertices[0], &hole_vertices[1]),
                builder::line(&hole_vertices[1], &hole_vertices[2]),
                builder::line(&hole_vertices[2], &hole_vertices[3]),
                builder::line(&hole_vertices[3], &hole_vertices[0]),
            ]);
            hole_wire.invert();
            boundaries.push(hole_wire);
        }

        Face::new(
            boundaries,
            Surface::Plane(truck_modeling::Plane::new(outer[0], outer[1], outer[3])),
        )
    }
}
