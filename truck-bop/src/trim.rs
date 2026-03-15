//! Face trimming loop construction.

use crate::{bopds::{SectionCurve, SplitFace, TrimmingEdge, TrimmingLoop}, BopDs, FaceId, SectionCurveId};
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

/// Builds split-face provenance records from trimming loops already stored in `BopDs`.
pub fn build_split_faces(bopds: &mut BopDs) -> usize {
    let all_loops = bopds.trimming_loops().to_vec();
    let split_faces: Vec<SplitFace> = all_loops
        .iter()
        .enumerate()
        .filter(|(_, loop_)| loop_.is_outer)
        .map(|(outer_index, outer_loop)| {
            let mut trimming_loops = vec![all_loops[outer_index].clone()];
            trimming_loops.extend(
                all_loops
                    .iter()
                    .enumerate()
                    .filter(|(inner_index, loop_)| {
                        !loop_.is_outer
                            && *inner_index != outer_index
                            && loop_.face == outer_loop.face
                            && loop_contained_by(loop_, outer_loop)
                            && !contained_by_other_outer(*inner_index, outer_index, &all_loops)
                    })
                    .map(|(_, loop_)| loop_.clone()),
            );
            let splitting_edges = collect_splitting_edges(&trimming_loops);
            SplitFace {
                original_face: outer_loop.face,
                trimming_loops,
                splitting_edges,
            }
        })
        .collect();

    for split_face in &split_faces {
        bopds.push_split_face(split_face.clone());
    }

    split_faces.len()
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
            reverse_trimming_loop(trimming_loop, outer_index == index);
            trimming_loop.signed_area = -trimming_loop.signed_area;
        }
    }
}

fn reverse_trimming_loop(trimming_loop: &mut TrimmingLoop, is_closed: bool) {
    trimming_loop.uv_points.reverse();
    trimming_loop.edges.reverse();
    for edge in &mut trimming_loop.edges {
        edge.uv_points.reverse();
    }

    if is_closed {
        trimming_loop.uv_points = close_polyline(
            trimming_loop.uv_points.clone(),
            f64::EPSILON,
        );
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

fn collect_splitting_edges(trimming_loops: &[TrimmingLoop]) -> Vec<SectionCurveId> {
    let mut splitting_edges = Vec::new();
    for trimming_loop in trimming_loops {
        for edge in &trimming_loop.edges {
            let Some(section_curve) = edge.section_curve else {
                continue;
            };
            if !splitting_edges.contains(&section_curve) {
                splitting_edges.push(section_curve);
            }
        }
    }
    splitting_edges
}

fn loop_contained_by(inner: &TrimmingLoop, outer: &TrimmingLoop) -> bool {
    polygon_centroid(&inner.uv_points)
        .filter(|point| !point_on_polygon_boundary(&outer.uv_points, *point))
        .is_some_and(|point| point_in_polygon(&outer.uv_points, point))
}

fn contained_by_other_outer(
    inner_index: usize,
    owner_outer_index: usize,
    all_loops: &[TrimmingLoop],
) -> bool {
    let inner = &all_loops[inner_index];
    all_loops
        .iter()
        .enumerate()
        .filter(|(outer_index, candidate)| {
            candidate.is_outer
                && *outer_index != owner_outer_index
                && candidate.face == inner.face
                && loop_contained_by(inner, candidate)
        })
        .any(|(_, candidate)| {
            let owner = &all_loops[owner_outer_index];
            candidate.signed_area.abs() < owner.signed_area.abs()
        })
}

fn point_in_polygon(polygon: &[Point2], point: Point2) -> bool {
    let mut inside = false;
    let mut prev = *polygon.last().unwrap();
    for &curr in polygon {
        let intersects = (curr.y > point.y) != (prev.y > point.y)
            && point.x < (prev.x - curr.x) * (point.y - curr.y) / (prev.y - curr.y) + curr.x;
        if intersects {
            inside = !inside;
        }
        prev = curr;
    }
    inside
}

fn polygon_centroid(polygon: &[Point2]) -> Option<Point2> {
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

fn open_polygon_vertices(polygon: &[Point2]) -> &[Point2] {
    if polygon.len() >= 2 && polygon.first() == polygon.last() {
        &polygon[..polygon.len() - 1]
    } else {
        polygon
    }
}

fn point_on_polygon_boundary(polygon: &[Point2], point: Point2) -> bool {
    polygon.windows(2).any(|segment| point_on_segment(segment[0], segment[1], point))
}

fn point_on_segment(start: Point2, end: Point2, point: Point2) -> bool {
    let edge = end - start;
    let offset = point - start;
    let cross = edge.x * offset.y - edge.y * offset.x;
    if cross.abs() > 1.0e-9 {
        return false;
    }

    let dot = edge.x * offset.x + edge.y * offset.y;
    if dot < -1.0e-9 {
        return false;
    }

    let length2 = edge.x * edge.x + edge.y * edge.y;
    dot <= length2 + 1.0e-9
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
            face_projection_available: [(FaceId(0), true), (FaceId(1), true)],
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

    #[test]
    fn trimming_loop_reversal_preserves_closed_polyline_and_edge_connectivity() {
        let mut loop_record = TrimmingLoop {
            face: FaceId(0),
            edges: vec![
                TrimmingEdge {
                    section_curve: Some(SectionCurveId(1)),
                    uv_points: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
                },
                TrimmingEdge {
                    section_curve: Some(SectionCurveId(2)),
                    uv_points: vec![Point2::new(1.0, 0.0), Point2::new(1.0, 1.0)],
                },
                TrimmingEdge {
                    section_curve: Some(SectionCurveId(3)),
                    uv_points: vec![Point2::new(1.0, 1.0), Point2::new(0.0, 1.0)],
                },
                TrimmingEdge {
                    section_curve: Some(SectionCurveId(4)),
                    uv_points: vec![Point2::new(0.0, 1.0), Point2::new(0.0, 0.0)],
                },
            ],
            signed_area: 1.0,
            uv_points: vec![
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 0.0),
                Point2::new(1.0, 1.0),
                Point2::new(0.0, 1.0),
                Point2::new(0.0, 0.0),
            ],
            is_outer: false,
        };

        reverse_trimming_loop(&mut loop_record, true);

        assert_eq!(loop_record.uv_points.first(), loop_record.uv_points.last());
        assert_eq!(loop_record.edges[0].section_curve, Some(SectionCurveId(4)));
        assert_eq!(loop_record.edges[0].uv_points, vec![Point2::new(0.0, 0.0), Point2::new(0.0, 1.0)]);
        assert_eq!(loop_record.edges[1].uv_points[0], loop_record.edges[0].uv_points[1]);
        assert_eq!(loop_record.edges[2].uv_points[0], loop_record.edges[1].uv_points[1]);
        assert_eq!(loop_record.edges[3].uv_points[0], loop_record.edges[2].uv_points[1]);
        assert_eq!(loop_record.edges[3].uv_points[1], loop_record.edges[0].uv_points[0]);
    }

    #[test]
    fn split_face_records_group_loops_by_original_face() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let section_curve_id = bopds.next_section_curve_id();
        bopds.push_section_curve(section_curve(section_curve_id));

        let loops = build_trimming_loops(&mut bopds, &[(FaceId(0), unit_square_face())]);
        assert_eq!(loops, 2);

        let built = build_split_faces(&mut bopds);

        assert_eq!(built, 1);
        let split_faces = bopds.split_faces();
        assert_eq!(split_faces.len(), 1);
        assert_eq!(split_faces[0].original_face, FaceId(0));
        assert_eq!(split_faces[0].trimming_loops.len(), 2);
    }

    #[test]
    fn split_face_records_capture_splitting_edges_once() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let section_curve_id = bopds.next_section_curve_id();
        bopds.push_section_curve(section_curve(section_curve_id));

        build_trimming_loops(&mut bopds, &[(FaceId(0), unit_square_face())]);
        build_split_faces(&mut bopds);

        let split_face = &bopds.split_faces()[0];
        assert_eq!(split_face.splitting_edges, vec![section_curve_id]);
    }

    #[test]
    fn split_face_records_are_queryable_from_bopds() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        build_trimming_loops(
            &mut bopds,
            &[(FaceId(0), unit_square_face()), (FaceId(1), unit_square_face())],
        );

        let built = build_split_faces(&mut bopds);

        assert_eq!(built, 2);
        let faces: Vec<FaceId> = bopds.split_faces().iter().map(|split_face| split_face.original_face).collect();
        assert_eq!(faces, vec![FaceId(0), FaceId(1)]);
        assert!(bopds.split_faces().iter().all(|split_face| split_face.trimming_loops.len() == 1));
    }

    #[test]
    fn split_face_records_emit_distinct_fragments_for_multiple_outer_loops_on_same_face() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let first_section = SectionCurveId(11);
        let second_section = SectionCurveId(12);
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            edges: vec![TrimmingEdge {
                section_curve: Some(first_section),
                uv_points: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 0.0),
                Point2::new(1.0, 1.0),
                Point2::new(0.0, 1.0),
                Point2::new(0.0, 0.0),
            ],
            signed_area: -1.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            edges: vec![TrimmingEdge {
                section_curve: Some(second_section),
                uv_points: vec![Point2::new(2.0, 0.0), Point2::new(3.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(2.0, 0.0),
                Point2::new(3.0, 0.0),
                Point2::new(3.0, 1.0),
                Point2::new(2.0, 1.0),
                Point2::new(2.0, 0.0),
            ],
            signed_area: -1.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            edges: vec![TrimmingEdge {
                section_curve: Some(SectionCurveId(99)),
                uv_points: vec![Point2::new(0.25, 0.25), Point2::new(0.75, 0.25)],
            }],
            uv_points: vec![
                Point2::new(0.25, 0.25),
                Point2::new(0.75, 0.25),
                Point2::new(0.75, 0.75),
                Point2::new(0.25, 0.75),
                Point2::new(0.25, 0.25),
            ],
            signed_area: 0.25,
            is_outer: false,
        });

        let built = build_split_faces(&mut bopds);

        assert_eq!(built, 2);
        assert_eq!(bopds.split_faces().len(), 2);
        assert_eq!(bopds.split_faces()[0].trimming_loops.len(), 2);
        assert_eq!(bopds.split_faces()[0].splitting_edges, vec![first_section, SectionCurveId(99)]);
        assert_eq!(bopds.split_faces()[1].trimming_loops.len(), 1);
        assert_eq!(bopds.split_faces()[1].splitting_edges, vec![second_section]);
    }

    #[test]
    fn split_face_records_assign_inner_loop_to_containing_outer_fragment_only() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let outer_a = SectionCurveId(11);
        let outer_b = SectionCurveId(12);
        let hole = SectionCurveId(13);
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            edges: vec![TrimmingEdge {
                section_curve: Some(outer_a),
                uv_points: vec![Point2::new(0.0, 0.0), Point2::new(4.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(0.0, 0.0),
                Point2::new(4.0, 0.0),
                Point2::new(4.0, 4.0),
                Point2::new(0.0, 4.0),
                Point2::new(0.0, 0.0),
            ],
            signed_area: -16.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            edges: vec![TrimmingEdge {
                section_curve: Some(outer_b),
                uv_points: vec![Point2::new(5.0, 0.0), Point2::new(7.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(5.0, 0.0),
                Point2::new(7.0, 0.0),
                Point2::new(7.0, 2.0),
                Point2::new(5.0, 2.0),
                Point2::new(5.0, 0.0),
            ],
            signed_area: -4.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            edges: vec![TrimmingEdge {
                section_curve: Some(hole),
                uv_points: vec![Point2::new(1.0, 1.0), Point2::new(2.0, 1.0)],
            }],
            uv_points: vec![
                Point2::new(1.0, 1.0),
                Point2::new(2.0, 1.0),
                Point2::new(2.0, 2.0),
                Point2::new(1.0, 2.0),
                Point2::new(1.0, 1.0),
            ],
            signed_area: 1.0,
            is_outer: false,
        });

        let built = build_split_faces(&mut bopds);

        assert_eq!(built, 2);
        assert_eq!(bopds.split_faces()[0].trimming_loops.len(), 2);
        assert_eq!(bopds.split_faces()[0].splitting_edges, vec![outer_a, hole]);
        assert_eq!(bopds.split_faces()[1].trimming_loops.len(), 1);
        assert_eq!(bopds.split_faces()[1].splitting_edges, vec![outer_b]);
    }

    #[test]
    fn split_face_records_do_not_duplicate_inner_loop_across_nested_outer_fragments() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let outer = SectionCurveId(21);
        let nested_outer = SectionCurveId(22);
        let hole = SectionCurveId(23);
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            edges: vec![TrimmingEdge {
                section_curve: Some(outer),
                uv_points: vec![Point2::new(0.0, 0.0), Point2::new(6.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(0.0, 0.0),
                Point2::new(6.0, 0.0),
                Point2::new(6.0, 6.0),
                Point2::new(0.0, 6.0),
                Point2::new(0.0, 0.0),
            ],
            signed_area: -36.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            edges: vec![TrimmingEdge {
                section_curve: Some(nested_outer),
                uv_points: vec![Point2::new(1.0, 1.0), Point2::new(5.0, 1.0)],
            }],
            uv_points: vec![
                Point2::new(1.0, 1.0),
                Point2::new(5.0, 1.0),
                Point2::new(5.0, 5.0),
                Point2::new(1.0, 5.0),
                Point2::new(1.0, 1.0),
            ],
            signed_area: -16.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            edges: vec![TrimmingEdge {
                section_curve: Some(hole),
                uv_points: vec![Point2::new(2.0, 2.0), Point2::new(3.0, 2.0)],
            }],
            uv_points: vec![
                Point2::new(2.0, 2.0),
                Point2::new(3.0, 2.0),
                Point2::new(3.0, 3.0),
                Point2::new(2.0, 3.0),
                Point2::new(2.0, 2.0),
            ],
            signed_area: 1.0,
            is_outer: false,
        });

        build_split_faces(&mut bopds);

        assert_eq!(bopds.split_faces()[0].trimming_loops.len(), 1);
        assert_eq!(bopds.split_faces()[0].splitting_edges, vec![outer]);
        assert_eq!(bopds.split_faces()[1].trimming_loops.len(), 2);
        assert_eq!(bopds.split_faces()[1].splitting_edges, vec![nested_outer, hole]);
    }

    #[test]
    fn split_face_records_skip_faces_without_outer_loops() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            edges: vec![TrimmingEdge {
                section_curve: Some(SectionCurveId(7)),
                uv_points: vec![Point2::new(0.25, 0.25), Point2::new(0.75, 0.25)],
            }],
            uv_points: vec![
                Point2::new(0.25, 0.25),
                Point2::new(0.75, 0.25),
                Point2::new(0.75, 0.75),
                Point2::new(0.25, 0.75),
                Point2::new(0.25, 0.25),
            ],
            signed_area: -0.25,
            is_outer: false,
        });

        let built = build_split_faces(&mut bopds);

        assert_eq!(built, 0);
        assert!(bopds.split_faces().is_empty());
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

    fn section_curve(section_curve_id: SectionCurveId) -> SectionCurve {
        SectionCurve {
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
            face_projection_available: [(FaceId(0), true), (FaceId(1), true)],
        }
    }
}
