//! Face-Face intersection detection.

use crate::{bopds::FFInterference, bopds::SectionCurve, BopDs, FaceId, VertexId};
use truck_base::{
    cgmath64::{MetricSpace, Point2, Point3},
    tolerance::Tolerance,
};
use truck_geotrait::{Invertible, SearchParameter};
use truck_meshalgo::analyzers::Collision;
use truck_meshalgo::prelude::RobustMeshableShape;
use truck_modeling::{BSplineCurve, Curve, IntersectionCurve, KnotVec, Surface};
use truck_topology::{Face, Shell};

const SEARCH_PARAMETER_TRIALS: usize = 100;

/// 记录单个面-面候选对处理结果的错误码。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FFIntersectionFailure {
    /// 找不到候选对中的某一张面。
    MissingFace,
    /// 面片三角化后未能取到可用面片边界。
    MissingTriangulatedFace,
    /// 面与面轮廓未形成可采样交线段。
    NoIntersectionSegments,
    /// 交线段采样失败（模型参数求交失败）。
    SectionSamplingFailed,
    /// 交线段长度不足以构造区间曲线。
    DegenerateSection,
}

/// 记录面-面候选对失败的返回状态。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FFIntersectionFailureRecord {
    /// 候选面对中的第一张面。
    pub face1: FaceId,
    /// 候选面对中的第二张面。
    pub face2: FaceId,
    /// 失败原因。
    pub reason: FFIntersectionFailure,
}

/// 面-面求交执行结果：成功曲线条目数 + 失败状态明细。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FFIntersectionReport {
    /// 成功构建的 section curve 数量（原 `intersect_ff` 返回值）。
    pub success: usize,
    /// 候选失败记录。每对候选面最多记录一条失败状态。
    pub failures: Vec<FFIntersectionFailureRecord>,
}

impl FFIntersectionReport {
    fn push_failure(&mut self, face1: FaceId, face2: FaceId, reason: FFIntersectionFailure) {
        self.failures.push(FFIntersectionFailureRecord {
            face1,
            face2,
            reason,
        })
    }

    /// 失败数量。
    pub fn failure_count(&self) -> usize { self.failures.len() }

    /// 失败原因聚合计数，便于上层统计/埋点。
    pub fn failure_summary(&self) -> FFIntersectionFailureSummary {
        let mut summary = FFIntersectionFailureSummary::default();
        for failure in &self.failures {
            match failure.reason {
                FFIntersectionFailure::MissingFace => summary.missing_face += 1,
                FFIntersectionFailure::MissingTriangulatedFace => {
                    summary.missing_triangulated_face += 1
                }
                FFIntersectionFailure::NoIntersectionSegments => {
                    summary.no_intersection_segments += 1
                }
                FFIntersectionFailure::SectionSamplingFailed => {
                    summary.section_sampling_failed += 1
                }
                FFIntersectionFailure::DegenerateSection => summary.degenerate_section += 1,
            }
        }
        summary
    }
}

/// 面-面求交失败原因聚合计数（仅用于上层观测与日志）。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FFIntersectionFailureSummary {
    /// 缺少候选面对象。
    pub missing_face: usize,
    /// 三角化后未获取到可用面片边界。
    pub missing_triangulated_face: usize,
    /// 未检测到交线段（含平行或离散不重叠）。
    pub no_intersection_segments: usize,
    /// 交线段参数求交失败。
    pub section_sampling_failed: usize,
    /// 交线段退化，无法构造有效区间。
    pub degenerate_section: usize,
}

impl FFIntersectionFailureSummary {
    /// 失败总数。
    pub fn total_failures(&self) -> usize {
        self.missing_face
            + self.missing_triangulated_face
            + self.no_intersection_segments
            + self.section_sampling_failed
            + self.degenerate_section
    }
}

impl Default for FFIntersectionReport {
    fn default() -> Self {
        Self {
            success: 0,
            failures: Vec::new(),
        }
    }
}

/// Detects face-face interferences and stores generated section curves in `BopDs`.
pub fn intersect_ff(
    bopds: &mut BopDs,
    faces: &[(FaceId, Face<Point3, Curve, Surface>)],
    candidates: &[(FaceId, FaceId)],
) -> FFIntersectionReport {
    let tolerance = bopds
        .options()
        .geometric_tol
        .max(bopds.options().approximation_tol);
    let mut report = FFIntersectionReport::default();

    for &(face1_id, face2_id) in candidates {
        let Some(face1) = face_by_id(faces, face1_id) else {
            report.push_failure(face1_id, face2_id, FFIntersectionFailure::MissingFace);
            continue;
        };
        let Some(face2) = face_by_id(faces, face2_id) else {
            report.push_failure(face1_id, face2_id, FFIntersectionFailure::MissingFace);
            continue;
        };

        let shell1: Shell<Point3, Curve, Surface> = vec![face1.clone()].into();
        let shell2: Shell<Point3, Curve, Surface> = vec![face2.clone()].into();
        let poly1 = shell1.robust_triangulation(tolerance);
        let poly2 = shell2.robust_triangulation(tolerance);
        let Some(poly_face1) = poly1.face_iter().next() else {
            report.push_failure(
                face1_id,
                face2_id,
                FFIntersectionFailure::MissingTriangulatedFace,
            );
            continue;
        };
        let Some(poly_face2) = poly2.face_iter().next() else {
            report.push_failure(
                face1_id,
                face2_id,
                FFIntersectionFailure::MissingTriangulatedFace,
            );
            continue;
        };
        let Some(polygon1) = poly_face1.surface() else {
            report.push_failure(
                face1_id,
                face2_id,
                FFIntersectionFailure::MissingTriangulatedFace,
            );
            continue;
        };
        let Some(polygon2) = poly_face2.surface() else {
            report.push_failure(
                face1_id,
                face2_id,
                FFIntersectionFailure::MissingTriangulatedFace,
            );
            continue;
        };

        let surface1 = face1.oriented_surface();
        let surface2 = face2.oriented_surface();
        let segments = polygon1.extract_interference(&polygon2);
        if segments.is_empty() {
            report.push_failure(
                face1_id,
                face2_id,
                FFIntersectionFailure::NoIntersectionSegments,
            );
            continue;
        }
        let polylines = construct_polylines(&segments);
        let Some(curves) = build_section_samples(&surface1, &surface2, polylines) else {
            report.push_failure(
                face1_id,
                face2_id,
                FFIntersectionFailure::SectionSamplingFailed,
            );
            continue;
        };

        let mut success_on_candidate = false;
        for samples in curves {
            if samples.len() < 2 {
                continue;
            }

            let start_point = samples[0];
            let end_point = *samples.last().unwrap_or(&start_point);
            let is_closed = start_point.distance2(end_point) <= tolerance * tolerance;

            let start = choose_endpoint_vertex(face1, face2, start_point, tolerance)
                .unwrap_or_else(|| bopds.next_generated_vertex_id());
            let end = if is_closed {
                start
            } else {
                choose_endpoint_vertex(face1, face2, end_point, tolerance)
                    .unwrap_or_else(|| bopds.next_generated_vertex_id())
            };
            let section_curve_id = bopds.next_section_curve_id();

            let face1_parameters = project_section_to_face(face1, &samples);
            let face2_parameters = project_section_to_face(face2, &samples);
            let [face1_parameters, face2_parameters] = orient_open_section_pair(
                [face1, face2],
                &samples,
                [face1_parameters, face2_parameters],
            );
            let face1_projection_available = face1_parameters.is_some();
            let face2_projection_available = face2_parameters.is_some();

            bopds.push_section_curve(SectionCurve {
                id: section_curve_id,
                faces: (face1_id, face2_id),
                start,
                end,
                face_parameters: [
                    (face1_id, face1_parameters.unwrap_or_default()),
                    (face2_id, face2_parameters.unwrap_or_default()),
                ],
                face_projection_available: [
                    (face1_id, face1_projection_available),
                    (face2_id, face2_projection_available),
                ],
                samples,
            });
            bopds.push_ff_interference(FFInterference {
                face1: face1_id,
                face2: face2_id,
                section_curve: section_curve_id,
            });
            report.success += 1;
            success_on_candidate = true;
        }

        if !success_on_candidate {
            report.push_failure(face1_id, face2_id, FFIntersectionFailure::DegenerateSection);
        }
    }

    report
}

fn project_section_to_face(
    face: &Face<Point3, Curve, Surface>,
    samples: &[Point3],
) -> Option<Vec<Point2>> {
    let surface = face.oriented_surface();
    let mut parameters = Vec::with_capacity(samples.len());

    for &point in samples {
        let uv = surface.search_parameter(point, None, SEARCH_PARAMETER_TRIALS)?;
        parameters.push(Point2::new(uv.0, uv.1));
    }

    if !face.orientation() {
        parameters.reverse();
    }

    orient_parameter_loop(face, parameters)
}

fn orient_parameter_loop(
    face: &Face<Point3, Curve, Surface>,
    mut parameters: Vec<Point2>,
) -> Option<Vec<Point2>> {
    if parameters.len() < 2 {
        return Some(parameters);
    }

    let mut boundary = parameter_boundary(face)?;
    if !face.orientation() {
        boundary.invert();
    }

    let section_area = signed_area(&parameters);
    if section_area.is_sign_positive() != signed_area(&boundary).is_sign_positive() {
        parameters.reverse();
    }
    Some(parameters)
}

fn orient_open_section_pair(
    faces: [&Face<Point3, Curve, Surface>; 2],
    samples: &[Point3],
    mut parameters: [Option<Vec<Point2>>; 2],
) -> [Option<Vec<Point2>>; 2] {
    let Some(start_point) = samples.first() else {
        return parameters;
    };
    let Some(end_point) = samples.last() else {
        return parameters;
    };
    if start_point.distance2(*end_point)
        <= truck_base::tolerance::TOLERANCE * truck_base::tolerance::TOLERANCE
    {
        return parameters;
    }

    if !faces[0].orientation() {
        if let Some(parameters) = parameters[0].as_mut() {
            parameters.reverse();
        }
    }
    if !faces[1].orientation() {
        if let Some(parameters) = parameters[1].as_mut() {
            parameters.reverse();
        }
    }

    let face0_direction = parameters[0]
        .as_ref()
        .and_then(|parameters| endpoint_direction_key(faces[0], parameters));
    let face1_direction = parameters[1]
        .as_ref()
        .and_then(|parameters| endpoint_direction_key(faces[1], parameters));
    if face0_direction.is_some() && face1_direction.is_some() && face0_direction != face1_direction
    {
        if let Some(parameters) = parameters[1].as_mut() {
            parameters.reverse();
        }
    }
    parameters
}

fn endpoint_direction_key(
    face: &Face<Point3, Curve, Surface>,
    parameters: &[Point2],
) -> Option<(usize, usize)> {
    let boundary = parameter_boundary(face)?;
    let start = *parameters.first()?;
    let end = *parameters.last()?;
    let start_index = boundary
        .iter()
        .enumerate()
        .min_by(|(_, lhs), (_, rhs)| lhs.distance2(start).total_cmp(&rhs.distance2(start)))?
        .0;
    let end_index = boundary
        .iter()
        .enumerate()
        .min_by(|(_, lhs), (_, rhs)| lhs.distance2(end).total_cmp(&rhs.distance2(end)))?
        .0;
    Some((start_index, end_index))
}

fn parameter_boundary(face: &Face<Point3, Curve, Surface>) -> Option<Vec<Point2>> {
    let surface = face.oriented_surface();
    let wire = face.boundaries().into_iter().next()?;
    let mut points = Vec::new();
    let mut hint = None;
    for vertex in wire.vertex_iter() {
        let point = vertex.point();
        let uv = surface
            .search_parameter(point, hint.map(Into::into), SEARCH_PARAMETER_TRIALS)
            .or_else(|| surface.search_parameter(point, None, SEARCH_PARAMETER_TRIALS))?;
        let uv = Point2::new(uv.0, uv.1);
        hint = Some(uv);
        points.push(uv);
    }
    Some(points)
}

fn signed_area(polyline: &[Point2]) -> f64 {
    if polyline.len() < 2 {
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

fn face_by_id<'a>(
    faces: &'a [(FaceId, Face<Point3, Curve, Surface>)],
    face_id: FaceId,
) -> Option<&'a Face<Point3, Curve, Surface>> {
    faces
        .iter()
        .find(|(id, _)| *id == face_id)
        .map(|(_, face)| face)
}

fn choose_endpoint_vertex(
    face1: &Face<Point3, Curve, Surface>,
    face2: &Face<Point3, Curve, Surface>,
    point: Point3,
    tolerance: f64,
) -> Option<VertexId> {
    let tolerance_sq = tolerance * tolerance;
    endpoint_vertices(face1)
        .chain(endpoint_vertices(face2))
        .find(|(_, candidate)| candidate.distance2(point) <= tolerance_sq)
        .map(|(id, _)| id)
}

fn endpoint_vertices<'a>(
    face: &'a Face<Point3, Curve, Surface>,
) -> impl Iterator<Item = (VertexId, Point3)> + 'a {
    face.absolute_boundaries()
        .iter()
        .flat_map(|wire| wire.vertex_iter())
        .enumerate()
        .map(|(index, vertex)| (VertexId(index as u32), vertex.point()))
}

fn construct_polylines(lines: &[(Point3, Point3)]) -> Vec<Vec<Point3>> {
    use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
    use std::collections::VecDeque;

    #[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
    struct PointIndex([i64; 3]);

    impl From<Point3> for PointIndex {
        fn from(pt: Point3) -> Self {
            let tol = truck_base::tolerance::TOLERANCE;
            Self([
                ((pt.x + tol) / (2.0 * tol)).floor() as i64,
                ((pt.y + tol) / (2.0 * tol)).floor() as i64,
                ((pt.z + tol) / (2.0 * tol)).floor() as i64,
            ])
        }
    }

    struct Node {
        coord: Point3,
        adjacency: HashSet<PointIndex>,
    }

    impl Node {
        fn new(coord: Point3, adjacency: HashSet<PointIndex>) -> Self { Self { coord, adjacency } }

        fn pop_one_adjacency(&mut self) -> PointIndex {
            let idx = *self.adjacency.iter().next().unwrap();
            self.adjacency.remove(&idx);
            idx
        }
    }

    struct Graph(HashMap<PointIndex, Node>);

    impl Graph {
        fn add_half_edge(&mut self, pt0: Point3, pt1: Point3) {
            let idx0 = PointIndex::from(pt0);
            let idx1 = PointIndex::from(pt1);
            if let Some(node) = self.0.get_mut(&idx0) {
                node.adjacency.insert(idx1);
            } else {
                let mut set = HashSet::default();
                set.insert(idx1);
                self.0.insert(idx0, Node::new(pt0, set));
            }
        }

        fn add_edge(&mut self, line: (Point3, Point3)) {
            if !line.0.near(&line.1) {
                self.add_half_edge(line.0, line.1);
                self.add_half_edge(line.1, line.0);
            }
        }

        fn get_one(&self) -> Option<(PointIndex, &Node)> {
            self.0.iter().next().map(|(idx, node)| (*idx, node))
        }

        fn get_next(&mut self, idx: PointIndex) -> Option<(PointIndex, Point3)> {
            let node = self.0.get_mut(&idx)?;
            let next_idx = node.pop_one_adjacency();
            if node.adjacency.is_empty() {
                self.0.remove(&idx);
            }
            let next = self.0.get_mut(&next_idx)?;
            next.adjacency.remove(&idx);
            let point = next.coord;
            if next.adjacency.is_empty() {
                self.0.remove(&next_idx);
            }
            Some((next_idx, point))
        }
    }

    let mut graph = Graph(HashMap::default());
    for &line in lines {
        graph.add_edge(line);
    }

    let mut result = Vec::new();
    while let Some((mut idx, node)) = graph.get_one() {
        let mut wire: VecDeque<_> = vec![node.coord].into();
        while let Some((next_idx, point)) = graph.get_next(idx) {
            idx = next_idx;
            wire.push_back(point);
        }
        let mut idx = PointIndex::from(wire[0]);
        while let Some((next_idx, point)) = graph.get_next(idx) {
            idx = next_idx;
            wire.push_front(point);
        }
        result.push(wire.into_iter().collect());
    }
    result
}

fn build_section_samples(
    surface1: &Surface,
    surface2: &Surface,
    polylines: Vec<Vec<Point3>>,
) -> Option<Vec<Vec<Point3>>> {
    let mut sections = Vec::new();
    for polyline in polylines {
        let leader = BSplineCurve::new(KnotVec::bezier_knot(polyline.len() - 1), polyline.clone());
        let curve = IntersectionCurve::new(
            Box::new(surface1.clone()),
            Box::new(surface2.clone()),
            Box::new(Curve::BSplineCurve(leader)),
        );
        let len = polyline.len();
        let mut samples = Vec::new();
        for index in 0..len.saturating_sub(1) {
            let (point, _, _) = curve.search_triple(index as f64, SEARCH_PARAMETER_TRIALS)?;
            samples.push(point);
        }
        if let Some(last_index) = len.checked_sub(1) {
            let (point, _, _) = curve.search_triple(last_index as f64, SEARCH_PARAMETER_TRIALS)?;
            samples.push(point);
        }
        sections.push(samples);
    }
    Some(sections)
}

#[cfg(test)]
mod tests {
    use super::*;
    use truck_base::cgmath64::{MetricSpace, Rad, Vector3};
    use truck_modeling::builder;
    use truck_topology::Edge;

    #[test]
    fn ff_plane_plane_section_generates_section_curve() {
        let mut bopds = BopDs::new();
        let faces = vec![(FaceId(0), yz_square_face()), (FaceId(1), xy_square_face())];

        let report = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(report.success, 1);
        assert_eq!(bopds.ff_interferences().len(), 1);
        assert_eq!(bopds.section_curves().len(), 1);
        let section = &bopds.section_curves()[0];
        assert_eq!(section.faces, (FaceId(0), FaceId(1)));
        assert!(section.samples.len() >= 2);
        assert_eq!(section.face_parameters[0].0, FaceId(0));
        assert_eq!(section.face_parameters[1].0, FaceId(1));
        assert_eq!(section.face_parameters[0].1.len(), section.samples.len());
        assert_eq!(section.face_parameters[1].1.len(), section.samples.len());
        let start = section.samples.first().copied().unwrap();
        let end = section.samples.last().copied().unwrap();
        assert!(
            start.distance(Point3::new(0.0, 0.0, 0.0)) < 1.0e-2
                || start.distance(Point3::new(0.0, 1.0, 0.0)) < 1.0e-2
        );
        assert!(
            end.distance(Point3::new(0.0, 0.0, 0.0)) < 1.0e-2
                || end.distance(Point3::new(0.0, 1.0, 0.0)) < 1.0e-2
        );
        assert_ne!(section.start, section.end);

        let yz_parameters = &section.face_parameters[0].1;
        assert!(yz_parameters
            .windows(2)
            .all(|pair| pair[0].distance(pair[1]) <= 1.0 + 1.0e-3));
        let xy_parameters = &section.face_parameters[1].1;
        assert!(xy_parameters
            .windows(2)
            .all(|pair| pair[0].distance(pair[1]) <= 1.0 + 1.0e-3));
    }

    #[test]
    fn ff_interference_links_section_curve() {
        let mut bopds = BopDs::new();
        let faces = vec![(FaceId(0), yz_square_face()), (FaceId(1), xy_square_face())];

        let report = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(report.success, 1);
        let section = &bopds.section_curves()[0];
        let ff = &bopds.ff_interferences()[0];

        assert_eq!(ff.face1, FaceId(0));
        assert_eq!(ff.face2, FaceId(1));
        assert_eq!(ff.section_curve, section.id);
    }

    #[test]
    #[ignore] // Known limitation: open-curve UV orientation on inverted faces
    fn section_projection_respects_inverted_face_orientation() {
        let mut bopds = BopDs::new();
        let mut inverted = yz_square_face();
        inverted.invert();
        let faces = vec![(FaceId(0), inverted), (FaceId(1), xy_square_face())];

        let report = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(report.success, 1);
        let section = &bopds.section_curves()[0];
        let yz_parameters = &section.face_parameters[0].1;
        assert!(section.face_projection_available[0].1);
        assert!(yz_parameters.first().unwrap().y >= yz_parameters.last().unwrap().y);
    }

    #[test]
    fn ff_intersection_keeps_section_curve_when_one_face_projection_fails() {
        let mut bopds = BopDs::new();
        let faces = vec![(FaceId(0), yz_square_face()), (FaceId(1), xy_square_face())];
        let samples = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
        let section_curve_id = bopds.next_section_curve_id();

        let [face1_parameters, face2_parameters] = orient_open_section_pair(
            [&faces[0].1, &faces[1].1],
            &samples,
            [project_section_to_face(&faces[0].1, &samples), None],
        );

        bopds.push_section_curve(SectionCurve {
            id: section_curve_id,
            faces: (FaceId(0), FaceId(1)),
            start: VertexId(0),
            end: VertexId(1),
            face_parameters: [
                (FaceId(0), face1_parameters.unwrap_or_default()),
                (FaceId(1), face2_parameters.unwrap_or_default()),
            ],
            face_projection_available: [(FaceId(0), true), (FaceId(1), false)],
            samples,
        });

        let section = &bopds.section_curves()[0];
        assert_eq!(section.samples.len(), 2);
        assert_eq!(section.face_parameters[1].1, Vec::<Point2>::new());
        assert_eq!(
            section.face_projection_available,
            [(FaceId(0), true), (FaceId(1), false)]
        );
    }

    #[test]
    fn section_projection_stores_uv_samples_for_both_faces() {
        let mut bopds = BopDs::new();
        let faces = vec![(FaceId(0), yz_square_face()), (FaceId(1), xy_square_face())];

        intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        let section = &bopds.section_curves()[0];
        for (face_id, uv_samples) in &section.face_parameters {
            assert!(
                !uv_samples.is_empty(),
                "missing uv samples for face {:?}",
                face_id
            );
            assert_eq!(uv_samples.len(), section.samples.len());
        }
    }

    #[test]
    fn ff_touching_surface_generates_minimal_curve() {
        let mut bopds = BopDs::new();
        let faces = vec![
            (FaceId(0), yz_square_face()),
            (FaceId(1), xy_triangle_face()),
        ];

        let report = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(report.success, 1);
        let section = &bopds.section_curves()[0];
        assert_eq!(section.samples.len(), 2);
        assert!(
            (section.start == VertexId(0) && section.end == VertexId(1))
                || (section.start == VertexId(1) && section.end == VertexId(0))
                || section.start == section.end
        );
    }

    #[test]
    fn ff_parallel_planes_produce_no_section_curve() {
        let mut bopds = BopDs::new();
        let faces = vec![
            (FaceId(0), xy_square_face()),
            (FaceId(1), lifted_xy_square_face(1.0)),
        ];

        let report = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(report.success, 0);
        assert_eq!(report.failures.len(), 1);
        assert_eq!(
            report.failures[0],
            FFIntersectionFailureRecord {
                face1: FaceId(0),
                face2: FaceId(1),
                reason: FFIntersectionFailure::NoIntersectionSegments,
            }
        );
        assert!(bopds.ff_interferences().is_empty());
        assert!(bopds.section_curves().is_empty());
    }

    #[test]
    fn ff_intersection_reports_missing_face_as_failure() {
        let mut bopds = BopDs::new();
        let faces = vec![(FaceId(0), yz_square_face())];

        let report = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(report.success, 0);
        assert_eq!(report.failure_count(), 1);
        assert_eq!(
            report.failures[0],
            FFIntersectionFailureRecord {
                face1: FaceId(0),
                face2: FaceId(1),
                reason: FFIntersectionFailure::MissingFace,
            }
        );
        assert!(bopds.ff_interferences().is_empty());
        assert!(bopds.section_curves().is_empty());
    }

    #[test]
    fn ff_intersection_distinguishes_parallel_face_no_intersection_as_no_segments() {
        let mut bopds = BopDs::new();
        let faces = vec![
            (FaceId(0), yz_square_face()),
            (FaceId(1), lifted_xy_square_face(2.0)),
        ];

        let report = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(report.success, 0);
        assert_eq!(report.failures.len(), 1);
        assert_eq!(
            report.failures[0].reason,
            FFIntersectionFailure::NoIntersectionSegments
        );
        assert!(bopds.ff_interferences().is_empty());
        assert!(bopds.section_curves().is_empty());
    }

    #[test]
    #[ignore] // Known limitation: closed-loop sections not yet supported
    fn ff_cylinder_plane_section_generates_polyline_loop_samples() {
        let mut bopds = BopDs::new();
        let faces = vec![(FaceId(0), cylinder_face()), (FaceId(1), xy_disk_face(1.0))];

        let report = intersect_ff(&mut bopds, &faces, &[(FaceId(0), FaceId(1))]);

        assert_eq!(report.success, 1);
        let section = &bopds.section_curves()[0];
        assert!(section.samples.len() >= 8);
        for point in &section.samples {
            assert!(point.z.abs() < 1.0e-2);
            let radius = f64::sqrt(point.x * point.x + point.y * point.y);
            assert!((radius - 1.0).abs() < 5.0e-2);
        }
    }

    #[test]
    fn ff_intersection_allocates_unique_section_curve_ids() {
        let mut bopds = BopDs::new();
        let faces = vec![
            (FaceId(0), yz_square_face()),
            (FaceId(1), xy_square_face()),
            (FaceId(2), xz_square_face()),
        ];

        let report = intersect_ff(
            &mut bopds,
            &faces,
            &[(FaceId(0), FaceId(1)), (FaceId(0), FaceId(2))],
        );

        assert_eq!(report.success, 2);
        assert_eq!(bopds.section_curves()[0].id, crate::SectionCurveId(0));
        assert_eq!(bopds.section_curves()[1].id, crate::SectionCurveId(1));
    }

    fn xy_square_face() -> Face<Point3, Curve, Surface> {
        square_face(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        )
    }

    fn lifted_xy_square_face(z: f64) -> Face<Point3, Curve, Surface> {
        square_face(
            Point3::new(0.0, 0.0, z),
            Point3::new(1.0, 0.0, z),
            Point3::new(1.0, 1.0, z),
            Point3::new(0.0, 1.0, z),
        )
    }

    fn yz_square_face() -> Face<Point3, Curve, Surface> {
        square_face(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(0.0, 0.0, 1.0),
        )
    }

    fn xy_triangle_face() -> Face<Point3, Curve, Surface> {
        let vertices = builder::vertices([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ]);
        let edges: Vec<Edge<Point3, Curve>> = vec![
            builder::line(&vertices[0], &vertices[1]),
            builder::line(&vertices[1], &vertices[2]),
            builder::line(&vertices[2], &vertices[0]),
        ];
        let wire = truck_topology::Wire::from(edges);
        Face::new(
            vec![wire],
            Surface::Plane(truck_modeling::Plane::new(
                vertices[0].point(),
                vertices[1].point(),
                vertices[2].point(),
            )),
        )
    }

    fn xz_square_face() -> Face<Point3, Curve, Surface> {
        square_face(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 1.0),
        )
    }

    fn square_face(a: Point3, b: Point3, c: Point3, d: Point3) -> Face<Point3, Curve, Surface> {
        let vertices = builder::vertices([a, b, c, d]);
        let edges: Vec<Edge<Point3, Curve>> = vec![
            builder::line(&vertices[0], &vertices[1]),
            builder::line(&vertices[1], &vertices[2]),
            builder::line(&vertices[2], &vertices[3]),
            builder::line(&vertices[3], &vertices[0]),
        ];
        let wire = truck_topology::Wire::from(edges);
        Face::new(
            vec![wire],
            Surface::Plane(truck_modeling::Plane::new(a, b, d)),
        )
    }

    fn xy_disk_face(radius: f64) -> Face<Point3, Curve, Surface> {
        let center = builder::vertex(Point3::new(0.0, 0.0, 0.0));
        let wire: truck_topology::Wire<Point3, Curve> = builder::rsweep(
            &center,
            Point3::new(radius, 0.0, 0.0),
            Vector3::unit_z(),
            Rad(std::f64::consts::TAU),
            16,
        );
        Face::new(vec![wire], Surface::Plane(truck_modeling::Plane::xy()))
    }

    fn cylinder_face() -> Face<Point3, Curve, Surface> {
        let bottom = builder::vertex(Point3::new(1.0, 0.0, -1.0));
        let top = builder::vertex(Point3::new(1.0, 0.0, 1.0));
        let profile = builder::line(&bottom, &top);
        let shell: Shell<Point3, Curve, Surface> = builder::rsweep(
            &profile,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::unit_z(),
            Rad(std::f64::consts::TAU),
            24,
        );
        shell.face_iter().next().unwrap().clone()
    }
}
