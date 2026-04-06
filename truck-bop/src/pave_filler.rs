//! Pave filler orchestration module.
//!
//! This module provides a minimal scheduler-style entry that collects
//! broad-phase candidates, dispatches geometry intersections, and drives
//! the cut/trim downstream preparation stages.

use std::any::TypeId;
use std::slice;

use crate::{
    broad_phase::CandidatePairs,
    intersect::{intersect_ee_into_bopds, intersect_ef, intersect_ff, intersect_ve, intersect_vf},
    trim, BopDs, EdgeId, FaceId, VertexId,
};
use truck_base::cgmath64::{EuclideanSpace, Point3};
use truck_geotrait::{
    BoundedCurve, Invertible, ParametricCurve, ParametricSurface, SearchNearestParameter,
    SearchParameter,
};
use truck_geotrait::{D1, D2};
use truck_modeling::{Curve, Surface};
use truck_topology::{Edge, Face, Vertex};

/// Execution counters for the current scheduler run.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BopOperationReport {
    /// Number of vertex-vertex candidates handled (currently recorded for completeness).
    pub vv: usize,
    /// Number of vertex-edge interferences discovered.
    pub ve: usize,
    /// Number of vertex-face interferences discovered.
    pub vf: usize,
    /// Number of edge-edge interferences discovered.
    pub ee: usize,
    /// Number of edge-face interferences discovered.
    pub ef: usize,
    /// Number of face-face interferences discovered.
    pub ff: usize,
    /// Number of face-face candidate pairs处理失败计数。
    pub ff_failures: usize,
    /// face1 或 face2 不存在于 candidate 映射表。
    pub ff_missing_face: usize,
    /// 面片三角化后边界缺失导致失败。
    pub ff_missing_triangulated_face: usize,
    /// 未检测到交线段（平行、错位或离散空）。
    pub ff_no_intersection_segments: usize,
    /// 曲线-曲面参数求交采样失败。
    pub ff_section_sampling_failed: usize,
    /// 采样退化，未生成有效区间。
    pub ff_degenerate_section: usize,
    /// Number of pave blocks currently stored in `BopDs` after split.
    pub pave_blocks: usize,
    /// Number of split-edge records prepared by the scheduler.
    pub split_edges: usize,
    /// Number of trimming loops built from section curves.
    pub trimming_loops: usize,
    /// Number of split faces built from trimming loops.
    pub split_faces: usize,
}

impl BopOperationReport {
    fn clear(&mut self) { *self = Self::default(); }

    /// Total number of interference events across all supported kinds.
    pub fn total_interference_count(&self) -> usize {
        self.ve + self.ee + self.vf + self.ef + self.ff
    }
}

/// Minimal front-end scheduler for intersection and pave preparation.
#[derive(Debug)]
pub struct PaveFiller {
    report: BopOperationReport,
}

impl PaveFiller {
    /// Creates a new filler with zeroed counters.
    pub fn new() -> Self {
        Self {
            report: BopOperationReport::default(),
        }
    }

    /// Returns the latest report snapshot.
    pub fn report(&self) -> BopOperationReport { self.report }

    /// Resets counters and transient state for a new run.
    pub fn prepare(&mut self) { self.report.clear(); }

    /// Runs vertex-edge / edge-edge / vertex-face / edge-face / face-face intersections.
    ///
    /// The method accepts a separate `faces_ff` parameter because face-face
    /// intersection currently depends on modeling face types.
    pub fn run_ve_ee_vf_ef_ff<C, S>(
        &mut self,
        bopds: &mut BopDs,
        vertices: &[(VertexId, Vertex<Point3>)],
        edges: &[(EdgeId, Edge<Point3, C>)],
        faces: &[(FaceId, Face<Point3, C, S>)],
        faces_ff: &[(FaceId, Face<Point3, Curve, Surface>)],
        candidates: &CandidatePairs,
    ) -> BopOperationReport
    where
        C: Clone
            + 'static
            + BoundedCurve<Point = Point3>
            + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
            + Invertible
            + SearchNearestParameter<D1, Point = Point3>,
        S: Clone
            + 'static
            + Invertible
            + ParametricSurface<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
            + SearchNearestParameter<D2, Point = Point3>
            + truck_geotrait::ParametricSurface3D,
    {
        self.prepare();

        self.report.vv = candidates.vv.len();
        self.report.ve = intersect_ve(bopds, vertices, edges, &candidates.ve);
        self.report.ee = intersect_ee_into_bopds(bopds, edges, &candidates.ee);
        self.report.vf = intersect_vf(bopds, vertices, faces, &candidates.vf);
        self.report.ef = intersect_ef(bopds, edges, faces, &candidates.ef);
        let ff_report = intersect_ff(bopds, faces_ff, &candidates.ff);
        let ff_failure_summary = ff_report.failure_summary();
        self.report.ff = ff_report.success;
        self.report.ff_failures = ff_failure_summary.total_failures();
        self.report.ff_missing_face = ff_failure_summary.missing_face;
        self.report.ff_missing_triangulated_face = ff_failure_summary.missing_triangulated_face;
        self.report.ff_no_intersection_segments = ff_failure_summary.no_intersection_segments;
        self.report.ff_section_sampling_failed = ff_failure_summary.section_sampling_failed;
        self.report.ff_degenerate_section = ff_failure_summary.degenerate_section;

        self.report
    }

    /// Runs vertex-edge / edge-edge / vertex-face / edge-face intersections.
    pub fn run_ve_ee_vf_ef<C, S>(
        &mut self,
        bopds: &mut BopDs,
        vertices: &[(VertexId, Vertex<Point3>)],
        edges: &[(EdgeId, Edge<Point3, C>)],
        faces: &[(FaceId, Face<Point3, C, S>)],
        candidates: &CandidatePairs,
    ) -> BopOperationReport
    where
        C: Clone
            + 'static
            + BoundedCurve<Point = Point3>
            + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
            + Invertible
            + SearchNearestParameter<D1, Point = Point3>,
        S: Clone
            + 'static
            + Invertible
            + ParametricSurface<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
            + SearchNearestParameter<D2, Point = Point3>
            + truck_geotrait::ParametricSurface3D,
    {
        self.prepare();

        self.report.vv = candidates.vv.len();
        self.report.ve = intersect_ve(bopds, vertices, edges, &candidates.ve);
        self.report.ee = intersect_ee_into_bopds(bopds, edges, &candidates.ee);
        self.report.vf = intersect_vf(bopds, vertices, faces, &candidates.vf);
        self.report.ef = intersect_ef(bopds, edges, faces, &candidates.ef);
        if let Some(faces_ff) = Self::faces_as_curve_surface(faces) {
            let ff_report = intersect_ff(bopds, faces_ff, &candidates.ff);
            let ff_failure_summary = ff_report.failure_summary();
            self.report.ff = ff_report.success;
            self.report.ff_failures = ff_failure_summary.total_failures();
            self.report.ff_missing_face = ff_failure_summary.missing_face;
            self.report.ff_missing_triangulated_face = ff_failure_summary.missing_triangulated_face;
            self.report.ff_no_intersection_segments = ff_failure_summary.no_intersection_segments;
            self.report.ff_section_sampling_failed = ff_failure_summary.section_sampling_failed;
            self.report.ff_degenerate_section = ff_failure_summary.degenerate_section;
        }

        self.report
    }

    #[allow(unsafe_code)]
    fn faces_as_curve_surface<C, S>(
        faces: &[(FaceId, Face<Point3, C, S>)],
    ) -> Option<&[(FaceId, Face<Point3, Curve, Surface>)]>
    where
        C: 'static,
        S: 'static, {
        if TypeId::of::<C>() != TypeId::of::<Curve>()
            || TypeId::of::<S>() != TypeId::of::<Surface>()
        {
            return None;
        }

        let len = faces.len();
        let ptr = faces.as_ptr() as *const (FaceId, Face<Point3, Curve, Surface>);

        // SAFETY:
        // This cast is sound only when `C == Curve` and `S == Surface`,
        // ensured by the TypeId checks above.
        Some(unsafe { slice::from_raw_parts(ptr, len) })
    }

    /// Splits all pave blocks for the provided edges.
    pub fn split_pave_blocks<I>(&mut self, bopds: &mut BopDs, edge_ids: I) -> usize
    where I: IntoIterator<Item = EdgeId> {
        for edge_id in edge_ids {
            bopds.split_pave_blocks_for_edge(edge_id);
        }
        self.report.pave_blocks = bopds.pave_blocks().len();
        self.report.pave_blocks
    }

    /// Builds split-edge-level intermediate results from the settled pave-block partition.
    pub fn make_split_edges(&mut self, bopds: &mut BopDs) -> usize {
        bopds.clear_split_edges();

        let materialized = bopds
            .pave_blocks()
            .iter()
            .enumerate()
            .map(|(index, block)| {
                crate::SplitEdgeRecord::new(
                    crate::SplitEdgeId(u32::MAX),
                    block.original_edge,
                    block.start_vertex,
                    block.end_vertex,
                    block.param_range,
                    bopds.common_block_for_pave_block(crate::PaveBlockId(index as u32)),
                    block.unsplittable,
                )
            })
            .collect::<Vec<_>>();

        for split_edge in materialized {
            bopds.push_split_edge(split_edge);
        }

        self.report.split_edges = bopds.split_edges().len();
        self.report.split_edges
    }

    /// Builds trimming loops for target faces.
    pub fn build_trimming_loops<C, S>(
        &mut self,
        bopds: &mut BopDs,
        faces: &[(FaceId, Face<Point3, C, S>)],
    ) -> usize
    where
        C: Clone,
        S: Clone + Invertible + SearchParameter<D2, Point = Point3>,
    {
        self.report.trimming_loops = trim::build_trimming_loops(bopds, faces);
        self.report.trimming_loops
    }

    /// Builds split faces from previously built trimming loops.
    pub fn build_split_faces(&mut self, bopds: &mut BopDs) -> usize {
        self.report.split_faces = trim::build_split_faces(bopds);
        self.report.split_faces
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{bopds::Pave, BopOptions, CandidatePairs};
    use truck_base::cgmath64::Point3;
    use truck_modeling::{builder, Curve, Surface};
    use truck_topology::{Edge, Face, Vertex};

    #[test]
    fn pave_filler_runs_empty_input() {
        let mut bopds = BopDs::new();
        let mut filler = PaveFiller::new();
        let empty_faces: Vec<(FaceId, Face<Point3, Curve, Surface>)> = Vec::new();

        let report = filler.run_ve_ee_vf_ef_ff(
            &mut bopds,
            &[],
            &[],
            &empty_faces,
            &empty_faces,
            &CandidatePairs::default(),
        );

        assert_eq!(report, BopOperationReport::default());
        assert_eq!(filler.report(), BopOperationReport::default());
        assert_eq!(filler.split_pave_blocks(&mut bopds, std::iter::empty()), 0);
        assert_eq!(filler.make_split_edges(&mut bopds), 0);
        assert_eq!(
            filler.build_trimming_loops::<Curve, Surface>(&mut bopds, &empty_faces),
            0
        );
        assert_eq!(filler.build_split_faces(&mut bopds), 0);
    }

    #[test]
    fn pave_filler_reports_ff_failures_by_reason() {
        let mut bopds = BopDs::new();
        let mut filler = PaveFiller::new();

        let mut candidates = CandidatePairs::default();
        candidates.ff.push((FaceId(0), FaceId(1)));
        candidates.ff.push((FaceId(0), FaceId(2)));

        let existing_faces: Vec<(FaceId, Face<Point3, Curve, Surface>)> = vec![
            (FaceId(0), unit_square_face()),
            (FaceId(1), yz_square_face(5.0)),
        ];

        let report = filler.run_ve_ee_vf_ef_ff::<Curve, Surface>(
            &mut bopds,
            &[],
            &[],
            &existing_faces,
            &existing_faces,
            &candidates,
        );

        assert_eq!(report.ff, 0);
        assert_eq!(report.ff_failures, 2);
        assert_eq!(report.ff_missing_face, 1);
        assert_eq!(report.ff_no_intersection_segments, 1);
        assert_eq!(report.ff_missing_triangulated_face, 0);
        assert_eq!(report.ff_section_sampling_failed, 0);
        assert_eq!(report.ff_degenerate_section, 0);
        assert!(bopds.ff_interferences().is_empty());
        assert!(bopds.section_curves().is_empty());
    }

    #[test]
    fn pave_filler_partitions_ff_candidates_into_success_and_failures() {
        let mut bopds = BopDs::new();
        let mut filler = PaveFiller::new();

        let mut candidates = CandidatePairs::default();
        // 成功：单位 XY 平面与 x=0 平面相交，产出一条区段线。
        candidates.ff.push((FaceId(0), FaceId(1)));
        // 无交线：两平面平行且不重叠。
        candidates.ff.push((FaceId(0), FaceId(2)));
        // face 缺失：第二张面不存在。
        candidates.ff.push((FaceId(0), FaceId(99)));

        let existing_faces: Vec<(FaceId, Face<Point3, Curve, Surface>)> = vec![
            (
                FaceId(0),
                square_face(
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(1.0, 1.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                ),
            ),
            (FaceId(1), yz_square_face(0.0)),
            (FaceId(2), yz_square_face(5.0)),
        ];

        let report = filler.run_ve_ee_vf_ef_ff::<Curve, Surface>(
            &mut bopds,
            &[],
            &[],
            &existing_faces,
            &existing_faces,
            &candidates,
        );

        assert_eq!(report.ff + report.ff_failures, candidates.ff.len());
        assert_eq!(report.ff, 1);
        assert_eq!(report.ff_failures, 2);
        assert_eq!(report.ff_missing_face, 1);
        assert_eq!(report.ff_no_intersection_segments, 1);
        assert_eq!(report.ff_missing_triangulated_face, 0);
        assert_eq!(report.ff_section_sampling_failed, 0);
        assert_eq!(report.ff_degenerate_section, 0);
    }

    #[test]
    fn pave_filler_reports_counts() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-3,
            ..BopOptions::default()
        });

        let vertex = Vertex::new(Point3::new(0.25, 0.0, 0.0));
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let vertices = vec![(VertexId(0), vertex)];
        let edges = vec![(EdgeId(10), edge)];

        let mut pairs = CandidatePairs::default();
        pairs.ve.push((VertexId(0), EdgeId(10)));

        // Ensure endpoint paves are available, so VE insertion can split existing blocks.
        bopds.rebuild_paves_for_edges(&edges);

        let mut filler = PaveFiller::new();
        let empty_faces: Vec<(FaceId, Face<Point3, Curve, Surface>)> = Vec::new();
        let report = filler.run_ve_ee_vf_ef_ff::<Curve, Surface>(
            &mut bopds,
            &vertices,
            &edges,
            &[],
            &empty_faces,
            &pairs,
        );

        assert_eq!(report.vv, 0);
        assert_eq!(report.ve, 1);
        assert_eq!(report.ee, 0);
        assert_eq!(report.vf, 0);
        assert_eq!(report.ef, 0);
        assert_eq!(report.ff, 0);

        let split_blocks =
            filler.split_pave_blocks(&mut bopds, edges.iter().map(|(edge_id, _)| *edge_id));
        assert_eq!(split_blocks, 2);
        let split_edges = filler.make_split_edges(&mut bopds);
        assert_eq!(split_edges, 2);

        let faces: Vec<(FaceId, Face<Point3, Curve, Surface>)> = Vec::new();
        let trimming = filler.build_trimming_loops(&mut bopds, &faces);
        let split_faces = filler.build_split_faces(&mut bopds);

        let report = filler.report();
        assert!(report.total_interference_count() >= report.vv);
        assert_eq!(report.split_edges, 2);
        assert_eq!(bopds.split_edges().len(), 2);
        assert_eq!(report.trimming_loops, trimming);
        assert_eq!(report.split_faces, split_faces);
    }

    #[test]
    fn make_split_edges_materializes_one_record_per_final_pave_block() {
        let mut bopds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-8,
            ..BopOptions::default()
        });
        let edge_id = EdgeId(10);

        bopds.push_pave(Pave::new(edge_id, VertexId(1), 0.0, 1.0e-8).unwrap());
        bopds.push_pave(Pave::new(edge_id, VertexId(2), 0.25, 1.0e-8).unwrap());
        bopds.push_pave(Pave::new(edge_id, VertexId(3), 0.75, 1.0e-8).unwrap());
        bopds.push_pave(Pave::new(edge_id, VertexId(4), 1.0, 1.0e-8).unwrap());
        bopds.rebuild_pave_blocks_from_paves(edge_id);

        let expected_blocks = bopds.pave_blocks().to_vec();
        assert_eq!(expected_blocks.len(), 3);

        let mut filler = PaveFiller::new();
        let split_edges = filler.make_split_edges(&mut bopds);

        assert_eq!(split_edges, expected_blocks.len());
        assert_eq!(bopds.split_edges().len(), expected_blocks.len());

        for (record, block) in bopds.split_edges().iter().zip(expected_blocks.iter()) {
            assert_eq!(record.original_edge, block.original_edge);
            assert_eq!(record.start_vertex, block.start_vertex);
            assert_eq!(record.end_vertex, block.end_vertex);
            assert_eq!(record.param_range, block.param_range);
            assert_eq!(record.unsplittable, block.unsplittable);
        }
    }

    #[test]
    fn split_edges_count_matches_pave_blocks() {
        let mut bopds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-8,
            ..BopOptions::default()
        });
        let edge_id = EdgeId(20);

        bopds.push_pave(Pave::new(edge_id, VertexId(5), 0.0, 1.0e-8).unwrap());
        bopds.push_pave(Pave::new(edge_id, VertexId(6), 0.5, 1.0e-8).unwrap());
        bopds.push_pave(Pave::new(edge_id, VertexId(7), 1.0, 1.0e-8).unwrap());
        bopds.rebuild_pave_blocks_from_paves(edge_id);

        let mut filler = PaveFiller::new();
        let count = filler.make_split_edges(&mut bopds);

        assert_eq!(count, bopds.pave_blocks().len());
        assert_eq!(filler.report().split_edges, bopds.split_edges().len());
    }

    #[test]
    fn make_split_edges_preserves_unsplittable_micro_segments() {
        let mut bopds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let edge_id = EdgeId(30);

        bopds.push_pave(Pave::new(edge_id, VertexId(8), 0.0, 1.0e-9).unwrap());
        bopds.push_pave(Pave::new(edge_id, VertexId(9), 5.0e-7, 1.0e-9).unwrap());
        bopds.push_pave(Pave::new(edge_id, VertexId(10), 1.0, 1.0e-9).unwrap());
        bopds.rebuild_pave_blocks_from_paves(edge_id);

        assert!(bopds.pave_blocks()[0].unsplittable);

        let mut filler = PaveFiller::new();
        filler.make_split_edges(&mut bopds);

        let first = &bopds.split_edges()[0];
        assert!(first.unsplittable);
        assert_eq!(first.param_range, bopds.pave_blocks()[0].param_range);
        assert_eq!(first.start_vertex, VertexId(8));
        assert_eq!(first.end_vertex, VertexId(9));
    }

    #[test]
    fn make_split_edges_carries_common_block_link_when_present() {
        use crate::bopds::CommonBlock;

        let mut bopds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-8,
            ..BopOptions::default()
        });
        let edge_id = EdgeId(40);

        bopds.push_pave(Pave::new(edge_id, VertexId(20), 0.0, 1.0e-8).unwrap());
        bopds.push_pave(Pave::new(edge_id, VertexId(21), 0.5, 1.0e-8).unwrap());
        bopds.push_pave(Pave::new(edge_id, VertexId(22), 1.0, 1.0e-8).unwrap());
        bopds.rebuild_pave_blocks_from_paves(edge_id);

        let common_block_id = bopds.push_common_block(CommonBlock::new(
            vec![crate::PaveBlockId(0)],
            vec![FaceId(99)],
            Some(edge_id),
        ));

        let mut filler = PaveFiller::new();
        let split_edges = filler.make_split_edges(&mut bopds);

        assert_eq!(split_edges, 2);
        assert_eq!(bopds.split_edges()[0].common_block, Some(common_block_id));
        assert_eq!(bopds.split_edges()[1].common_block, None);

        let linked_ids: Vec<_> = bopds
            .split_edges_for_common_block(common_block_id)
            .map(|record| record.id)
            .collect();
        assert_eq!(linked_ids, vec![crate::SplitEdgeId(0)]);
    }

    #[test]
    fn pavefiller_builds_faceinfo_commonblock() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-6,
            ..BopOptions::default()
        });

        let edge_id = bopds.register_edge_source(0);
        let face_id = bopds.register_face_source(1);
        let edge = line_edge(Point3::new(0.0, 0.5, 0.0), Point3::new(0.0, 0.5, 1.0));
        let face = unit_square_face();
        let edges = vec![(edge_id, edge)];
        let faces = vec![(face_id, face)];

        let pairs = crate::generate_candidate_pairs_from_bopds(&bopds, &[], &edges, &faces);
        assert!(pairs.ef.contains(&(edge_id, face_id)));

        bopds.rebuild_paves_for_edges(&edges);
        let mut filler = PaveFiller::new();
        let report = filler.run_ve_ee_vf_ef_ff::<Curve, Surface>(
            &mut bopds,
            &[],
            &edges,
            &faces,
            &faces,
            &pairs,
        );

        assert!(report.ef > 0);
        let face_info = bopds
            .face_info(face_id)
            .expect("face info should be created");
        let all_pave_blocks = face_info
            .on_pave_blocks
            .iter()
            .chain(&face_info.sc_pave_blocks)
            .chain(&face_info.in_pave_blocks)
            .collect::<Vec<_>>();
        assert!(all_pave_blocks
            .iter()
            .any(|pave_block| bopds.common_block_for_pave_block(**pave_block).is_some()));

        assert!(!bopds.common_blocks().is_empty());
    }

    fn line_edge(start: Point3, end: Point3) -> Edge<Point3, Curve> {
        let vertices = builder::vertices([start, end]);
        builder::line(&vertices[0], &vertices[1])
    }

    fn yz_square_face(x_offset: f64) -> Face<Point3, Curve, Surface> {
        square_face(
            Point3::new(x_offset, 0.0, 0.0),
            Point3::new(x_offset, 1.0, 0.0),
            Point3::new(x_offset, 1.0, 1.0),
            Point3::new(x_offset, 0.0, 1.0),
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

    fn unit_square_face() -> Face<Point3, Curve, Surface> {
        let vertices = builder::vertices([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ]);
        let edges: Vec<Edge<Point3, Curve>> = vec![
            builder::line(&vertices[0], &vertices[1]),
            builder::line(&vertices[1], &vertices[2]),
            builder::line(&vertices[2], &vertices[3]),
            builder::line(&vertices[3], &vertices[0]),
        ];
        Face::new(
            vec![truck_topology::Wire::from(edges)],
            Surface::Plane(truck_modeling::Plane::new(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            )),
        )
    }
}
