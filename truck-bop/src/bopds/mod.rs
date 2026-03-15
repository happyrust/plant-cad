//! Boolean operation data structure

mod arena;
mod common_block;
mod face_info;
mod ids;
mod interference;
mod pave;
mod pave_block;
pub(crate) mod shape_info;

pub use interference::{
    EEInterference, EFInterference, FFInterference, InterferenceTable, SectionCurve,
    TrimmingEdge, TrimmingLoop, VEInterference, VFInterference, VVInterference,
};
pub use ids::{
    CommonBlockId, EdgeId, FaceId, PaveBlockId, SectionCurveId, ShapeId, VertexId,
};
pub use pave::Pave;
pub use pave_block::PaveBlock;

use rustc_hash::FxHashMap;
use crate::BopOptions;
use shape_info::{ShapeInfo, ShapeKind};
use truck_base::cgmath64::Point3;
use truck_geotrait::{BoundedCurve, Invertible, ParametricCurve};
use truck_topology::Edge;

/// Data structure
#[derive(Debug)]
pub struct BopDs {
    options: BopOptions,
    shapes: Vec<ShapeInfo>,
    vertex_to_shape: FxHashMap<VertexId, ShapeId>,
    edge_to_shape: FxHashMap<EdgeId, ShapeId>,
    face_to_shape: FxHashMap<FaceId, ShapeId>,
    interferences: InterferenceTable,
    paves: Vec<Pave>,
    pave_blocks: Vec<PaveBlock>,
    next_section_curve_id: u32,
    next_generated_vertex_id: u32,
}

impl BopDs {
    /// Create new BopDs
    pub fn new() -> Self {
        Self {
            options: BopOptions::default(),
            shapes: Vec::new(),
            vertex_to_shape: FxHashMap::default(),
            edge_to_shape: FxHashMap::default(),
            face_to_shape: FxHashMap::default(),
            interferences: InterferenceTable::default(),
            paves: Vec::new(),
            pave_blocks: Vec::new(),
            next_section_curve_id: 0,
            next_generated_vertex_id: 1_000_000,
        }
    }

    /// Create new BopDs with explicit options.
    pub fn with_options(options: BopOptions) -> Self {
        Self {
            options,
            ..Self::new()
        }
    }

    /// Borrow the configured options.
    pub fn options(&self) -> &BopOptions {
        &self.options
    }

    /// Register vertex source
    pub fn register_vertex_source(&mut self, operand_rank: u8) -> VertexId {
        let shape_id = ShapeId(self.shapes.len() as u32);
        self.shapes.push(ShapeInfo {
            kind: ShapeKind::Vertex,
            operand_rank,
            is_source: true,
        });
        let vertex_id = VertexId(shape_id.0);
        self.vertex_to_shape.insert(vertex_id, shape_id);
        vertex_id
    }

    /// Register edge source
    pub fn register_edge_source(&mut self, operand_rank: u8) -> EdgeId {
        let shape_id = ShapeId(self.shapes.len() as u32);
        self.shapes.push(ShapeInfo {
            kind: ShapeKind::Edge,
            operand_rank,
            is_source: true,
        });
        let edge_id = EdgeId(shape_id.0);
        self.edge_to_shape.insert(edge_id, shape_id);
        edge_id
    }

    /// Register face source
    pub fn register_face_source(&mut self, operand_rank: u8) -> FaceId {
        let shape_id = ShapeId(self.shapes.len() as u32);
        self.shapes.push(ShapeInfo {
            kind: ShapeKind::Face,
            operand_rank,
            is_source: true,
        });
        let face_id = FaceId(shape_id.0);
        self.face_to_shape.insert(face_id, shape_id);
        face_id
    }

    /// Get shape ID for vertex
    pub fn vertex_shape_id(&self, vertex_id: VertexId) -> Option<ShapeId> {
        self.vertex_to_shape.get(&vertex_id).copied()
    }

    /// Get shape ID for edge
    pub fn edge_shape_id(&self, edge_id: EdgeId) -> Option<ShapeId> {
        self.edge_to_shape.get(&edge_id).copied()
    }

    /// Get shape ID for face
    pub fn face_shape_id(&self, face_id: FaceId) -> Option<ShapeId> {
        self.face_to_shape.get(&face_id).copied()
    }

    /// Get shape info
    pub fn shape_info(&self, shape_id: ShapeId) -> Option<&ShapeInfo> {
        self.shapes.get(shape_id.0 as usize)
    }

    /// Get shape info for a registered vertex.
    pub fn vertex_shape_info(&self, vertex_id: VertexId) -> Option<&ShapeInfo> {
        self.vertex_shape_id(vertex_id).and_then(|shape_id| self.shape_info(shape_id))
    }

    /// Get shape info for a registered edge.
    pub fn edge_shape_info(&self, edge_id: EdgeId) -> Option<&ShapeInfo> {
        self.edge_shape_id(edge_id).and_then(|shape_id| self.shape_info(shape_id))
    }

    /// Get shape info for a registered face.
    pub fn face_shape_info(&self, face_id: FaceId) -> Option<&ShapeInfo> {
        self.face_shape_id(face_id).and_then(|shape_id| self.shape_info(shape_id))
    }

    /// Store a vertex-vertex interference.
    pub fn push_vv_interference(&mut self, interference: VVInterference) {
        self.interferences.push_vv(interference);
    }

    /// Store a vertex-edge interference.
    pub fn push_ve_interference(&mut self, interference: VEInterference) {
        self.interferences.push_ve(interference);
    }

    /// Store a vertex-face interference.
    pub fn push_vf_interference(&mut self, interference: VFInterference) {
        self.interferences.push_vf(interference);
    }

    /// Store an edge-edge interference.
    pub fn push_ee_interference(&mut self, interference: EEInterference) {
        self.interferences.push_ee(interference);
    }

    /// Store an edge-face interference.
    pub fn push_ef_interference(&mut self, interference: EFInterference) {
        self.interferences.push_ef(interference);
    }

    /// Store a face-face interference.
    pub fn push_ff_interference(&mut self, interference: FFInterference) {
        self.interferences.push_ff(interference);
    }

    /// Allocate a fresh section curve identifier.
    pub fn next_section_curve_id(&mut self) -> SectionCurveId {
        let id = SectionCurveId(self.next_section_curve_id);
        self.next_section_curve_id += 1;
        id
    }

    /// Allocate a generated vertex identifier for synthesized section endpoints.
    pub fn next_generated_vertex_id(&mut self) -> VertexId {
        let id = VertexId(self.next_generated_vertex_id);
        self.next_generated_vertex_id += 1;
        id
    }

    /// Store a section curve.
    pub fn push_section_curve(&mut self, section_curve: SectionCurve) {
        self.interferences.push_section_curve(section_curve);
    }

    /// Store a trimming loop.
    pub fn push_trimming_loop(&mut self, trimming_loop: TrimmingLoop) {
        self.interferences.push_trimming_loop(trimming_loop);
    }

    /// Store a pave.
    pub fn push_pave(&mut self, pave: Pave) {
        self.paves.push(pave);
        self.normalize_edge_paves(pave.edge);
    }

    /// Rebuild sorted, deduplicated pave lists for all known edges while ensuring endpoints exist.
    pub fn rebuild_paves_for_edges<C>(&mut self, edges: &[(EdgeId, Edge<Point3, C>)])
    where
        C: Clone + ParametricCurve<Point = Point3> + BoundedCurve + Invertible,
    {
        for &(edge_id, ref edge) in edges {
            self.ensure_edge_endpoints(edge_id, edge);
            self.normalize_edge_paves(edge_id);
            self.rebuild_pave_blocks_for_edge(edge_id);
        }
    }

    /// Borrow stored paves for a specific edge.
    pub fn paves_for_edge(&self, edge_id: EdgeId) -> Vec<Pave> {
        self.paves
            .iter()
            .copied()
            .filter(|pave| pave.edge == edge_id)
            .collect()
    }

    /// Borrow stored pave blocks for a specific edge.
    pub fn pave_blocks_for_edge(&self, edge_id: EdgeId) -> Vec<PaveBlock> {
        self.pave_blocks
            .iter()
            .copied()
            .filter(|block| block.original_edge == edge_id)
            .collect()
    }

    /// Borrow all stored vertex-vertex interferences.
    pub fn vv_interferences(&self) -> &[VVInterference] {
        self.interferences.vv()
    }

    /// Borrow all stored vertex-edge interferences.
    pub fn ve_interferences(&self) -> &[VEInterference] {
        self.interferences.ve()
    }

    /// Borrow all stored vertex-face interferences.
    pub fn vf_interferences(&self) -> &[VFInterference] {
        self.interferences.vf()
    }

    /// Borrow all stored edge-edge interferences.
    pub fn ee_interferences(&self) -> &[EEInterference] {
        self.interferences.ee()
    }

    /// Borrow all stored edge-face interferences.
    pub fn ef_interferences(&self) -> &[EFInterference] {
        self.interferences.ef()
    }

    /// Borrow all stored face-face interferences.
    pub fn ff_interferences(&self) -> &[FFInterference] {
        self.interferences.ff()
    }

    /// Borrow all stored section curves.
    pub fn section_curves(&self) -> &[SectionCurve] {
        self.interferences.section_curves()
    }

    /// Borrow all stored trimming loops.
    pub fn trimming_loops(&self) -> &[TrimmingLoop] {
        self.interferences.trimming_loops()
    }

    /// Borrow all stored paves.
    pub fn paves(&self) -> &[Pave] {
        &self.paves
    }

    /// Borrow all stored pave blocks.
    pub fn pave_blocks(&self) -> &[PaveBlock] {
        &self.pave_blocks
    }

    fn ensure_edge_endpoints<C>(&mut self, edge_id: EdgeId, edge: &Edge<Point3, C>)
    where
        C: Clone + ParametricCurve<Point = Point3> + BoundedCurve + Invertible,
    {
        let curve = edge.oriented_curve();
        let (start, end) = curve.range_tuple();
        let tolerance = self.options.parametric_tol;
        let start_vertex = self.next_generated_vertex_id();
        let end_vertex = self.next_generated_vertex_id();

        self.insert_or_merge_pave(Pave::new(edge_id, start_vertex, start, tolerance)
            .expect("parametric tolerance is validated when BopOptions is created"));
        self.insert_or_merge_pave(Pave::new(edge_id, end_vertex, end, tolerance)
            .expect("parametric tolerance is validated when BopOptions is created"));
    }

    fn insert_or_merge_pave(&mut self, pave: Pave) {
        if let Some(existing) = self.paves.iter_mut().find(|existing| {
            existing.edge == pave.edge
                && (existing.parameter - pave.parameter).abs() <= existing.tolerance.max(pave.tolerance)
        }) {
            if pave.vertex.0 < existing.vertex.0 {
                existing.vertex = pave.vertex;
            }
            existing.tolerance = existing.tolerance.max(pave.tolerance);
            return;
        }

        self.paves.push(pave);
    }

    fn normalize_edge_paves(&mut self, edge_id: EdgeId) {
        let mut edge_paves: Vec<Pave> = self
            .paves
            .iter()
            .copied()
            .filter(|pave| pave.edge == edge_id)
            .collect();
        if edge_paves.is_empty() {
            return;
        }

        edge_paves.sort_by(|lhs, rhs| lhs.parameter.total_cmp(&rhs.parameter));

        let mut normalized: Vec<Pave> = Vec::with_capacity(edge_paves.len());
        for pave in edge_paves {
            if let Some(previous) = normalized.last_mut() {
                let tolerance = previous.tolerance.max(pave.tolerance);
                if (pave.parameter - previous.parameter).abs() <= tolerance {
                    if pave.vertex.0 < previous.vertex.0 {
                        previous.vertex = pave.vertex;
                    }
                    previous.tolerance = tolerance;
                    continue;
                }
            }
            normalized.push(pave);
        }

        self.paves.retain(|pave| pave.edge != edge_id);
        self.paves.extend(normalized);
    }

    fn rebuild_pave_blocks_for_edge(&mut self, edge_id: EdgeId) {
        let edge_paves = self.paves_for_edge(edge_id);
        self.pave_blocks.retain(|block| block.original_edge != edge_id);
        self.pave_blocks.extend(
            edge_paves
                .windows(2)
                .map(|pair| PaveBlock::from_pave_pair(pair[0], pair[1], self.options.parametric_tol)),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers_source_entities_with_rank() {
        let mut ds = BopDs::new();
        let vertex = ds.register_vertex_source(0);
        let edge = ds.register_edge_source(1);
        let face = ds.register_face_source(1);

        assert!(ds.vertex_shape_id(vertex).is_some());
        assert!(ds.edge_shape_id(edge).is_some());
        assert!(ds.face_shape_id(face).is_some());
    }

    #[test]
    fn returns_shape_info_for_registered_face() {
        let mut ds = BopDs::new();
        let face = ds.register_face_source(1);
        let shape_id = ds.face_shape_id(face).unwrap();
        let info = ds.shape_info(shape_id).unwrap();
        assert!(info.is_source);
    }

    #[test]
    fn stores_vv_interference_records() {
        let mut ds = BopDs::new();
        let interference = VVInterference { vertex1: VertexId(0), vertex2: VertexId(1) };

        ds.push_vv_interference(interference);

        assert_eq!(ds.vv_interferences(), &[interference]);
    }

    #[test]
    fn stores_ve_interference_records_and_paves() {
        let mut ds = BopDs::new();
        let interference = VEInterference {
            vertex: VertexId(0),
            edge: EdgeId(1),
            parameter: 0.25,
        };
        let pave = Pave::new(EdgeId(1), VertexId(0), 0.25, 1.0e-6).unwrap();

        ds.push_ve_interference(interference);
        ds.push_pave(pave);

        assert_eq!(ds.ve_interferences(), &[interference]);
        assert_eq!(ds.paves(), &[pave]);
    }

    #[test]
    fn pave_sorting_sorts_unsorted_edge_paves() {
        let mut ds = BopDs::new();

        ds.push_pave(Pave::new(EdgeId(1), VertexId(2), 0.75, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(1), VertexId(1), 0.25, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(1), VertexId(3), 0.5, 1.0e-8).unwrap());

        let parameters: Vec<f64> = ds.paves_for_edge(EdgeId(1)).into_iter().map(|pave| pave.parameter).collect();
        assert_eq!(parameters, vec![0.25, 0.5, 0.75]);
    }

    #[test]
    fn pave_deduplication_merges_close_parameters() {
        let mut ds = BopDs::with_options(BopOptions { parametric_tol: 1.0e-5, ..BopOptions::default() });

        ds.push_pave(Pave::new(EdgeId(1), VertexId(4), 0.5, 1.0e-5).unwrap());
        ds.push_pave(Pave::new(EdgeId(1), VertexId(2), 0.500009, 1.0e-5).unwrap());

        let paves = ds.paves_for_edge(EdgeId(1));
        assert_eq!(paves.len(), 1);
        assert_eq!(paves[0].vertex, VertexId(2));
        assert!((paves[0].parameter - 0.5).abs() < 1.0e-12);
    }

    #[test]
    fn pave_sorting_adds_missing_edge_endpoints() {
        let mut ds = BopDs::with_options(BopOptions { parametric_tol: 1.0e-8, ..BopOptions::default() });
        let edge = line_edge(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );

        ds.push_pave(Pave::new(EdgeId(1), VertexId(10), 0.25, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(1), edge)]);

        let parameters: Vec<f64> = ds.paves_for_edge(EdgeId(1)).into_iter().map(|pave| pave.parameter).collect();
        assert_eq!(parameters, vec![0.0, 0.25, 1.0]);
    }

    #[test]
    fn pave_sorting_allocates_generated_vertex_ids_for_missing_endpoints() {
        let mut ds = BopDs::with_options(BopOptions { parametric_tol: 1.0e-8, ..BopOptions::default() });
        let edge = line_edge(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );

        ds.push_pave(Pave::new(EdgeId(2), VertexId(10), 0.25, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(2), edge)]);

        let paves = ds.paves_for_edge(EdgeId(2));
        let endpoint_vertices: Vec<VertexId> = paves
            .into_iter()
            .filter(|pave| pave.parameter == 0.0 || pave.parameter == 1.0)
            .map(|pave| pave.vertex)
            .collect();

        assert_eq!(endpoint_vertices.len(), 2);
        assert!(endpoint_vertices.iter().all(|vertex| vertex.0 >= 1_000_000));
        assert_ne!(endpoint_vertices[0], endpoint_vertices[1]);
    }

    #[test]
    fn pave_sorting_reuses_existing_generated_endpoint_ids_on_rebuild() {
        let mut ds = BopDs::with_options(BopOptions { parametric_tol: 1.0e-8, ..BopOptions::default() });
        let edge = line_edge(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );

        ds.rebuild_paves_for_edges(&[(EdgeId(4), edge.clone())]);
        let first_vertices: Vec<VertexId> = ds
            .paves_for_edge(EdgeId(4))
            .into_iter()
            .map(|pave| pave.vertex)
            .collect();

        ds.rebuild_paves_for_edges(&[(EdgeId(4), edge)]);
        let second_vertices: Vec<VertexId> = ds
            .paves_for_edge(EdgeId(4))
            .into_iter()
            .map(|pave| pave.vertex)
            .collect();

        assert_eq!(first_vertices, second_vertices);
        assert!(second_vertices.iter().all(|vertex| vertex.0 >= 1_000_000));
    }

    #[test]
    fn pave_sorting_rebuild_is_idempotent() {
        let mut ds = BopDs::with_options(BopOptions { parametric_tol: 1.0e-8, ..BopOptions::default() });
        let edge = line_edge(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );

        ds.push_pave(Pave::new(EdgeId(1), VertexId(10), 0.5, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(1), edge.clone())]);
        let first = ds.paves_for_edge(EdgeId(1));

        ds.rebuild_paves_for_edges(&[(EdgeId(1), edge)]);
        let second = ds.paves_for_edge(EdgeId(1));

        assert_eq!(first, second);
    }

    #[test]
    fn paveblock_creation_creates_one_block_per_consecutive_pair() {
        let mut ds = BopDs::with_options(BopOptions { parametric_tol: 1.0e-8, ..BopOptions::default() });
        let edge = line_edge(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );

        ds.push_pave(Pave::new(EdgeId(1), VertexId(10), 0.75, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(1), VertexId(11), 0.25, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(1), edge)]);

        let blocks = ds.pave_blocks_for_edge(EdgeId(1));
        assert_eq!(blocks.len(), 3);
        assert_eq!(blocks[0].param_range, (0.0, 0.25));
        assert_eq!(blocks[1].param_range, (0.25, 0.75));
        assert_eq!(blocks[2].param_range, (0.75, 1.0));
    }

    #[test]
    fn paveblock_creation_uses_sorted_ranges_and_vertices() {
        let mut ds = BopDs::with_options(BopOptions { parametric_tol: 1.0e-8, ..BopOptions::default() });
        let edge = line_edge(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );

        ds.push_pave(Pave::new(EdgeId(3), VertexId(41), 0.6, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(3), VertexId(40), 0.2, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(3), edge)]);

        let blocks = ds.pave_blocks_for_edge(EdgeId(3));
        assert_eq!(blocks[1].original_edge, EdgeId(3));
        assert_eq!(blocks[1].start_vertex, VertexId(40));
        assert_eq!(blocks[1].end_vertex, VertexId(41));
        assert_eq!(blocks[1].param_range, (0.2, 0.6));
    }

    #[test]
    fn paveblock_creation_reorders_reversed_pave_pair_vertices() {
        let mut ds = BopDs::with_options(BopOptions { parametric_tol: 1.0e-8, ..BopOptions::default() });

        ds.push_pave(Pave::new(EdgeId(4), VertexId(41), 0.8, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(4), VertexId(40), 0.2, 1.0e-8).unwrap());

        ds.pave_blocks.clear();
        ds.pave_blocks.push(PaveBlock::from_pave_pair(ds.paves[0], ds.paves[1], ds.options.parametric_tol));

        let block = ds.pave_blocks_for_edge(EdgeId(4))[0];
        assert_eq!(block.start_vertex, VertexId(40));
        assert_eq!(block.end_vertex, VertexId(41));
        assert_eq!(block.param_range, (0.2, 0.8));
    }

    #[test]
    fn paveblock_micro_segment_marks_unsplittable() {
        let mut ds = BopDs::with_options(BopOptions { parametric_tol: 1.0e-4, ..BopOptions::default() });
        let edge = line_edge(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );

        ds.push_pave(Pave::new(EdgeId(5), VertexId(50), 0.25, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(5), VertexId(51), 0.25001, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(5), edge)]);

        let blocks = ds.pave_blocks_for_edge(EdgeId(5));
        let micro = blocks.iter().find(|block| block.param_range == (0.25, 0.25001)).unwrap();
        assert!(micro.unsplittable);
    }

    #[test]
    fn paveblock_creation_edge_with_n_paves_produces_n_minus_one_blocks() {
        let mut ds = BopDs::with_options(BopOptions { parametric_tol: 1.0e-8, ..BopOptions::default() });
        let edge = line_edge(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );

        ds.push_pave(Pave::new(EdgeId(8), VertexId(80), 0.2, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(8), VertexId(81), 0.4, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(8), VertexId(82), 0.9, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(8), edge)]);

        let paves = ds.paves_for_edge(EdgeId(8));
        let blocks = ds.pave_blocks_for_edge(EdgeId(8));
        assert_eq!(blocks.len(), paves.len() - 1);
    }

    #[test]
    fn stores_vf_interference_records() {
        let mut ds = BopDs::new();
        let interference = VFInterference {
            vertex: VertexId(0),
            face: FaceId(1),
            parameters: (0.5, 0.25),
        };

        ds.push_vf_interference(interference);

        assert_eq!(ds.vf_interferences(), &[interference]);
    }

    #[test]
    fn stores_ee_interference_records() {
        let mut ds = BopDs::new();
        let interference = EEInterference {
            edge1: EdgeId(0),
            edge2: EdgeId(1),
            t_a: 0.25,
            t_b: 0.75,
        };

        ds.push_ee_interference(interference);

        assert_eq!(ds.ee_interferences(), &[interference]);
    }

    #[test]
    fn stores_ef_interference_records() {
        let mut ds = BopDs::new();
        let interference = EFInterference {
            edge: EdgeId(0),
            face: FaceId(1),
            parameter: 0.5,
            surface_parameters: (0.5, 0.25),
        };

        ds.push_ef_interference(interference);

        assert_eq!(ds.ef_interferences(), &[interference]);
    }

    #[test]
    fn stores_ff_interference_records_and_section_curves() {
        let mut ds = BopDs::new();
        let curve_id = ds.next_section_curve_id();
        let section_curve = SectionCurve {
            id: curve_id,
            faces: (FaceId(0), FaceId(1)),
            start: VertexId(100),
            end: VertexId(101),
            samples: vec![Point3::new(0.0, 0.0, 0.0)],
            face_parameters: [
                (FaceId(0), vec![truck_base::cgmath64::Point2::new(0.0, 0.0)]),
                (FaceId(1), vec![truck_base::cgmath64::Point2::new(0.0, 0.0)]),
            ],
        };
        let interference = FFInterference {
            face1: FaceId(0),
            face2: FaceId(1),
            section_curve: curve_id,
        };

        ds.push_section_curve(section_curve.clone());
        ds.push_ff_interference(interference);

        assert_eq!(ds.section_curves(), &[section_curve]);
        assert_eq!(ds.ff_interferences(), &[interference]);
    }

    #[test]
    fn stores_trimming_loop_records() {
        let mut ds = BopDs::new();
        let trimming_loop = TrimmingLoop {
            face: FaceId(3),
            edges: vec![TrimmingEdge {
                section_curve: Some(SectionCurveId(0)),
                uv_points: vec![
                    truck_base::cgmath64::Point2::new(0.0, 0.0),
                    truck_base::cgmath64::Point2::new(1.0, 0.0),
                ],
            }],
            uv_points: vec![
                truck_base::cgmath64::Point2::new(0.0, 0.0),
                truck_base::cgmath64::Point2::new(1.0, 0.0),
                truck_base::cgmath64::Point2::new(0.0, 0.0),
            ],
            signed_area: 1.0,
            is_outer: true,
        };

        ds.push_trimming_loop(trimming_loop.clone());

        assert_eq!(ds.trimming_loops(), &[trimming_loop]);
    }

    fn line_edge(start: Point3, end: Point3) -> Edge<Point3, truck_modeling::Curve> {
        let vertices = truck_modeling::builder::vertices([start, end]);
        truck_modeling::builder::line(&vertices[0], &vertices[1])
    }
}
