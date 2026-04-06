//! Boolean operation data structure

mod arena;
mod common_block;
mod face_info;
mod ids;
mod interference;
mod pave;
mod pave_block;
mod split_edge;
pub(crate) mod shape_info;

pub use common_block::CommonBlock;
pub use face_info::FaceInfo;
pub use ids::{
    CommonBlockId, EdgeId, FaceId, PaveBlockId, SectionCurveId, ShapeId, SplitEdgeId, VertexId,
};
pub use interference::{
    EEInterference, EEInterferenceKind, EFInterference, FFInterference, InterferenceTable,
    MergedVertex, SectionCurve, SewnEdge, SewnEdgePair, SewnEdgeSource, SewnPath, SplitFace,
    TrimmingEdge, TrimmingEdgeProvenance, TrimmingLoop, TrimmingTopologyKey, VEInterference,
    VFInterference, VVInterference,
};
pub use pave::Pave;
pub use pave_block::PaveBlock;
pub use split_edge::SplitEdgeRecord;

use crate::BopOptions;
use crate::PointClassification;
use rustc_hash::FxHashMap;
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
    face_infos: FxHashMap<FaceId, FaceInfo>,
    interferences: InterferenceTable,
    common_blocks: Vec<CommonBlock>,
    common_block_generation: u64,
    common_block_snapshot_version: u64,
    pave_block_to_common_block: FxHashMap<PaveBlockId, CommonBlockId>,
    interference_pairs: FxHashMap<(ShapeId, ShapeId), usize>,
    paves: Vec<Pave>,
    pave_blocks: Vec<PaveBlock>,
    split_edges: Vec<SplitEdgeRecord>,
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
            face_infos: FxHashMap::default(),
            interferences: InterferenceTable::default(),
            common_blocks: Vec::new(),
            common_block_generation: 0,
            common_block_snapshot_version: 0,
            pave_block_to_common_block: FxHashMap::default(),
            interference_pairs: FxHashMap::default(),
            paves: Vec::new(),
            pave_blocks: Vec::new(),
            split_edges: Vec::new(),
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
    pub fn options(&self) -> &BopOptions { &self.options }

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
        self.vertex_shape_id(vertex_id)
            .and_then(|shape_id| self.shape_info(shape_id))
    }

    /// Get shape info for a registered edge.
    pub fn edge_shape_info(&self, edge_id: EdgeId) -> Option<&ShapeInfo> {
        self.edge_shape_id(edge_id)
            .and_then(|shape_id| self.shape_info(shape_id))
    }

    /// Get shape info for a registered face.
    pub fn face_shape_info(&self, face_id: FaceId) -> Option<&ShapeInfo> {
        self.face_shape_id(face_id)
            .and_then(|shape_id| self.shape_info(shape_id))
    }

    /// Returns the face-state record when it was already created.
    pub fn face_info(&self, face_id: FaceId) -> Option<&FaceInfo> { self.face_infos.get(&face_id) }

    /// Returns the face-state record, creating an empty one on demand.
    pub fn ensure_face_info(&mut self, face_id: FaceId) -> &mut FaceInfo {
        self.face_infos.entry(face_id).or_default()
    }

    /// Updates the face-state record, creating it if absent.
    pub fn update_face_info<F>(&mut self, face_id: FaceId, f: F)
    where F: FnOnce(&mut FaceInfo) {
        let info = self.ensure_face_info(face_id);
        f(info);
    }

    /// Marks face-state as consumed in snapshot mode and returns its version.
    pub fn mark_face_info_snapshot(&mut self, face_id: FaceId) -> Option<u64> {
        let info = self.face_infos.get_mut(&face_id)?;
        info.mark_clean();
        Some(info.version)
    }

    /// Registers a boundary vertex into the face-state pools.
    pub fn push_face_on_vertex(&mut self, face_id: FaceId, vertex_id: VertexId) -> bool {
        self.ensure_face_info(face_id).push_on_vertex(vertex_id)
    }

    /// Registers an interior vertex into the face-state pools.
    pub fn push_face_in_vertex(&mut self, face_id: FaceId, vertex_id: VertexId) -> bool {
        self.ensure_face_info(face_id).push_in_vertex(vertex_id)
    }

    /// Registers a section vertex into the face-state pools.
    pub fn push_face_sc_vertex(&mut self, face_id: FaceId, vertex_id: VertexId) -> bool {
        self.ensure_face_info(face_id).push_sc_vertex(vertex_id)
    }

    /// Registers a boundary pave block into the face-state pools.
    pub fn push_face_on_pave_block(&mut self, face_id: FaceId, pave_block_id: PaveBlockId) -> bool {
        self.ensure_face_info(face_id)
            .push_on_pave_block(pave_block_id)
    }

    /// Registers an interior pave block into the face-state pools.
    pub fn push_face_in_pave_block(&mut self, face_id: FaceId, pave_block_id: PaveBlockId) -> bool {
        self.ensure_face_info(face_id)
            .push_in_pave_block(pave_block_id)
    }

    /// Registers a section pave block into the face-state pools.
    pub fn push_face_sc_pave_block(&mut self, face_id: FaceId, pave_block_id: PaveBlockId) -> bool {
        self.ensure_face_info(face_id)
            .push_sc_pave_block(pave_block_id)
    }

    /// Borrows all common blocks stored in the data structure.
    pub fn common_blocks(&self) -> &[CommonBlock] { &self.common_blocks }

    /// Borrows a common block by its identifier.
    pub fn common_block(&self, common_block_id: CommonBlockId) -> Option<&CommonBlock> {
        self.common_blocks.get(common_block_id.0 as usize)
    }

    /// Iterates all common blocks that contain the given face.
    pub fn common_blocks_for_face(&self, face_id: FaceId) -> impl Iterator<Item = &CommonBlock> {
        self.common_blocks
            .iter()
            .filter(move |common_block| common_block.contains_face(face_id))
    }

    /// Returns all common-block identifiers for the given face that contains one or more
    /// of the provided pave blocks.
    pub fn commonblock_from_paveblocks_for_face(
        &self,
        face_id: FaceId,
        pave_blocks: &[PaveBlockId],
    ) -> Vec<CommonBlockId> {
        let mut common_block_ids = Vec::new();
        for (index, common_block) in self.common_blocks.iter().enumerate() {
            if !common_block.is_effective() {
                continue;
            }
            if !common_block.contains_face(face_id) {
                continue;
            }
            if common_block
                .pave_blocks
                .iter()
                .any(|pave_block| pave_blocks.contains(pave_block))
            {
                common_block_ids.push(CommonBlockId(index as u32));
            }
        }
        common_block_ids
    }

    /// Iterates all effective common blocks associated with the given edge.
    pub fn common_blocks_for_edge(&self, edge_id: EdgeId) -> impl Iterator<Item = &CommonBlock> {
        self.common_blocks.iter().filter(move |common_block| {
            common_block.is_effective() && common_block.representative_edge == Some(edge_id)
        })
    }

    /// Stores a common block and returns its identifier.
    pub fn push_common_block(&mut self, common_block: CommonBlock) -> CommonBlockId {
        let common_block_id = CommonBlockId(self.common_blocks.len() as u32);
        for pave_block_id in common_block.pave_blocks.iter().copied() {
            self.pave_block_to_common_block
                .insert(pave_block_id, common_block_id);
        }
        self.common_block_generation += 1;
        self.common_blocks.push(common_block);
        common_block_id
    }

    /// Updates an existing common block in place and refreshes its pave-block lookup links.
    pub fn update_common_block(
        &mut self,
        common_block_id: CommonBlockId,
        common_block: CommonBlock,
    ) -> bool {
        let Some(slot) = self.common_blocks.get_mut(common_block_id.0 as usize) else {
            return false;
        };
        for pave_block_id in slot.pave_blocks.iter().copied() {
            self.pave_block_to_common_block.remove(&pave_block_id);
        }
        for pave_block_id in common_block.pave_blocks.iter().copied() {
            self.pave_block_to_common_block
                .insert(pave_block_id, common_block_id);
        }
        *slot = common_block;
        self.common_block_generation += 1;
        true
    }

    /// Removes a common block by identifier and returns it, if present.
    pub fn remove_common_block(&mut self, common_block_id: CommonBlockId) -> Option<CommonBlock> {
        if self.common_blocks.len() <= common_block_id.0 as usize {
            return None;
        }

        let removed = self.common_blocks.swap_remove(common_block_id.0 as usize);
        self.rebuild_common_block_index();
        self.common_block_generation += 1;
        Some(removed)
    }

    /// Clears all common blocks and mapping state.
    pub fn clear_common_blocks(&mut self) {
        self.common_blocks.clear();
        self.pave_block_to_common_block.clear();
        self.common_block_generation += 1;
    }

    /// Returns the common-block identifier associated with a pave block.
    pub fn common_block_for_pave_block(&self, pave_block_id: PaveBlockId) -> Option<CommonBlockId> {
        self.pave_block_to_common_block.get(&pave_block_id).copied()
    }

    /// Returns the pave-block identifier for a block matching the given edge and parameter span.
    pub fn find_pave_block_by_range(
        &self,
        edge_id: EdgeId,
        start: f64,
        end: f64,
        tolerance: f64,
    ) -> Option<PaveBlockId> {
        let (start, end) = if start <= end {
            (start, end)
        } else {
            (end, start)
        };
        self.pave_blocks
            .iter()
            .enumerate()
            .find(|(_, block)| {
                block.original_edge == edge_id
                    && (block.param_range.0 - start).abs() <= tolerance
                    && (block.param_range.1 - end).abs() <= tolerance
            })
            .map(|(index, _)| PaveBlockId(index as u32))
    }

    /// Stores a symmetric interference-pair index.
    pub fn set_interference_pair_index(&mut self, lhs: ShapeId, rhs: ShapeId, index: usize) {
        self.interference_pairs
            .insert(canonical_shape_pair(lhs, rhs), index);
    }

    /// Returns the stored interference-pair index, independent of argument order.
    pub fn interference_pair_index(&self, lhs: ShapeId, rhs: ShapeId) -> Option<usize> {
        self.interference_pairs
            .get(&canonical_shape_pair(lhs, rhs))
            .copied()
    }

    /// Returns current common-block generation.
    pub fn common_block_generation(&self) -> u64 { self.common_block_generation }

    /// Snapshot marker for common-block lifecycle management.
    pub fn mark_common_blocks_snapshot(&mut self) -> u64 {
        self.common_block_snapshot_version = self.common_block_generation;
        self.common_block_snapshot_version
    }

    /// Returns the last reported snapshot marker.
    pub fn common_blocks_snapshot_version(&self) -> u64 { self.common_block_snapshot_version }

    /// Removes stale index entries by rebuild on demand.
    fn rebuild_common_block_index(&mut self) {
        self.pave_block_to_common_block.clear();
        for (id, block) in self.common_blocks.iter().enumerate() {
            let block_id = CommonBlockId(id as u32);
            for &pave_block_id in &block.pave_blocks {
                self.pave_block_to_common_block
                    .insert(pave_block_id, block_id);
            }
        }
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

    /// Store a split face fragment.
    pub fn push_split_face(&mut self, split_face: SplitFace) {
        self.interferences.push_split_face(split_face);
    }

    /// Store a merged vertex cluster.
    pub fn push_merged_vertex(&mut self, merged_vertex: MergedVertex) {
        self.interferences.push_merged_vertex(merged_vertex);
    }

    /// Store a sewn boundary path.
    pub fn push_sewn_path(&mut self, sewn_path: SewnPath) {
        self.interferences.push_sewn_path(sewn_path);
    }

    /// Update a split face classification in place.
    pub fn set_split_face_classification(
        &mut self,
        index: usize,
        representative_point: Point3,
        classification: PointClassification,
    ) {
        if let Some(split_face) = self.interferences.split_faces.get_mut(index) {
            split_face.representative_point = Some(representative_point);
            split_face.classification = Some(classification);
        }
    }

    /// Store a pave.
    pub fn push_pave(&mut self, pave: Pave) {
        self.paves.push(pave);
        self.normalize_edge_paves(pave.edge);
    }

    /// Appends an extra pave to the block that contains the parameter.
    pub fn append_ext_pave(&mut self, edge_id: EdgeId, pave: Pave) -> bool {
        let tolerance = self.options.parametric_tol.max(pave.tolerance);
        let Some(index) = self.pave_blocks.iter().position(|block| {
            block.original_edge == edge_id
                && pave.parameter + tolerance >= block.param_range.0
                && pave.parameter - tolerance <= block.param_range.1
        }) else {
            return false;
        };
        self.pave_blocks[index].append_ext_pave(pave, self.options.parametric_tol)
    }

    /// Rebuilds the base block partition from the normalized edge paves only.
    pub fn rebuild_pave_blocks_from_paves(&mut self, edge_id: EdgeId) {
        let edge_paves = self.paves_for_edge(edge_id);
        self.pave_blocks.retain(|block| {
            block.original_edge != edge_id || !block.ext_paves.is_empty() || block.split_result.is_some()
        });
        self.pave_blocks.extend(
            edge_paves.windows(2).map(|pair| {
                PaveBlock::from_pave_pair(pair[0], pair[1], self.options.parametric_tol)
            }),
        );
    }

    /// Splits every block on an edge using its endpoint paves plus block-local extra paves.
    pub fn split_pave_blocks_for_edge(&mut self, edge_id: EdgeId) {
        let original_blocks: Vec<PaveBlock> = self
            .pave_blocks
            .iter()
            .filter(|block| block.original_edge == edge_id)
            .cloned()
            .collect();
        self.pave_blocks
            .retain(|block| block.original_edge != edge_id);

        for mut block in original_blocks {
            let ordered = block.ordered_paves(self.options.parametric_tol);
            block.ext_paves.clear();
            let split_result = ordered
                .windows(2)
                .map(|pair| (pair[0].vertex, pair[1].vertex))
                .collect::<Vec<_>>();
            block.split_result = Some(split_result.clone());

            if split_result.len() <= 1 {
                self.pave_blocks.push(block);
                continue;
            }

            self.pave_blocks.extend(ordered.windows(2).map(|pair| {
                PaveBlock::from_pave_pair(pair[0], pair[1], self.options.parametric_tol)
            }));
        }
    }

    /// Rebuild sorted, deduplicated pave lists for all known edges while ensuring endpoints exist.
    pub fn rebuild_paves_for_edges<C>(&mut self, edges: &[(EdgeId, Edge<Point3, C>)])
    where C: Clone + ParametricCurve<Point = Point3> + BoundedCurve + Invertible {
        for &(edge_id, ref edge) in edges {
            self.ensure_edge_endpoints(edge_id, edge);
            self.normalize_edge_paves(edge_id);
            self.rebuild_pave_blocks_from_paves(edge_id);
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
            .cloned()
            .filter(|block| block.original_edge == edge_id)
            .collect()
    }

    /// Borrow all stored vertex-vertex interferences.
    pub fn vv_interferences(&self) -> &[VVInterference] { self.interferences.vv() }

    /// Borrow all stored vertex-edge interferences.
    pub fn ve_interferences(&self) -> &[VEInterference] { self.interferences.ve() }

    /// Borrow all stored vertex-face interferences.
    pub fn vf_interferences(&self) -> &[VFInterference] { self.interferences.vf() }

    /// Borrow all stored edge-edge interferences.
    pub fn ee_interferences(&self) -> &[EEInterference] { self.interferences.ee() }

    /// Borrow all stored edge-face interferences.
    pub fn ef_interferences(&self) -> &[EFInterference] { self.interferences.ef() }

    /// Borrow all stored face-face interferences.
    pub fn ff_interferences(&self) -> &[FFInterference] { self.interferences.ff() }

    /// Borrow all stored section curves.
    pub fn section_curves(&self) -> &[SectionCurve] { self.interferences.section_curves() }

    /// Borrow all stored trimming loops.
    pub fn trimming_loops(&self) -> &[TrimmingLoop] { self.interferences.trimming_loops() }

    /// Borrow all stored split face fragments.
    pub fn split_faces(&self) -> &[SplitFace] { self.interferences.split_faces() }

    /// Borrow all merged vertex clusters.
    pub fn merged_vertices(&self) -> &[MergedVertex] { self.interferences.merged_vertices() }

    /// Borrow all sewn boundary paths.
    pub fn sewn_paths(&self) -> &[SewnPath] { self.interferences.sewn_paths() }

    /// Borrow all stored paves.
    pub fn paves(&self) -> &[Pave] { &self.paves }

    /// Borrow all stored pave blocks.
    pub fn pave_blocks(&self) -> &[PaveBlock] { &self.pave_blocks }

    /// Stores a materialized split-edge fact and returns its identifier.
    pub fn push_split_edge(&mut self, mut split_edge: SplitEdgeRecord) -> SplitEdgeId {
        let split_edge_id = SplitEdgeId(self.split_edges.len() as u32);
        split_edge.id = split_edge_id;
        self.split_edges.push(split_edge);
        split_edge_id
    }

    /// Borrows all materialized split-edge facts.
    pub fn split_edges(&self) -> &[SplitEdgeRecord] { &self.split_edges }

    /// Borrows a materialized split-edge fact by identifier.
    pub fn split_edge(&self, split_edge_id: SplitEdgeId) -> Option<&SplitEdgeRecord> {
        self.split_edges.get(split_edge_id.0 as usize)
    }

    /// Borrows all materialized split-edge facts for a source edge.
    pub fn split_edges_for_edge(
        &self,
        edge_id: EdgeId,
    ) -> impl Iterator<Item = &SplitEdgeRecord> + '_ {
        self.split_edges
            .iter()
            .filter(move |split_edge| split_edge.original_edge == edge_id)
    }

    /// Borrows all materialized split-edge facts linked to a common block.
    pub fn split_edges_for_common_block(
        &self,
        common_block_id: CommonBlockId,
    ) -> impl Iterator<Item = &SplitEdgeRecord> + '_ {
        self.split_edges
            .iter()
            .filter(move |split_edge| split_edge.common_block == Some(common_block_id))
    }

    /// Clears all materialized split-edge facts.
    pub fn clear_split_edges(&mut self) { self.split_edges.clear(); }

    /// Clear previously computed merged vertex clusters.
    pub fn clear_merged_vertices(&mut self) { self.interferences.merged_vertices.clear(); }

    /// Clear previously computed sewn boundary paths.
    pub fn clear_sewn_paths(&mut self) { self.interferences.sewn_paths.clear(); }

    fn ensure_edge_endpoints<C>(&mut self, edge_id: EdgeId, edge: &Edge<Point3, C>)
    where C: Clone + ParametricCurve<Point = Point3> + BoundedCurve + Invertible {
        let curve = edge.oriented_curve();
        let (start, end) = curve.range_tuple();
        let tolerance = self.options.parametric_tol;
        let start_vertex = self.next_generated_vertex_id();
        let end_vertex = self.next_generated_vertex_id();

        self.insert_or_merge_pave(
            Pave::new(edge_id, start_vertex, start, tolerance)
                .expect("parametric tolerance is validated when BopOptions is created"),
        );
        self.insert_or_merge_pave(
            Pave::new(edge_id, end_vertex, end, tolerance)
                .expect("parametric tolerance is validated when BopOptions is created"),
        );
    }

    fn insert_or_merge_pave(&mut self, pave: Pave) {
        if let Some(existing) = self.paves.iter_mut().find(|existing| {
            existing.edge == pave.edge
                && (existing.parameter - pave.parameter).abs()
                    <= existing.tolerance.max(pave.tolerance)
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
}

fn canonical_shape_pair(lhs: ShapeId, rhs: ShapeId) -> (ShapeId, ShapeId) {
    if lhs <= rhs {
        (lhs, rhs)
    } else {
        (rhs, lhs)
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
        let interference = VVInterference {
            vertex1: VertexId(0),
            vertex2: VertexId(1),
        };

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

        let parameters: Vec<f64> = ds
            .paves_for_edge(EdgeId(1))
            .into_iter()
            .map(|pave| pave.parameter)
            .collect();
        assert_eq!(parameters, vec![0.25, 0.5, 0.75]);
    }

    #[test]
    fn pave_deduplication_merges_close_parameters() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-5,
            ..BopOptions::default()
        });

        ds.push_pave(Pave::new(EdgeId(1), VertexId(4), 0.5, 1.0e-5).unwrap());
        ds.push_pave(Pave::new(EdgeId(1), VertexId(2), 0.500009, 1.0e-5).unwrap());

        let paves = ds.paves_for_edge(EdgeId(1));
        assert_eq!(paves.len(), 1);
        assert_eq!(paves[0].vertex, VertexId(2));
        assert!((paves[0].parameter - 0.5).abs() < 1.0e-12);
    }

    #[test]
    fn pave_sorting_adds_missing_edge_endpoints() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-8,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        ds.push_pave(Pave::new(EdgeId(1), VertexId(10), 0.25, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(1), edge)]);

        let parameters: Vec<f64> = ds
            .paves_for_edge(EdgeId(1))
            .into_iter()
            .map(|pave| pave.parameter)
            .collect();
        assert_eq!(parameters, vec![0.0, 0.25, 1.0]);
    }

    #[test]
    fn pave_sorting_allocates_generated_vertex_ids_for_missing_endpoints() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-8,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

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
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-8,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

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
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-8,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        ds.push_pave(Pave::new(EdgeId(1), VertexId(10), 0.5, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(1), edge.clone())]);
        let first = ds.paves_for_edge(EdgeId(1));

        ds.rebuild_paves_for_edges(&[(EdgeId(1), edge)]);
        let second = ds.paves_for_edge(EdgeId(1));

        assert_eq!(first, second);
    }

    #[test]
    fn paveblock_creation_creates_one_block_per_consecutive_pair() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-8,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

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
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-8,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

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
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-8,
            ..BopOptions::default()
        });

        ds.push_pave(Pave::new(EdgeId(4), VertexId(41), 0.8, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(4), VertexId(40), 0.2, 1.0e-8).unwrap());

        ds.pave_blocks.clear();
        ds.pave_blocks.push(PaveBlock::from_pave_pair(
            ds.paves[0],
            ds.paves[1],
            ds.options.parametric_tol,
        ));

        let block = ds.pave_blocks_for_edge(EdgeId(4))[0].clone();
        assert_eq!(block.start_vertex, VertexId(40));
        assert_eq!(block.end_vertex, VertexId(41));
        assert_eq!(block.param_range, (0.2, 0.8));
    }

    #[test]
    fn paveblock_micro_segment_marks_unsplittable() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-4,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        ds.push_pave(Pave::new(EdgeId(5), VertexId(50), 0.25, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(5), VertexId(51), 0.25001, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(5), edge)]);

        let blocks = ds.pave_blocks_for_edge(EdgeId(5));
        let micro = blocks
            .iter()
            .find(|block| block.param_range == (0.25, 0.25001))
            .unwrap();
        assert!(micro.unsplittable);
    }

    #[test]
    fn paveblock_creation_edge_with_n_paves_produces_n_minus_one_blocks() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-8,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        ds.push_pave(Pave::new(EdgeId(8), VertexId(80), 0.2, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(8), VertexId(81), 0.4, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(8), VertexId(82), 0.9, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(8), edge)]);

        let paves = ds.paves_for_edge(EdgeId(8));
        let blocks = ds.pave_blocks_for_edge(EdgeId(8));
        assert_eq!(blocks.len(), paves.len() - 1);
    }

    #[test]
    fn paveblock_split_merges_extra_paves_in_deterministic_order() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        ds.rebuild_paves_for_edges(&[(EdgeId(9), edge)]);
        assert!(ds.append_ext_pave(
            EdgeId(9),
            Pave::new(EdgeId(9), VertexId(91), 0.75, 1.0e-6).unwrap()
        ));
        assert!(ds.append_ext_pave(
            EdgeId(9),
            Pave::new(EdgeId(9), VertexId(90), 0.25, 1.0e-6).unwrap()
        ));
        assert!(!ds.append_ext_pave(
            EdgeId(9),
            Pave::new(EdgeId(9), VertexId(89), 0.2500005, 1.0e-6).unwrap()
        ));

        ds.split_pave_blocks_for_edge(EdgeId(9));

        let blocks = ds.pave_blocks_for_edge(EdgeId(9));
        let ranges: Vec<(f64, f64)> = blocks.iter().map(|block| block.param_range).collect();
        let vertices: Vec<(VertexId, VertexId)> = blocks
            .iter()
            .map(|block| (block.start_vertex, block.end_vertex))
            .collect();

        assert_eq!(ranges, vec![(0.0, 0.25), (0.25, 0.75), (0.75, 1.0)]);
        assert_eq!(
            vertices,
            vec![
                (VertexId(1_000_000), VertexId(90)),
                (VertexId(90), VertexId(91)),
                (VertexId(91), VertexId(1_000_001))
            ]
        );
    }

    #[test]
    fn paveblock_split_records_child_linkage_before_replacement() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        ds.rebuild_paves_for_edges(&[(EdgeId(10), edge)]);
        assert!(ds.append_ext_pave(
            EdgeId(10),
            Pave::new(EdgeId(10), VertexId(101), 0.5, 1.0e-6).unwrap()
        ));

        let before = ds.pave_blocks_for_edge(EdgeId(10));
        assert_eq!(before.len(), 1);
        assert!(before[0].split_result.is_none());

        ds.split_pave_blocks_for_edge(EdgeId(10));

        let after = ds.pave_blocks_for_edge(EdgeId(10));
        assert_eq!(after.len(), 2);
        assert_eq!(after[0].param_range, (0.0, 0.5));
        assert_eq!(after[1].param_range, (0.5, 1.0));
    }

    #[test]
    fn rebuild_pave_blocks_from_paves_discards_extra_split_state() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        ds.rebuild_paves_for_edges(&[(EdgeId(11), edge)]);
        assert!(ds.append_ext_pave(
            EdgeId(11),
            Pave::new(EdgeId(11), VertexId(111), 0.5, 1.0e-6).unwrap()
        ));
        ds.split_pave_blocks_for_edge(EdgeId(11));
        assert_eq!(ds.pave_blocks_for_edge(EdgeId(11)).len(), 2);

        ds.rebuild_pave_blocks_from_paves(EdgeId(11));

        let rebuilt = ds.pave_blocks_for_edge(EdgeId(11));
        assert_eq!(rebuilt.len(), 1);
        assert!(rebuilt[0].ext_paves.is_empty());
        assert!(rebuilt[0].split_result.is_none());
        assert_eq!(rebuilt[0].param_range, (0.0, 1.0));
    }

    #[test]
    fn paveblock_rebuild_is_idempotent() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        ds.push_pave(Pave::new(EdgeId(13), VertexId(130), 0.25, 1.0e-6).unwrap());
        ds.push_pave(Pave::new(EdgeId(13), VertexId(131), 0.75, 1.0e-6).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(13), edge)]);

        let first = ds.pave_blocks_for_edge(EdgeId(13));
        ds.rebuild_pave_blocks_from_paves(EdgeId(13));
        let second = ds.pave_blocks_for_edge(EdgeId(13));

        assert_eq!(first, second);
    }

    #[test]
    fn split_pave_blocks_repeated_run_keeps_same_ranges() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        ds.rebuild_paves_for_edges(&[(EdgeId(14), edge)]);
        assert!(ds.append_ext_pave(
            EdgeId(14),
            Pave::new(EdgeId(14), VertexId(140), 0.25, 1.0e-6).unwrap()
        ));
        assert!(ds.append_ext_pave(
            EdgeId(14),
            Pave::new(EdgeId(14), VertexId(141), 0.75, 1.0e-6).unwrap()
        ));

        ds.split_pave_blocks_for_edge(EdgeId(14));
        let first = ds.pave_blocks_for_edge(EdgeId(14));
        let first_ranges: Vec<_> = first.iter().map(|block| block.param_range).collect();
        let first_vertices: Vec<_> = first
            .iter()
            .map(|block| (block.start_vertex, block.end_vertex))
            .collect();

        ds.split_pave_blocks_for_edge(EdgeId(14));
        let second = ds.pave_blocks_for_edge(EdgeId(14));
        let second_ranges: Vec<_> = second.iter().map(|block| block.param_range).collect();
        let second_vertices: Vec<_> = second
            .iter()
            .map(|block| (block.start_vertex, block.end_vertex))
            .collect();

        assert_eq!(first_ranges, vec![(0.0, 0.25), (0.25, 0.75), (0.75, 1.0)]);
        assert_eq!(first_ranges, second_ranges);
        assert_eq!(first_vertices, second_vertices);
        assert!(second.iter().all(|block| block.ext_paves.is_empty()));
        assert!(second.iter().all(|block| {
            block
                .split_result
                .as_ref()
                .is_some_and(|children| children.len() == 1)
        }));
    }

    #[test]
    fn paveblock_split_preserves_explicit_unsplittable_micro_segments() {
        let mut ds = BopDs::with_options(BopOptions {
            parametric_tol: 1.0e-4,
            ..BopOptions::default()
        });
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        ds.push_pave(Pave::new(EdgeId(12), VertexId(120), 0.25, 1.0e-8).unwrap());
        ds.push_pave(Pave::new(EdgeId(12), VertexId(121), 0.25001, 1.0e-8).unwrap());
        ds.rebuild_paves_for_edges(&[(EdgeId(12), edge)]);
        ds.split_pave_blocks_for_edge(EdgeId(12));

        let blocks = ds.pave_blocks_for_edge(EdgeId(12));
        let micro = blocks
            .iter()
            .find(|block| block.param_range == (0.25, 0.25001))
            .unwrap();
        assert!(micro.unsplittable);
        assert!(micro.ext_paves.is_empty());
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
            kind: EEInterferenceKind::VertexHit,
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
            face_projection_sample: Some(Point3::new(0.0, 0.0, 0.0)),
            face_projection_available: true,
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
            face_projection_available: [(FaceId(0), true), (FaceId(1), true)],
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
    fn generated_vertex_ids_are_monotonic_and_unique() {
        let mut ds = BopDs::new();

        let first = ds.next_generated_vertex_id();
        let second = ds.next_generated_vertex_id();
        let third = ds.next_generated_vertex_id();

        assert!(first < second);
        assert!(second < third);
        assert_ne!(first, second);
        assert_ne!(second, third);
        assert_ne!(first, third);

        ds.push_merged_vertex(MergedVertex {
            id: third,
            original_vertices: vec![third],
            point: Point3::new(0.0, 0.0, 0.0),
        });

        let fourth = ds.next_generated_vertex_id();
        assert!(third < fourth);
    }

    #[test]
    fn section_curve_ids_are_monotonic_and_unique() {
        let mut ds = BopDs::new();

        let first = ds.next_section_curve_id();
        let second = ds.next_section_curve_id();
        let third = ds.next_section_curve_id();

        assert!(first < second);
        assert!(second < third);
        assert_ne!(first, second);
        assert_ne!(second, third);
        assert_ne!(first, third);

        ds.push_section_curve(SectionCurve {
            id: third,
            faces: (FaceId(0), FaceId(1)),
            start: VertexId(10),
            end: VertexId(11),
            samples: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            face_parameters: [
                (
                    FaceId(0),
                    vec![
                        truck_base::cgmath64::Point2::new(0.0, 0.0),
                        truck_base::cgmath64::Point2::new(1.0, 0.0),
                    ],
                ),
                (
                    FaceId(1),
                    vec![
                        truck_base::cgmath64::Point2::new(0.0, 0.0),
                        truck_base::cgmath64::Point2::new(1.0, 0.0),
                    ],
                ),
            ],
            face_projection_available: [(FaceId(0), true), (FaceId(1), true)],
        });

        let fourth = ds.next_section_curve_id();
        assert!(third < fourth);
    }

    #[test]
    fn stores_trimming_loop_records() {
        let mut ds = BopDs::new();
        let trimming_loop = TrimmingLoop {
            face: FaceId(3),
            vertex_ids: vec![VertexId(1), VertexId(2)],
            edges: vec![TrimmingEdge {
                provenance: TrimmingEdgeProvenance::SectionCurve {
                    section_curve: SectionCurveId(0),
                },
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

    #[test]
    fn stores_split_face_records() {
        let mut ds = BopDs::new();
        let split_face = SplitFace {
            original_face: FaceId(3),
            operand_rank: 0,
            trimming_loops: vec![TrimmingLoop {
                face: FaceId(3),
                vertex_ids: vec![VertexId(1), VertexId(2)],
                edges: vec![TrimmingEdge {
                    provenance: TrimmingEdgeProvenance::SectionCurve {
                        section_curve: SectionCurveId(0),
                    },
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
            }],
            splitting_edges: vec![SectionCurveId(0)],
            representative_point: Some(Point3::new(0.5, 0.5, 0.0)),
            classification: Some(PointClassification::Inside),
        };

        ds.push_split_face(split_face.clone());

        assert_eq!(ds.split_faces(), &[split_face]);
    }

    #[test]
    fn stores_merged_vertex_records() {
        let mut ds = BopDs::new();
        let merged = MergedVertex {
            id: VertexId(12),
            original_vertices: vec![VertexId(12), VertexId(14)],
            point: Point3::new(0.5, 0.5, 0.0),
        };

        ds.push_merged_vertex(merged.clone());

        assert_eq!(ds.merged_vertices(), &[merged]);
    }

    #[test]
    fn clear_merged_vertices_removes_existing_clusters() {
        let mut ds = BopDs::new();
        ds.push_merged_vertex(MergedVertex {
            id: VertexId(1),
            original_vertices: vec![VertexId(1)],
            point: Point3::new(0.0, 0.0, 0.0),
        });

        ds.clear_merged_vertices();

        assert!(ds.merged_vertices().is_empty());
    }

    #[test]
    fn clear_sewn_paths_removes_existing_paths() {
        let mut ds = BopDs::new();
        ds.push_sewn_path(SewnPath {
            edges: vec![SewnEdge {
                source: SewnEdgeSource {
                    face: FaceId(0),
                    loop_index: 0,
                    edge_index: 0,
                    topology_key: TrimmingTopologyKey::SourceBoundary(EdgeId(11)),
                },
                face: FaceId(0),
                loop_index: 0,
                edge_index: 0,
                start_vertex: VertexId(1),
                end_vertex: VertexId(2),
                reversed: false,
                topology_key: TrimmingTopologyKey::SourceBoundary(EdgeId(11)),
                sewn_pair: Some(SewnEdgePair::new(
                    SewnEdgeSource {
                        face: FaceId(0),
                        loop_index: 0,
                        edge_index: 0,
                        topology_key: TrimmingTopologyKey::SourceBoundary(EdgeId(11)),
                    },
                    SewnEdgeSource {
                        face: FaceId(1),
                        loop_index: 0,
                        edge_index: 0,
                        topology_key: TrimmingTopologyKey::SourceBoundary(EdgeId(12)),
                    },
                )),
            }],
            is_closed: true,
        });

        ds.clear_sewn_paths();

        assert!(ds.sewn_paths().is_empty());
    }

    #[test]
    fn updates_split_face_classification_in_place() {
        let mut ds = BopDs::new();
        ds.push_split_face(SplitFace {
            original_face: FaceId(3),
            operand_rank: 1,
            trimming_loops: vec![],
            splitting_edges: vec![],
            representative_point: None,
            classification: None,
        });

        ds.set_split_face_classification(
            0,
            Point3::new(1.0, 2.0, 3.0),
            PointClassification::OnBoundary,
        );

        assert_eq!(
            ds.split_faces()[0].representative_point,
            Some(Point3::new(1.0, 2.0, 3.0))
        );
        assert_eq!(
            ds.split_faces()[0].classification,
            Some(PointClassification::OnBoundary)
        );
    }

    #[test]
    fn bopds_can_bind_pave_block_to_common_block() {
        use super::common_block::CommonBlock;

        let mut ds = BopDs::new();
        let common_block_id = ds.push_common_block(CommonBlock::new(
            vec![PaveBlockId(3), PaveBlockId(4)],
            vec![FaceId(7), FaceId(8)],
            Some(EdgeId(21)),
        ));

        assert_eq!(
            ds.common_block_for_pave_block(PaveBlockId(3)),
            Some(common_block_id)
        );
        assert_eq!(
            ds.common_block_for_pave_block(PaveBlockId(4)),
            Some(common_block_id)
        );
        assert_eq!(
            ds.common_blocks()[common_block_id.0 as usize].faces,
            vec![FaceId(7), FaceId(8)]
        );
    }

    #[test]
    fn bopds_face_common_block_roundtrip() {
        use super::common_block::CommonBlock;

        let mut ds = BopDs::new();
        let mut first = CommonBlock::new(vec![PaveBlockId(1)], vec![FaceId(2)], Some(EdgeId(3)));
        first.register_witness_edge(EdgeId(7));
        first.register_witness_face(FaceId(8));
        let second = CommonBlock::new(vec![PaveBlockId(2)], vec![FaceId(3)], Some(EdgeId(11)));

        let block_id = ds.push_common_block(first);
        let second_id = ds.push_common_block(second);
        assert_eq!(ds.common_block_generation(), 2);
        assert_eq!(ds.common_blocks_for_face(FaceId(2)).count(), 1);
        assert_eq!(
            ds.common_block_for_pave_block(PaveBlockId(2)),
            Some(second_id)
        );
        assert_eq!(ds.common_blocks_snapshot_version(), 0);
        assert_eq!(ds.mark_common_blocks_snapshot(), 2);
        assert_eq!(ds.common_blocks_snapshot_version(), 2);

        let blocks_for_face: Vec<_> = ds.common_blocks_for_face(FaceId(2)).collect();
        assert_eq!(blocks_for_face.len(), 1);
        assert_eq!(blocks_for_face[0].contains_witness_edge(EdgeId(7)), true);
        assert!(ds.common_block_for_pave_block(PaveBlockId(1)).is_some());

        let removed = ds
            .remove_common_block(block_id)
            .expect("common block should be removed");
        assert_eq!(removed.pave_blocks, vec![PaveBlockId(1)]);
        assert_eq!(ds.common_block_generation(), 3);
        assert_eq!(ds.common_blocks().len(), 1);
        assert_eq!(ds.common_block_for_pave_block(PaveBlockId(1)), None);
        assert_eq!(
            ds.common_block_for_pave_block(PaveBlockId(2)),
            Some(CommonBlockId(0))
        );

        ds.push_common_block(CommonBlock::new(
            vec![PaveBlockId(9)],
            vec![FaceId(3)],
            Some(EdgeId(10)),
        ));
        assert_eq!(ds.common_block_generation(), 4);
        assert_eq!(ds.common_blocks_for_face(FaceId(2)).count(), 0);

        ds.clear_common_blocks();
        assert_eq!(ds.common_block_generation(), 5);
        assert!(ds.common_blocks().is_empty());
        assert_eq!(ds.common_blocks_snapshot_version(), 2);
    }

    #[test]
    fn commonblock_from_paveblocks_for_face() {
        use super::common_block::CommonBlock;

        let mut ds = BopDs::new();

        let first = CommonBlock::new(
            vec![PaveBlockId(1), PaveBlockId(2)],
            vec![FaceId(1), FaceId(2)],
            Some(EdgeId(10)),
        );
        let second = CommonBlock::new(vec![PaveBlockId(3)], vec![FaceId(1)], Some(EdgeId(11)));
        let third = CommonBlock::new(vec![PaveBlockId(4)], vec![FaceId(3)], None);

        let first_id = ds.push_common_block(first);
        let second_id = ds.push_common_block(second);
        let third_id = ds.push_common_block(third);
        assert_eq!(ds.common_blocks_for_face(FaceId(1)).count(), 2);

        assert_eq!(
            ds.commonblock_from_paveblocks_for_face(FaceId(1), &[PaveBlockId(1)]),
            vec![first_id]
        );
        assert_eq!(
            ds.commonblock_from_paveblocks_for_face(FaceId(1), &[PaveBlockId(3)]),
            vec![second_id]
        );
        assert_eq!(
            ds.commonblock_from_paveblocks_for_face(FaceId(1), &[PaveBlockId(2), PaveBlockId(3)]),
            vec![first_id, second_id]
        );
        assert!(ds
            .commonblock_from_paveblocks_for_face(FaceId(2), &[PaveBlockId(3)])
            .is_empty());
        assert!(ds
            .commonblock_from_paveblocks_for_face(FaceId(3), &[PaveBlockId(4)])
            .contains(&third_id));
        assert_eq!(
            ds.common_blocks_for_edge(EdgeId(10)).collect::<Vec<_>>(),
            vec![&ds.common_blocks()[first_id.0 as usize]]
        );
    }

    #[test]
    fn stores_overlap_ee_interference_records() {
        let mut ds = BopDs::new();
        let interference = EEInterference {
            edge1: EdgeId(0),
            edge2: EdgeId(1),
            t_a: 0.0,
            t_b: 0.5,
            kind: EEInterferenceKind::OverlapHit,
        };

        ds.push_ee_interference(interference);

        assert_eq!(ds.ee_interferences(), &[interference]);
    }

    #[test]
    fn bopds_can_store_interference_pair_index() {
        let mut ds = BopDs::new();
        let a = ShapeId(3);
        let b = ShapeId(9);

        ds.set_interference_pair_index(a, b, 41);

        assert_eq!(ds.interference_pair_index(a, b), Some(41));
        assert_eq!(ds.interference_pair_index(b, a), Some(41));
    }

    #[test]
    fn bopds_creates_face_info_lazily() {
        let mut ds = BopDs::new();
        let face = FaceId(12);

        assert!(ds.push_face_sc_pave_block(face, PaveBlockId(99)));
        assert!(ds.push_face_on_vertex(face, VertexId(55)));
        assert!(!ds.push_face_sc_pave_block(face, PaveBlockId(99)));

        let info = ds
            .face_info(face)
            .expect("face info should be created on demand");
        assert_eq!(info.sc_pave_blocks, vec![PaveBlockId(99)]);
        assert_eq!(info.on_vertices, vec![VertexId(55)]);
    }

    #[test]
    fn bopds_mark_face_info_snapshot_returns_version_and_clears_dirty() {
        let mut ds = BopDs::new();
        let face = FaceId(42);

        ds.update_face_info(face, |info| {
            info.push_sc_vertex(VertexId(7));
            assert!(info.touched());
        });
        assert_eq!(ds.mark_face_info_snapshot(face), Some(1));
        assert!(!ds.face_info(face).expect("face info exists").touched());
        assert_eq!(ds.mark_face_info_snapshot(face), Some(1));
    }

    #[test]
    fn bopds_mark_face_info_snapshot_no_record_returns_none() {
        let mut ds = BopDs::new();
        assert_eq!(ds.mark_face_info_snapshot(FaceId(88)), None);
    }

    #[test]
    fn bopds_can_store_split_edge_records() {
        let mut ds = BopDs::new();
        let first_id = ds.push_split_edge(SplitEdgeRecord::new(
            SplitEdgeId(u32::MAX),
            EdgeId(5),
            VertexId(10),
            VertexId(11),
            (0.0, 0.5),
            Some(CommonBlockId(2)),
            false,
        ));
        let second_id = ds.push_split_edge(SplitEdgeRecord::new(
            SplitEdgeId(u32::MAX),
            EdgeId(5),
            VertexId(11),
            VertexId(12),
            (0.5, 1.0),
            None,
            true,
        ));

        assert_eq!(first_id, SplitEdgeId(0));
        assert_eq!(second_id, SplitEdgeId(1));
        assert_eq!(ds.split_edges().len(), 2);
        assert_eq!(ds.split_edge(first_id).unwrap().param_range, (0.0, 0.5));
        assert!(ds.split_edge(second_id).unwrap().unsplittable);

        let edge_ranges: Vec<_> = ds
            .split_edges_for_edge(EdgeId(5))
            .map(|record| record.param_range)
            .collect();
        assert_eq!(edge_ranges, vec![(0.0, 0.5), (0.5, 1.0)]);

        let common_block_links: Vec<_> = ds
            .split_edges_for_common_block(CommonBlockId(2))
            .map(|record| record.id)
            .collect();
        assert_eq!(common_block_links, vec![SplitEdgeId(0)]);

        ds.clear_split_edges();
        assert!(ds.split_edges().is_empty());
        assert!(ds.split_edge(SplitEdgeId(0)).is_none());
    }

    fn line_edge(start: Point3, end: Point3) -> Edge<Point3, truck_modeling::Curve> {
        let vertices = truck_modeling::builder::vertices([start, end]);
        truck_modeling::builder::line(&vertices[0], &vertices[1])
    }
}
