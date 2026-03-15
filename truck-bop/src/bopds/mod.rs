//! Boolean operation data structure

mod arena;
mod common_block;
mod face_info;
mod ids;
mod interference;
mod pave;
mod pave_block;
pub(crate) mod shape_info;

pub use interference::{EEInterference, EFInterference, InterferenceTable, VEInterference, VFInterference, VVInterference};
pub use ids::{
    CommonBlockId, EdgeId, FaceId, PaveBlockId, SectionCurveId, ShapeId, VertexId,
};
pub use pave::Pave;

use rustc_hash::FxHashMap;
use crate::BopOptions;
use shape_info::{ShapeInfo, ShapeKind};

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

    /// Store a pave.
    pub fn push_pave(&mut self, pave: Pave) {
        self.paves.push(pave);
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

    /// Borrow all stored paves.
    pub fn paves(&self) -> &[Pave] {
        &self.paves
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
}
