//! Boolean operation data structure

mod arena;
mod common_block;
mod face_info;
mod ids;
mod interference;
mod pave;
mod pave_block;
mod shape_info;

pub use ids::{
    CommonBlockId, EdgeId, FaceId, PaveBlockId, SectionCurveId, ShapeId, VertexId,
};

use rustc_hash::FxHashMap;
use crate::{BopOptions, BopError};
use shape_info::{ShapeInfo, ShapeKind};

/// Data structure
#[derive(Debug)]
pub struct BopDs {
    options: BopOptions,
    shapes: Vec<ShapeInfo>,
    vertex_to_shape: FxHashMap<VertexId, ShapeId>,
    edge_to_shape: FxHashMap<EdgeId, ShapeId>,
    face_to_shape: FxHashMap<FaceId, ShapeId>,
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
        }
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
}
