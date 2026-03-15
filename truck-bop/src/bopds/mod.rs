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

/// Data structure
#[derive(Debug)]
pub struct BopDs;
