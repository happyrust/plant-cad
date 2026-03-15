//! Boolean operation data structure

mod arena;
mod ids;

pub use ids::{
    CommonBlockId, EdgeId, FaceId, PaveBlockId, SectionCurveId, ShapeId, VertexId,
};

/// Data structure
#[derive(Debug)]
pub struct BopDs;
