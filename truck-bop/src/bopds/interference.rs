//! Interference structures

use crate::{EdgeId, FaceId, VertexId};

/// Vertex-vertex interference
#[derive(Debug)]
pub struct VVInterference {
    /// First vertex
    pub vertex1: VertexId,
    /// Second vertex
    pub vertex2: VertexId,
}

/// Vertex-edge interference
#[derive(Debug)]
pub struct VEInterference {
    /// Vertex
    pub vertex: VertexId,
    /// Edge
    pub edge: EdgeId,
    /// Parameter on edge
    pub parameter: f64,
}

/// Edge-edge interference
#[derive(Debug)]
pub struct EEInterference {
    /// First edge
    pub edge1: EdgeId,
    /// Second edge
    pub edge2: EdgeId,
}

/// Face-face interference
#[derive(Debug)]
pub struct FFInterference {
    /// First face
    pub face1: FaceId,
    /// Second face
    pub face2: FaceId,
}

/// Interference table
#[derive(Debug, Default)]
pub struct InterferenceTable {
    /// Vertex-vertex interferences
    pub vv: Vec<VVInterference>,
    /// Vertex-edge interferences
    pub ve: Vec<VEInterference>,
    /// Edge-edge interferences
    pub ee: Vec<EEInterference>,
    /// Face-face interferences
    pub ff: Vec<FFInterference>,
}
