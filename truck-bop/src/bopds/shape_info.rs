//! Shape information structures

/// Shape kind
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShapeKind {
    /// Vertex
    Vertex,
    /// Edge
    Edge,
    /// Face
    Face,
    /// Solid
    Solid,
}

/// Shape information
#[derive(Debug)]
pub struct ShapeInfo {
    /// Shape kind
    pub kind: ShapeKind,
    /// Operand rank (0, 1, 2, ...)
    pub operand_rank: u8,
    /// Whether this is a source entity
    pub is_source: bool,
}
