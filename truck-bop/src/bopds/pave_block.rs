//! Pave block structure

use crate::EdgeId;

/// Pave block - represents a segment of an edge between two paves
#[derive(Debug)]
pub struct PaveBlock {
    /// Original edge
    pub original_edge: EdgeId,
    /// Parameter range (start, end)
    pub param_range: (f64, f64),
}
