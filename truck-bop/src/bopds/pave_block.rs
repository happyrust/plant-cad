//! Pave block structure

use crate::{EdgeId, VertexId};

use super::Pave;

/// Pave block - represents a segment of an edge between two paves
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PaveBlock {
    /// Original edge
    pub original_edge: EdgeId,
    /// Start pave vertex.
    pub start_vertex: VertexId,
    /// End pave vertex.
    pub end_vertex: VertexId,
    /// Parameter range (start, end)
    pub param_range: (f64, f64),
    /// True when this block is too small to split reliably.
    pub unsplittable: bool,
}

impl PaveBlock {
    /// Create a pave block from consecutive paves on the same edge.
    pub fn from_pave_pair(start: Pave, end: Pave, parametric_tol: f64) -> Self {
        debug_assert_eq!(start.edge, end.edge);
        let start_parameter = start.parameter.min(end.parameter);
        let end_parameter = start.parameter.max(end.parameter);
        Self {
            original_edge: start.edge,
            start_vertex: start.vertex,
            end_vertex: end.vertex,
            param_range: (start_parameter, end_parameter),
            unsplittable: (end_parameter - start_parameter) < parametric_tol,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pave_block_marks_micro_segments_unsplittable() {
        let start = Pave {
            edge: EdgeId(2),
            vertex: VertexId(10),
            parameter: 0.5,
            tolerance: 1.0e-6,
        };
        let end = Pave {
            edge: EdgeId(2),
            vertex: VertexId(11),
            parameter: 0.500000005,
            tolerance: 1.0e-6,
        };

        let block = PaveBlock::from_pave_pair(start, end, 1.0e-8);

        assert!(block.unsplittable);
        assert_eq!(block.param_range, (0.5, 0.500000005));
    }
}
