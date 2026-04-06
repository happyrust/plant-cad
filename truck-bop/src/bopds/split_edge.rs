//! Materialized split-edge facts derived from final pave blocks.

use crate::{CommonBlockId, EdgeId, SplitEdgeId, VertexId};

/// Materialized split-edge fact derived from a final edge partition.
#[derive(Debug, Clone, PartialEq)]
pub struct SplitEdgeRecord {
    /// Stable identifier of this materialized split edge.
    pub id: SplitEdgeId,
    /// Source edge from which this split edge originates.
    pub original_edge: EdgeId,
    /// Start vertex of the split edge segment.
    pub start_vertex: VertexId,
    /// End vertex of the split edge segment.
    pub end_vertex: VertexId,
    /// Parametric range on the source edge.
    pub param_range: (f64, f64),
    /// Shared-segment linkage derived from the authoritative common-block mapping.
    pub common_block: Option<CommonBlockId>,
    /// True when the segment is too small to split further in a stable way.
    pub unsplittable: bool,
}

impl SplitEdgeRecord {
    /// Creates a materialized split-edge record.
    pub fn new(
        id: SplitEdgeId,
        original_edge: EdgeId,
        start_vertex: VertexId,
        end_vertex: VertexId,
        param_range: (f64, f64),
        common_block: Option<CommonBlockId>,
        unsplittable: bool,
    ) -> Self {
        Self {
            id,
            original_edge,
            start_vertex,
            end_vertex,
            param_range,
            common_block,
            unsplittable,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_edge_record_stores_original_edge_and_param_range() {
        let record = SplitEdgeRecord::new(
            SplitEdgeId(4),
            EdgeId(7),
            VertexId(10),
            VertexId(11),
            (0.25, 0.75),
            Some(CommonBlockId(3)),
            true,
        );

        assert_eq!(record.id, SplitEdgeId(4));
        assert_eq!(record.original_edge, EdgeId(7));
        assert_eq!(record.start_vertex, VertexId(10));
        assert_eq!(record.end_vertex, VertexId(11));
        assert_eq!(record.param_range, (0.25, 0.75));
        assert_eq!(record.common_block, Some(CommonBlockId(3)));
        assert!(record.unsplittable);
    }
}
