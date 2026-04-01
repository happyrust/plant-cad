//! Pave block structure

use crate::{EdgeId, VertexId};

use super::Pave;

/// Pave block - represents a segment of an edge between two paves
#[derive(Debug, Clone, PartialEq)]
pub struct PaveBlock {
    /// Original edge
    pub original_edge: EdgeId,
    /// Start pave vertex.
    pub start_vertex: VertexId,
    /// End pave vertex.
    pub end_vertex: VertexId,
    /// Parameter range (start, end)
    pub param_range: (f64, f64),
    /// Extra split facts discovered after the base block was built.
    pub ext_paves: Vec<Pave>,
    /// Child blocks created by splitting this block, when available.
    pub split_result: Option<Vec<(VertexId, VertexId)>>,
    /// True when this block is too small to split reliably.
    pub unsplittable: bool,
}

impl PaveBlock {
    /// Create a pave block from consecutive paves on the same edge.
    pub fn from_pave_pair(start: Pave, end: Pave, parametric_tol: f64) -> Self {
        debug_assert_eq!(start.edge, end.edge);
        let (start, end) = if start.parameter <= end.parameter {
            (start, end)
        } else {
            (end, start)
        };
        let start_parameter = start.parameter;
        let end_parameter = end.parameter;
        Self {
            original_edge: start.edge,
            start_vertex: start.vertex,
            end_vertex: end.vertex,
            param_range: (start_parameter, end_parameter),
            ext_paves: Vec::new(),
            split_result: None,
            unsplittable: (end_parameter - start_parameter) < parametric_tol,
        }
    }

    /// Returns the endpoint paves plus any extra split paves in deterministic parameter order.
    pub fn ordered_paves(&self, parametric_tol: f64) -> Vec<Pave> {
        let mut paves = vec![
            Pave::new(
                self.original_edge,
                self.start_vertex,
                self.param_range.0,
                parametric_tol,
            )
            .expect("parametric tolerance is validated when BopOptions is created"),
            Pave::new(
                self.original_edge,
                self.end_vertex,
                self.param_range.1,
                parametric_tol,
            )
            .expect("parametric tolerance is validated when BopOptions is created"),
        ];
        paves.extend(self.ext_paves.iter().copied());
        paves.sort_by(|lhs, rhs| lhs.parameter.total_cmp(&rhs.parameter));

        let mut normalized: Vec<Pave> = Vec::with_capacity(paves.len());
        for pave in paves {
            if let Some(previous) = normalized.last_mut() {
                let tolerance = previous.tolerance.max(pave.tolerance).max(parametric_tol);
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

        normalized
    }

    /// Appends a split pave unless it matches one of the endpoints or an existing extra pave.
    pub fn append_ext_pave(&mut self, pave: Pave, parametric_tol: f64) -> bool {
        debug_assert_eq!(pave.edge, self.original_edge);
        let merged = self.ordered_paves(parametric_tol);
        if merged.iter().any(|existing| {
            (existing.parameter - pave.parameter).abs()
                <= existing.tolerance.max(pave.tolerance).max(parametric_tol)
        }) {
            return false;
        }
        self.ext_paves.push(pave);
        self.split_result = None;
        true
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

    #[test]
    fn pave_block_reorders_vertices_with_parameters() {
        let higher = Pave {
            edge: EdgeId(7),
            vertex: VertexId(70),
            parameter: 0.8,
            tolerance: 1.0e-6,
        };
        let lower = Pave {
            edge: EdgeId(7),
            vertex: VertexId(71),
            parameter: 0.2,
            tolerance: 1.0e-6,
        };

        let block = PaveBlock::from_pave_pair(higher, lower, 1.0e-8);

        assert_eq!(block.start_vertex, VertexId(71));
        assert_eq!(block.end_vertex, VertexId(70));
        assert_eq!(block.param_range, (0.2, 0.8));
    }

    #[test]
    fn pave_block_orders_and_deduplicates_extra_paves() {
        let mut block = PaveBlock::from_pave_pair(
            Pave::new(EdgeId(3), VertexId(30), 0.0, 1.0e-6).unwrap(),
            Pave::new(EdgeId(3), VertexId(31), 1.0, 1.0e-6).unwrap(),
            1.0e-6,
        );
        assert!(block.append_ext_pave(
            Pave::new(EdgeId(3), VertexId(33), 0.75, 1.0e-6).unwrap(),
            1.0e-6
        ));
        assert!(block.append_ext_pave(
            Pave::new(EdgeId(3), VertexId(32), 0.25, 1.0e-6).unwrap(),
            1.0e-6
        ));
        assert!(!block.append_ext_pave(
            Pave::new(EdgeId(3), VertexId(29), 0.2500005, 1.0e-6).unwrap(),
            1.0e-6
        ));

        let ordered = block.ordered_paves(1.0e-6);
        let parameters: Vec<f64> = ordered.iter().map(|pave| pave.parameter).collect();
        let vertices: Vec<VertexId> = ordered.iter().map(|pave| pave.vertex).collect();

        assert_eq!(parameters, vec![0.0, 0.25, 0.75, 1.0]);
        assert_eq!(
            vertices,
            vec![VertexId(30), VertexId(32), VertexId(33), VertexId(31)]
        );
    }
}
