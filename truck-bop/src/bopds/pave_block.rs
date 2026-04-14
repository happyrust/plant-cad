//! Pave block structure — represents a segment of an edge between two paves.
//!
//! Modeled after OCCT's `BOPDS_PaveBlock`: supports extra paves (intermediate
//! split points) and an `update` operation that produces sub-blocks.

use crate::{EdgeId, PaveBlockId, VertexId};

use super::Pave;

/// Pave block — a segment of an edge between two endpoint paves,
/// with optional intermediate "extra" paves for further splitting.
#[allow(missing_docs)]
#[derive(Debug, Clone, PartialEq)]
pub struct PaveBlock {
    pub original_edge: EdgeId,
    pub start_vertex: VertexId,
    pub end_vertex: VertexId,
    pub param_range: (f64, f64),
    pub unsplittable: bool,
    /// Intermediate paves within this block, sorted by parameter.
    ext_paves: Vec<Pave>,
    /// The split edge assigned to this block after splitting (if any).
    split_edge: Option<EdgeId>,
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
            unsplittable: (end_parameter - start_parameter) < parametric_tol,
            ext_paves: Vec::new(),
            split_edge: None,
        }
    }

    /// All intermediate paves within this block, sorted by parameter.
    pub fn ext_paves(&self) -> &[Pave] {
        &self.ext_paves
    }

    /// Add an intermediate pave within this block's parameter range.
    /// Returns true if the pave was added (parameter is within range and not duplicate).
    pub fn add_ext_pave(&mut self, pave: Pave, parametric_tol: f64) -> bool {
        if pave.parameter <= self.param_range.0 + parametric_tol
            || pave.parameter >= self.param_range.1 - parametric_tol
        {
            return false;
        }
        if self
            .ext_paves
            .iter()
            .any(|p| (p.parameter - pave.parameter).abs() < parametric_tol)
        {
            return false;
        }
        self.ext_paves.push(pave);
        self.ext_paves
            .sort_by(|a, b| a.parameter.partial_cmp(&b.parameter).unwrap());
        true
    }

    /// Returns `true` if intermediate paves exist.
    pub fn has_ext_paves(&self) -> bool {
        !self.ext_paves.is_empty()
    }

    /// Checks whether the given parameter lies within this block's range.
    pub fn contains_parameter(&self, param: f64, tol: f64) -> bool {
        param >= self.param_range.0 - tol && param <= self.param_range.1 + tol
    }

    /// The split edge assigned after splitting (if any).
    pub fn split_edge(&self) -> Option<EdgeId> {
        self.split_edge
    }

    /// Assign a split edge to this pave block.
    pub fn set_split_edge(&mut self, edge: EdgeId) {
        self.split_edge = Some(edge);
    }

    /// Split this block by its ext_paves, producing sub-blocks.
    /// Returns the list of sub-blocks (including this block's endpoints
    /// combined with ext_pave positions). The original block is consumed.
    pub fn update(
        &self,
        parametric_tol: f64,
        id_start: PaveBlockId,
    ) -> Vec<(PaveBlockId, PaveBlock)> {
        if self.ext_paves.is_empty() {
            return vec![(id_start, self.clone())];
        }

        let mut all_paves = Vec::with_capacity(self.ext_paves.len() + 2);
        all_paves.push(Pave {
            edge: self.original_edge,
            vertex: self.start_vertex,
            parameter: self.param_range.0,
            tolerance: parametric_tol,
        });
        all_paves.extend(self.ext_paves.iter().cloned());
        all_paves.push(Pave {
            edge: self.original_edge,
            vertex: self.end_vertex,
            parameter: self.param_range.1,
            tolerance: parametric_tol,
        });

        all_paves
            .windows(2)
            .enumerate()
            .map(|(i, pair)| {
                let id = PaveBlockId(id_start.0 + i as u32);
                (id, PaveBlock::from_pave_pair(pair[0], pair[1], parametric_tol))
            })
            .collect()
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
    fn ext_pave_within_range_is_added() {
        let block = make_block(0.0, 1.0);

        let mut b = block;
        let added = b.add_ext_pave(
            Pave {
                edge: EdgeId(1),
                vertex: VertexId(50),
                parameter: 0.5,
                tolerance: 1.0e-6,
            },
            1.0e-8,
        );

        assert!(added);
        assert_eq!(b.ext_paves().len(), 1);
    }

    #[test]
    fn ext_pave_outside_range_is_rejected() {
        let mut b = make_block(0.0, 1.0);
        let added = b.add_ext_pave(
            Pave {
                edge: EdgeId(1),
                vertex: VertexId(50),
                parameter: 1.5,
                tolerance: 1.0e-6,
            },
            1.0e-8,
        );

        assert!(!added);
        assert!(b.ext_paves().is_empty());
    }

    #[test]
    fn update_without_ext_paves_returns_self() {
        let block = make_block(0.0, 1.0);
        let result = block.update(1.0e-8, PaveBlockId(0));

        assert_eq!(result.len(), 1);
        assert_eq!(result[0].1.param_range, (0.0, 1.0));
    }

    #[test]
    fn update_with_one_ext_pave_produces_two_sub_blocks() {
        let mut block = make_block(0.0, 1.0);
        block.add_ext_pave(
            Pave {
                edge: EdgeId(1),
                vertex: VertexId(50),
                parameter: 0.4,
                tolerance: 1.0e-6,
            },
            1.0e-8,
        );

        let result = block.update(1.0e-8, PaveBlockId(100));

        assert_eq!(result.len(), 2);
        assert_eq!(result[0].0, PaveBlockId(100));
        assert_eq!(result[0].1.param_range, (0.0, 0.4));
        assert_eq!(result[1].0, PaveBlockId(101));
        assert_eq!(result[1].1.param_range, (0.4, 1.0));
    }

    #[test]
    fn update_with_two_ext_paves_produces_three_sub_blocks() {
        let mut block = make_block(0.0, 1.0);
        block.add_ext_pave(
            Pave {
                edge: EdgeId(1),
                vertex: VertexId(50),
                parameter: 0.3,
                tolerance: 1.0e-6,
            },
            1.0e-8,
        );
        block.add_ext_pave(
            Pave {
                edge: EdgeId(1),
                vertex: VertexId(51),
                parameter: 0.7,
                tolerance: 1.0e-6,
            },
            1.0e-8,
        );

        let result = block.update(1.0e-8, PaveBlockId(0));

        assert_eq!(result.len(), 3);
        assert!((result[0].1.param_range.1 - 0.3).abs() < 1.0e-12);
        assert!((result[1].1.param_range.0 - 0.3).abs() < 1.0e-12);
        assert!((result[1].1.param_range.1 - 0.7).abs() < 1.0e-12);
        assert!((result[2].1.param_range.0 - 0.7).abs() < 1.0e-12);
    }

    fn make_block(start_param: f64, end_param: f64) -> PaveBlock {
        PaveBlock::from_pave_pair(
            Pave {
                edge: EdgeId(1),
                vertex: VertexId(10),
                parameter: start_param,
                tolerance: 1.0e-6,
            },
            Pave {
                edge: EdgeId(1),
                vertex: VertexId(11),
                parameter: end_param,
                tolerance: 1.0e-6,
            },
            1.0e-8,
        )
    }
}
