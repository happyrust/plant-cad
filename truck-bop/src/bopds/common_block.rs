//! Common block structure — represents an edge segment shared by multiple faces.
//!
//! Modeled after OCCT's `BOPDS_CommonBlock`: groups one or more PaveBlocks that
//! share the same geometric edge curve, and records which faces they belong to.

use crate::{FaceId, PaveBlockId};

/// A common block groups pave blocks that lie on the same geometric edge
/// (potentially from different operands) and records the faces that share them.
#[allow(missing_docs)]
#[derive(Debug, Clone)]
pub struct CommonBlock {
    pave_blocks: Vec<PaveBlockId>,
    faces: Vec<FaceId>,
    tolerance: f64,
}

impl CommonBlock {
    /// Create a new common block with one initial pave block.
    pub fn new(initial_block: PaveBlockId, tolerance: f64) -> Self {
        Self {
            pave_blocks: vec![initial_block],
            faces: Vec::new(),
            tolerance,
        }
    }

    /// All pave blocks in this common block.
    pub fn pave_blocks(&self) -> &[PaveBlockId] {
        &self.pave_blocks
    }

    /// The first (representative) pave block.
    pub fn representative(&self) -> PaveBlockId {
        self.pave_blocks[0]
    }

    /// Add a pave block (deduplicated).
    pub fn add_pave_block(&mut self, pb: PaveBlockId) {
        if !self.pave_blocks.contains(&pb) {
            self.pave_blocks.push(pb);
        }
    }

    pub fn contains_pave_block(&self, pb: PaveBlockId) -> bool {
        self.pave_blocks.contains(&pb)
    }

    pub fn faces(&self) -> &[FaceId] {
        &self.faces
    }

    pub fn add_face(&mut self, face: FaceId) {
        if !self.faces.contains(&face) {
            self.faces.push(face);
        }
    }

    pub fn contains_face(&self, face: FaceId) -> bool {
        self.faces.contains(&face)
    }

    pub fn tolerance(&self) -> f64 {
        self.tolerance
    }

    pub fn set_tolerance(&mut self, tolerance: f64) {
        self.tolerance = tolerance;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn common_block_tracks_pave_blocks_and_faces() {
        let mut cb = CommonBlock::new(PaveBlockId(0), 1.0e-6);
        cb.add_pave_block(PaveBlockId(1));
        cb.add_face(FaceId(10));
        cb.add_face(FaceId(20));

        assert_eq!(cb.representative(), PaveBlockId(0));
        assert_eq!(cb.pave_blocks().len(), 2);
        assert!(cb.contains_pave_block(PaveBlockId(1)));
        assert!(!cb.contains_pave_block(PaveBlockId(99)));
        assert!(cb.contains_face(FaceId(10)));
        assert_eq!(cb.faces().len(), 2);
    }

    #[test]
    fn common_block_deduplicates_additions() {
        let mut cb = CommonBlock::new(PaveBlockId(0), 1.0e-6);
        cb.add_pave_block(PaveBlockId(0));
        cb.add_face(FaceId(5));
        cb.add_face(FaceId(5));

        assert_eq!(cb.pave_blocks().len(), 1);
        assert_eq!(cb.faces().len(), 1);
    }
}
