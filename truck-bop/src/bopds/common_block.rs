//! Common block structure

use crate::{EdgeId, FaceId, PaveBlockId};

/// Common block grouping one or more shared pave blocks.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CommonBlock {
    /// Pave blocks that belong to the same shared geometric segment.
    pub pave_blocks: Vec<PaveBlockId>,
    /// Faces participating in this shared segment.
    pub faces: Vec<FaceId>,
    /// Canonical edge identity, when one source edge can represent the group.
    pub representative_edge: Option<EdgeId>,
}

impl CommonBlock {
    /// Creates a common block and normalizes duplicate pave blocks and faces.
    pub fn new(
        mut pave_blocks: Vec<PaveBlockId>,
        mut faces: Vec<FaceId>,
        representative_edge: Option<EdgeId>,
    ) -> Self {
        dedup_preserve_order(&mut pave_blocks);
        dedup_preserve_order(&mut faces);
        Self {
            pave_blocks,
            faces,
            representative_edge,
        }
    }

    /// Appends a pave block to the group if it is not already present.
    pub fn push_pave_block(&mut self, pave_block: PaveBlockId) {
        if !self.pave_blocks.contains(&pave_block) {
            self.pave_blocks.push(pave_block);
        }
    }

    /// Appends a face to the group if it is not already present.
    pub fn push_face(&mut self, face: FaceId) {
        if !self.faces.contains(&face) {
            self.faces.push(face);
        }
    }
}

fn dedup_preserve_order<T: PartialEq + Copy>(items: &mut Vec<T>) {
    let mut deduped = Vec::with_capacity(items.len());
    for item in items.iter().copied() {
        if !deduped.contains(&item) {
            deduped.push(item);
        }
    }
    *items = deduped;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn common_block_can_group_multiple_pave_blocks() {
        let mut block = CommonBlock::new(vec![PaveBlockId(1)], vec![FaceId(2)], Some(EdgeId(9)));

        block.push_pave_block(PaveBlockId(3));
        block.push_pave_block(PaveBlockId(1));
        block.push_face(FaceId(4));
        block.push_face(FaceId(2));

        assert_eq!(block.pave_blocks, vec![PaveBlockId(1), PaveBlockId(3)]);
        assert_eq!(block.faces, vec![FaceId(2), FaceId(4)]);
        assert_eq!(block.representative_edge, Some(EdgeId(9)));
    }
}
