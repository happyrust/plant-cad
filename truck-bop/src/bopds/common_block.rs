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
    /// Marks whether this shared segment is active in the current operation.
    pub is_active: bool,
    /// Edges proving the shared segment's geometric relation.
    pub witness_edges: Vec<EdgeId>,
    /// Faces proving the shared segment's geometric relation.
    pub witness_faces: Vec<FaceId>,
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
            is_active: true,
            witness_edges: Vec::new(),
            witness_faces: Vec::new(),
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

    /// Returns true when there is no pave block and no face.
    pub fn is_empty(&self) -> bool { self.pave_blocks.is_empty() && self.faces.is_empty() }

    /// Returns true when this block should participate in downstream logic.
    pub fn is_effective(&self) -> bool { self.is_active && !self.pave_blocks.is_empty() }

    /// Returns true if the block contains the provided pave block.
    pub fn contains_pave_block(&self, pave_block: PaveBlockId) -> bool {
        self.pave_blocks.contains(&pave_block)
    }

    /// Returns true if the block contains the provided face.
    pub fn contains_face(&self, face: FaceId) -> bool { self.faces.contains(&face) }

    /// Returns true if the block contains the provided witness edge.
    pub fn contains_witness_edge(&self, edge: EdgeId) -> bool { self.witness_edges.contains(&edge) }

    /// Returns true if the block contains the provided witness face.
    pub fn contains_witness_face(&self, face: FaceId) -> bool { self.witness_faces.contains(&face) }

    /// Registers a witness edge with deduplicated semantics.
    pub fn register_witness_edge(&mut self, edge: EdgeId) {
        if !self.witness_edges.contains(&edge) {
            self.witness_edges.push(edge);
        }
    }

    /// Registers a witness face with deduplicated semantics.
    pub fn register_witness_face(&mut self, face: FaceId) {
        if !self.witness_faces.contains(&face) {
            self.witness_faces.push(face);
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
        assert!(block.is_active);
        assert!(block.is_effective());
    }

    #[test]
    fn common_block_dedup_members() {
        let mut block = CommonBlock::new(
            vec![PaveBlockId(1), PaveBlockId(1)],
            vec![FaceId(2), FaceId(2)],
            Some(EdgeId(9)),
        );

        block.push_pave_block(PaveBlockId(1));
        block.push_face(FaceId(2));
        block.register_witness_edge(EdgeId(9));
        block.register_witness_face(FaceId(2));

        assert_eq!(block.pave_blocks, vec![PaveBlockId(1)]);
        assert_eq!(block.faces, vec![FaceId(2)]);
        assert_eq!(block.witness_edges, vec![EdgeId(9)]);
        assert_eq!(block.witness_faces, vec![FaceId(2)]);
    }

    #[test]
    fn common_block_registers_witnesses_and_queries() {
        let mut block = CommonBlock::new(vec![PaveBlockId(1)], vec![FaceId(2)], Some(EdgeId(9)));

        block.register_witness_edge(EdgeId(10));
        block.register_witness_face(FaceId(3));

        assert!(block.contains_pave_block(PaveBlockId(1)));
        assert!(!block.contains_pave_block(PaveBlockId(2)));
        assert!(block.contains_face(FaceId(2)));
        assert!(!block.contains_face(FaceId(8)));
        assert!(block.contains_witness_edge(EdgeId(10)));
        assert!(!block.contains_witness_edge(EdgeId(11)));
        assert!(block.contains_witness_face(FaceId(3)));
        assert!(!block.contains_witness_face(FaceId(4)));
    }

    #[test]
    fn common_block_collects_multiple_pave_blocks() {
        let block = CommonBlock::new(
            vec![PaveBlockId(1), PaveBlockId(3)],
            vec![FaceId(2)],
            Some(EdgeId(9)),
        );

        assert!(!block.is_empty());
        assert!(block.is_effective());
        assert_eq!(block.pave_blocks, vec![PaveBlockId(1), PaveBlockId(3)]);
    }
}
