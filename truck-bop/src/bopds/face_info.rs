//! Face information structure

use crate::{PaveBlockId, VertexId};

/// Face information grouped by boolean-state semantics.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct FaceInfo {
    /// Vertices that lie on the face boundary.
    pub on_vertices: Vec<VertexId>,
    /// Vertices that lie inside the face domain.
    pub in_vertices: Vec<VertexId>,
    /// Vertices that belong to face-face section geometry.
    pub sc_vertices: Vec<VertexId>,
    /// Pave blocks that lie on the face boundary.
    pub on_pave_blocks: Vec<PaveBlockId>,
    /// Pave blocks that lie inside the face domain.
    pub in_pave_blocks: Vec<PaveBlockId>,
    /// Pave blocks that come from section geometry.
    pub sc_pave_blocks: Vec<PaveBlockId>,
}

impl FaceInfo {
    /// Registers a boundary vertex if it was not recorded yet.
    pub fn push_on_vertex(&mut self, vertex: VertexId) -> bool {
        push_unique(&mut self.on_vertices, vertex)
    }

    /// Registers an interior vertex if it was not recorded yet.
    pub fn push_in_vertex(&mut self, vertex: VertexId) -> bool {
        push_unique(&mut self.in_vertices, vertex)
    }

    /// Registers a section vertex if it was not recorded yet.
    pub fn push_sc_vertex(&mut self, vertex: VertexId) -> bool {
        push_unique(&mut self.sc_vertices, vertex)
    }

    /// Registers a boundary pave block if it was not recorded yet.
    pub fn push_on_pave_block(&mut self, pave_block: PaveBlockId) -> bool {
        push_unique(&mut self.on_pave_blocks, pave_block)
    }

    /// Registers an interior pave block if it was not recorded yet.
    pub fn push_in_pave_block(&mut self, pave_block: PaveBlockId) -> bool {
        push_unique(&mut self.in_pave_blocks, pave_block)
    }

    /// Registers a section pave block if it was not recorded yet.
    pub fn push_sc_pave_block(&mut self, pave_block: PaveBlockId) -> bool {
        push_unique(&mut self.sc_pave_blocks, pave_block)
    }
}

fn push_unique<T: PartialEq + Copy>(items: &mut Vec<T>, item: T) -> bool {
    if items.contains(&item) {
        false
    } else {
        items.push(item);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_info_starts_empty() {
        let info = FaceInfo::default();
        assert!(info.on_vertices.is_empty());
        assert!(info.in_vertices.is_empty());
        assert!(info.sc_vertices.is_empty());
        assert!(info.on_pave_blocks.is_empty());
        assert!(info.in_pave_blocks.is_empty());
        assert!(info.sc_pave_blocks.is_empty());
    }

    #[test]
    fn face_info_tracks_on_in_sc_separately() {
        let mut info = FaceInfo::default();

        info.push_on_vertex(VertexId(1));
        info.push_in_vertex(VertexId(2));
        info.push_sc_vertex(VertexId(3));
        info.push_on_pave_block(PaveBlockId(10));
        info.push_in_pave_block(PaveBlockId(11));
        info.push_sc_pave_block(PaveBlockId(12));

        assert_eq!(info.on_vertices, vec![VertexId(1)]);
        assert_eq!(info.in_vertices, vec![VertexId(2)]);
        assert_eq!(info.sc_vertices, vec![VertexId(3)]);
        assert_eq!(info.on_pave_blocks, vec![PaveBlockId(10)]);
        assert_eq!(info.in_pave_blocks, vec![PaveBlockId(11)]);
        assert_eq!(info.sc_pave_blocks, vec![PaveBlockId(12)]);
    }

    #[test]
    fn face_info_deduplicates_repeated_block_registration() {
        let mut info = FaceInfo::default();

        info.push_on_pave_block(PaveBlockId(8));
        info.push_on_pave_block(PaveBlockId(8));
        info.push_sc_vertex(VertexId(5));
        info.push_sc_vertex(VertexId(5));

        assert_eq!(info.on_pave_blocks, vec![PaveBlockId(8)]);
        assert_eq!(info.sc_vertices, vec![VertexId(5)]);
    }
}
