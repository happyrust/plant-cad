//! Face information structure

use crate::{PaveBlockId, VertexId};

/// Face information
#[derive(Debug, Default)]
pub struct FaceInfo {
    /// Vertices on this face
    pub on_vertices: Vec<VertexId>,
    /// Pave blocks in this face
    pub in_pave_blocks: Vec<PaveBlockId>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_info_starts_empty() {
        let info = FaceInfo::default();
        assert!(info.on_vertices.is_empty());
        assert!(info.in_pave_blocks.is_empty());
    }
}
