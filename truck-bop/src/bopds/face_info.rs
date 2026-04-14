//! Face information structure — tracks In/On/Section sub-shapes for a face.
//!
//! After intersection, each face maintains three categories of sub-shapes:
//! - **On**: vertices/edges that lie on the face boundary
//! - **In**: vertices/edges that lie strictly inside the face
//! - **Section**: vertices/edges from face-face intersection curves

use crate::{PaveBlockId, VertexId};

/// Face information with In / On / Section classification.
#[allow(missing_docs)]
#[derive(Debug, Default, Clone)]
pub struct FaceInfo {
    pub on_vertices: Vec<VertexId>,
    pub on_pave_blocks: Vec<PaveBlockId>,

    pub in_vertices: Vec<VertexId>,
    pub in_pave_blocks: Vec<PaveBlockId>,

    pub section_vertices: Vec<VertexId>,
    pub section_pave_blocks: Vec<PaveBlockId>,
}

impl FaceInfo {
    /// Reset all sub-shape classifications.
    pub fn clear(&mut self) {
        self.on_vertices.clear();
        self.on_pave_blocks.clear();
        self.in_vertices.clear();
        self.in_pave_blocks.clear();
        self.section_vertices.clear();
        self.section_pave_blocks.clear();
    }

    /// Record a vertex that lies on the face boundary.
    pub fn add_on_vertex(&mut self, v: VertexId) {
        if !self.on_vertices.contains(&v) {
            self.on_vertices.push(v);
        }
    }

    /// Record a pave block that lies on the face boundary.
    pub fn add_on_pave_block(&mut self, pb: PaveBlockId) {
        if !self.on_pave_blocks.contains(&pb) {
            self.on_pave_blocks.push(pb);
        }
    }

    /// Record a vertex that lies strictly inside the face.
    pub fn add_in_vertex(&mut self, v: VertexId) {
        if !self.in_vertices.contains(&v) {
            self.in_vertices.push(v);
        }
    }

    /// Record a pave block that lies strictly inside the face.
    pub fn add_in_pave_block(&mut self, pb: PaveBlockId) {
        if !self.in_pave_blocks.contains(&pb) {
            self.in_pave_blocks.push(pb);
        }
    }

    /// Record a vertex from face-face intersection.
    pub fn add_section_vertex(&mut self, v: VertexId) {
        if !self.section_vertices.contains(&v) {
            self.section_vertices.push(v);
        }
    }

    /// Record a pave block from face-face intersection.
    pub fn add_section_pave_block(&mut self, pb: PaveBlockId) {
        if !self.section_pave_blocks.contains(&pb) {
            self.section_pave_blocks.push(pb);
        }
    }

    /// Returns `true` if no sub-shapes have been recorded.
    pub fn is_empty(&self) -> bool {
        self.on_vertices.is_empty()
            && self.on_pave_blocks.is_empty()
            && self.in_vertices.is_empty()
            && self.in_pave_blocks.is_empty()
            && self.section_vertices.is_empty()
            && self.section_pave_blocks.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn face_info_starts_empty() {
        let info = FaceInfo::default();
        assert!(info.on_vertices.is_empty());
        assert!(info.in_pave_blocks.is_empty());
        assert!(info.is_empty());
    }

    #[test]
    fn face_info_in_on_section_are_independent() {
        let mut info = FaceInfo::default();
        info.add_on_vertex(VertexId(1));
        info.add_in_vertex(VertexId(2));
        info.add_section_vertex(VertexId(3));
        info.add_on_pave_block(PaveBlockId(10));
        info.add_in_pave_block(PaveBlockId(20));
        info.add_section_pave_block(PaveBlockId(30));

        assert_eq!(info.on_vertices, vec![VertexId(1)]);
        assert_eq!(info.in_vertices, vec![VertexId(2)]);
        assert_eq!(info.section_vertices, vec![VertexId(3)]);
        assert!(!info.is_empty());
    }

    #[test]
    fn face_info_clear_resets_all() {
        let mut info = FaceInfo::default();
        info.add_on_vertex(VertexId(1));
        info.add_in_pave_block(PaveBlockId(10));
        info.add_section_vertex(VertexId(5));

        info.clear();

        assert!(info.is_empty());
    }

    #[test]
    fn face_info_deduplicates() {
        let mut info = FaceInfo::default();
        info.add_on_vertex(VertexId(1));
        info.add_on_vertex(VertexId(1));
        info.add_in_pave_block(PaveBlockId(10));
        info.add_in_pave_block(PaveBlockId(10));

        assert_eq!(info.on_vertices.len(), 1);
        assert_eq!(info.in_pave_blocks.len(), 1);
    }
}
