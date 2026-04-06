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
    /// Monotonic version, increments when new facts are appended.
    pub version: u64,
    /// If true, there is unread update since last snapshot.
    pub is_dirty: bool,
}

/// Snapshot view for vertex-class membership.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaceVertexClasses {
    /// Any vertex in the `on` pool.
    pub on: bool,
    /// Any vertex in the `in` pool.
    pub in_: bool,
    /// Any vertex in the `sc` pool.
    pub sc: bool,
}

impl FaceInfo {
    /// Constructs an empty record with explicit initial version.
    pub fn with_version(version: u64) -> Self {
        Self {
            version,
            ..Self::default()
        }
    }

    /// Returns whether the record has been updated since last snapshot.
    pub fn touched(&self) -> bool { self.is_dirty }

    /// Clears pooled data and resets dirty state.
    pub fn clear(&mut self) {
        self.on_vertices.clear();
        self.in_vertices.clear();
        self.sc_vertices.clear();
        self.on_pave_blocks.clear();
        self.in_pave_blocks.clear();
        self.sc_pave_blocks.clear();
        self.is_dirty = false;
    }

    /// Current number of boundary vertices.
    pub fn on_count(&self) -> usize { self.on_vertices.len() }

    /// Current number of inner vertices.
    pub fn in_count(&self) -> usize { self.in_vertices.len() }

    /// Current number of section vertices.
    pub fn sc_count(&self) -> usize { self.sc_vertices.len() }

    /// Returns `true` when any vertex belongs to any pool.
    pub fn contains_vertex(&self, vertex: VertexId) -> bool {
        self.on_vertices.contains(&vertex)
            || self.in_vertices.contains(&vertex)
            || self.sc_vertices.contains(&vertex)
    }

    /// Returns `true` when any pave block belongs to any pool.
    pub fn contains_pave_block(&self, pave_block: PaveBlockId) -> bool {
        self.on_pave_blocks.contains(&pave_block)
            || self.in_pave_blocks.contains(&pave_block)
            || self.sc_pave_blocks.contains(&pave_block)
    }

    /// Returns the non-empty class set of this face-info record.
    pub fn vertex_classes(&self) -> FaceVertexClasses {
        FaceVertexClasses {
            on: !self.on_vertices.is_empty(),
            in_: !self.in_vertices.is_empty(),
            sc: !self.sc_vertices.is_empty(),
        }
    }

    /// Marks this record as updated without changing semantic data.
    pub fn mark_dirty(&mut self) {
        self.version += 1;
        self.is_dirty = true;
    }

    /// Mark this record consumed as snapshot.
    pub fn mark_clean(&mut self) { self.is_dirty = false; }

    /// Registers a boundary vertex if it was not recorded yet.
    pub fn push_on_vertex(&mut self, vertex: VertexId) -> bool {
        let changed = push_unique(&mut self.on_vertices, vertex);
        if changed {
            self.mark_dirty();
        }
        changed
    }

    /// Registers an interior vertex if it was not recorded yet.
    pub fn push_in_vertex(&mut self, vertex: VertexId) -> bool {
        let changed = push_unique(&mut self.in_vertices, vertex);
        if changed {
            self.mark_dirty();
        }
        changed
    }

    /// Registers a section vertex if it was not recorded yet.
    pub fn push_sc_vertex(&mut self, vertex: VertexId) -> bool {
        let changed = push_unique(&mut self.sc_vertices, vertex);
        if changed {
            self.mark_dirty();
        }
        changed
    }

    /// Registers a boundary pave block if it was not recorded yet.
    pub fn push_on_pave_block(&mut self, pave_block: PaveBlockId) -> bool {
        let changed = push_unique(&mut self.on_pave_blocks, pave_block);
        if changed {
            self.mark_dirty();
        }
        changed
    }

    /// Registers an interior pave block if it was not recorded yet.
    pub fn push_in_pave_block(&mut self, pave_block: PaveBlockId) -> bool {
        let changed = push_unique(&mut self.in_pave_blocks, pave_block);
        if changed {
            self.mark_dirty();
        }
        changed
    }

    /// Registers a section pave block if it was not recorded yet.
    pub fn push_sc_pave_block(&mut self, pave_block: PaveBlockId) -> bool {
        let changed = push_unique(&mut self.sc_pave_blocks, pave_block);
        if changed {
            self.mark_dirty();
        }
        changed
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

    #[test]
    fn face_info_records_state_changes() {
        let mut info = FaceInfo::with_version(7);
        assert_eq!(info.version, 7);
        assert!(!info.touched());
        assert!(info.push_on_vertex(VertexId(1)));
        assert!(info.touched());
        assert_eq!(info.version, 8);
        assert!(!info.push_on_vertex(VertexId(1)));
        assert_eq!(info.version, 8);

        assert_eq!(
            info.vertex_classes(),
            FaceVertexClasses {
                on: true,
                in_: false,
                sc: false,
            }
        );
        assert!(info.contains_vertex(VertexId(1)));
        assert_eq!(info.on_count(), 1);
    }

    #[test]
    fn face_info_state_merge_uses_latest() {
        let mut info = FaceInfo::default();
        info.push_sc_pave_block(PaveBlockId(3));
        let version_after_first = info.version;
        info.push_sc_pave_block(PaveBlockId(4));
        assert!(info.version > version_after_first);
        let latest_version = info.version;

        info.mark_clean();
        assert!(!info.touched());
        info.push_sc_pave_block(PaveBlockId(4));
        assert_eq!(info.version, latest_version);
    }

    #[test]
    fn face_info_queries_is_deterministic() {
        let mut info = FaceInfo::default();
        let version = info.version;
        assert!(!info.contains_pave_block(PaveBlockId(9)));
        info.push_in_pave_block(PaveBlockId(9));
        assert!(info.contains_pave_block(PaveBlockId(9)));
        assert_eq!(info.in_pave_blocks.len(), 1);
        assert_eq!(
            info.vertex_classes(),
            FaceVertexClasses {
                on: false,
                in_: false,
                sc: false,
            }
        );
        info.push_in_vertex(VertexId(11));
        assert_eq!(
            info.vertex_classes(),
            FaceVertexClasses {
                on: false,
                in_: true,
                sc: false,
            }
        );
        info.clear();
        assert!(!info.touched());
        assert_eq!(info.on_count(), 0);
        assert_eq!(info.version, version + 2);
    }
}
