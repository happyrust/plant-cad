//! Typed identifiers for BOP data structures

/// Shape identifier
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ShapeId(pub u32);

/// Vertex identifier
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub u32);

/// Edge identifier
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EdgeId(pub u32);

/// Face identifier
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FaceId(pub u32);

/// Pave block identifier
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct PaveBlockId(pub u32);

/// Common block identifier
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CommonBlockId(pub u32);

/// Section curve identifier
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SectionCurveId(pub u32);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn typed_ids_are_orderable() {
        assert!(VertexId(1) < VertexId(2));
    }

    #[test]
    fn typed_ids_are_hashable() {
        let mut set = std::collections::HashSet::new();
        set.insert(EdgeId(7));
        assert!(set.contains(&EdgeId(7)));
    }
}
