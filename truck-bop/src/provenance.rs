//! Provenance tracking for boolean operation results.
//!
//! Records which input topological entities (edges, faces) each output entity
//! originates from, enabling downstream consumers to trace results back to
//! their source geometry without modifying `truck-topology` core types.

use crate::bopds::{EdgeId, FaceId, SectionCurveId, VertexId};
use rustc_hash::FxHashMap;

/// Describes where an output edge or face originated.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SourceOrigin {
    /// Derived from an original boundary edge in the input solid.
    OriginalEdge(EdgeId),
    /// Derived from an intersection section curve.
    SectionCurve(SectionCurveId),
    /// Derived from an original face in the input solid.
    OriginalFace(FaceId),
    /// Synthesized during loop repair or other internal operations;
    /// no traceable input entity.
    Synthesized,
}

/// Per-edge provenance: which source edges contributed to this output edge.
#[derive(Debug, Clone)]
pub struct EdgeProvenance {
    /// BopDs vertex pair identifying this rebuilt edge (canonical, undirected).
    pub start_vertex: VertexId,
    /// BopDs vertex pair identifying this rebuilt edge (canonical, undirected).
    pub end_vertex: VertexId,
    /// Source edge(s) that this output edge was derived from.
    pub sources: Vec<SourceOrigin>,
}

/// Per-face provenance: which source face this output face was derived from.
#[derive(Debug, Clone, Copy)]
pub struct FaceProvenance {
    /// The BopDs FaceId of the input face that was split/trimmed.
    pub original_face: FaceId,
    /// Which operand (0 or 1) the source face belongs to.
    pub operand_rank: u8,
}

/// Provenance map returned alongside boolean operation results.
///
/// Provides a complete mapping from output topology back to input topology
/// without requiring any changes to `truck-topology` core types.
#[derive(Debug, Default)]
pub struct ProvenanceMap {
    /// Per-output-face provenance, indexed parallel to the output face list.
    pub faces: Vec<FaceProvenance>,
    /// Per-output-edge provenance, keyed by canonical `(VertexId, VertexId)`.
    pub edges: FxHashMap<(VertexId, VertexId), Vec<SourceOrigin>>,
    /// Vertex merge mapping: original VertexId → canonical VertexId.
    pub merged_vertices: FxHashMap<VertexId, VertexId>,
}

impl ProvenanceMap {
    /// Records that an output face at the given index originates from `face_id`
    /// belonging to operand `rank`.
    pub fn record_face(&mut self, original_face: FaceId, operand_rank: u8) {
        self.faces.push(FaceProvenance {
            original_face,
            operand_rank,
        });
    }

    /// Records that an output edge (identified by its canonical vertex pair)
    /// originates from `source`.
    pub fn record_edge(&mut self, start: VertexId, end: VertexId, source: SourceOrigin) {
        let key = if start <= end {
            (start, end)
        } else {
            (end, start)
        };
        let sources = self.edges.entry(key).or_default();
        if !sources.contains(&source) {
            sources.push(source);
        }
    }

    /// Look up provenance for an edge by its canonical vertex pair.
    pub fn edge_sources(&self, start: VertexId, end: VertexId) -> Option<&[SourceOrigin]> {
        let key = if start <= end {
            (start, end)
        } else {
            (end, start)
        };
        self.edges.get(&key).map(|v| v.as_slice())
    }
}
