//! Interference structures

use crate::{EdgeId, FaceId, PointClassification, SectionCurveId, VertexId};
use truck_base::cgmath64::Point2;

/// A merged vertex cluster built from selected fragments.
#[derive(Debug, Clone, PartialEq)]
pub struct MergedVertex {
    /// Canonical vertex identifier representing the merged cluster.
    pub id: VertexId,
    /// Source vertices absorbed into this merged vertex.
    pub original_vertices: Vec<VertexId>,
    /// Canonical merged location in model space.
    pub point: truck_base::cgmath64::Point3,
}

/// Shared topology identity used across trimming, sewing, and rebuild.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum TrimmingTopologyKey {
    /// Identity of a real source boundary edge from the original face topology.
    SourceBoundary(EdgeId),
    /// Identity of a generated section curve from face-face intersection.
    SectionCurve(SectionCurveId),
    /// Identity of a generated trimming edge that has no source topology.
    Generated {
        /// Source face that owns the generated trimmed edge.
        face: FaceId,
        /// Loop slot containing the generated trimmed edge.
        loop_index: usize,
        /// Edge slot inside the loop for this generated trimmed edge.
        edge_index: usize,
    },
}

/// Provenance of a trimming edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum TrimmingEdgeProvenance {
    /// The edge comes directly from a source face boundary.
    SourceBoundary {
        /// Source boundary edge identity.
        edge: EdgeId,
    },
    /// The edge comes from a face-face section curve.
    SectionCurve {
        /// Section-curve identity shared by both owning faces.
        section_curve: SectionCurveId,
    },
    /// The edge is synthesized within trimming/rebuild and has no source edge.
    Generated,
}

impl TrimmingEdgeProvenance {
    /// Returns the topology key implied by this provenance for a specific trimmed edge slot.
    pub fn topology_key(
        self,
        face: FaceId,
        loop_index: usize,
        edge_index: usize,
    ) -> TrimmingTopologyKey {
        match self {
            Self::SourceBoundary { edge } => TrimmingTopologyKey::SourceBoundary(edge),
            Self::SectionCurve { section_curve } => {
                TrimmingTopologyKey::SectionCurve(section_curve)
            }
            Self::Generated => TrimmingTopologyKey::Generated {
                face,
                loop_index,
                edge_index,
            },
        }
    }

    /// Returns the source boundary identity when available.
    pub fn source_boundary(self) -> Option<EdgeId> {
        match self {
            Self::SourceBoundary { edge } => Some(edge),
            _ => None,
        }
    }

    /// Returns the section-curve identity when available.
    pub fn section_curve(self) -> Option<SectionCurveId> {
        match self {
            Self::SectionCurve { section_curve } => Some(section_curve),
            _ => None,
        }
    }
}

/// An oriented fragment edge after sewing equivalent endpoints together.
#[derive(Debug, Clone, PartialEq)]
pub struct SewnEdge {
    /// Source trimmed edge identity within the owning fragment.
    pub source: SewnEdgeSource,
    /// Source face that owns the trimmed edge.
    pub face: FaceId,
    /// Source trimming loop index within the split face.
    pub loop_index: usize,
    /// Source edge index within the trimming loop.
    pub edge_index: usize,
    /// Canonical start vertex after endpoint merging.
    pub start_vertex: VertexId,
    /// Canonical end vertex after endpoint merging.
    pub end_vertex: VertexId,
    /// Whether the edge orientation had to be reversed to follow the sewn path.
    pub reversed: bool,
    /// Topology identity carried from the source trimming edge.
    pub topology_key: TrimmingTopologyKey,
    /// Counterpart source edge when this edge was sewn to another fragment edge.
    pub sewn_pair: Option<SewnEdgePair>,
}

/// Identity of the source trimmed edge that produced a sewn edge record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SewnEdgeSource {
    /// Source face that owns the trimmed edge.
    pub face: FaceId,
    /// Source trimming loop index within the split face.
    pub loop_index: usize,
    /// Source edge index within the trimming loop.
    pub edge_index: usize,
    /// Unified topology identity for the source trimmed edge.
    pub topology_key: TrimmingTopologyKey,
}

/// Relation linking two fragment edges that were sewn together.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SewnEdgePair {
    /// Source identity for one sewn fragment edge.
    pub first: SewnEdgeSource,
    /// Source identity for the opposing sewn fragment edge.
    pub second: SewnEdgeSource,
}

impl SewnEdgePair {
    /// Creates a deterministic sewn-pair relation independent of input order.
    pub fn new(first: SewnEdgeSource, second: SewnEdgeSource) -> Self {
        if first <= second {
            Self { first, second }
        } else {
            Self {
                first: second,
                second: first,
            }
        }
    }
}

impl PartialOrd for SewnEdgeSource {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(other)) }
}

impl Ord for SewnEdgeSource {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.face
            .cmp(&other.face)
            .then(self.loop_index.cmp(&other.loop_index))
            .then(self.edge_index.cmp(&other.edge_index))
            .then(self.topology_key.cmp(&other.topology_key))
    }
}

/// A continuous oriented boundary assembled from sewn edges.
#[derive(Debug, Clone, PartialEq)]
pub struct SewnPath {
    /// Ordered sewn edges in traversal order.
    pub edges: Vec<SewnEdge>,
    /// Whether the path closes back on its start vertex.
    pub is_closed: bool,
}

/// A face fragment created while splitting a source face.
#[derive(Debug, Clone, PartialEq)]
pub struct SplitFace {
    /// Source face that was split.
    pub original_face: FaceId,
    /// Source operand rank for the fragment's owning face.
    pub operand_rank: u8,
    /// Trimming loops that bound the fragment.
    pub trimming_loops: Vec<TrimmingLoop>,
    /// Section curves that contributed split boundaries to this fragment.
    pub splitting_edges: Vec<SectionCurveId>,
    /// Representative point used for classification against the opposite operand.
    pub representative_point: Option<truck_base::cgmath64::Point3>,
    /// Classification state against the opposite operand.
    pub classification: Option<PointClassification>,
}

/// A trimming edge represented in a face's parameter space.
#[derive(Debug, Clone, PartialEq)]
pub struct TrimmingEdge {
    /// Explicit provenance for this trimming edge.
    pub provenance: TrimmingEdgeProvenance,
    /// Polyline vertices in face UV space.
    pub uv_points: Vec<Point2>,
}

/// A closed trimming loop for a split face.
#[derive(Debug, Clone, PartialEq)]
pub struct TrimmingLoop {
    /// Source face that owns this loop.
    pub face: FaceId,
    /// Fragment-local vertex identifiers aligned with the loop polyline traversal.
    /// Closed loops may repeat the first vertex id at the end so `vertex_ids` stays aligned
    /// with the closed `uv_points` polyline.
    pub vertex_ids: Vec<VertexId>,
    /// Ordered edges forming the loop.
    pub edges: Vec<TrimmingEdge>,
    /// Flattened closed polyline; first point equals last point within tolerance.
    pub uv_points: Vec<Point2>,
    /// Signed area in parameter space.
    pub signed_area: f64,
    /// Whether this loop is the outer boundary.
    pub is_outer: bool,
}

/// Vertex-vertex interference
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VVInterference {
    /// First vertex
    pub vertex1: VertexId,
    /// Second vertex
    pub vertex2: VertexId,
}

/// Vertex-edge interference
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VEInterference {
    /// Vertex
    pub vertex: VertexId,
    /// Edge
    pub edge: EdgeId,
    /// Parameter on edge
    pub parameter: f64,
}

/// Vertex-face interference
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VFInterference {
    /// Vertex
    pub vertex: VertexId,
    /// Face
    pub face: FaceId,
    /// Parameters on face surface
    pub parameters: (f64, f64),
}

/// Semantic kind of an edge-edge interference.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EEInterferenceKind {
    /// The edges meet at a single shared vertex.
    VertexHit,
    /// The edges overlap along a shared segment.
    OverlapHit,
}

/// Edge-edge interference
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EEInterference {
    /// First edge
    pub edge1: EdgeId,
    /// Second edge
    pub edge2: EdgeId,
    /// Parameter on the first edge curve
    pub t_a: f64,
    /// Parameter on the second edge curve
    pub t_b: f64,
    /// Distinguishes point contacts from shared overlap facts.
    pub kind: EEInterferenceKind,
}

/// Edge-face interference
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EFInterference {
    /// Edge
    pub edge: EdgeId,
    /// Face
    pub face: FaceId,
    /// Parameter on edge curve
    pub parameter: f64,
    /// Parameters on face surface
    pub surface_parameters: (f64, f64),
}

/// Face-face interference
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FFInterference {
    /// First face
    pub face1: FaceId,
    /// Second face
    pub face2: FaceId,
    /// Generated section curve
    pub section_curve: SectionCurveId,
}

/// A section curve generated by a face-face intersection.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionCurve {
    /// Curve identifier.
    pub id: SectionCurveId,
    /// The two source faces.
    pub faces: (FaceId, FaceId),
    /// First endpoint on the section, if open.
    pub start: VertexId,
    /// Second endpoint on the section, if open.
    pub end: VertexId,
    /// Polyline samples describing the section in model space.
    pub samples: Vec<truck_base::cgmath64::Point3>,
    /// Polyline samples of the section projected into each source face's parameter space.
    pub face_parameters: [(FaceId, Vec<Point2>); 2],
    /// Whether UV projection succeeded for each source face.
    pub face_projection_available: [(FaceId, bool); 2],
}

/// Interference table
#[derive(Debug, Default)]
pub struct InterferenceTable {
    /// Vertex-vertex interferences
    pub vv: Vec<VVInterference>,
    /// Vertex-edge interferences
    pub ve: Vec<VEInterference>,
    /// Vertex-face interferences
    pub vf: Vec<VFInterference>,
    /// Edge-edge interferences
    pub ee: Vec<EEInterference>,
    /// Edge-face interferences
    pub ef: Vec<EFInterference>,
    /// Face-face interferences
    pub ff: Vec<FFInterference>,
    /// Section curves generated by face-face intersections.
    pub section_curves: Vec<SectionCurve>,
    /// Trimming loops generated per face from boundaries and section edges.
    pub trimming_loops: Vec<TrimmingLoop>,
    /// Split face fragments with provenance information.
    pub split_faces: Vec<SplitFace>,
    /// Vertex clusters merged during topology rebuild.
    pub merged_vertices: Vec<MergedVertex>,
    /// Oriented boundaries reconstructed from sewn fragment edges.
    pub sewn_paths: Vec<SewnPath>,
}

impl InterferenceTable {
    /// Store a vertex-vertex interference.
    pub fn push_vv(&mut self, interference: VVInterference) { self.vv.push(interference); }

    /// Store a vertex-edge interference.
    pub fn push_ve(&mut self, interference: VEInterference) { self.ve.push(interference); }

    /// Store a vertex-face interference.
    pub fn push_vf(&mut self, interference: VFInterference) { self.vf.push(interference); }

    /// Store an edge-edge interference.
    pub fn push_ee(&mut self, interference: EEInterference) { self.ee.push(interference); }

    /// Store an edge-face interference.
    pub fn push_ef(&mut self, interference: EFInterference) { self.ef.push(interference); }

    /// Store a face-face interference.
    pub fn push_ff(&mut self, interference: FFInterference) { self.ff.push(interference); }

    /// Store a section curve.
    pub fn push_section_curve(&mut self, section_curve: SectionCurve) {
        self.section_curves.push(section_curve);
    }

    /// Store a trimming loop.
    pub fn push_trimming_loop(&mut self, trimming_loop: TrimmingLoop) {
        self.trimming_loops.push(trimming_loop);
    }

    /// Store a split face fragment.
    pub fn push_split_face(&mut self, split_face: SplitFace) { self.split_faces.push(split_face); }

    /// Store a merged vertex cluster.
    pub fn push_merged_vertex(&mut self, merged_vertex: MergedVertex) {
        self.merged_vertices.push(merged_vertex);
    }

    /// Store a sewn boundary path.
    pub fn push_sewn_path(&mut self, sewn_path: SewnPath) { self.sewn_paths.push(sewn_path); }

    /// Borrow all vertex-vertex interferences.
    pub fn vv(&self) -> &[VVInterference] { &self.vv }

    /// Borrow all vertex-edge interferences.
    pub fn ve(&self) -> &[VEInterference] { &self.ve }

    /// Borrow all vertex-face interferences.
    pub fn vf(&self) -> &[VFInterference] { &self.vf }

    /// Borrow all edge-edge interferences.
    pub fn ee(&self) -> &[EEInterference] { &self.ee }

    /// Borrow all edge-face interferences.
    pub fn ef(&self) -> &[EFInterference] { &self.ef }

    /// Borrow all face-face interferences.
    pub fn ff(&self) -> &[FFInterference] { &self.ff }

    /// Borrow all section curves.
    pub fn section_curves(&self) -> &[SectionCurve] { &self.section_curves }

    /// Borrow all trimming loops.
    pub fn trimming_loops(&self) -> &[TrimmingLoop] { &self.trimming_loops }

    /// Borrow all split face fragments.
    pub fn split_faces(&self) -> &[SplitFace] { &self.split_faces }

    /// Borrow all merged vertex clusters.
    pub fn merged_vertices(&self) -> &[MergedVertex] { &self.merged_vertices }

    /// Borrow all sewn boundary paths.
    pub fn sewn_paths(&self) -> &[SewnPath] { &self.sewn_paths }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stores_vv_interference_records() {
        let mut table = InterferenceTable::default();
        let interference = VVInterference {
            vertex1: VertexId(1),
            vertex2: VertexId(2),
        };

        table.push_vv(interference);

        assert_eq!(table.vv(), &[interference]);
    }

    #[test]
    fn stores_ve_interference_records() {
        let mut table = InterferenceTable::default();
        let interference = VEInterference {
            vertex: VertexId(1),
            edge: EdgeId(2),
            parameter: 0.25,
        };

        table.push_ve(interference);

        assert_eq!(table.ve(), &[interference]);
    }

    #[test]
    fn stores_vf_interference_records() {
        let mut table = InterferenceTable::default();
        let interference = VFInterference {
            vertex: VertexId(1),
            face: FaceId(2),
            parameters: (0.25, 0.75),
        };

        table.push_vf(interference);

        assert_eq!(table.vf(), &[interference]);
    }

    #[test]
    fn stores_ee_interference_records() {
        let mut table = InterferenceTable::default();
        let interference = EEInterference {
            edge1: EdgeId(1),
            edge2: EdgeId(2),
            t_a: 0.25,
            t_b: 0.75,
            kind: EEInterferenceKind::VertexHit,
        };

        table.push_ee(interference);

        assert_eq!(table.ee(), &[interference]);
    }

    #[test]
    fn stores_ee_interference_semantics() {
        let mut table = InterferenceTable::default();
        let vertex = EEInterference {
            edge1: EdgeId(1),
            edge2: EdgeId(2),
            t_a: 0.25,
            t_b: 0.75,
            kind: EEInterferenceKind::VertexHit,
        };
        let overlap = EEInterference {
            edge1: EdgeId(1),
            edge2: EdgeId(2),
            t_a: 0.0,
            t_b: 0.5,
            kind: EEInterferenceKind::OverlapHit,
        };

        table.push_ee(vertex);
        table.push_ee(overlap);

        assert_eq!(table.ee(), &[vertex, overlap]);
    }

    #[test]
    fn stores_ef_interference_records() {
        let mut table = InterferenceTable::default();
        let interference = EFInterference {
            edge: EdgeId(1),
            face: FaceId(2),
            parameter: 0.25,
            surface_parameters: (0.25, 0.75),
        };

        table.push_ef(interference);

        assert_eq!(table.ef(), &[interference]);
    }

    #[test]
    fn stores_ff_interference_and_section_curve_records() {
        let mut table = InterferenceTable::default();
        let section_curve = SectionCurve {
            id: SectionCurveId(3),
            faces: (FaceId(1), FaceId(2)),
            start: VertexId(4),
            end: VertexId(5),
            samples: vec![truck_base::cgmath64::Point3::new(0.0, 0.0, 0.0)],
            face_parameters: [
                (FaceId(1), vec![Point2::new(0.0, 0.0)]),
                (FaceId(2), vec![Point2::new(1.0, 1.0)]),
            ],
            face_projection_available: [(FaceId(1), true), (FaceId(2), true)],
        };
        let interference = FFInterference {
            face1: FaceId(1),
            face2: FaceId(2),
            section_curve: SectionCurveId(3),
        };

        table.push_section_curve(section_curve.clone());
        table.push_ff(interference);

        assert_eq!(table.section_curves(), &[section_curve]);
        assert_eq!(table.ff(), &[interference]);
    }

    #[test]
    fn stores_trimming_loop_records() {
        let mut table = InterferenceTable::default();
        let trimming_loop = TrimmingLoop {
            face: FaceId(1),
            vertex_ids: vec![VertexId(1), VertexId(2)],
            edges: vec![TrimmingEdge {
                provenance: TrimmingEdgeProvenance::SectionCurve {
                    section_curve: SectionCurveId(2),
                },
                uv_points: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 0.0),
                Point2::new(0.0, 0.0),
            ],
            signed_area: 1.0,
            is_outer: true,
        };

        table.push_trimming_loop(trimming_loop.clone());

        assert_eq!(table.trimming_loops(), &[trimming_loop]);
    }

    #[test]
    fn stores_split_face_records() {
        let mut table = InterferenceTable::default();
        let loop_record = TrimmingLoop {
            face: FaceId(1),
            vertex_ids: vec![VertexId(1), VertexId(2)],
            edges: vec![TrimmingEdge {
                provenance: TrimmingEdgeProvenance::SectionCurve {
                    section_curve: SectionCurveId(2),
                },
                uv_points: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 0.0),
                Point2::new(0.0, 0.0),
            ],
            signed_area: 1.0,
            is_outer: true,
        };
        let split_face = SplitFace {
            original_face: FaceId(1),
            operand_rank: 0,
            trimming_loops: vec![loop_record],
            splitting_edges: vec![SectionCurveId(2)],
            representative_point: Some(truck_base::cgmath64::Point3::new(0.5, 0.5, 0.0)),
            classification: Some(PointClassification::Inside),
        };

        table.push_split_face(split_face.clone());

        assert_eq!(table.split_faces(), &[split_face]);
    }

    #[test]
    fn stores_merged_vertex_records() {
        let mut table = InterferenceTable::default();
        let merged = MergedVertex {
            id: VertexId(9),
            original_vertices: vec![VertexId(9), VertexId(11)],
            point: truck_base::cgmath64::Point3::new(1.0, 2.0, 3.0),
        };

        table.push_merged_vertex(merged.clone());

        assert_eq!(table.merged_vertices(), &[merged]);
    }

    #[test]
    fn stores_sewn_path_records() {
        let mut table = InterferenceTable::default();
        let path = SewnPath {
            edges: vec![SewnEdge {
                source: SewnEdgeSource {
                    face: FaceId(2),
                    loop_index: 0,
                    edge_index: 1,
                    topology_key: TrimmingTopologyKey::SourceBoundary(EdgeId(9)),
                },
                face: FaceId(2),
                loop_index: 0,
                edge_index: 1,
                start_vertex: VertexId(4),
                end_vertex: VertexId(5),
                reversed: false,
                topology_key: TrimmingTopologyKey::SectionCurve(SectionCurveId(8)),
                sewn_pair: Some(SewnEdgePair::new(
                    SewnEdgeSource {
                        face: FaceId(2),
                        loop_index: 0,
                        edge_index: 1,
                        topology_key: TrimmingTopologyKey::SourceBoundary(EdgeId(9)),
                    },
                    SewnEdgeSource {
                        face: FaceId(3),
                        loop_index: 1,
                        edge_index: 0,
                        topology_key: TrimmingTopologyKey::SourceBoundary(EdgeId(10)),
                    },
                )),
            }],
            is_closed: false,
        };

        table.push_sewn_path(path.clone());

        assert_eq!(table.sewn_paths(), &[path]);
    }

    #[test]
    fn sewn_edge_pair_is_order_independent() {
        let lhs = SewnEdgeSource {
            face: FaceId(4),
            loop_index: 2,
            edge_index: 1,
            topology_key: TrimmingTopologyKey::SourceBoundary(EdgeId(12)),
        };
        let rhs = SewnEdgeSource {
            face: FaceId(1),
            loop_index: 0,
            edge_index: 3,
            topology_key: TrimmingTopologyKey::SourceBoundary(EdgeId(5)),
        };

        assert_eq!(SewnEdgePair::new(lhs, rhs), SewnEdgePair::new(rhs, lhs));
    }

    #[test]
    fn trimming_provenance_maps_to_topology_key_variants() {
        let source = TrimmingEdgeProvenance::SourceBoundary { edge: EdgeId(7) };
        let section = TrimmingEdgeProvenance::SectionCurve {
            section_curve: SectionCurveId(3),
        };
        let generated = TrimmingEdgeProvenance::Generated;

        assert_eq!(
            source.topology_key(FaceId(1), 2, 3),
            TrimmingTopologyKey::SourceBoundary(EdgeId(7))
        );
        assert_eq!(
            section.topology_key(FaceId(1), 2, 3),
            TrimmingTopologyKey::SectionCurve(SectionCurveId(3))
        );
        assert_eq!(
            generated.topology_key(FaceId(1), 2, 3),
            TrimmingTopologyKey::Generated {
                face: FaceId(1),
                loop_index: 2,
                edge_index: 3,
            }
        );
    }
}
