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
    /// Optional source section curve identifier when the edge came from a section.
    pub section_curve: Option<SectionCurveId>,
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
    /// Optional source topological edge identifier, when available.
    pub original_edge: Option<EdgeId>,
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
            .then(self.original_edge.cmp(&other.original_edge))
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

/// Provenance of a trimming edge: which input entity it originates from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TrimmingEdgeSource {
    /// Edge comes from an original face boundary in the input solid.
    OriginalBoundaryEdge(EdgeId),
    /// Edge comes from a segment of an intersection section curve.
    SectionSegment {
        /// The parent section curve.
        curve: SectionCurveId,
        /// Zero-based index of the segment within the polyline discretization.
        segment_index: u32,
    },
    /// Edge has no traceable source (e.g. synthesized during loop repair).
    Unattributed,
}

/// A trimming edge represented in a face's parameter space.
#[derive(Debug, Clone, PartialEq)]
pub struct TrimmingEdge {
    /// Where this edge originates from.
    pub source: TrimmingEdgeSource,
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
        };

        table.push_ee(interference);

        assert_eq!(table.ee(), &[interference]);
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
                source: TrimmingEdgeSource::SectionSegment { curve: SectionCurveId(2), segment_index: 0 },
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
                source: TrimmingEdgeSource::SectionSegment { curve: SectionCurveId(2), segment_index: 0 },
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
                    original_edge: Some(EdgeId(9)),
                },
                face: FaceId(2),
                loop_index: 0,
                edge_index: 1,
                start_vertex: VertexId(4),
                end_vertex: VertexId(5),
                reversed: false,
                section_curve: Some(SectionCurveId(8)),
                sewn_pair: Some(SewnEdgePair::new(
                    SewnEdgeSource {
                        face: FaceId(2),
                        loop_index: 0,
                        edge_index: 1,
                        original_edge: Some(EdgeId(9)),
                    },
                    SewnEdgeSource {
                        face: FaceId(3),
                        loop_index: 1,
                        edge_index: 0,
                        original_edge: Some(EdgeId(10)),
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
            original_edge: Some(EdgeId(12)),
        };
        let rhs = SewnEdgeSource {
            face: FaceId(1),
            loop_index: 0,
            edge_index: 3,
            original_edge: Some(EdgeId(5)),
        };

        assert_eq!(SewnEdgePair::new(lhs, rhs), SewnEdgePair::new(rhs, lhs));
    }
}
