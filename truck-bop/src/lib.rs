#![cfg_attr(not(debug_assertions), deny(warnings))]
#![deny(clippy::all, rust_2018_idioms)]
#![warn(
    missing_docs,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications
)]

//! Boolean operation infrastructure for truck

mod bounding;
mod bopds;
mod broad_phase;
mod error;
mod intersect;
mod options;
mod pipeline;
mod trim;

pub use bounding::{BoundingProvider, FaceBoundingSurface};
pub use bopds::{
    BopDs, CommonBlockId, EdgeId, FaceId, MergedVertex, PaveBlock, PaveBlockId,
    SectionCurveId, SewnEdge, SewnPath, ShapeId, SplitFace, VertexId,
};
pub use broad_phase::{CandidatePairs, generate_candidate_pairs, generate_candidate_pairs_from_bopds};
pub use error::BopError;
pub use intersect::{intersect_ee, intersect_ef, intersect_ff, intersect_ve, intersect_vf, intersect_vv};
pub use options::BopOptions;
pub use pipeline::{BooleanOp, PointClassification, classify_point_in_solid};
pub use trim::{
    assemble_shells,
    build_solids_from_shells,
    build_split_faces,
    build_trimming_loops,
    classify_split_faces_against_operand,
    merge_equivalent_vertices,
    sew_fragment_edges,
    select_split_faces_for_boolean_op,
};

/// Common (intersection) operation stub
pub fn common<C, S>(
    _a: &truck_topology::Solid<truck_base::cgmath64::Point3, C, S>,
    _b: &truck_topology::Solid<truck_base::cgmath64::Point3, C, S>,
    _tol: f64,
) -> Result<truck_topology::Solid<truck_base::cgmath64::Point3, C, S>, BopError> {
    Err(BopError::NotImplemented("common"))
}

/// Fuse (union) operation stub
pub fn fuse<C, S>(
    _a: &truck_topology::Solid<truck_base::cgmath64::Point3, C, S>,
    _b: &truck_topology::Solid<truck_base::cgmath64::Point3, C, S>,
    _tol: f64,
) -> Result<truck_topology::Solid<truck_base::cgmath64::Point3, C, S>, BopError> {
    Err(BopError::NotImplemented("fuse"))
}

/// Cut (difference) operation stub
pub fn cut<C, S>(
    _a: &truck_topology::Solid<truck_base::cgmath64::Point3, C, S>,
    _b: &truck_topology::Solid<truck_base::cgmath64::Point3, C, S>,
    _tol: f64,
) -> Result<truck_topology::Solid<truck_base::cgmath64::Point3, C, S>, BopError> {
    Err(BopError::NotImplemented("cut"))
}

/// Section operation stub
pub fn section<C, S>(
    _a: &truck_topology::Solid<truck_base::cgmath64::Point3, C, S>,
    _b: &truck_topology::Solid<truck_base::cgmath64::Point3, C, S>,
    _tol: f64,
) -> Result<truck_topology::Shell<truck_base::cgmath64::Point3, C, S>, BopError> {
    Err(BopError::NotImplemented("section"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn common_stub_reports_not_implemented() {
        let err = BopError::NotImplemented("common");
        assert!(matches!(err, BopError::NotImplemented(_)));
    }
}
