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
mod options;
mod pipeline;

pub use bounding::BoundingProvider;
pub use bopds::{BopDs, CommonBlockId, EdgeId, FaceId, PaveBlockId, SectionCurveId, ShapeId, VertexId};
pub use broad_phase::{CandidatePairs, generate_candidate_pairs};
pub use error::BopError;
pub use options::BopOptions;
pub use pipeline::BooleanOp;

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
