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

mod bopds;
mod error;
mod options;
mod pipeline;

pub use bopds::{BopDs, CommonBlockId, EdgeId, FaceId, PaveBlockId, SectionCurveId, ShapeId, VertexId};
pub use error::BopError;
pub use options::BopOptions;
pub use pipeline::BooleanOp;
