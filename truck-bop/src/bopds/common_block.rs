//! Common block structure

use crate::{FaceId, PaveBlockId};

/// Common block - represents a shared edge segment between faces
#[allow(dead_code)]
#[derive(Debug)]
pub struct CommonBlock {
    /// Pave block
    pub pave_block: PaveBlockId,
    /// Faces sharing this block
    pub faces: Vec<FaceId>,
}
