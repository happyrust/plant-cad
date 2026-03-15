//! Edge-Face intersection detection.

use crate::{BopError, BopOptions, EdgeId, FaceId};

/// Detects intersection between an edge and a face (stub).
pub fn intersect_ef(
    _edge_id: EdgeId,
    _face_id: FaceId,
    _options: &BopOptions,
) -> Result<Vec<(EdgeId, FaceId, f64)>, BopError> {
    Ok(Vec::new())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ef_intersection_stub_returns_empty() {
        let opts = BopOptions::default();
        let result = intersect_ef(EdgeId(0), FaceId(0), &opts).unwrap();
        assert!(result.is_empty());
    }
}
