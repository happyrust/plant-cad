//! Edge-Edge intersection detection.

use crate::{BopError, BopOptions, EdgeId};

/// Detects intersection between two edges (stub).
pub fn intersect_ee(
    _edge_a_id: EdgeId,
    _edge_b_id: EdgeId,
    _options: &BopOptions,
) -> Result<Vec<(EdgeId, EdgeId, f64, f64)>, BopError> {
    Ok(Vec::new())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ee_intersection_stub_returns_empty() {
        let opts = BopOptions::default();
        let result = intersect_ee(EdgeId(0), EdgeId(1), &opts).unwrap();
        assert!(result.is_empty());
    }
}
