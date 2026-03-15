//! Vertex-Face intersection detection.

use crate::{BopError, BopOptions, VertexId};
use truck_topology::Vertex;
use truck_base::cgmath64::Point3;

/// Detects if a vertex lies on or near a face surface (stub).
pub fn intersect_vf(
    _vertex_id: VertexId,
    _vertex: &Vertex<Point3>,
    _options: &BopOptions,
) -> Result<Option<VertexId>, BopError> {
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use truck_modeling::builder;

    #[test]
    fn vf_intersection_stub_returns_none() {
        let vertex = builder::vertex(Point3::new(0.0, 0.0, 0.0));
        let opts = BopOptions::default();
        let result = intersect_vf(VertexId(0), &vertex, &opts).unwrap();
        assert!(result.is_none());
    }
}
