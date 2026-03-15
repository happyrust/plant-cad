//! Pave structure - represents a vertex on an edge with parameter

use crate::{BopError, VertexId};

/// Pave - vertex on an edge with parameter
#[derive(Debug)]
pub struct Pave {
    /// Vertex ID
    pub vertex: VertexId,
    /// Parameter on edge
    pub parameter: f64,
    /// Tolerance
    pub tolerance: f64,
}

impl Pave {
    /// Create a new pave
    pub fn new(vertex: VertexId, parameter: f64, tolerance: f64) -> Result<Self, BopError> {
        if tolerance <= 0.0 {
            return Err(BopError::InvalidTolerance);
        }
        Ok(Self {
            vertex,
            parameter,
            tolerance,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pave_orders_by_parameter() {
        let a = Pave::new(VertexId(0), 0.25, 1.0e-6).unwrap();
        let b = Pave::new(VertexId(1), 0.75, 1.0e-6).unwrap();
        assert!(a.parameter < b.parameter);
    }
}
