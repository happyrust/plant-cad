//! Error types

/// Boolean operation error
#[derive(Debug, thiserror::Error)]
pub enum BopError {
    /// Invalid tolerance configuration
    #[error("invalid tolerance configuration")]
    InvalidTolerance,
    /// Unsupported geometry
    #[error("unsupported geometry")]
    UnsupportedGeometry,
    /// Topology invariant broken
    #[error("topology invariant broken")]
    TopologyInvariantBroken,
    /// Missing shape
    #[error("missing shape: {0:?}")]
    MissingShape(crate::ShapeId),
    /// Internal invariant violated
    #[error("internal invariant violated: {0}")]
    InternalInvariant(&'static str),
    /// Not implemented
    #[error("not implemented: {0}")]
    NotImplemented(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn displays_not_implemented_error() {
        let err = BopError::NotImplemented("phase-1 stub");
        assert!(err.to_string().contains("phase-1 stub"));
    }
}
