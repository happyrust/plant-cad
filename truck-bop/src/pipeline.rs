//! Boolean operation pipeline

/// Boolean operation type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOp {
    /// Union/Fuse
    Fuse,
    /// Intersection/Common
    Common,
    /// Subtraction/Cut
    Cut,
    /// Section
    Section,
}

/// Pipeline execution report
#[derive(Debug)]
pub struct PipelineReport;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boolean_op_debug_name_is_stable() {
        assert_eq!(format!("{:?}", BooleanOp::Common), "Common");
    }
}
