//! Configuration options

use crate::BopError;

/// Boolean operation options
#[derive(Debug, Clone)]
pub struct BopOptions {
    /// Geometric tolerance for proximity tests
    pub geometric_tol: f64,
    /// Parametric tolerance for curve/surface parameters
    pub parametric_tol: f64,
    /// Angular tolerance for orientation tests
    pub angular_tol: f64,
    /// Approximation tolerance for meshing
    pub approximation_tol: f64,
    /// Enable face-face intersection fallback
    pub enable_ff_fallback: bool,
}

impl Default for BopOptions {
    fn default() -> Self {
        Self {
            geometric_tol: 1.0e-6,
            parametric_tol: 1.0e-8,
            angular_tol: 1.0e-10,
            approximation_tol: 1.0e-4,
            enable_ff_fallback: true,
        }
    }
}

impl BopOptions {
    /// Validate the options
    pub fn validate(&self) -> Result<(), BopError> {
        if self.geometric_tol <= 0.0 {
            return Err(BopError::InvalidTolerance);
        }
        if self.parametric_tol <= 0.0 {
            return Err(BopError::InvalidTolerance);
        }
        if self.angular_tol <= 0.0 {
            return Err(BopError::InvalidTolerance);
        }
        if self.approximation_tol <= 0.0 {
            return Err(BopError::InvalidTolerance);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_options_validate() { BopOptions::default().validate().unwrap(); }

    #[test]
    fn rejects_non_positive_tolerance() {
        let opts = BopOptions {
            geometric_tol: 0.0,
            ..BopOptions::default()
        };
        assert!(opts.validate().is_err());
    }
}
