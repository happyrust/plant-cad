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
mod bounding;
mod broad_phase;
mod error;
mod intersect;
mod options;
pub mod pave_filler;
mod pipeline;
pub use pave_filler::{BopOperationReport, PaveFiller};
mod trim;

pub use bopds::{
    BopDs, CommonBlockId, EdgeId, FaceId, MergedVertex, PaveBlock, PaveBlockId, SectionCurveId,
    SewnEdge, SewnPath, ShapeId, SplitEdgeId, SplitEdgeRecord, SplitFace, TrimmingEdge,
    TrimmingEdgeProvenance, TrimmingLoop, TrimmingTopologyKey, VertexId,
};
pub use bounding::{BoundingProvider, FaceBoundingSurface};
pub use broad_phase::{
    generate_candidate_pairs, generate_candidate_pairs_from_bopds, CandidatePairs,
};
pub use error::BopError;
pub use intersect::{
    intersect_ee, intersect_ee_into_bopds, intersect_ef, intersect_ff, intersect_ve, intersect_vf,
    intersect_vv,
};
pub use options::BopOptions;
pub use pipeline::{classify_point_in_solid, run_boolean_pipeline, BooleanOp, PointClassification};
pub use trim::{
    assemble_shells, build_solids_from_shells, build_split_faces, build_trimming_loops,
    classify_split_faces_against_operand, merge_equivalent_vertices,
    select_split_faces_for_boolean_op, sew_fragment_edges,
};
use truck_base::cgmath64::{EuclideanSpace, Point3};
use truck_geotrait::{
    BoundedCurve, Invertible, ParametricCurve, ParametricSurface, ParametricSurface3D,
    SearchNearestParameter, SearchParameter, D1, D2,
};
use truck_topology::Solid;

/// Common (intersection) operation stub
pub fn common<C, S>(
    a: &Solid<Point3, C, S>,
    b: &Solid<Point3, C, S>,
    tol: f64,
) -> Result<Solid<Point3, C, S>, BopError>
where
    C: Clone
        + 'static
        + BoundedCurve<Point = Point3>
        + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + Invertible
        + SearchNearestParameter<D1, Point = Point3>,
    S: Clone
        + 'static
        + FaceBoundingSurface
        + Invertible
        + ParametricSurface<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + ParametricSurface3D
        + SearchParameter<D2, Point = Point3>
        + SearchNearestParameter<D2, Point = Point3>,
    truck_modeling::Line<Point3>: truck_modeling::ToSameGeometry<C>,
{
    if !solids_bboxes_overlap(a, b) {
        return Err(BopError::UnsupportedGeometry);
    }
    let shells = run_boolean_pipeline(BooleanOp::Common, a, b, tol)?;
    if shells.is_empty() {
        return Err(BopError::UnsupportedGeometry);
    }
    let mut solids = build_solids_from_shells(shells)?;
    if solids.len() != 1 {
        return Err(BopError::UnsupportedGeometry);
    }
    solids.pop().ok_or(BopError::UnsupportedGeometry)
}

/// Fuse (union) operation stub
pub fn fuse<C, S>(
    a: &Solid<Point3, C, S>,
    b: &Solid<Point3, C, S>,
    tol: f64,
) -> Result<Solid<Point3, C, S>, BopError>
where
    C: Clone
        + 'static
        + BoundedCurve<Point = Point3>
        + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + Invertible
        + SearchNearestParameter<D1, Point = Point3>,
    S: Clone
        + 'static
        + FaceBoundingSurface
        + Invertible
        + ParametricSurface<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + ParametricSurface3D
        + SearchParameter<D2, Point = Point3>
        + SearchNearestParameter<D2, Point = Point3>,
    truck_modeling::Line<Point3>: truck_modeling::ToSameGeometry<C>,
{
    if !solids_bboxes_overlap(a, b) {
        return Err(BopError::UnsupportedGeometry);
    }
    let shells = run_boolean_pipeline(BooleanOp::Fuse, a, b, tol)?;
    if shells.is_empty() {
        return Err(BopError::UnsupportedGeometry);
    }
    let mut solids = build_solids_from_shells(shells)?;
    if solids.len() != 1 {
        return Err(BopError::UnsupportedGeometry);
    }
    solids.pop().ok_or(BopError::UnsupportedGeometry)
}

/// Cut (difference) operation stub
pub fn cut<C, S>(
    a: &Solid<Point3, C, S>,
    b: &Solid<Point3, C, S>,
    tol: f64,
) -> Result<Solid<Point3, C, S>, BopError>
where
    C: Clone
        + 'static
        + BoundedCurve<Point = Point3>
        + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + Invertible
        + SearchNearestParameter<D1, Point = Point3>,
    S: Clone
        + 'static
        + FaceBoundingSurface
        + Invertible
        + ParametricSurface<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + ParametricSurface3D
        + SearchParameter<D2, Point = Point3>
        + SearchNearestParameter<D2, Point = Point3>,
    truck_modeling::Line<Point3>: truck_modeling::ToSameGeometry<C>,
{
    if !solids_bboxes_overlap(a, b) {
        return Ok(a.clone());
    }
    let shells = run_boolean_pipeline(BooleanOp::Cut, a, b, tol)?;
    if shells.is_empty() {
        return Ok(a.clone());
    }
    let mut solids = build_solids_from_shells(shells)?;
    if solids.len() != 1 {
        return Err(BopError::UnsupportedGeometry);
    }
    solids.pop().ok_or(BopError::UnsupportedGeometry)
}

/// Section operation stub
pub fn section<C, S>(
    a: &Solid<Point3, C, S>,
    b: &Solid<Point3, C, S>,
    tol: f64,
) -> Result<truck_topology::Shell<Point3, C, S>, BopError>
where
    C: Clone
        + 'static
        + BoundedCurve<Point = Point3>
        + ParametricCurve<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + Invertible
        + SearchNearestParameter<D1, Point = Point3>,
    S: Clone
        + 'static
        + FaceBoundingSurface
        + Invertible
        + ParametricSurface<Point = Point3, Vector = <Point3 as EuclideanSpace>::Diff>
        + ParametricSurface3D
        + SearchParameter<D2, Point = Point3>
        + SearchNearestParameter<D2, Point = Point3>,
    truck_modeling::Line<Point3>: truck_modeling::ToSameGeometry<C>,
{
    let mut shells = run_boolean_pipeline(BooleanOp::Section, a, b, tol)?;
    if shells.len() != 1 {
        return Err(BopError::UnsupportedGeometry);
    }
    shells.pop().ok_or(BopError::UnsupportedGeometry)
}

fn solids_bboxes_overlap<C, S>(lhs: &Solid<Point3, C, S>, rhs: &Solid<Point3, C, S>) -> bool {
    let lhs_bbox = truck_base::bounding_box::BoundingBox::from_iter(
        lhs.boundaries()
            .iter()
            .flat_map(|shell| shell.vertex_iter().map(|vertex| vertex.point())),
    );
    let rhs_bbox = truck_base::bounding_box::BoundingBox::from_iter(
        rhs.boundaries()
            .iter()
            .flat_map(|shell| shell.vertex_iter().map(|vertex| vertex.point())),
    );

    if lhs_bbox.is_empty() || rhs_bbox.is_empty() {
        return false;
    }

    !(lhs_bbox ^ rhs_bbox).is_empty()
}

#[cfg(test)]
mod tests {
    use super::*;
    use truck_base::bounding_box::BoundingBox;
    use truck_base::cgmath64::Point3;
    use truck_modeling::{primitive, Curve, Surface};
    use truck_topology::{Shell, Solid};

    #[test]
    fn common_api_is_reachable_for_standard_types() {
        let _fn: fn(
            &Solid<Point3, Curve, Surface>,
            &Solid<Point3, Curve, Surface>,
            f64,
        ) -> Result<Solid<Point3, Curve, Surface>, BopError> = common::<Curve, Surface>;
    }

    #[test]
    fn section_api_is_reachable_for_standard_types() {
        let _fn: fn(
            &Solid<Point3, Curve, Surface>,
            &Solid<Point3, Curve, Surface>,
            f64,
        ) -> Result<Shell<Point3, Curve, Surface>, BopError> = section::<Curve, Surface>;
    }

    #[test]
    fn section_api_returns_unsupported_for_non_intersecting_operands() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(5.0, 5.0, 5.0),
            Point3::new(6.0, 6.0, 6.0),
        ]));

        assert!(matches!(
            section(&lhs, &rhs, 1.0e-6),
            Err(BopError::UnsupportedGeometry)
        ));
    }

    #[test]
    fn section_api_returns_unsupported_when_multiple_shells_exist() {
        let mut lhs_boundaries = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]))
        .into_boundaries();
        lhs_boundaries.extend(
            primitive::cuboid(BoundingBox::from_iter([
                Point3::new(2.5, 0.0, 0.0),
                Point3::new(3.5, 1.0, 1.0),
            ]))
            .into_boundaries(),
        );
        let lhs: Solid<Point3, Curve, Surface> = Solid::new(lhs_boundaries);

        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(-1.0, 0.5, -0.5),
            Point3::new(5.0, 1.5, 1.5),
        ]));

        assert!(matches!(
            section(&lhs, &rhs, 1.0e-6),
            Err(BopError::UnsupportedGeometry)
        ));
    }

    #[test]
    fn section_api_returns_unsupported_when_assembly_fails() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 2.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(1.0, 0.5, 0.0),
            Point3::new(3.0, 1.5, 2.0),
        ]));

        assert!(matches!(
            section(&lhs, &rhs, 1.0e-6),
            Err(BopError::UnsupportedGeometry)
        ));
    }

    #[test]
    fn section_api_disjoint_operands_keep_scheme_a_contract() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(5.0, 5.0, 5.0),
            Point3::new(6.0, 6.0, 6.0),
        ]));

        assert!(matches!(
            section(&lhs, &rhs, 1.0e-6),
            Err(BopError::UnsupportedGeometry)
        ));

        assert!(matches!(
            common(&lhs, &rhs, 1.0e-6),
            Err(BopError::UnsupportedGeometry)
        ));
        assert!(cut(&lhs, &rhs, 1.0e-6).is_ok());
        assert!(matches!(
            fuse(&lhs, &rhs, 1.0e-6),
            Err(BopError::UnsupportedGeometry)
        ));
    }

    #[test]
    fn common_cut_fuse_smoke_no_notimplemented() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 2.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(1.0, 0.5, 0.0),
            Point3::new(3.0, 1.5, 2.0),
        ]));

        assert!(!matches!(
            common(&lhs, &rhs, 1.0e-6),
            Err(BopError::NotImplemented(_))
        ));
        assert!(!matches!(
            cut(&lhs, &rhs, 1.0e-6),
            Err(BopError::NotImplemented(_))
        ));
        assert!(!matches!(
            fuse(&lhs, &rhs, 1.0e-6),
            Err(BopError::NotImplemented(_))
        ));
    }

    #[test]
    fn disjoint_operands_follow_scheme_a_common_fuse_cut_contract() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(5.0, 5.0, 5.0),
            Point3::new(6.0, 6.0, 6.0),
        ]));

        assert!(matches!(
            common(&lhs, &rhs, 1.0e-6),
            Err(BopError::UnsupportedGeometry)
        ));

        let cut_result = cut(&lhs, &rhs, 1.0e-6).unwrap();
        assert_eq!(cut_result.boundaries().len(), lhs.boundaries().len());

        assert!(matches!(
            fuse(&lhs, &rhs, 1.0e-6),
            Err(BopError::UnsupportedGeometry)
        ));
    }
}
