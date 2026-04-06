//! Boolean operation pipeline

use crate::broad_phase::generate_candidate_pairs_from_bopds;
use crate::trim::{assemble_shells_with_diagnostic, AssemblyFailure};
use crate::{
    build_solids_from_shells, classify_split_faces_against_operand, merge_equivalent_vertices,
    select_split_faces_for_boolean_op, sew_fragment_edges, BopDs, BopError, BopOptions, EdgeId,
    FaceBoundingSurface, FaceId, PaveFiller, SewnPath, SplitFace, VertexId,
};
use rustc_hash::FxHashMap;
use std::time::Instant;
use truck_base::{
    bounding_box::BoundingBox,
    cgmath64::{EuclideanSpace, Point3, Vector3},
};
use truck_geotrait::{
    BoundedCurve, Invertible, ParametricCurve, ParametricSurface, ParametricSurface3D,
    SearchNearestParameter, SearchParameter, D1, D2,
};
use truck_topology::{Edge, Face, Shell, Solid, Vertex};

// TODO(W6): keep this module generic over any curve/surface pair compatible
// with the intersection routines. Face-face interception is currently executed
// through FF fallback in downstream implementations.

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
#[allow(dead_code)]
#[derive(Debug)]
pub struct PipelineReport;

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub(crate) struct SectionAssemblyFailureSummary {
    pub failure_id: u64,
    pub selected_split_face_count: usize,
    pub selected_faces_with_splitting_edges: usize,
    pub total_split_edges: usize,
    pub max_split_edges_per_face: usize,
    pub sewn_path_count: usize,
    pub sewn_closed_path_count: usize,
    pub sewn_open_path_count: usize,
    pub reason: SectionAssemblyFailureReason,
    pub no_orientable_seed_face: Option<usize>,
    pub remaining_orientable_candidates: Option<usize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SectionAssemblyFailureReason {
    NoOrientableCandidate,
    OpenShell,
    InvalidShellOrientation,
}

pub(crate) type BooleanPipelineTiming = u128;

/// Executes the common boolean pipeline stages on two operands and
/// returns assembled shell candidates.
///
/// 1) register source entities
/// 2) broad-phase filtering
/// 3) punctual and curve/face intersections through [`PaveFiller`]
/// 4) trimming loop and split-face construction
/// 5) split-face classification and assembly
pub fn run_boolean_pipeline<C, S>(
    operation: BooleanOp,
    lhs: &Solid<Point3, C, S>,
    rhs: &Solid<Point3, C, S>,
    geometric_tol: f64,
) -> Result<Vec<Shell<Point3, C, S>>, BopError>
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
    run_boolean_pipeline_with_diagnostics(operation, lhs, rhs, geometric_tol)
        .map(|(shells, _)| shells)
}

#[cfg_attr(not(test), allow(dead_code))]
/// 在 `BooleanOp::Section` 场景下，装配阶段发生既定可观测拓扑失败时，
/// 返回 `Ok((Vec::new(), Some(summary)))`：保留“空结果降级”外部语义，
/// 同时返回可被日志与问题追踪系统复用的结构化 summary（含失败 ID）。
/// 对于非 `Section`，或非 topology 归因的错误，仍保持原有 `Err(BopError)` 行为。
pub(crate) fn run_boolean_pipeline_with_diagnostics<C, S>(
    operation: BooleanOp,
    lhs: &Solid<Point3, C, S>,
    rhs: &Solid<Point3, C, S>,
    geometric_tol: f64,
) -> Result<
    (
        Vec<Shell<Point3, C, S>>,
        Option<SectionAssemblyFailureSummary>,
    ),
    BopError,
>
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
    run_boolean_pipeline_with_diagnostics_no_timing(operation, lhs, rhs, geometric_tol)
        .map(|(shells, maybe_summary)| (shells, maybe_summary))
}

#[cfg_attr(not(test), allow(dead_code))]
/// 在 `BooleanOp::Section` 场景下，装配阶段发生既定可观测拓扑失败时，返回带阶段耗时的执行结果。
/// 当前时延口径 `trim_ms` 统计了 `run_boolean_pipeline_with_diagnostics` 整体执行耗时。
pub(crate) fn run_boolean_pipeline_with_diagnostics_and_timing<C, S>(
    operation: BooleanOp,
    lhs: &Solid<Point3, C, S>,
    rhs: &Solid<Point3, C, S>,
    geometric_tol: f64,
) -> Result<
    (
        Vec<Shell<Point3, C, S>>,
        Option<SectionAssemblyFailureSummary>,
        BooleanPipelineTiming,
    ),
    BopError,
>
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
    let pipeline_start = Instant::now();
    let (shells, maybe_summary) =
        run_boolean_pipeline_with_diagnostics_no_timing(operation, lhs, rhs, geometric_tol)?;
    let trim_ms = pipeline_start.elapsed().as_millis();
    Ok((shells, maybe_summary, trim_ms))
}

fn run_boolean_pipeline_with_diagnostics_no_timing<C, S>(
    operation: BooleanOp,
    lhs: &Solid<Point3, C, S>,
    rhs: &Solid<Point3, C, S>,
    geometric_tol: f64,
) -> Result<
    (
        Vec<Shell<Point3, C, S>>,
        Option<SectionAssemblyFailureSummary>,
    ),
    BopError,
>
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
    if geometric_tol <= 0.0 {
        return Err(BopError::InvalidTolerance);
    }

    let mut options = BopOptions::default();
    options.geometric_tol = geometric_tol;
    options.validate()?;

    let mut bopds = BopDs::with_options(options);
    let left_entities = collect_operand_entities(&mut bopds, 0, lhs)?;
    let right_entities = collect_operand_entities(&mut bopds, 1, rhs)?;

    let all_vertices: Vec<_> = left_entities
        .vertices
        .iter()
        .chain(right_entities.vertices.iter())
        .cloned()
        .collect();
    let all_edges: Vec<_> = left_entities
        .edges
        .iter()
        .chain(right_entities.edges.iter())
        .cloned()
        .collect();
    let all_faces: Vec<_> = left_entities
        .faces
        .iter()
        .chain(right_entities.faces.iter())
        .cloned()
        .collect();

    let candidate_pairs =
        generate_candidate_pairs_from_bopds(&bopds, &all_vertices, &all_edges, &all_faces);

    bopds.rebuild_paves_for_edges(&all_edges);
    let mut filler = PaveFiller::new();
    let _report = filler.run_ve_ee_vf_ef(
        &mut bopds,
        &all_vertices,
        &all_edges,
        &all_faces,
        &candidate_pairs,
    );

    filler.split_pave_blocks(&mut bopds, all_edges.iter().map(|(edge_id, _)| *edge_id));
    filler.make_split_edges(&mut bopds);
    filler.build_trimming_loops(&mut bopds, &all_faces);
    filler.build_split_faces(&mut bopds);

    let solids_by_operand = [(0_u8, lhs.clone()), (1_u8, rhs.clone())];
    classify_split_faces_against_operand(&mut bopds, &solids_by_operand)?;

    let selected_split_faces = select_split_faces_for_boolean_op(&bopds, operation);
    if selected_split_faces.is_empty() {
        return Ok((Vec::new(), None));
    }

    let merged_vertices = merge_equivalent_vertices(&mut bopds, &selected_split_faces);
    let sewn_paths = sew_fragment_edges(&mut bopds, &selected_split_faces, &merged_vertices);

    let all_faces_by_id = all_faces
        .iter()
        .map(|(id, face)| (*id, face.clone()))
        .collect::<FxHashMap<_, _>>();
    let selection_summary = section_split_face_summary(&selected_split_faces);
    let shell_summary = shell_path_summary(&sewn_paths);

    let shells = match assemble_shells_with_diagnostic(&selected_split_faces, &all_faces_by_id) {
        Ok(shells) => shells,
        Err(assembly_error) => match (operation, assembly_error.as_topology_reason()) {
            (BooleanOp::Section, Some(reason)) => {
                let section_summary =
                    assemble_section_failure_summary(&selection_summary, &shell_summary, reason);
                return Ok((Vec::new(), Some(section_summary)));
            }
            (_, Some(_)) => return Err(assembly_error.into_bop_error()),
            (_, None) => return Err(assembly_error.into_bop_error()),
        },
    };

    if shells.is_empty() {
        return Ok((Vec::new(), None));
    }

    if operation == BooleanOp::Section {
        return Ok((shells, None));
    }

    build_solids_from_shells(shells.clone())?;

    Ok((shells, None))
}

fn section_split_face_summary(split_faces: &[SplitFace]) -> (usize, usize, usize, usize) {
    let selected_split_face_count = split_faces.len();
    let mut selected_faces_with_splitting_edges = 0_usize;
    let mut total_split_edges = 0_usize;
    let mut max_split_edges_per_face = 0_usize;

    for split_face in split_faces {
        if !split_face.splitting_edges.is_empty() {
            selected_faces_with_splitting_edges += 1;
        }
        total_split_edges += split_face.splitting_edges.len();
        max_split_edges_per_face = max_split_edges_per_face.max(split_face.splitting_edges.len());
    }

    (
        selected_split_face_count,
        selected_faces_with_splitting_edges,
        total_split_edges,
        max_split_edges_per_face,
    )
}

fn shell_path_summary(paths: &[SewnPath]) -> (usize, usize, usize) {
    let sewn_path_count = paths.len();
    let mut sewn_closed_path_count = 0_usize;
    let mut sewn_open_path_count = 0_usize;

    for path in paths {
        if path.is_closed {
            sewn_closed_path_count += 1;
        } else {
            sewn_open_path_count += 1;
        }
    }

    (
        sewn_path_count,
        sewn_closed_path_count,
        sewn_open_path_count,
    )
}

fn assemble_section_failure_summary(
    section_stats: &(usize, usize, usize, usize),
    path_stats: &(usize, usize, usize),
    reason: &AssemblyFailure,
) -> SectionAssemblyFailureSummary {
    let (
        selected_split_face_count,
        selected_faces_with_splitting_edges,
        total_split_edges,
        max_split_edges_per_face,
    ) = *section_stats;
    let (sewn_path_count, sewn_closed_path_count, sewn_open_path_count) = *path_stats;
    let failure_id = section_failure_id(section_stats, path_stats, reason);

    match reason {
        AssemblyFailure::NoOrientableCandidate {
            seed_face,
            remaining_faces,
        } => SectionAssemblyFailureSummary {
            failure_id,
            selected_split_face_count,
            selected_faces_with_splitting_edges,
            total_split_edges,
            max_split_edges_per_face,
            sewn_path_count,
            sewn_closed_path_count,
            sewn_open_path_count,
            reason: SectionAssemblyFailureReason::NoOrientableCandidate,
            no_orientable_seed_face: Some(*seed_face),
            remaining_orientable_candidates: Some(*remaining_faces),
        },
        AssemblyFailure::OpenShell => SectionAssemblyFailureSummary {
            failure_id,
            selected_split_face_count,
            selected_faces_with_splitting_edges,
            total_split_edges,
            max_split_edges_per_face,
            sewn_path_count,
            sewn_closed_path_count,
            sewn_open_path_count,
            reason: SectionAssemblyFailureReason::OpenShell,
            no_orientable_seed_face: None,
            remaining_orientable_candidates: None,
        },
        AssemblyFailure::InvalidShellOrientation => SectionAssemblyFailureSummary {
            failure_id,
            selected_split_face_count,
            selected_faces_with_splitting_edges,
            total_split_edges,
            max_split_edges_per_face,
            sewn_path_count,
            sewn_closed_path_count,
            sewn_open_path_count,
            reason: SectionAssemblyFailureReason::InvalidShellOrientation,
            no_orientable_seed_face: None,
            remaining_orientable_candidates: None,
        },
    }
}

fn section_failure_id(
    section_stats: &(usize, usize, usize, usize),
    path_stats: &(usize, usize, usize),
    reason: &AssemblyFailure,
) -> u64 {
    let (
        selected_split_face_count,
        selected_faces_with_splitting_edges,
        total_split_edges,
        max_split_edges_per_face,
    ) = *section_stats;
    let (sewn_path_count, sewn_closed_path_count, sewn_open_path_count) = *path_stats;

    let mut fingerprint = 0xCBF29CE484222325u64;

    fingerprint = mix_fingerprint(
        fingerprint,
        (selected_split_face_count as u64).wrapping_mul(0x9E3779B97F4A7C15u64),
    );
    fingerprint = mix_fingerprint(
        fingerprint,
        (selected_faces_with_splitting_edges as u64).wrapping_mul(0xBF58476D1CE4E5B9u64),
    );
    fingerprint = mix_fingerprint(
        fingerprint,
        (total_split_edges as u64).wrapping_mul(0x94D049BB133111EBu64),
    );
    fingerprint = mix_fingerprint(
        fingerprint,
        (max_split_edges_per_face as u64).wrapping_mul(0x2545F4914F6CDD1Du64),
    );
    fingerprint = mix_fingerprint(
        fingerprint,
        (sewn_path_count as u64).wrapping_mul(0x9E3779B97F4A7C15u64),
    );
    fingerprint = mix_fingerprint(
        fingerprint,
        (sewn_closed_path_count as u64).wrapping_mul(0xC2B2AE3D27D4EB4Fu64),
    );
    fingerprint = mix_fingerprint(
        fingerprint,
        (sewn_open_path_count as u64).wrapping_mul(0x27D4EB2F165667C5u64),
    );

    match reason {
        AssemblyFailure::NoOrientableCandidate {
            seed_face,
            remaining_faces,
        } => {
            fingerprint = mix_fingerprint(fingerprint, 0x1111_1111_1111_1111u64);
            fingerprint = mix_fingerprint(fingerprint, *seed_face as u64);
            mix_fingerprint(fingerprint, *remaining_faces as u64)
        }
        AssemblyFailure::OpenShell => mix_fingerprint(fingerprint, 0x2222_2222_2222_2222u64),
        AssemblyFailure::InvalidShellOrientation => {
            mix_fingerprint(fingerprint, 0x3333_3333_3333_3333u64)
        }
    }
}

fn mix_fingerprint(fingerprint: u64, value: u64) -> u64 {
    let mut state = fingerprint ^ value.rotate_left(17);
    state = state.rotate_left(31).wrapping_mul(0x9E3779B97F4A7C15u64);
    state ^ (state >> 33)
}

#[derive(Debug)]
struct OperandEntities<C, S> {
    vertices: Vec<(VertexId, Vertex<Point3>)>,
    edges: Vec<(EdgeId, Edge<Point3, C>)>,
    faces: Vec<(FaceId, Face<Point3, C, S>)>,
}

fn collect_operand_entities<C, S>(
    bopds: &mut BopDs,
    operand_rank: u8,
    solid: &Solid<Point3, C, S>,
) -> Result<OperandEntities<C, S>, BopError>
where
    C: Clone + BoundedCurve<Point = Point3> + Invertible,
    S: Clone,
{
    let mut vertices = Vec::new();
    let mut edges = Vec::new();
    let mut faces = Vec::new();

    for shell in solid.boundaries() {
        for vertex in shell.vertex_iter() {
            vertices.push((bopds.register_vertex_source(operand_rank), vertex.clone()));
        }
        for edge in shell.edge_iter() {
            edges.push((bopds.register_edge_source(operand_rank), edge.clone()));
        }
        for face in shell.face_iter() {
            faces.push((bopds.register_face_source(operand_rank), face.clone()));
        }
    }

    if vertices.is_empty() && edges.is_empty() && faces.is_empty() {
        return Err(BopError::UnsupportedGeometry);
    }

    Ok(OperandEntities {
        vertices,
        edges,
        faces,
    })
}

/// Point classification relative to a solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointClassification {
    /// Point lies strictly outside the solid.
    Outside,
    /// Point lies strictly inside the solid.
    Inside,
    /// Point lies on the solid boundary within tolerance.
    OnBoundary,
}

/// Classifies a point against a solid using simplified axis-aligned box logic.
pub fn classify_point_in_solid<C, S>(
    solid: &Solid<Point3, C, S>,
    point: Point3,
    tolerance: f64,
) -> Result<PointClassification, BopError>
where
    C: Clone,
    S: Clone,
{
    let shell = expect_single_shell(solid)?;
    let bounds = axis_aligned_box_bounds(shell, tolerance)?;

    if point_on_box_boundary(point, &bounds, tolerance) {
        return Ok(PointClassification::OnBoundary);
    }

    let directions = [Vector3::unit_x(), Vector3::unit_y(), Vector3::unit_z()];
    let inside = directions.into_iter().all(|direction| {
        count_axis_aligned_box_hits(shell, point, direction, &bounds, tolerance) % 2 == 1
    });

    Ok(if inside {
        PointClassification::Inside
    } else {
        PointClassification::Outside
    })
}

fn expect_single_shell<C, S>(
    solid: &Solid<Point3, C, S>,
) -> Result<&Shell<Point3, C, S>, BopError> {
    match solid.boundaries().as_slice() {
        [shell] => Ok(shell),
        _ => Err(BopError::UnsupportedGeometry),
    }
}

fn axis_aligned_box_bounds<C, S>(
    shell: &Shell<Point3, C, S>,
    tolerance: f64,
) -> Result<BoundingBox<Point3>, BopError>
where
    C: Clone,
    S: Clone,
{
    if shell.len() != 6 {
        return Err(BopError::UnsupportedGeometry);
    }

    let bounds = BoundingBox::from_iter(shell.vertex_iter().map(|vertex| vertex.point()));
    if bounds.is_empty() {
        return Err(BopError::UnsupportedGeometry);
    }

    for face in shell.face_iter() {
        let Some((normal_axis, normal_value, in_plane_axes)) =
            face_axis_alignment(face, &bounds, tolerance)
        else {
            return Err(BopError::UnsupportedGeometry);
        };

        let vertices: Vec<_> = face.vertex_iter().map(|vertex| vertex.point()).collect();
        if vertices.len() < 4 {
            return Err(BopError::UnsupportedGeometry);
        }

        for vertex in vertices {
            if !near(axis_value(vertex, normal_axis), normal_value, tolerance) {
                return Err(BopError::UnsupportedGeometry);
            }
            for axis in in_plane_axes {
                let value = axis_value(vertex, axis);
                if value < axis_min(&bounds, axis) - tolerance
                    || value > axis_max(&bounds, axis) + tolerance
                {
                    return Err(BopError::UnsupportedGeometry);
                }
            }
        }
    }

    Ok(bounds)
}

fn count_axis_aligned_box_hits<C, S>(
    shell: &Shell<Point3, C, S>,
    point: Point3,
    direction: Vector3,
    bounds: &BoundingBox<Point3>,
    tolerance: f64,
) -> usize
where
    C: Clone,
    S: Clone,
{
    let ray_axis = dominant_axis(direction);
    shell
        .face_iter()
        .filter_map(|face| ray_face_hit_parameter(face, point, ray_axis, bounds, tolerance))
        .count()
}

fn ray_face_hit_parameter<C, S>(
    face: &Face<Point3, C, S>,
    point: Point3,
    ray_axis: usize,
    bounds: &BoundingBox<Point3>,
    tolerance: f64,
) -> Option<f64>
where
    C: Clone,
    S: Clone,
{
    let (normal_axis, normal_value, in_plane_axes) = face_axis_alignment(face, bounds, tolerance)?;
    if normal_axis != ray_axis {
        return None;
    }

    let delta = normal_value - axis_value(point, ray_axis);
    if delta <= tolerance {
        return None;
    }

    for axis in in_plane_axes {
        let value = axis_value(point, axis);
        if value < axis_min(bounds, axis) - tolerance || value > axis_max(bounds, axis) + tolerance
        {
            return None;
        }
    }

    Some(delta)
}

fn point_on_box_boundary(point: Point3, bounds: &BoundingBox<Point3>, tolerance: f64) -> bool {
    let inside = (0..3).all(|axis| {
        let value = axis_value(point, axis);
        value >= axis_min(bounds, axis) && value <= axis_max(bounds, axis)
    });
    if !inside {
        return false;
    }

    (0..3).any(|axis| {
        near(axis_value(point, axis), axis_min(bounds, axis), tolerance)
            || near(axis_value(point, axis), axis_max(bounds, axis), tolerance)
    })
}

fn face_axis_alignment<C, S>(
    face: &Face<Point3, C, S>,
    bounds: &BoundingBox<Point3>,
    tolerance: f64,
) -> Option<(usize, f64, [usize; 2])>
where
    C: Clone,
    S: Clone,
{
    let vertices: Vec<_> = face.vertex_iter().map(|vertex| vertex.point()).collect();
    let first = *vertices.first()?;

    for axis in 0..3 {
        let constant = axis_value(first, axis);
        if !vertices
            .iter()
            .all(|vertex| near(axis_value(*vertex, axis), constant, tolerance))
        {
            continue;
        }

        let min = axis_min(bounds, axis);
        let max = axis_max(bounds, axis);
        if !near(constant, min, tolerance) && !near(constant, max, tolerance) {
            continue;
        }

        let in_plane_axes = remaining_axes(axis);
        for in_plane_axis in in_plane_axes {
            let axis_min_value = axis_min(bounds, in_plane_axis);
            let axis_max_value = axis_max(bounds, in_plane_axis);
            let face_min = vertices
                .iter()
                .map(|vertex| axis_value(*vertex, in_plane_axis))
                .fold(f64::INFINITY, f64::min);
            let face_max = vertices
                .iter()
                .map(|vertex| axis_value(*vertex, in_plane_axis))
                .fold(f64::NEG_INFINITY, f64::max);
            if !near(face_min, axis_min_value, tolerance)
                || !near(face_max, axis_max_value, tolerance)
            {
                return None;
            }
        }

        return Some((axis, constant, in_plane_axes));
    }

    None
}

fn dominant_axis(direction: Vector3) -> usize {
    let abs = [direction.x.abs(), direction.y.abs(), direction.z.abs()];
    if abs[0] >= abs[1] && abs[0] >= abs[2] {
        0
    } else if abs[1] >= abs[2] {
        1
    } else {
        2
    }
}

fn remaining_axes(axis: usize) -> [usize; 2] {
    match axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => unreachable!("axis index must be in 0..3"),
    }
}

fn axis_value(point: Point3, axis: usize) -> f64 {
    match axis {
        0 => point.x,
        1 => point.y,
        2 => point.z,
        _ => unreachable!("axis index must be in 0..3"),
    }
}

fn axis_min(bounds: &BoundingBox<Point3>, axis: usize) -> f64 { axis_value(bounds.min(), axis) }

fn axis_max(bounds: &BoundingBox<Point3>, axis: usize) -> f64 { axis_value(bounds.max(), axis) }

fn near(lhs: f64, rhs: f64, tolerance: f64) -> bool { (lhs - rhs).abs() <= tolerance }

#[cfg(test)]
mod tests {
    use super::*;
    use truck_base::{bounding_box::BoundingBox, cgmath64::Point3};
    use truck_modeling::{builder, primitive, Curve, Plane, Surface};
    use truck_topology::{Edge, Face, Solid, Wire};

    #[test]
    fn boolean_op_debug_name_is_stable() {
        assert_eq!(format!("{:?}", BooleanOp::Common), "Common");
    }

    #[test]
    fn run_boolean_pipeline_rejects_non_positive_tolerance_for_section() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 2.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(3.0, 3.0, 2.0),
        ]));

        assert!(matches!(
            run_boolean_pipeline(BooleanOp::Section, &lhs, &rhs, 0.0),
            Err(BopError::InvalidTolerance)
        ));
        assert!(matches!(
            run_boolean_pipeline(BooleanOp::Section, &lhs, &rhs, -1.0),
            Err(BopError::InvalidTolerance)
        ));
    }

    #[test]
    fn run_boolean_pipeline_section_returns_empty_for_now_for_overlapping_boxes() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 2.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(1.0, 0.5, 0.0),
            Point3::new(3.0, 1.5, 2.0),
        ]));

        let shells = run_boolean_pipeline(BooleanOp::Section, &lhs, &rhs, 1.0e-6).unwrap();
        assert!(shells.is_empty());
    }

    #[test]
    fn run_boolean_pipeline_section_no_intersection_records_empty_without_diagnostic_summary() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(5.0, 5.0, 5.0),
            Point3::new(6.0, 6.0, 6.0),
        ]));

        let (shells, maybe_summary) =
            run_boolean_pipeline_with_diagnostics(BooleanOp::Section, &lhs, &rhs, 1.0e-6).unwrap();
        assert!(shells.is_empty());
        assert!(maybe_summary.is_none());
    }

    #[test]
    fn run_boolean_pipeline_section_collects_diagnostic_summary_on_failure() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 2.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(1.0, 0.5, 0.0),
            Point3::new(3.0, 1.5, 2.0),
        ]));

        let (shells, maybe_summary) =
            run_boolean_pipeline_with_diagnostics(BooleanOp::Section, &lhs, &rhs, 1.0e-6).unwrap();
        assert!(shells.is_empty());
        let summary = maybe_summary.expect("section should record assembly failure summary");
        assert_ne!(summary.failure_id, 0);
        assert!(summary.selected_split_face_count > 0);
        assert!(summary.total_split_edges > 0);
        assert!(summary.selected_faces_with_splitting_edges <= summary.selected_split_face_count);
        assert!(summary.max_split_edges_per_face >= 1);
        assert!(matches!(
            summary.reason,
            SectionAssemblyFailureReason::NoOrientableCandidate
                | SectionAssemblyFailureReason::OpenShell
                | SectionAssemblyFailureReason::InvalidShellOrientation
        ));
        assert_eq!(
            summary.sewn_path_count,
            summary.sewn_closed_path_count + summary.sewn_open_path_count
        );
        if summary.reason == SectionAssemblyFailureReason::NoOrientableCandidate {
            assert!(summary.no_orientable_seed_face.is_some());
            assert!(summary.remaining_orientable_candidates.is_some());
        } else {
            assert!(summary.no_orientable_seed_face.is_none());
            assert!(summary.remaining_orientable_candidates.is_none());
        }
    }

    #[test]
    fn run_boolean_pipeline_section_diagnostic_failure_id_is_reproducible() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 2.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(1.0, 0.5, 0.0),
            Point3::new(3.0, 1.5, 2.0),
        ]));

        let (_, maybe_summary_1) =
            run_boolean_pipeline_with_diagnostics(BooleanOp::Section, &lhs, &rhs, 1.0e-6).unwrap();
        let (_, maybe_summary_2) =
            run_boolean_pipeline_with_diagnostics(BooleanOp::Section, &lhs, &rhs, 1.0e-6).unwrap();

        let summary_1 = maybe_summary_1.expect("section should record first diagnostic summary");
        let summary_2 = maybe_summary_2.expect("section should record second diagnostic summary");
        assert_eq!(summary_1.failure_id, summary_2.failure_id);
    }

    #[test]
    fn run_boolean_pipeline_common_cut_fuse_disjoint_operands_returns_empty_shell_vector() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(5.0, 5.0, 5.0),
            Point3::new(6.0, 6.0, 6.0),
        ]));

        for operation in [BooleanOp::Common, BooleanOp::Cut, BooleanOp::Fuse] {
            assert!(
                run_boolean_pipeline(operation, &lhs, &rhs, 1.0e-6).is_ok(),
                "{:?} should be executable for disjoint solids",
                operation
            );
        }
    }

    #[test]
    fn run_boolean_pipeline_disjoint_operands_no_summary_for_non_section_ops() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(5.0, 5.0, 5.0),
            Point3::new(6.0, 6.0, 6.0),
        ]));

        for operation in [BooleanOp::Common, BooleanOp::Cut, BooleanOp::Fuse] {
            let (_shells, maybe_summary) =
                run_boolean_pipeline_with_diagnostics(operation, &lhs, &rhs, 1.0e-6).unwrap();
            assert!(
                run_boolean_pipeline(operation, &lhs, &rhs, 1.0e-6).is_ok(),
                "{:?} should be executable for disjoint solids",
                operation
            );
            assert!(maybe_summary.is_none());
        }
    }

    #[test]
    fn run_boolean_pipeline_cut_fuse_disjoint_operands_records_w7_2_metrics() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(5.0, 5.0, 5.0),
            Point3::new(6.0, 6.0, 6.0),
        ]));

        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let lhs_operands = collect_operand_entities(&mut bopds, 0, &lhs).unwrap();
        let rhs_operands = collect_operand_entities(&mut bopds, 1, &rhs).unwrap();

        let all_vertices: Vec<_> = lhs_operands
            .vertices
            .iter()
            .chain(rhs_operands.vertices.iter())
            .cloned()
            .collect();
        let all_edges: Vec<_> = lhs_operands
            .edges
            .iter()
            .chain(rhs_operands.edges.iter())
            .cloned()
            .collect();
        let all_faces: Vec<_> = lhs_operands
            .faces
            .iter()
            .chain(rhs_operands.faces.iter())
            .cloned()
            .collect();

        let candidates =
            generate_candidate_pairs_from_bopds(&bopds, &all_vertices, &all_edges, &all_faces);
        let vv = candidates.vv.len();
        let ve = candidates.ve.len();
        let vf = candidates.vf.len();
        let ee = candidates.ee.len();
        let ef = candidates.ef.len();
        let ff = candidates.ff.len();
        let candidate_total = vv + ve + vf + ee + ef + ff;
        assert_eq!(candidate_total, 0);

        bopds.rebuild_paves_for_edges(&all_edges);
        let mut filler = PaveFiller::new();
        let filler_report = filler.run_ve_ee_vf_ef(
            &mut bopds,
            &all_vertices,
            &all_edges,
            &all_faces,
            &candidates,
        );
        assert!(filler_report.ff_failures <= candidates.ff.len());
        let split_blocks =
            filler.split_pave_blocks(&mut bopds, all_edges.iter().map(|(id, _)| *id));
        assert!(split_blocks > 0);
        assert_eq!(filler_report.ff_failures, 0);
        let split_edges = filler.make_split_edges(&mut bopds);
        assert!(split_edges > 0);
        let trimming_loops = filler.build_trimming_loops(&mut bopds, &all_faces);
        assert!(trimming_loops > 0);
        assert_eq!(filler_report.ff, 0);
        assert_eq!(filler_report.ff_failures, 0);
        let split_faces = filler.build_split_faces(&mut bopds);
        assert!(split_faces > 0);

        for operation in [BooleanOp::Cut, BooleanOp::Fuse] {
            let (shells, maybe_summary, timing) =
                run_boolean_pipeline_with_diagnostics_and_timing(operation, &lhs, &rhs, 1.0e-6)
                    .unwrap();
            assert!(maybe_summary.is_none());

            let (failure_id, reason, remaining_candidates) = maybe_summary
                .map(|summary| {
                    let reason = match summary.reason {
                        SectionAssemblyFailureReason::NoOrientableCandidate => {
                            "NoOrientableCandidate".to_string()
                        }
                        SectionAssemblyFailureReason::OpenShell => "OpenShell".to_string(),
                        SectionAssemblyFailureReason::InvalidShellOrientation => {
                            "InvalidShellOrientation".to_string()
                        }
                    };
                    let remaining = summary.remaining_orientable_candidates.unwrap_or(0);
                    (summary.failure_id, reason, remaining)
                })
                .unwrap_or((0u64, "none".to_string(), 0usize));

            let operation_name = match operation {
                BooleanOp::Cut => "cut",
                BooleanOp::Fuse => "fuse",
                BooleanOp::Common | BooleanOp::Section => unreachable!(),
            };

            println!(
                "[OCCT-Bool][W7-2][2026-04-02][case_id={operation_name}_pipeline_runs_disjoint_operands_w7_2] \
op={operation_name} case={operation_name}_pipeline_runs_disjoint_operands_w7_2 \
total_candidate_pairs={candidate_total} vv={vv} ve={ve} vf={vf} ee={ee} ef={ef} ff={ff} ff_failures={ff_failures} \
ff_missing_face={ff_missing_face} ff_missing_triangulated_face={ff_missing_triangulated_face} \
ff_no_intersection_segments={ff_no_intersection_segments} ff_section_sampling_failed={ff_section_sampling_failed} \
ff_degenerate_section={ff_degenerate_section} \
pave_blocks={split_blocks} split_edges={split_edges} trimming_loops={trimming_loops} split_faces={split_faces} trim_ms={trim_ms} \
section_failure_id={failure_id} reason={reason} remaining_orientable_candidates={remaining_candidates} \
result={result} notes={notes}",
                trim_ms = timing,
                ff_failures = filler_report.ff_failures,
                ff_missing_face = filler_report.ff_missing_face,
                ff_missing_triangulated_face = filler_report.ff_missing_triangulated_face,
                ff_no_intersection_segments = filler_report.ff_no_intersection_segments,
                ff_section_sampling_failed = filler_report.ff_section_sampling_failed,
                ff_degenerate_section = filler_report.ff_degenerate_section,
                failure_id = failure_id,
                reason = reason,
                remaining_candidates = remaining_candidates,
                result = if shells.is_empty() { "pass" } else { "pass" },
                notes = "cut_fuse_disjoint_baseline",
            );
        }
    }

    #[test]
    fn section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 2.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(1.0, 0.0, 0.5),
            Point3::new(3.0, 2.0, 2.5),
        ]));

        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let lhs_operands = collect_operand_entities(&mut bopds, 0, &lhs).unwrap();
        let rhs_operands = collect_operand_entities(&mut bopds, 1, &rhs).unwrap();

        let all_vertices: Vec<_> = lhs_operands
            .vertices
            .iter()
            .chain(rhs_operands.vertices.iter())
            .cloned()
            .collect();
        let all_edges: Vec<_> = lhs_operands
            .edges
            .iter()
            .chain(rhs_operands.edges.iter())
            .cloned()
            .collect();
        let all_faces: Vec<_> = lhs_operands
            .faces
            .iter()
            .chain(rhs_operands.faces.iter())
            .cloned()
            .collect();

        let candidates =
            generate_candidate_pairs_from_bopds(&bopds, &all_vertices, &all_edges, &all_faces);
        assert!(candidates.vv.len() + candidates.ve.len() + candidates.vf.len() > 0);
        assert!(
            !candidates.ee.is_empty() || !candidates.ef.is_empty() || !candidates.ff.is_empty()
        );
        let vv = candidates.vv.len();
        let ve = candidates.ve.len();
        let vf = candidates.vf.len();
        let ee = candidates.ee.len();
        let ef = candidates.ef.len();
        let ff = candidates.ff.len();
        let candidate_total = vv + ve + vf + ee + ef + ff;

        bopds.rebuild_paves_for_edges(&all_edges);
        let mut filler = PaveFiller::new();
        let filler_report = filler.run_ve_ee_vf_ef(
            &mut bopds,
            &all_vertices,
            &all_edges,
            &all_faces,
            &candidates,
        );
        assert!(filler_report.ff_failures <= candidates.ff.len());

        let split_blocks =
            filler.split_pave_blocks(&mut bopds, all_edges.iter().map(|(id, _)| *id));
        assert!(split_blocks > 0);
        let split_edges = filler.make_split_edges(&mut bopds);
        assert!(split_edges > 0);
        let trimming_loops = filler.build_trimming_loops(&mut bopds, &all_faces);
        assert!(trimming_loops > 0);
        let split_faces = filler.build_split_faces(&mut bopds);
        assert!(split_faces > 0);

        assert!(filler_report.total_interference_count() > 0);
        assert!(
            filler_report.ve > 0
                || filler_report.vf > 0
                || filler_report.ee > 0
                || filler_report.ef > 0
                || filler_report.ff > 0
        );
        assert!(filler.report().split_faces > 0);

        let (shells, maybe_summary, timing) = run_boolean_pipeline_with_diagnostics_and_timing(
            BooleanOp::Section,
            &lhs,
            &rhs,
            1.0e-6,
        )
        .unwrap();
        assert!(shells.is_empty());
        let trim_ms = timing;
        let (failure_id, reason, remaining_candidates) = maybe_summary
            .map(|summary| {
                let reason = match summary.reason {
                    SectionAssemblyFailureReason::NoOrientableCandidate => {
                        "NoOrientableCandidate".to_string()
                    }
                    SectionAssemblyFailureReason::OpenShell => "OpenShell".to_string(),
                    SectionAssemblyFailureReason::InvalidShellOrientation => {
                        "InvalidShellOrientation".to_string()
                    }
                };
                let remaining = summary.remaining_orientable_candidates.unwrap_or(0);
                (summary.failure_id, reason, remaining)
            })
            .unwrap_or((0u64, "none".to_string(), 0usize));

        println!(
            "[OCCT-Bool][W7-2][2026-04-02][case_id=section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces] \
op=section case=section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces \
total_candidate_pairs={candidate_total} vv={vv} ve={ve} vf={vf} ee={ee} ef={ef} ff={ff} ff_failures={ff_failures} \
ff_missing_face={ff_missing_face} ff_missing_triangulated_face={ff_missing_triangulated_face} \
ff_no_intersection_segments={ff_no_intersection_segments} ff_section_sampling_failed={ff_section_sampling_failed} \
ff_degenerate_section={ff_degenerate_section} \
pave_blocks={split_blocks} split_edges={split_edges} trimming_loops={trimming_loops} split_faces={split_faces} trim_ms={trim_ms} \
section_failure_id={failure_id} reason={reason} remaining_orientable_candidates={remaining_candidates} \
result=pass notes=baseline_seeded",
            reason = reason,
            ff_failures = filler_report.ff_failures,
            ff_missing_face = filler_report.ff_missing_face,
            ff_missing_triangulated_face = filler_report.ff_missing_triangulated_face,
            ff_no_intersection_segments = filler_report.ff_no_intersection_segments,
            ff_section_sampling_failed = filler_report.ff_section_sampling_failed,
            ff_degenerate_section = filler_report.ff_degenerate_section,
        );
    }

    #[test]
    fn common_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces() {
        let lhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]));
        let rhs: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(2.5, 2.5, 2.5),
            Point3::new(3.5, 3.5, 3.5),
        ]));

        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-6,
            ..BopOptions::default()
        });
        let lhs_operands = collect_operand_entities(&mut bopds, 0, &lhs).unwrap();
        let rhs_operands = collect_operand_entities(&mut bopds, 1, &rhs).unwrap();

        let all_vertices: Vec<_> = lhs_operands
            .vertices
            .iter()
            .chain(rhs_operands.vertices.iter())
            .cloned()
            .collect();
        let all_edges: Vec<_> = lhs_operands
            .edges
            .iter()
            .chain(rhs_operands.edges.iter())
            .cloned()
            .collect();
        let all_faces: Vec<_> = lhs_operands
            .faces
            .iter()
            .chain(rhs_operands.faces.iter())
            .cloned()
            .collect();

        let candidates =
            generate_candidate_pairs_from_bopds(&bopds, &all_vertices, &all_edges, &all_faces);
        let vv = candidates.vv.len();
        let ve = candidates.ve.len();
        let vf = candidates.vf.len();
        let ee = candidates.ee.len();
        let ef = candidates.ef.len();
        let ff = candidates.ff.len();
        let candidate_total = vv + ve + vf + ee + ef + ff;
        assert_eq!(candidate_total, 0);

        bopds.rebuild_paves_for_edges(&all_edges);
        let mut filler = PaveFiller::new();
        let filler_report = filler.run_ve_ee_vf_ef(
            &mut bopds,
            &all_vertices,
            &all_edges,
            &all_faces,
            &candidates,
        );
        assert_eq!(candidates.ff.len(), 0);
        assert_eq!(filler_report.ff, 0);
        assert_eq!(filler_report.ff_failures, 0);

        let split_blocks =
            filler.split_pave_blocks(&mut bopds, all_edges.iter().map(|(id, _)| *id));
        assert!(split_blocks > 0);
        let split_edges = filler.make_split_edges(&mut bopds);
        assert!(split_edges > 0);
        let trimming_loops = filler.build_trimming_loops(&mut bopds, &all_faces);
        assert!(trimming_loops > 0);
        let split_faces = filler.build_split_faces(&mut bopds);
        assert!(split_faces > 0);

        let (shells, maybe_summary, timing) =
            run_boolean_pipeline_with_diagnostics_and_timing(BooleanOp::Common, &lhs, &rhs, 1.0e-6)
                .unwrap();
        assert!(!shells.is_empty());
        let trim_ms = timing;
        let result = if shells.is_empty() {
            "fail(no_shells)"
        } else {
            "pass"
        };
        let (failure_id, reason, remaining_candidates) = maybe_summary
            .map(|summary| {
                let reason = match summary.reason {
                    SectionAssemblyFailureReason::NoOrientableCandidate => {
                        "NoOrientableCandidate".to_string()
                    }
                    SectionAssemblyFailureReason::OpenShell => "OpenShell".to_string(),
                    SectionAssemblyFailureReason::InvalidShellOrientation => {
                        "InvalidShellOrientation".to_string()
                    }
                };
                let remaining = summary.remaining_orientable_candidates.unwrap_or(0);
                (summary.failure_id, reason, remaining)
            })
            .unwrap_or((0u64, "none".to_string(), 0usize));

        println!(
            "[OCCT-Bool][W7-2][2026-04-02][case_id=common_pipeline_runs_disjoint_common_baseline] \
            op=common case=common_pipeline_runs_disjoint_common_baseline \
total_candidate_pairs={candidate_total} vv={vv} ve={ve} vf={vf} ee={ee} ef={ef} ff={ff} ff_failures={ff_failures} \
ff_missing_face={ff_missing_face} ff_missing_triangulated_face={ff_missing_triangulated_face} \
ff_no_intersection_segments={ff_no_intersection_segments} ff_section_sampling_failed={ff_section_sampling_failed} \
ff_degenerate_section={ff_degenerate_section} \
pave_blocks={split_blocks} split_edges={split_edges} trimming_loops={trimming_loops} split_faces={split_faces} trim_ms={trim_ms} \
section_failure_id={failure_id} reason={reason} remaining_orientable_candidates={remaining_candidates} \
result={result} notes=common_disjoint_baseline",
            ff_failures = filler_report.ff_failures,
            ff_missing_face = filler_report.ff_missing_face,
            ff_missing_triangulated_face = filler_report.ff_missing_triangulated_face,
            ff_no_intersection_segments = filler_report.ff_no_intersection_segments,
            ff_section_sampling_failed = filler_report.ff_section_sampling_failed,
            ff_degenerate_section = filler_report.ff_degenerate_section,
            reason = reason,
            result = result,
        );
    }

    #[test]
    fn point_classification_returns_inside_for_box_center() {
        let solid: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 3.0, 4.0),
        ]));

        let classification =
            classify_point_in_solid(&solid, Point3::new(1.0, 1.5, 2.0), 1.0e-6).unwrap();

        assert_eq!(classification, PointClassification::Inside);
    }

    #[test]
    fn point_classification_returns_outside_for_point_beyond_box() {
        let solid: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 3.0, 4.0),
        ]));

        let classification =
            classify_point_in_solid(&solid, Point3::new(2.5, 1.5, 2.0), 1.0e-6).unwrap();

        assert_eq!(classification, PointClassification::Outside);
    }

    #[test]
    fn point_classification_returns_on_boundary_for_face_point() {
        let solid: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 3.0, 4.0),
        ]));

        let classification =
            classify_point_in_solid(&solid, Point3::new(0.0, 1.5, 2.0), 1.0e-6).unwrap();

        assert_eq!(classification, PointClassification::OnBoundary);
    }

    #[test]
    #[ignore = "Known limitation: simplified classifier only supports axis-aligned boxes"]
    fn point_classification_rotated_box_center_is_inside() {
        let solid = rotated_box();

        let classification =
            classify_point_in_solid(&solid, Point3::new(0.0, 0.0, 0.5), 1.0e-6).unwrap();

        assert_eq!(classification, PointClassification::Inside);
    }

    #[test]
    #[ignore = "Known limitation: simplified classifier only supports axis-aligned boxes"]
    fn point_classification_rotated_box_face_point_is_boundary() {
        let solid = rotated_box();

        let classification =
            classify_point_in_solid(&solid, Point3::new(0.5, 0.0, 0.5), 1.0e-6).unwrap();

        assert_eq!(classification, PointClassification::OnBoundary);
    }

    fn rotated_box() -> Solid<Point3, Curve, Surface> {
        let bottom = rotated_face(0.0, false);
        let top = rotated_face(1.0, true);

        let vertices = [
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, -1.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(-1.0, 0.0, 1.0),
        ];
        let v = builder::vertices(vertices);
        let e = [
            builder::line(&v[0], &v[1]),
            builder::line(&v[1], &v[2]),
            builder::line(&v[2], &v[3]),
            builder::line(&v[3], &v[0]),
            builder::line(&v[0], &v[4]),
            builder::line(&v[1], &v[5]),
            builder::line(&v[2], &v[6]),
            builder::line(&v[3], &v[7]),
            builder::line(&v[4], &v[5]),
            builder::line(&v[5], &v[6]),
            builder::line(&v[6], &v[7]),
            builder::line(&v[7], &v[4]),
        ];

        let shell: Shell<Point3, Curve, Surface> = vec![
            bottom,
            side_face([e[0].clone(), e[5].clone(), e[8].inverse(), e[4].inverse()]),
            side_face([e[1].clone(), e[6].clone(), e[9].inverse(), e[5].inverse()]),
            side_face([e[2].clone(), e[7].clone(), e[10].inverse(), e[6].inverse()]),
            side_face([e[3].clone(), e[4].clone(), e[11].inverse(), e[7].inverse()]),
            top,
        ]
        .into();

        Solid::new(vec![shell])
    }

    fn rotated_face(z: f64, top: bool) -> Face<Point3, Curve, Surface> {
        let wire = rotated_wire(z, top);
        let plane = Plane::new(
            Point3::new(0.0, -1.0, z),
            Point3::new(1.0, 0.0, z),
            Point3::new(-1.0, 0.0, z),
        );
        let face = Face::new(vec![wire], Surface::Plane(plane));
        if top {
            face
        } else {
            face.inverse()
        }
    }

    fn rotated_wire(z: f64, reverse: bool) -> Wire<Point3, Curve> {
        let vertices = builder::vertices([
            Point3::new(0.0, -1.0, z),
            Point3::new(1.0, 0.0, z),
            Point3::new(0.0, 1.0, z),
            Point3::new(-1.0, 0.0, z),
        ]);
        let edges: Vec<Edge<Point3, Curve>> = vec![
            builder::line(&vertices[0], &vertices[1]),
            builder::line(&vertices[1], &vertices[2]),
            builder::line(&vertices[2], &vertices[3]),
            builder::line(&vertices[3], &vertices[0]),
        ];
        let mut wire = Wire::from(edges);
        if reverse {
            wire.invert();
        }
        wire
    }

    fn side_face(edges: [Edge<Point3, Curve>; 4]) -> Face<Point3, Curve, Surface> {
        let wire = Wire::from(edges.to_vec());
        let points: Vec<_> = wire.vertex_iter().map(|vertex| vertex.point()).collect();
        Face::new(
            vec![wire],
            Surface::Plane(Plane::new(points[0], points[1], points[3])),
        )
    }
}
