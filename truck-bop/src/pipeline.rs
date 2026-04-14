//! Boolean operation pipeline

use crate::{fclass2d::FClass2d, BopError};
use truck_base::{
    bounding_box::BoundingBox,
    cgmath64::{InnerSpace, MetricSpace, Point2, Point3, Vector3},
};
use truck_geotrait::{Invertible, ParametricSurface, SearchNearestParameter, D2};
use truck_topology::{Face, Shell, Solid};

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

/// Classifies a point against a solid.
///
/// Tries, in order: (1) AABB fast path for axis-aligned boxes,
/// (2) general ray-casting against parametric faces,
/// (3) nearest-face normal heuristic as final fallback.
pub fn classify_point_in_solid<C, S>(
    solid: &Solid<Point3, C, S>,
    point: Point3,
    tolerance: f64,
) -> Result<PointClassification, BopError>
where
    C: Clone,
    S: Clone
        + Invertible
        + ParametricSurface<Point = Point3, Vector = Vector3>
        + SearchNearestParameter<D2, Point = Point3>,
{
    let shell = expect_single_shell(solid)?;

    if let Ok(result) = classify_point_in_aabb(shell, point, tolerance) {
        return Ok(result);
    }

    classify_point_by_ray_casting(shell, point, tolerance)
}

fn classify_point_in_aabb<C, S>(
    shell: &Shell<Point3, C, S>,
    point: Point3,
    tolerance: f64,
) -> Result<PointClassification, BopError>
where
    C: Clone,
    S: Clone,
{
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

const NEAREST_FACE_TRIALS: usize = 100;
const RAY_NEWTON_MAX_ITERS: usize = 50;
const RAY_NEWTON_TOL: f64 = 1.0e-10;
const RAY_UV_SEARCH_TRIALS: usize = 20;
const RAY_MAX_DIRECTIONS: usize = 3;

// ── Ray-casting point classifier ─────────────────────────────────────────────

fn classify_point_by_ray_casting<C, S>(
    shell: &Shell<Point3, C, S>,
    point: Point3,
    tolerance: f64,
) -> Result<PointClassification, BopError>
where
    C: Clone,
    S: Clone
        + Invertible
        + ParametricSurface<Point = Point3, Vector = Vector3>
        + SearchNearestParameter<D2, Point = Point3>,
{
    if let Some(cls) = boundary_proximity_check(shell, point, tolerance) {
        return Ok(cls);
    }

    let directions = [
        Vector3::new(0.8017, 0.2673, 0.5345).normalize(),
        Vector3::new(-0.3714, 0.7428, 0.5571).normalize(),
        Vector3::new(0.4472, -0.8944, 0.0).normalize(),
    ];

    for direction in &directions[..RAY_MAX_DIRECTIONS] {
        match try_ray_classify(shell, point, *direction, tolerance) {
            RayCastOutcome::Classified(cls) => return Ok(cls),
            RayCastOutcome::Degenerate => continue,
        }
    }

    classify_point_by_nearest_face(shell, point, tolerance)
}

fn boundary_proximity_check<C, S>(
    shell: &Shell<Point3, C, S>,
    point: Point3,
    tolerance: f64,
) -> Option<PointClassification>
where
    C: Clone,
    S: Clone
        + Invertible
        + ParametricSurface<Point = Point3, Vector = Vector3>
        + SearchNearestParameter<D2, Point = Point3>,
{
    for face in shell.face_iter() {
        let surface = face.oriented_surface();
        if let Some(uv) = surface.search_nearest_parameter(point, None, NEAREST_FACE_TRIALS) {
            let closest = surface.subs(uv.0, uv.1);
            if closest.distance(point) <= tolerance {
                return Some(PointClassification::OnBoundary);
            }
        }
    }
    None
}

enum RayCastOutcome {
    Classified(PointClassification),
    Degenerate,
}

fn try_ray_classify<C, S>(
    shell: &Shell<Point3, C, S>,
    origin: Point3,
    direction: Vector3,
    tolerance: f64,
) -> RayCastOutcome
where
    C: Clone,
    S: Clone
        + Invertible
        + ParametricSurface<Point = Point3, Vector = Vector3>
        + SearchNearestParameter<D2, Point = Point3>,
{
    let mut all_hits: Vec<f64> = Vec::new();

    for face in shell.face_iter() {
        let surface = face.oriented_surface();
        let uv_hints = face_boundary_uv_hints(&face, &surface);
        let classifier = FClass2d::from_face(&face, &surface, tolerance, None);
        let mut face_hits: Vec<f64> = Vec::new();

        for hint in &uv_hints {
            let Some((t, u, v)) =
                ray_surface_newton(&surface, origin, direction, hint.0, hint.1)
            else {
                continue;
            };

            if t.abs() <= tolerance {
                return RayCastOutcome::Degenerate;
            }
            if t <= 0.0 {
                continue;
            }
            if face_hits.iter().any(|&h| (h - t).abs() < tolerance) {
                continue;
            }
            let cls = classifier.classify(Point2::new(u, v));
            if cls != PointClassification::Outside {
                face_hits.push(t);
            }
        }

        all_hits.extend(face_hits);
    }

    dedup_hits(&mut all_hits, tolerance);

    RayCastOutcome::Classified(if all_hits.len() % 2 == 1 {
        PointClassification::Inside
    } else {
        PointClassification::Outside
    })
}

fn face_boundary_uv_hints<C, S>(
    face: &Face<Point3, C, S>,
    surface: &S,
) -> Vec<(f64, f64)>
where
    C: Clone,
    S: SearchNearestParameter<D2, Point = Point3>,
{
    let mut hints = Vec::new();
    for wire in face.boundaries() {
        for vertex in wire.vertex_iter() {
            if let Some(uv) =
                surface.search_nearest_parameter(vertex.point(), None, RAY_UV_SEARCH_TRIALS)
            {
                hints.push(uv);
            }
        }
    }
    hints
}

/// Solve surface(u,v) = origin + t * direction via Newton iteration.
/// Returns (t, u, v) if converged with t > 0.
fn ray_surface_newton<S>(
    surface: &S,
    origin: Point3,
    direction: Vector3,
    u0: f64,
    v0: f64,
) -> Option<(f64, f64, f64)>
where
    S: ParametricSurface<Point = Point3, Vector = Vector3>,
{
    let mut u = u0;
    let mut v = v0;
    let s0 = surface.subs(u, v);
    let mut t = (s0 - origin).dot(direction) / direction.magnitude2();

    for _ in 0..RAY_NEWTON_MAX_ITERS {
        let s = surface.subs(u, v);
        let residual = s - origin - direction * t;
        if residual.magnitude2() < RAY_NEWTON_TOL * RAY_NEWTON_TOL {
            return Some((t, u, v));
        }

        let su = surface.uder(u, v);
        let sv = surface.vder(u, v);
        let nd = -direction;

        let Some((du, dv, dt)) = solve_3x3(su, sv, nd, residual) else {
            return None;
        };

        u -= du;
        v -= dv;
        t -= dt;
    }
    None
}

/// Solve [col0 | col1 | col2] * x = rhs via Cramer's rule.
fn solve_3x3(
    col0: Vector3,
    col1: Vector3,
    col2: Vector3,
    rhs: Vector3,
) -> Option<(f64, f64, f64)> {
    let det = col0.dot(col1.cross(col2));
    if det.abs() < 1.0e-14 {
        return None;
    }
    let inv = 1.0 / det;
    let x0 = rhs.dot(col1.cross(col2)) * inv;
    let x1 = col0.dot(rhs.cross(col2)) * inv;
    let x2 = col0.dot(col1.cross(rhs)) * inv;
    Some((x0, x1, x2))
}

fn dedup_hits(hits: &mut Vec<f64>, tolerance: f64) {
    hits.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    hits.dedup_by(|a, b| (*a - *b).abs() < tolerance);
}

// ── Nearest-face fallback classifier ────────────────────────────────────────

fn classify_point_by_nearest_face<C, S>(
    shell: &Shell<Point3, C, S>,
    point: Point3,
    tolerance: f64,
) -> Result<PointClassification, BopError>
where
    C: Clone,
    S: Clone
        + Invertible
        + ParametricSurface<Point = Point3, Vector = Vector3>
        + SearchNearestParameter<D2, Point = Point3>,
{
    let mut best_distance = f64::INFINITY;
    let mut best_normal: Option<Vector3> = None;
    let mut best_closest: Option<Point3> = None;

    for face in shell.face_iter() {
        let surface = face.oriented_surface();
        let Some(uv) = surface.search_nearest_parameter(point, None, NEAREST_FACE_TRIALS) else {
            continue;
        };

        let closest = surface.subs(uv.0, uv.1);
        let dist = closest.distance(point);

        if dist < best_distance {
            let du = surface.uder(uv.0, uv.1);
            let dv = surface.vder(uv.0, uv.1);
            let normal = du.cross(dv);
            if normal.magnitude2() > f64::EPSILON * f64::EPSILON {
                best_distance = dist;
                best_normal = Some(normal.normalize());
                best_closest = Some(closest);
            }
        }
    }

    let normal = best_normal.ok_or(BopError::UnsupportedGeometry)?;
    let closest = best_closest.ok_or(BopError::UnsupportedGeometry)?;

    if best_distance <= tolerance {
        return Ok(PointClassification::OnBoundary);
    }

    let to_point = point - closest;
    if to_point.dot(normal) > 0.0 {
        Ok(PointClassification::Outside)
    } else {
        Ok(PointClassification::Inside)
    }
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
    use truck_topology::{Face, Solid, Wire};

    #[test]
    fn boolean_op_debug_name_is_stable() {
        assert_eq!(format!("{:?}", BooleanOp::Common), "Common");
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
    fn point_classification_rotated_box_center_is_inside() {
        let solid = rotated_box();

        let classification =
            classify_point_in_solid(&solid, Point3::new(0.0, 0.0, 0.5), 1.0e-6).unwrap();

        assert_eq!(classification, PointClassification::Inside);
    }

    #[test]
    fn point_classification_rotated_box_face_point_is_boundary() {
        let solid = rotated_box();

        // (0.5, -0.5, 0.5) lies on the side face between v[0](0,-1,0) and v[1](1,0,0)
        // where the plane equation is x - y = 1
        let classification =
            classify_point_in_solid(&solid, Point3::new(0.5, -0.5, 0.5), 1.0e-6).unwrap();

        assert_eq!(classification, PointClassification::OnBoundary);
    }

    fn rotated_box() -> Solid<Point3, Curve, Surface> {
        let v = builder::vertices([
            Point3::new(0.0, -1.0, 0.0),  // 0: bottom
            Point3::new(1.0, 0.0, 0.0),   // 1
            Point3::new(0.0, 1.0, 0.0),   // 2
            Point3::new(-1.0, 0.0, 0.0),  // 3
            Point3::new(0.0, -1.0, 1.0),  // 4: top
            Point3::new(1.0, 0.0, 1.0),   // 5
            Point3::new(0.0, 1.0, 1.0),   // 6
            Point3::new(-1.0, 0.0, 1.0),  // 7
        ]);

        let eb = [
            builder::line(&v[0], &v[1]),
            builder::line(&v[1], &v[2]),
            builder::line(&v[2], &v[3]),
            builder::line(&v[3], &v[0]),
        ];
        let ev = [
            builder::line(&v[0], &v[4]),
            builder::line(&v[1], &v[5]),
            builder::line(&v[2], &v[6]),
            builder::line(&v[3], &v[7]),
        ];
        let et = [
            builder::line(&v[4], &v[5]),
            builder::line(&v[5], &v[6]),
            builder::line(&v[6], &v[7]),
            builder::line(&v[7], &v[4]),
        ];

        let bottom_wire = Wire::from(vec![
            eb[0].inverse(), eb[3].inverse(), eb[2].inverse(), eb[1].inverse(),
        ]);
        let bottom_plane = Plane::new(v[0].point(), v[3].point(), v[1].point());
        let bottom = Face::new(vec![bottom_wire], Surface::Plane(bottom_plane));

        let top_wire = Wire::from(vec![
            et[0].clone(), et[1].clone(), et[2].clone(), et[3].clone(),
        ]);
        let top_plane = Plane::new(v[4].point(), v[5].point(), v[7].point());
        let top = Face::new(vec![top_wire], Surface::Plane(top_plane));

        let side = |b: usize, _t: usize| -> Face<Point3, Curve, Surface> {
            let nb = (b + 1) % 4;
            let wire = Wire::from(vec![
                eb[b].clone(),
                ev[nb].clone(),
                et[b].inverse(),
                ev[b].inverse(),
            ]);
            let p0 = v[b].point();
            let p1 = v[nb].point();
            let p3 = v[b + 4].point();
            Face::new(vec![wire], Surface::Plane(Plane::new(p0, p1, p3)))
        };

        let shell: Shell<Point3, Curve, Surface> = vec![
            bottom,
            side(0, 4),
            side(1, 5),
            side(2, 6),
            side(3, 7),
            top,
        ]
        .into();

        Solid::new(vec![shell])
    }
}
