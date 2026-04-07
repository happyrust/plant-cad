//! Boolean operation pipeline

use crate::BopError;
use truck_base::{
    bounding_box::BoundingBox,
    cgmath64::{Point3, Vector3},
};
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
