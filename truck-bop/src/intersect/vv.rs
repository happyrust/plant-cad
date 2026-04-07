//! Vertex-vertex intersection detection.

use crate::{bopds::VVInterference, BopDs, VertexId};
use truck_base::cgmath64::{MetricSpace, Point3};
use truck_topology::Vertex;

/// Detects vertex-vertex interferences and stores them in `BopDs`.
pub fn intersect_vv(
    bopds: &mut BopDs,
    vertices: &[(VertexId, Vertex<Point3>)],
    candidates: &[(VertexId, VertexId)],
) -> usize {
    let tolerance_sq = bopds.options().geometric_tol * bopds.options().geometric_tol;

    let mut count = 0;
    for &(vertex1, vertex2) in candidates {
        let Some(point1) = vertex_point(vertices, vertex1) else {
            continue;
        };
        let Some(point2) = vertex_point(vertices, vertex2) else {
            continue;
        };

        if point1.distance2(point2) <= tolerance_sq {
            bopds.push_vv_interference(VVInterference { vertex1, vertex2 });
            count += 1;
        }
    }

    count
}

fn vertex_point(vertices: &[(VertexId, Vertex<Point3>)], vertex_id: VertexId) -> Option<Point3> {
    vertices
        .iter()
        .find(|(id, _)| *id == vertex_id)
        .map(|(_, vertex)| vertex.point())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::BopOptions;

    #[test]
    fn vv_intersection_detects_coincident_vertices() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-3,
            ..BopOptions::default()
        });
        let vertices = vec![
            (VertexId(0), Vertex::new(Point3::new(0.0, 0.0, 0.0))),
            (VertexId(1), Vertex::new(Point3::new(0.0, 0.0, 0.0))),
        ];

        let count = intersect_vv(&mut bopds, &vertices, &[(VertexId(0), VertexId(1))]);

        assert_eq!(count, 1);
        assert_eq!(
            bopds.vv_interferences(),
            &[VVInterference {
                vertex1: VertexId(0),
                vertex2: VertexId(1)
            }]
        );
    }

    #[test]
    fn vv_intersection_detects_vertices_within_tolerance() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 0.05,
            ..BopOptions::default()
        });
        let vertices = vec![
            (VertexId(0), Vertex::new(Point3::new(0.0, 0.0, 0.0))),
            (VertexId(1), Vertex::new(Point3::new(0.03, 0.0, 0.0))),
        ];

        let count = intersect_vv(&mut bopds, &vertices, &[(VertexId(0), VertexId(1))]);

        assert_eq!(count, 1);
        assert_eq!(
            bopds.vv_interferences(),
            &[VVInterference {
                vertex1: VertexId(0),
                vertex2: VertexId(1)
            }]
        );
    }

    #[test]
    fn vv_intersection_ignores_vertices_beyond_tolerance() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 0.01,
            ..BopOptions::default()
        });
        let vertices = vec![
            (VertexId(0), Vertex::new(Point3::new(0.0, 0.0, 0.0))),
            (VertexId(1), Vertex::new(Point3::new(0.03, 0.0, 0.0))),
        ];

        let count = intersect_vv(&mut bopds, &vertices, &[(VertexId(0), VertexId(1))]);

        assert_eq!(count, 0);
        assert!(bopds.vv_interferences().is_empty());
    }
}
