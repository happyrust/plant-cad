//! Vertex-edge intersection detection.

use crate::{
    bopds::{Pave, VEInterference},
    BopDs, EdgeId, VertexId,
};
use truck_base::cgmath64::{MetricSpace, Point3};
use truck_geotrait::{BoundedCurve, Invertible, SearchNearestParameter, D1};
use truck_topology::{Edge, Vertex};

const SEARCH_PARAMETER_TRIALS: usize = 100;

/// Detects vertex-edge interferences and stores them in `BopDs`.
pub fn intersect_ve<C>(
    bopds: &mut BopDs,
    vertices: &[(VertexId, Vertex<Point3>)],
    edges: &[(EdgeId, Edge<Point3, C>)],
    candidates: &[(VertexId, EdgeId)],
) -> usize
where
    C: Clone
        + BoundedCurve<Point = Point3>
        + Invertible
        + SearchNearestParameter<D1, Point = Point3>,
{
    let tolerance = bopds.options().geometric_tol;
    let tolerance_sq = tolerance * tolerance;

    let mut count = 0;
    for &(vertex_id, edge_id) in candidates {
        let Some(point) = vertex_point(vertices, vertex_id) else {
            continue;
        };
        let Some(edge) = edge_by_id(edges, edge_id) else {
            continue;
        };

        let curve = edge.oriented_curve();
        let (start, end) = curve.range_tuple();
        let Some(parameter) =
            curve.search_nearest_parameter(point, (start, end), SEARCH_PARAMETER_TRIALS)
        else {
            continue;
        };
        if parameter < start || parameter > end {
            continue;
        }

        let projection = curve.subs(parameter);
        if projection.distance2(point) > tolerance_sq {
            continue;
        }

        let pave = Pave::new(edge_id, vertex_id, parameter, tolerance)
            .expect("geometric tolerance is validated when BopOptions is created");
        bopds.push_pave(pave);
        bopds.push_ve_interference(VEInterference {
            vertex: vertex_id,
            edge: edge_id,
            parameter,
        });
        count += 1;
    }

    count
}

fn vertex_point(vertices: &[(VertexId, Vertex<Point3>)], vertex_id: VertexId) -> Option<Point3> {
    vertices
        .iter()
        .find(|(id, _)| *id == vertex_id)
        .map(|(_, vertex)| vertex.point())
}

fn edge_by_id<C>(edges: &[(EdgeId, Edge<Point3, C>)], edge_id: EdgeId) -> Option<&Edge<Point3, C>> {
    edges
        .iter()
        .find(|(id, _)| *id == edge_id)
        .map(|(_, edge)| edge)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::BopOptions;
    use truck_modeling::builder;

    #[test]
    fn ve_intersection_detects_vertex_on_edge_and_generates_pave() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-3,
            ..BopOptions::default()
        });
        let vertex = Vertex::new(Point3::new(0.25, 0.0, 0.0));
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        let count = intersect_ve(
            &mut bopds,
            &[(VertexId(0), vertex)],
            &[(EdgeId(10), edge)],
            &[(VertexId(0), EdgeId(10))],
        );

        assert_eq!(count, 1);
        assert_eq!(
            bopds.ve_interferences(),
            &[VEInterference {
                vertex: VertexId(0),
                edge: EdgeId(10),
                parameter: 0.25,
            }]
        );
        assert_eq!(
            bopds.paves(),
            &[Pave::new(EdgeId(10), VertexId(0), 0.25, 1.0e-3).unwrap()]
        );
    }

    #[test]
    fn ve_intersection_detects_vertex_within_tolerance() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 0.01,
            ..BopOptions::default()
        });
        let vertex = Vertex::new(Point3::new(0.5, 0.005, 0.0));
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        let count = intersect_ve(
            &mut bopds,
            &[(VertexId(0), vertex)],
            &[(EdgeId(10), edge)],
            &[(VertexId(0), EdgeId(10))],
        );

        assert_eq!(count, 1);
        assert_eq!(bopds.ve_interferences().len(), 1);
        assert_eq!(bopds.paves().len(), 1);
        assert!((bopds.paves()[0].parameter - 0.5).abs() < 1.0e-9);
    }

    #[test]
    fn ve_intersection_ignores_vertex_beyond_tolerance() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 0.01,
            ..BopOptions::default()
        });
        let vertex = Vertex::new(Point3::new(0.5, 0.03, 0.0));
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        let count = intersect_ve(
            &mut bopds,
            &[(VertexId(0), vertex)],
            &[(EdgeId(10), edge)],
            &[(VertexId(0), EdgeId(10))],
        );

        assert_eq!(count, 0);
        assert!(bopds.ve_interferences().is_empty());
        assert!(bopds.paves().is_empty());
    }

    #[test]
    fn ve_intersection_ignores_vertex_projecting_beyond_edge_bounds() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 0.2,
            ..BopOptions::default()
        });
        let vertex = Vertex::new(Point3::new(1.15, 0.0, 0.0));
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        let count = intersect_ve(
            &mut bopds,
            &[(VertexId(0), vertex)],
            &[(EdgeId(10), edge)],
            &[(VertexId(0), EdgeId(10))],
        );

        assert_eq!(count, 0);
        assert!(bopds.ve_interferences().is_empty());
        assert!(bopds.paves().is_empty());
    }

    #[test]
    fn ve_intersection_generates_pave_within_edge_bounds() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-3,
            ..BopOptions::default()
        });
        let vertex = Vertex::new(Point3::new(0.8, 0.0, 0.0));
        let edge = line_edge(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));

        let count = intersect_ve(
            &mut bopds,
            &[(VertexId(0), vertex)],
            &[(EdgeId(10), edge.clone())],
            &[(VertexId(0), EdgeId(10))],
        );

        let parameter = bopds.paves()[0].parameter;
        let (start, end) = edge.oriented_curve().range_tuple();

        assert_eq!(count, 1);
        assert!(parameter >= start && parameter <= end);
    }

    fn line_edge(start: Point3, end: Point3) -> Edge<Point3, truck_modeling::Curve> {
        let vertices = builder::vertices([start, end]);
        builder::line(&vertices[0], &vertices[1])
    }
}
