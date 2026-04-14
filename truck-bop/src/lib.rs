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
mod fclass2d;
pub(crate) mod geometry_utils;
mod intersect;
mod options;
mod pipeline;
/// Provenance tracking for boolean operation results.
pub mod provenance;
mod trim;

pub use fclass2d::FClass2d;

pub use bopds::{
    BopDs, CommonBlock, CommonBlockId, EdgeId, FaceId, FaceInfo, MergedVertex, PaveBlock,
    PaveBlockId, SectionCurveId, SewnEdge, SewnPath, ShapeId, SplitFace, VertexId,
};
pub use bounding::{BoundingProvider, FaceBoundingSurface};
pub use broad_phase::{
    generate_candidate_pairs, generate_candidate_pairs_from_bopds, CandidatePairs,
};
pub use error::BopError;
pub use intersect::{
    intersect_ee, intersect_ef, intersect_ff, intersect_ve, intersect_vf, intersect_vv,
};
pub use options::BopOptions;
pub use pipeline::{classify_point_in_solid, BooleanOp, PointClassification};
pub use provenance::{ProvenanceMap, SourceOrigin};
pub use trim::{
    assemble_shells, build_solids_from_shells, build_split_faces, build_trimming_loops,
    classify_split_faces_against_operand, merge_equivalent_vertices,
    select_split_faces_for_boolean_op, sew_fragment_edges,
};

use rustc_hash::FxHashMap;
use truck_base::cgmath64::Point3;
use truck_modeling::{Curve, Surface};
use truck_topology::{Edge, Face, Shell, Solid, Vertex};

type P = Point3;

/// Boolean operation result with provenance tracking.
#[derive(Debug)]
pub struct BooleanResult {
    /// Result solids.
    pub solids: Vec<Solid<P, Curve, Surface>>,
    /// Maps each output face/edge back to its input origin.
    pub provenance: ProvenanceMap,
}

/// Run the full boolean pipeline for the given operation.
fn run_boolean_pipeline(
    a: &Solid<P, Curve, Surface>,
    b: &Solid<P, Curve, Surface>,
    tol: f64,
    operation: BooleanOp,
) -> Result<BooleanResult, BopError> {
    let options = BopOptions {
        geometric_tol: tol,
        parametric_tol: tol * 0.01,
        ..BopOptions::default()
    };
    let mut ds = BopDs::with_options(options);

    let (vertices, edges, faces) = register_solid_shapes(&mut ds, a, 0);
    let (vertices_b, edges_b, faces_b) = register_solid_shapes(&mut ds, b, 1);

    let mut all_vertices = vertices;
    all_vertices.extend(vertices_b);
    let mut all_edges = edges;
    all_edges.extend(edges_b);
    let mut all_faces = faces;
    all_faces.extend(faces_b);

    let candidates =
        generate_candidate_pairs(&all_vertices, &all_edges, &all_faces, ds.options());

    let cross_operand = |id_a, id_b| -> bool {
        let rank_a = ds.face_shape_info(id_a).map(|si| si.operand_rank);
        let rank_b = ds.face_shape_info(id_b).map(|si| si.operand_rank);
        rank_a.is_some() && rank_b.is_some() && rank_a != rank_b
    };

    let cross_ff: Vec<_> = candidates
        .ff
        .iter()
        .copied()
        .filter(|&(a, b)| cross_operand(a, b))
        .collect();

    intersect_vv(&mut ds, &all_vertices, &candidates.vv);
    intersect_ve(&mut ds, &all_vertices, &all_edges, &candidates.ve);
    intersect_vf(&mut ds, &all_vertices, &all_faces, &candidates.vf);

    for &(ea_id, eb_id) in &candidates.ee {
        let ea = all_edges.iter().find(|(id, _)| *id == ea_id).map(|(_, e)| e);
        let eb = all_edges.iter().find(|(id, _)| *id == eb_id).map(|(_, e)| e);
        if let (Some(ea), Some(eb)) = (ea, eb) {
            let _ = intersect_ee(ea_id, eb_id, ea, eb, ds.options());
        }
    }

    intersect_ef(&mut ds, &all_edges, &all_faces, &candidates.ef);
    intersect_ff(&mut ds, &all_faces, &cross_ff);

    build_trimming_loops(&mut ds, &all_faces);
    build_split_faces(&mut ds);

    let solids_by_operand = vec![(0u8, a.clone()), (1u8, b.clone())];
    classify_split_faces_against_operand(&mut ds, &solids_by_operand, &all_faces)?;

    let selected = select_split_faces_for_boolean_op(&ds, operation);
    if selected.is_empty() {
        let all_boundary = ds.split_faces().iter().all(|sf|
            sf.classification == Some(PointClassification::OnBoundary));
        if all_boundary && !ds.split_faces().is_empty() {
            let passthrough_solid = match operation {
                BooleanOp::Common | BooleanOp::Fuse => a.clone(),
                BooleanOp::Cut => a.clone(),
                BooleanOp::Section => {
                    return Ok(BooleanResult {
                        solids: Vec::new(),
                        provenance: ProvenanceMap::default(),
                    });
                }
            };
            let prov = provenance_for_passthrough(&passthrough_solid, &ds);
            return Ok(BooleanResult {
                solids: vec![passthrough_solid],
                provenance: prov,
            });
        }
        return Ok(BooleanResult {
            solids: Vec::new(),
            provenance: ProvenanceMap::default(),
        });
    }

    let merged = merge_equivalent_vertices(&mut ds, &selected, &all_faces);
    sew_fragment_edges(&mut ds, &selected, &merged);

    let faces_by_id: FxHashMap<FaceId, Face<P, Curve, Surface>> = all_faces
        .iter()
        .map(|(id, f)| (*id, f.clone()))
        .collect();
    let (shells, provenance) = assemble_shells(&mut ds, &selected, &faces_by_id, &merged)?;
    let solids = build_solids_from_shells(shells)?;

    Ok(BooleanResult { solids, provenance })
}

fn register_solid_shapes(
    ds: &mut BopDs,
    solid: &Solid<P, Curve, Surface>,
    rank: u8,
) -> (
    Vec<(VertexId, Vertex<P>)>,
    Vec<(EdgeId, Edge<P, Curve>)>,
    Vec<(FaceId, Face<P, Curve, Surface>)>,
) {
    let mut vertices = Vec::new();
    let mut edges = Vec::new();
    let mut faces = Vec::new();

    for shell in solid.boundaries() {
        for face in shell.face_iter() {
            let face_id = ds.register_face_source(rank);
            faces.push((face_id, face.clone()));

            for wire in face.boundaries() {
                for edge in wire.edge_iter() {
                    let edge_id = ds.register_edge_source(rank);
                    edges.push((edge_id, edge.clone()));

                    let vid = ds.register_vertex_source(rank);
                    vertices.push((vid, edge.front().clone()));
                }
            }
        }
    }

    (vertices, edges, faces)
}

fn provenance_for_passthrough(
    solid: &Solid<P, Curve, Surface>,
    ds: &BopDs,
) -> ProvenanceMap {
    let mut prov = ProvenanceMap::default();
    for sf in ds.split_faces() {
        prov.record_face(sf.original_face, sf.operand_rank);
    }
    for shell in solid.boundaries() {
        for face in shell.face_iter() {
            for wire in face.boundaries() {
                let verts: Vec<_> = wire.vertex_iter().collect();
                for i in 0..verts.len() {
                    let next = (i + 1) % verts.len();
                    let start_id = VertexId(i as u32);
                    let end_id = VertexId(next as u32);
                    prov.record_edge(
                        start_id,
                        end_id,
                        SourceOrigin::OriginalEdge(EdgeId(i as u32)),
                    );
                }
            }
        }
    }
    prov
}

/// Common (intersection) operation.
pub fn common(
    a: &Solid<P, Curve, Surface>,
    b: &Solid<P, Curve, Surface>,
    tol: f64,
) -> Result<Vec<Solid<P, Curve, Surface>>, BopError> {
    run_boolean_pipeline(a, b, tol, BooleanOp::Common).map(|r| r.solids)
}

/// Common (intersection) with provenance tracking.
pub fn common_with_provenance(
    a: &Solid<P, Curve, Surface>,
    b: &Solid<P, Curve, Surface>,
    tol: f64,
) -> Result<BooleanResult, BopError> {
    run_boolean_pipeline(a, b, tol, BooleanOp::Common)
}

/// Fuse (union) operation.
pub fn fuse(
    a: &Solid<P, Curve, Surface>,
    b: &Solid<P, Curve, Surface>,
    tol: f64,
) -> Result<Vec<Solid<P, Curve, Surface>>, BopError> {
    run_boolean_pipeline(a, b, tol, BooleanOp::Fuse).map(|r| r.solids)
}

/// Fuse (union) with provenance tracking.
pub fn fuse_with_provenance(
    a: &Solid<P, Curve, Surface>,
    b: &Solid<P, Curve, Surface>,
    tol: f64,
) -> Result<BooleanResult, BopError> {
    run_boolean_pipeline(a, b, tol, BooleanOp::Fuse)
}

/// Cut (difference) operation.
pub fn cut(
    a: &Solid<P, Curve, Surface>,
    b: &Solid<P, Curve, Surface>,
    tol: f64,
) -> Result<Vec<Solid<P, Curve, Surface>>, BopError> {
    run_boolean_pipeline(a, b, tol, BooleanOp::Cut).map(|r| r.solids)
}

/// Cut (difference) with provenance tracking.
pub fn cut_with_provenance(
    a: &Solid<P, Curve, Surface>,
    b: &Solid<P, Curve, Surface>,
    tol: f64,
) -> Result<BooleanResult, BopError> {
    run_boolean_pipeline(a, b, tol, BooleanOp::Cut)
}

/// Section operation — returns a shell of faces lying on the intersection of two solids.
///
/// Unlike `common`/`fuse`/`cut`, section produces an **open** shell consisting
/// of the faces that lie on the shared boundary between the two operands.
pub fn section(
    a: &Solid<P, Curve, Surface>,
    b: &Solid<P, Curve, Surface>,
    tol: f64,
) -> Result<Shell<P, Curve, Surface>, BopError> {
    let options = BopOptions {
        geometric_tol: tol,
        parametric_tol: tol * 0.01,
        ..BopOptions::default()
    };
    let mut ds = BopDs::with_options(options);

    let (vertices, edges, faces) = register_solid_shapes(&mut ds, a, 0);
    let (vertices_b, edges_b, faces_b) = register_solid_shapes(&mut ds, b, 1);

    let mut all_vertices = vertices;
    all_vertices.extend(vertices_b);
    let mut all_edges = edges;
    all_edges.extend(edges_b);
    let mut all_faces = faces;
    all_faces.extend(faces_b);

    let candidates =
        generate_candidate_pairs(&all_vertices, &all_edges, &all_faces, ds.options());

    let cross_operand = |id_a, id_b| -> bool {
        let rank_a = ds.face_shape_info(id_a).map(|si| si.operand_rank);
        let rank_b = ds.face_shape_info(id_b).map(|si| si.operand_rank);
        rank_a.is_some() && rank_b.is_some() && rank_a != rank_b
    };

    let cross_ff: Vec<_> = candidates
        .ff
        .iter()
        .copied()
        .filter(|&(a, b)| cross_operand(a, b))
        .collect();

    intersect_ff(&mut ds, &all_faces, &cross_ff);

    let section_curves = ds.section_curves().to_vec();
    if section_curves.is_empty() {
        return Ok(Shell::new());
    }

    let mut section_faces: Vec<Face<P, Curve, Surface>> = Vec::new();

    for sc in &section_curves {
        if sc.samples.len() < 2 {
            continue;
        }

        let (face1_id, face2_id) = sc.faces;
        let original_face = all_faces
            .iter()
            .find(|(id, _)| *id == face1_id)
            .or_else(|| all_faces.iter().find(|(id, _)| *id == face2_id));

        let Some((_, ref_face)) = original_face else {
            continue;
        };
        let surface = ref_face.oriented_surface();

        let control_points: Vec<Point3> = sc.samples.clone();
        if control_points.len() < 2 {
            continue;
        }

        let knot_vec = truck_modeling::KnotVec::uniform_knot(control_points.len() - 1, 1);
        let bsp = truck_modeling::BSplineCurve::new(knot_vec, control_points);
        let curve = Curve::BSplineCurve(bsp);

        let v0 = Vertex::new(sc.samples[0]);
        let v1 = if sc.start == sc.end {
            v0.clone()
        } else {
            Vertex::new(*sc.samples.last().unwrap())
        };

        let edge = Edge::new(&v0, &v1, curve);
        let wire = truck_topology::Wire::from(vec![edge]);
        let face = Face::new(vec![wire], surface);
        section_faces.push(face);
    }

    Ok(section_faces.into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use truck_base::bounding_box::BoundingBox;
    use truck_modeling::primitive;

    fn box_solid(min: [f64; 3], max: [f64; 3]) -> Solid<P, Curve, Surface> {
        primitive::cuboid(BoundingBox::from_iter([
            P::new(min[0], min[1], min[2]),
            P::new(max[0], max[1], max[2]),
        ]))
    }

    #[test]
    fn identical_boxes_common() {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);

        let result = common(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "common failed: {:?}", result.err());
        let solids = result.unwrap();
        assert!(!solids.is_empty(), "common of identical boxes should produce a solid");
    }

    #[test]
    fn overlapping_boxes_fuse() {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);

        let result = fuse(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "fuse failed: {:?}", result.err());
    }

    #[test]
    fn overlapping_boxes_cut() {
        let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = box_solid([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);

        let result = cut(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "cut failed: {:?}", result.err());
    }

    #[test]
    fn adjacent_boxes_fuse() {
        let a = box_solid([0.0, 0.0, 0.0], [3.0, 3.0, 3.0]);
        let b = box_solid([0.0, 3.0, 0.0], [1.0, 4.0, 1.0]);

        let result = fuse(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "fuse failed: {:?}", result.err());
    }

    #[test]
    fn contained_box_fuse() {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.25, 0.25, 0.25], [0.75, 0.75, 0.75]);

        let result = fuse(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "fuse failed: {:?}", result.err());
    }
}
