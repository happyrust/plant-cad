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

/// Section operation result with provenance tracking.
#[derive(Debug)]
pub struct SectionResult {
    /// Result shell (open, containing section faces).
    pub shell: Shell<P, Curve, Surface>,
    /// Maps each section face back to the pair of input faces it was derived from.
    pub provenance: ProvenanceMap,
}

/// Run the full boolean pipeline for the given operation.
fn run_boolean_pipeline(
    a: &Solid<P, Curve, Surface>,
    b: &Solid<P, Curve, Surface>,
    tol: f64,
    operation: BooleanOp,
) -> Result<BooleanResult, BopError> {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        run_boolean_pipeline_inner(a, b, tol, operation)
    })) {
        Ok(result) => result,
        Err(_) => Err(BopError::TopologyInvariantBroken),
    }
}

fn run_boolean_pipeline_inner(
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
            if let Ok(hits) = intersect_ee(ea_id, eb_id, ea, eb, ds.options()) {
                for (e1, e2, t_a, t_b) in hits {
                    ds.push_ee_interference(bopds::EEInterference {
                        edge1: e1,
                        edge2: e2,
                        t_a,
                        t_b,
                    });
                    let vid = ds.next_generated_vertex_id();
                    let ptol = ds.options().parametric_tol;
                    let pave_a = bopds::Pave::new(e1, vid, t_a, ptol);
                    if let Ok(p) = pave_a {
                        ds.insert_or_merge_pave_public(p);
                    }
                    let vid2 = ds.next_generated_vertex_id();
                    let pave_b = bopds::Pave::new(e2, vid2, t_b, ptol);
                    if let Ok(p) = pave_b {
                        ds.insert_or_merge_pave_public(p);
                    }
                }
            }
        }
    }

    intersect_ef(&mut ds, &all_edges, &all_faces, &candidates.ef);
    intersect_ff(&mut ds, &all_faces, &cross_ff);

    let dirty_edge_ids: Vec<EdgeId> = ds.ee_interferences()
        .iter()
        .flat_map(|ee| [ee.edge1, ee.edge2])
        .collect();
    if !dirty_edge_ids.is_empty() {
        let dirty_edges: Vec<_> = all_edges
            .iter()
            .filter(|(id, _)| dirty_edge_ids.contains(id))
            .map(|(id, e)| (*id, e.clone()))
            .collect();
        ds.rebuild_paves_for_edges(&dirty_edges);
    }

    build_trimming_loops(&mut ds, &all_faces);
    build_split_faces(&mut ds);

    let solids_by_operand = vec![(0u8, a.clone()), (1u8, b.clone())];
    classify_split_faces_against_operand(&mut ds, &solids_by_operand, &all_faces)?;

    let selected = select_split_faces_for_boolean_op(&ds, operation);
    let selected = remove_coplanar_duplicates(selected, &all_faces, ds.options().geometric_tol);
    let all_ds_boundary = !ds.split_faces().is_empty()
        && ds.split_faces().iter().all(|sf|
            sf.classification == Some(PointClassification::OnBoundary));
    if matches!(operation, BooleanOp::Cut) && all_ds_boundary {
        return Ok(BooleanResult {
            solids: Vec::new(),
            provenance: ProvenanceMap::default(),
        });
    }
    if selected.is_empty() {
        if all_ds_boundary {
            match operation {
                BooleanOp::Fuse => {
                    let prov = provenance_for_passthrough(a, &ds);
                    return Ok(BooleanResult {
                        solids: vec![a.clone()],
                        provenance: prov,
                    });
                }
                BooleanOp::Common => {
                    let prov = provenance_for_passthrough(a, &ds);
                    return Ok(BooleanResult {
                        solids: vec![a.clone()],
                        provenance: prov,
                    });
                }
                BooleanOp::Cut => {
                    return Ok(BooleanResult {
                        solids: Vec::new(),
                        provenance: ProvenanceMap::default(),
                    });
                }
                BooleanOp::Section => {
                    return Ok(BooleanResult {
                        solids: Vec::new(),
                        provenance: ProvenanceMap::default(),
                    });
                }
            }
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
    let assembly_result = assemble_shells(&mut ds, &selected, &faces_by_id, &merged)
        .and_then(|(shells, prov)| {
            let solids = build_solids_from_shells(shells)?;
            Ok(BooleanResult { solids, provenance: prov })
        });

    match assembly_result {
        Ok(result) => Ok(result),
        Err(e) => Err(e),
    }
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

fn remove_coplanar_duplicates(
    selected: Vec<SplitFace>,
    all_faces: &[(FaceId, Face<P, Curve, Surface>)],
    tolerance: f64,
) -> Vec<SplitFace> {
    use truck_base::cgmath64::InnerSpace;

    let face_normal = |face_id: FaceId| -> Option<(truck_base::cgmath64::Vector3, Point3)> {
        let face = all_faces.iter().find(|(id, _)| *id == face_id)?.1.clone();
        match face.oriented_surface() {
            Surface::Plane(p) => Some((p.normal(), p.origin())),
            _ => None,
        }
    };

    let mut to_remove = vec![false; selected.len()];

    for i in 0..selected.len() {
        if to_remove[i] {
            continue;
        }
        let Some((n_i, o_i)) = face_normal(selected[i].original_face) else {
            continue;
        };
        for j in (i + 1)..selected.len() {
            if to_remove[j] {
                continue;
            }
            if selected[i].operand_rank == selected[j].operand_rank {
                continue;
            }
            let Some((n_j, o_j)) = face_normal(selected[j].original_face) else {
                continue;
            };
            let cross = n_i.cross(n_j);
            if cross.magnitude2() > tolerance * tolerance {
                continue;
            }
            let dist = n_i.dot(o_j - o_i).abs();
            if dist > tolerance {
                continue;
            }
            if selected[i].original_face == selected[j].original_face {
                continue;
            }
            if selected[j].operand_rank > selected[i].operand_rank {
                to_remove[j] = true;
            } else {
                to_remove[i] = true;
            }
        }
    }

    selected
        .into_iter()
        .enumerate()
        .filter(|(idx, _)| !to_remove[*idx])
        .map(|(_, sf)| sf)
        .collect()
}

fn provenance_for_passthrough(
    _solid: &Solid<P, Curve, Surface>,
    ds: &BopDs,
) -> ProvenanceMap {
    let mut prov = ProvenanceMap::default();
    for sf in ds.split_faces() {
        prov.record_face(sf.original_face, sf.operand_rank);
        for tl in &sf.trimming_loops {
            let open_vids = trim::open_loop_vertex_ids(tl);
            for i in 0..open_vids.len() {
                let next = (i + 1) % open_vids.len();
                let edge_source = tl.edges.get(i).map(|e| match e.source {
                    bopds::TrimmingEdgeSource::OriginalBoundaryEdge(eid) => {
                        SourceOrigin::OriginalEdge(eid)
                    }
                    bopds::TrimmingEdgeSource::SectionSegment { curve, .. } => {
                        SourceOrigin::SectionCurve(curve)
                    }
                    bopds::TrimmingEdgeSource::Unattributed => {
                        SourceOrigin::Synthesized
                    }
                });
                if let Some(source) = edge_source {
                    prov.record_edge(open_vids[i], open_vids[next], source);
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
    section_with_provenance(a, b, tol).map(|r| r.shell)
}

/// Section operation with provenance tracking.
pub fn section_with_provenance(
    a: &Solid<P, Curve, Surface>,
    b: &Solid<P, Curve, Surface>,
    tol: f64,
) -> Result<SectionResult, BopError> {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        section_inner(a, b, tol)
    })) {
        Ok(result) => result,
        Err(_) => Err(BopError::TopologyInvariantBroken),
    }
}

fn section_inner(
    a: &Solid<P, Curve, Surface>,
    b: &Solid<P, Curve, Surface>,
    tol: f64,
) -> Result<SectionResult, BopError> {
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
        return Ok(SectionResult {
            shell: Shell::new(),
            provenance: ProvenanceMap::default(),
        });
    }

    let mut section_faces: Vec<Face<P, Curve, Surface>> = Vec::new();
    let mut prov = ProvenanceMap::default();

    for sc in &section_curves {
        if sc.samples.len() < 2 {
            continue;
        }

        let (face1_id, face2_id) = sc.faces;
        let original_face = all_faces
            .iter()
            .find(|(id, _)| *id == face1_id)
            .or_else(|| all_faces.iter().find(|(id, _)| *id == face2_id));

        let Some((face_id, ref_face)) = original_face else {
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
        if sc.start == sc.end {
            let wire = truck_topology::Wire::from(vec![edge]);
            let face = Face::new(vec![wire], surface);
            section_faces.push(face);
        } else {
            let wire = truck_topology::Wire::from(vec![edge]);
            let face = Face::new_unchecked(vec![wire], surface);
            section_faces.push(face);
        }

        let operand = ds.face_shape_info(*face_id)
            .map(|si| si.operand_rank)
            .unwrap_or(0);
        prov.record_face(*face_id, operand);
        prov.record_edge(
            sc.start,
            sc.end,
            SourceOrigin::SectionCurve(sc.id),
        );
    }

    Ok(SectionResult {
        shell: section_faces.into(),
        provenance: prov,
    })
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

        let result = common_with_provenance(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "common failed: {:?}", result.err());
        let br = result.unwrap();
        assert_eq!(br.solids.len(), 1, "common of identical boxes → 1 solid");
        assert!(!br.provenance.faces.is_empty(), "provenance should have face entries");
    }

    #[test]
    fn identical_boxes_cut_returns_empty() {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);

        let result = cut(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "cut failed: {:?}", result.err());
        let solids = result.unwrap();
        assert!(solids.is_empty(), "cut of identical boxes → empty (a - a = ∅)");
    }

    #[test]
    fn boundary_only_cut_returns_empty_result() {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);

        let result = cut(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "boundary-only cut failed: {:?}", result.err());
        assert!(
            result.unwrap().is_empty(),
            "boundary-only cut should return an empty neutral result"
        );
    }

    #[test]
    fn overlapping_boxes_fuse() {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);

        let result = fuse_with_provenance(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "fuse failed: {:?}", result.err());
        let br = result.unwrap();
        assert_eq!(br.solids.len(), 1, "overlapping fuse → 1 solid");
        assert!(!br.provenance.faces.is_empty(), "provenance should have face entries");
    }

    #[test]
    fn overlapping_boxes_fuse_does_not_fallback_to_passthrough_solids() {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);

        let result = fuse_with_provenance(&a, &b, 1.0e-6);
        assert!(
            !matches!(result, Ok(BooleanResult { solids, provenance }) if solids.len() == 2 && provenance.faces.is_empty()),
            "overlapping fuse must not degrade to passthrough fallback",
        );
    }

    #[test]
    fn overlapping_boxes_cut() {
        let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = box_solid([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);

        let result = cut_with_provenance(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "cut failed: {:?}", result.err());
        let br = result.unwrap();
        assert_eq!(br.solids.len(), 1, "overlapping cut → 1 solid");
        assert!(!br.provenance.faces.is_empty(), "provenance should have face entries");
    }

    #[test]
    fn adjacent_boxes_fuse() {
        let a = box_solid([0.0, 0.0, 0.0], [3.0, 3.0, 3.0]);
        let b = box_solid([0.0, 3.0, 0.0], [1.0, 4.0, 1.0]);

        let result = fuse_with_provenance(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "fuse failed: {:?}", result.err());
        let br = result.unwrap();
        assert!(!br.solids.is_empty(), "adjacent fuse → at least 1 solid");
    }

    #[test]
    fn contained_box_fuse() {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.25, 0.25, 0.25], [0.75, 0.75, 0.75]);

        let result = fuse_with_provenance(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "fuse failed: {:?}", result.err());
        let br = result.unwrap();
        assert_eq!(br.solids.len(), 1, "contained fuse → 1 solid (outer envelope)");
    }

    #[test]
    fn disjoint_boxes_fuse() {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([5.0, 5.0, 5.0], [6.0, 6.0, 6.0]);

        let result = fuse(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "disjoint fuse failed: {:?}", result.err());
        let solids = result.unwrap();
        assert!(solids.len() >= 1, "disjoint fuse → at least 1 solid");
    }

    #[test]
    fn passthrough_provenance_no_fake_edges() {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);

        let result = fuse_with_provenance(&a, &b, 1.0e-6);
        assert!(result.is_ok());
        let br = result.unwrap();
        for (_, sources) in &br.provenance.edges {
            for src in sources {
                assert!(
                    !matches!(src, SourceOrigin::OriginalEdge(EdgeId(0))),
                    "no fake OriginalEdge(0) sentinels in provenance"
                );
            }
        }
    }

    #[test]
    fn section_with_provenance_returns_face_info() {
        let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = box_solid([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);

        let result = section_with_provenance(&a, &b, 1.0e-6);
        assert!(result.is_ok(), "section failed: {:?}", result.err());
        let sr = result.unwrap();
        if !sr.shell.is_empty() {
            assert!(!sr.provenance.faces.is_empty(), "section provenance should track faces");
        }
    }
}
