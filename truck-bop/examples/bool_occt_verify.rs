use truck_base::{bounding_box::BoundingBox, cgmath64::Point3};
use truck_modeling::{primitive, Curve, Surface};
use truck_topology::{Shell, Solid};

type P = Point3;

fn box_solid(min: [f64; 3], max: [f64; 3]) -> Solid<P, Curve, Surface> {
    primitive::cuboid(BoundingBox::from_iter([
        P::new(min[0], min[1], min[2]),
        P::new(max[0], max[1], max[2]),
    ]))
}

/// OCCT reference data for a boolean operation test case
struct OcctRef {
    name: &'static str,
    op: &'static str,
    expected_solids: Option<usize>,
    expected_faces: Option<usize>,
    expected_vertices: Option<usize>,
    expected_edges: Option<usize>,
    expected_section_curves: Option<usize>,
}

fn count_topo(solids: &[Solid<P, Curve, Surface>]) -> (usize, usize, usize) {
    let mut faces = 0;
    let mut edges = 0;
    let mut vertices = std::collections::HashSet::new();
    for solid in solids {
        for shell in solid.boundaries() {
            faces += shell.len();
            for face in shell.face_iter() {
                for wire in face.boundaries() {
                    edges += wire.len();
                    for v in wire.vertex_iter() {
                        let p = v.point();
                        let key = (
                            (p.x * 1e6) as i64,
                            (p.y * 1e6) as i64,
                            (p.z * 1e6) as i64,
                        );
                        vertices.insert(key);
                    }
                }
            }
        }
    }
    (faces, edges, vertices.len())
}

fn run_case(
    occt: &OcctRef,
    result: Result<Vec<Solid<P, Curve, Surface>>, truck_bop::BopError>,
) {
    match result {
        Ok(solids) => {
            let (faces, edges, vertices) = count_topo(&solids);
            let solid_ok = occt.expected_solids.map_or(true, |e| e == solids.len());
            let face_ok = occt.expected_faces.map_or(true, |e| e == faces);
            let vert_ok = occt.expected_vertices.map_or(true, |e| e == vertices);
            let edge_ok = occt.expected_edges.map_or(true, |e| e == edges);

            let status = if solid_ok && face_ok && vert_ok && edge_ok {
                "PASS"
            } else {
                "DIFF"
            };

            println!(
                "[{}] {:<30} {} → solids={}{} faces={}{} verts={}{} edges={}{}",
                status,
                occt.name,
                occt.op,
                solids.len(),
                if solid_ok { "" } else { "!" },
                faces,
                if face_ok { "" } else { "!" },
                vertices,
                if vert_ok { "" } else { "!" },
                edges,
                if edge_ok { "" } else { "!" },
            );
        }
        Err(e) => {
            println!("[FAIL] {:<30} {} → {:?}", occt.name, occt.op, e);
        }
    }
}

fn main() {
    let tol = 1.0e-6;
    println!("=== truck-bop vs OCCT reference verification ===\n");
    println!("Legend: PASS=matches OCCT, DIFF=topology mismatch (!), FAIL=error\n");

    // ═══════════════════════════════════════════════════════════════════
    // BOX-BOX test cases with OCCT BOPAlgo_BOP reference topology
    // Reference: OCCT 7.x BRepAlgoAPI_Fuse/Common/Cut on axis-aligned boxes
    // ═══════════════════════════════════════════════════════════════════

    // Case 1: Overlapping boxes — partial overlap
    // OCCT: Fuse → 1 solid, 12 faces, 20 vertices
    {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);
        let occt = OcctRef {
            name: "overlap_fuse",
            op: "Fuse",
            expected_solids: Some(1),
            expected_faces: Some(12),
            expected_vertices: Some(20),
            expected_edges: None,
            expected_section_curves: Some(6),
        };
        run_case(&occt, truck_bop::fuse(&a, &b, tol));
    }

    // Case 2: Overlapping boxes — Common
    // OCCT: Common → 1 solid, 6 faces, 8 vertices (the overlap cube)
    {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);
        let occt = OcctRef {
            name: "overlap_common",
            op: "Common",
            expected_solids: Some(1),
            expected_faces: Some(6),
            expected_vertices: Some(8),
            expected_edges: None,
            expected_section_curves: Some(6),
        };
        run_case(&occt, truck_bop::common(&a, &b, tol));
    }

    // Case 3: Overlapping boxes — Cut
    // OCCT: Cut → 1 solid, 9 faces, 14 vertices (L-shaped)
    {
        let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = box_solid([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);
        let occt = OcctRef {
            name: "overlap_cut",
            op: "Cut",
            expected_solids: Some(1),
            expected_faces: Some(9),
            expected_vertices: Some(14),
            expected_edges: None,
            expected_section_curves: Some(3),
        };
        run_case(&occt, truck_bop::cut(&a, &b, tol));
    }

    // Case 4: Identical boxes — Common should return same box
    // OCCT: Common → 1 solid, 6 faces, 8 vertices
    {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let occt = OcctRef {
            name: "identical_common",
            op: "Common",
            expected_solids: Some(1),
            expected_faces: Some(6),
            expected_vertices: Some(8),
            expected_edges: None,
            expected_section_curves: Some(0),
        };
        run_case(&occt, truck_bop::common(&a, &b, tol));
    }

    // Case 5: Identical boxes — Fuse should return same box
    // OCCT: Fuse → 1 solid, 6 faces, 8 vertices
    {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let occt = OcctRef {
            name: "identical_fuse",
            op: "Fuse",
            expected_solids: Some(1),
            expected_faces: Some(6),
            expected_vertices: Some(8),
            expected_edges: None,
            expected_section_curves: Some(0),
        };
        run_case(&occt, truck_bop::fuse(&a, &b, tol));
    }

    // Case 6: Contained box — Fuse should return outer box
    // OCCT: Fuse → 1 solid, 6 faces, 8 vertices (outer box unchanged)
    {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([0.25, 0.25, 0.25], [0.75, 0.75, 0.75]);
        let occt = OcctRef {
            name: "contained_fuse",
            op: "Fuse",
            expected_solids: Some(1),
            expected_faces: Some(6),
            expected_vertices: Some(8),
            expected_edges: None,
            expected_section_curves: Some(0),
        };
        run_case(&occt, truck_bop::fuse(&a, &b, tol));
    }

    // Case 7: Contained box — Common should return inner box
    // OCCT: Common → 1 solid, 6 faces, 8 vertices (inner box)
    {
        let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = box_solid([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);
        let occt = OcctRef {
            name: "contained_common",
            op: "Common",
            expected_solids: Some(1),
            expected_faces: Some(6),
            expected_vertices: Some(8),
            expected_edges: None,
            expected_section_curves: Some(0),
        };
        run_case(&occt, truck_bop::common(&a, &b, tol));
    }

    // Case 8: Contained box — Cut should return shell with hole
    // OCCT: Cut → 1 solid, 12 faces (6 outer + 6 inner hole), 16 vertices
    {
        let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = box_solid([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);
        let occt = OcctRef {
            name: "contained_cut",
            op: "Cut",
            expected_solids: Some(1),
            expected_faces: Some(12),
            expected_vertices: Some(16),
            expected_edges: None,
            expected_section_curves: Some(0),
        };
        run_case(&occt, truck_bop::cut(&a, &b, tol));
    }

    // Case 9: Edge-touching boxes — Fuse should merge into 1 solid
    // OCCT: Fuse → 1 solid, 10 faces, 12 vertices
    {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([1.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
        let occt = OcctRef {
            name: "edge_touch_fuse",
            op: "Fuse",
            expected_solids: Some(1),
            expected_faces: Some(10),
            expected_vertices: Some(12),
            expected_edges: None,
            expected_section_curves: Some(0),
        };
        run_case(&occt, truck_bop::fuse(&a, &b, tol));
    }

    // Case 10: Disjoint boxes — Fuse should return 2 separate solids
    // OCCT: Fuse → 2 solids, 12 faces total, 16 vertices
    {
        let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = box_solid([5.0, 5.0, 5.0], [6.0, 6.0, 6.0]);
        let occt = OcctRef {
            name: "disjoint_fuse",
            op: "Fuse",
            expected_solids: Some(2),
            expected_faces: Some(12),
            expected_vertices: Some(16),
            expected_edges: None,
            expected_section_curves: Some(0),
        };
        run_case(&occt, truck_bop::fuse(&a, &b, tol));
    }

    // Case 11: Adjacent boxes (shared face, partial overlap) — Fuse
    // OCCT: Fuse → 1 solid, 10 faces, 14 vertices
    {
        let a = box_solid([0.0, 0.0, 0.0], [3.0, 3.0, 3.0]);
        let b = box_solid([0.0, 3.0, 0.0], [1.0, 4.0, 1.0]);
        let occt = OcctRef {
            name: "adjacent_fuse",
            op: "Fuse",
            expected_solids: Some(1),
            expected_faces: Some(10),
            expected_vertices: Some(14),
            expected_edges: None,
            expected_section_curves: None,
        };
        run_case(&occt, truck_bop::fuse(&a, &b, tol));
    }

    // Case 12: Offset overlapping boxes — Fuse
    // OCCT: Fuse → 1 solid, 12 faces, 20 vertices
    {
        let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = box_solid([0.5, 0.5, 0.5], [2.5, 2.5, 2.5]);
        let occt = OcctRef {
            name: "offset_fuse",
            op: "Fuse",
            expected_solids: Some(1),
            expected_faces: Some(12),
            expected_vertices: Some(20),
            expected_edges: None,
            expected_section_curves: Some(6),
        };
        run_case(&occt, truck_bop::fuse(&a, &b, tol));
    }

    // Case 13: Offset overlapping boxes — Cut
    // OCCT: Cut → 1 solid, 9 faces, 14 vertices
    {
        let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = box_solid([0.5, 0.5, 0.5], [2.5, 2.5, 2.5]);
        let occt = OcctRef {
            name: "offset_cut",
            op: "Cut",
            expected_solids: Some(1),
            expected_faces: Some(9),
            expected_vertices: Some(14),
            expected_edges: None,
            expected_section_curves: Some(3),
        };
        run_case(&occt, truck_bop::cut(&a, &b, tol));
    }

    println!("\n=== done ===");
}
