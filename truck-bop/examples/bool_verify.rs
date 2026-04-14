use truck_base::{bounding_box::BoundingBox, cgmath64::{Point3, Vector3}};
use truck_modeling::{builder, primitive, Curve, Surface};
use truck_topology::{Shell, Solid};

type P = Point3;

fn box_solid(min: [f64; 3], max: [f64; 3]) -> Solid<P, Curve, Surface> {
    primitive::cuboid(BoundingBox::from_iter([
        P::new(min[0], min[1], min[2]),
        P::new(max[0], max[1], max[2]),
    ]))
}

fn cylinder_solid(center: P, radius: f64, height: f64, division: usize) -> Solid<P, Curve, Surface> {
    let axis = Vector3::new(0.0, 0.0, 1.0);
    let start = center + Vector3::new(radius, 0.0, 0.0);
    let circle_wire: truck_topology::Wire<P, Curve> = primitive::circle(start, center, axis, division);
    let disk: truck_topology::Face<P, Curve, Surface> = builder::try_attach_plane(&[circle_wire]).unwrap();
    let solid: Solid<P, Curve, Surface> = builder::tsweep(&disk, axis * height);
    solid
}

fn main() {
    let tol = 1.0e-6;

    println!("=== truck-bop boolean verification ===\n");

    let cases: Vec<(&str, Box<dyn Fn() -> Result<Vec<Solid<P, Curve, Surface>>, _>>)> = vec![
        ("overlapping_fuse", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
            let b = box_solid([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);
            truck_bop::fuse(&a, &b, tol)
        })),
        ("overlapping_cut", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
            let b = box_solid([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);
            truck_bop::cut(&a, &b, tol)
        })),
        ("identical_common", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
            let b = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
            truck_bop::common(&a, &b, tol)
        })),
        ("adjacent_fuse", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [3.0, 3.0, 3.0]);
            let b = box_solid([0.0, 3.0, 0.0], [1.0, 4.0, 1.0]);
            truck_bop::fuse(&a, &b, tol)
        })),
        ("contained_fuse", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
            let b = box_solid([0.25, 0.25, 0.25], [0.75, 0.75, 0.75]);
            truck_bop::fuse(&a, &b, tol)
        })),
        ("overlapping_common", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
            let b = box_solid([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);
            truck_bop::common(&a, &b, tol)
        })),
        ("edge_touch_fuse", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
            let b = box_solid([1.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
            truck_bop::fuse(&a, &b, tol)
        })),
        ("identical_fuse", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
            let b = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
            truck_bop::fuse(&a, &b, tol)
        })),
        ("contained_common", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
            let b = box_solid([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);
            truck_bop::common(&a, &b, tol)
        })),
        ("contained_cut", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
            let b = box_solid([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);
            truck_bop::cut(&a, &b, tol)
        })),
        ("disjoint_fuse", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
            let b = box_solid([5.0, 5.0, 5.0], [6.0, 6.0, 6.0]);
            truck_bop::fuse(&a, &b, tol)
        })),
        ("box_cyl_common", Box::new(|| {
            let a = box_solid([-2.0, -2.0, 0.0], [2.0, 2.0, 2.0]);
            let b = cylinder_solid(P::new(0.0, 0.0, 0.0), 1.0, 3.0, 8);
            truck_bop::common(&a, &b, tol)
        })),
        ("box_cyl_fuse", Box::new(|| {
            let a = box_solid([-2.0, -2.0, 0.0], [2.0, 2.0, 2.0]);
            let b = cylinder_solid(P::new(0.0, 0.0, 0.0), 1.0, 3.0, 8);
            truck_bop::fuse(&a, &b, tol)
        })),
        ("box_cyl_cut", Box::new(|| {
            let a = box_solid([-2.0, -2.0, 0.0], [2.0, 2.0, 2.0]);
            let b = cylinder_solid(P::new(0.0, 0.0, 0.0), 1.0, 3.0, 8);
            truck_bop::cut(&a, &b, tol)
        })),
    ];

    for (name, f) in &cases {
        match f() {
            Ok(solids) => println!("[PASS] {:<22} → {} solid(s)", name, solids.len()),
            Err(e) => println!("[FAIL] {:<22} → {:?}", name, e),
        }
    }

    println!("\n--- section tests ---");
    let section_cases: Vec<(&str, Box<dyn Fn() -> Result<Shell<P, Curve, Surface>, _>>)> = vec![
        ("box_box_section", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
            let b = box_solid([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);
            truck_bop::section(&a, &b, tol)
        })),
        ("box_cyl_section", Box::new(|| {
            let a = box_solid([-2.0, -2.0, 0.0], [2.0, 2.0, 2.0]);
            let b = cylinder_solid(P::new(0.0, 0.0, 0.0), 1.0, 3.0, 8);
            truck_bop::section(&a, &b, tol)
        })),
    ];
    for (name, f) in &section_cases {
        match f() {
            Ok(shell) => println!("[PASS] {:<22} → {} face(s)", name, shell.len()),
            Err(e) => println!("[FAIL] {:<22} → {:?}", name, e),
        }
    }

    println!("\n=== done ===");
}
