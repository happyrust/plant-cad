use truck_base::{bounding_box::BoundingBox, cgmath64::Point3};
use truck_modeling::{Curve, Surface};
use truck_topology::Solid;

type P = Point3;

fn box_solid(min: [f64; 3], max: [f64; 3]) -> Solid<P, Curve, Surface> {
    truck_modeling::primitive::cuboid(BoundingBox::from_iter([
        P::new(min[0], min[1], min[2]),
        P::new(max[0], max[1], max[2]),
    ]))
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
    ];

    for (name, f) in &cases {
        match f() {
            Ok(solids) => println!("[PASS] {:<22} → {} solid(s)", name, solids.len()),
            Err(e) => println!("[FAIL] {:<22} → {:?}", name, e),
        }
    }

    println!("\n=== done ===");
}
