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

fn _sphere_solid(center: P, radius: f64, division: usize) -> Solid<P, Curve, Surface> {
    use truck_base::cgmath64::Rad;
    let axis = Vector3::new(0.0, 0.0, 1.0);
    let start = center + Vector3::new(radius, 0.0, 0.0);
    let circle_wire: truck_topology::Wire<P, Curve> = primitive::circle(start, center, axis, division);
    let shell: truck_topology::Shell<P, Curve, Surface> = builder::rsweep(
        &circle_wire,
        center,
        Vector3::new(0.0, 1.0, 0.0),
        Rad(std::f64::consts::TAU),
        division,
    );
    Solid::new(vec![shell])
}

fn cylinder_solid(center: P, radius: f64, height: f64, division: usize) -> Solid<P, Curve, Surface> {
    let axis = Vector3::new(0.0, 0.0, 1.0);
    let start = center + Vector3::new(radius, 0.0, 0.0);
    let circle_wire: truck_topology::Wire<P, Curve> = primitive::circle(start, center, axis, division);
    let disk: truck_topology::Face<P, Curve, Surface> = builder::try_attach_plane(&[circle_wire]).unwrap();
    let solid: Solid<P, Curve, Surface> = builder::tsweep(&disk, axis * height);
    solid
}

fn run_with_timeout<F, R>(name: &str, f: F, timeout_secs: u64) -> Option<R>
where
    F: FnOnce() -> R + Send + 'static,
    R: Send + 'static,
{
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let result = f();
        let _ = tx.send(result);
    });
    match rx.recv_timeout(std::time::Duration::from_secs(timeout_secs)) {
        Ok(r) => Some(r),
        Err(_) => {
            println!("[TIMEOUT] {:<22} → exceeded {}s", name, timeout_secs);
            None
        }
    }
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
        ("cyl_cyl_fuse", Box::new(|| {
            let a = cylinder_solid(P::new(0.0, 0.0, 0.0), 1.0, 2.0, 8);
            let b = cylinder_solid(P::new(0.5, 0.0, 0.0), 1.0, 2.0, 8);
            truck_bop::fuse(&a, &b, tol)
        })),
        ("cyl_cyl_common", Box::new(|| {
            let a = cylinder_solid(P::new(0.0, 0.0, 0.0), 1.0, 2.0, 8);
            let b = cylinder_solid(P::new(0.5, 0.0, 0.0), 1.0, 2.0, 8);
            truck_bop::common(&a, &b, tol)
        })),
        ("offset_box_fuse", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
            let b = box_solid([0.5, 0.5, 0.5], [2.5, 2.5, 2.5]);
            truck_bop::fuse(&a, &b, tol)
        })),
        ("offset_box_cut", Box::new(|| {
            let a = box_solid([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
            let b = box_solid([0.5, 0.5, 0.5], [2.5, 2.5, 2.5]);
            truck_bop::cut(&a, &b, tol)
        })),
    ];

    for (name, f) in &cases {
        let start = std::time::Instant::now();
        let result = f();
        let elapsed = start.elapsed();
        if elapsed.as_secs() > 10 {
            println!("[SLOW] {:<22} → {:.1}s", name, elapsed.as_secs_f64());
        }
        match result {
            Ok(solids) => println!("[PASS] {:<22} → {} solid(s) ({:.1}s)", name, solids.len(), elapsed.as_secs_f64()),
            Err(e) => println!("[FAIL] {:<22} → {:?} ({:.1}s)", name, e, elapsed.as_secs_f64()),
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
        let start = std::time::Instant::now();
        let result = f();
        let elapsed = start.elapsed();
        match result {
            Ok(shell) => println!("[PASS] {:<22} → {} face(s) ({:.1}s)", name, shell.len(), elapsed.as_secs_f64()),
            Err(e) => println!("[FAIL] {:<22} → {:?} ({:.1}s)", name, e, elapsed.as_secs_f64()),
        }
    }

    println!("\n=== done ===");
}
