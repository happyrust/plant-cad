//! Face trimming loop construction.

use crate::{
    bopds::{
        MergedVertex, SectionCurve, SewnEdge, SewnEdgePair, SewnEdgeSource, SewnPath,
        SplitFace, TrimmingEdge, TrimmingLoop,
    },
    classify_point_in_solid,
    BopDs,
    BopError,
    BooleanOp,
    EdgeId,
    FaceId,
    PointClassification,
    SectionCurveId,
    VertexId,
};
use rustc_hash::FxHashMap;
use truck_base::cgmath64::{MetricSpace, Point2, Point3};
use truck_geotrait::{D2, Invertible, SearchParameter};
use truck_topology::shell::ShellCondition;
use truck_topology::{Face, Solid};

const SEARCH_PARAMETER_TRIALS: usize = 100;

/// Builds trimming loops for the provided faces from original boundaries and projected section edges.
pub fn build_trimming_loops<C, S>(
    bopds: &mut BopDs,
    faces: &[(FaceId, Face<Point3, C, S>)],
) -> usize
where
    C: Clone,
    S: Clone + Invertible + SearchParameter<D2, Point = Point3>,
{
    let tolerance = bopds.options().parametric_tol.max(bopds.options().geometric_tol);
    let section_curves = bopds.section_curves().to_vec();
    let mut built = 0;

    for &(face_id, ref face) in faces {
        let loops = build_loops_for_face(face_id, face, &section_curves, tolerance);
        built += loops.len();
        for trimming_loop in loops {
            bopds.push_trimming_loop(trimming_loop);
        }
    }

    built
}

/// Builds split-face provenance records from trimming loops already stored in `BopDs`.
pub fn build_split_faces(bopds: &mut BopDs) -> usize {
    let all_loops = bopds.trimming_loops().to_vec();
    let split_faces: Vec<SplitFace> = all_loops
        .iter()
        .enumerate()
        .filter(|(_, loop_)| loop_.is_outer)
        .map(|(outer_index, outer_loop)| {
            let mut trimming_loops = vec![all_loops[outer_index].clone()];
            trimming_loops.extend(
                all_loops
                    .iter()
                    .enumerate()
                    .filter(|(inner_index, loop_)| {
                        !loop_.is_outer
                            && *inner_index != outer_index
                            && loop_.face == outer_loop.face
                            && loop_contained_by(loop_, outer_loop)
                            && !contained_by_other_outer(*inner_index, outer_index, &all_loops)
                    })
                    .map(|(_, loop_)| loop_.clone()),
            );
            let splitting_edges = collect_splitting_edges(&trimming_loops);
            SplitFace {
                original_face: outer_loop.face,
                operand_rank: bopds
                    .face_shape_info(outer_loop.face)
                    .map_or(0, |shape_info| shape_info.operand_rank),
                trimming_loops,
                splitting_edges,
                representative_point: None,
                classification: None,
            }
        })
        .collect();

    for split_face in &split_faces {
        bopds.push_split_face(split_face.clone());
    }

    split_faces.len()
}

/// Classifies stored split-face fragments against the opposite operand solid.
pub fn classify_split_faces_against_operand<C, S>(
    bopds: &mut BopDs,
    solids_by_operand: &[(u8, Solid<Point3, C, S>)],
) -> Result<usize, BopError>
where
    C: Clone,
    S: Clone,
{
    let split_faces = bopds.split_faces().to_vec();
    let mut classified = 0;

    for (index, split_face) in split_faces.iter().enumerate() {
        let opposite_rank = match split_face.operand_rank {
            0 => 1,
            1 => 0,
            _ => continue,
        };

        let Some((_, solid)) = solids_by_operand.iter().find(|(rank, _)| *rank == opposite_rank) else {
            continue;
        };

        let representative_point = representative_point(split_face)
            .ok_or(BopError::UnsupportedGeometry)?;
        let classification = classify_point_in_solid(
            solid,
            representative_point,
            bopds.options().geometric_tol,
        )?;
        bopds.set_split_face_classification(index, representative_point, classification);
        classified += 1;
    }

    Ok(classified)
}

/// Selects classified split-face fragments according to the requested boolean operation.
pub fn select_split_faces_for_boolean_op(
    bopds: &BopDs,
    operation: BooleanOp,
) -> Vec<SplitFace> {
    bopds
        .split_faces()
        .iter()
        .filter(|split_face| should_select_split_face(split_face, operation))
        .cloned()
        .collect()
}

/// Merges equivalent vertices from the selected fragments and returns an original-to-canonical map.
pub fn merge_equivalent_vertices(
    bopds: &mut BopDs,
    split_faces: &[SplitFace],
) -> FxHashMap<VertexId, VertexId> {
    let tolerance = bopds.options().geometric_tol;
    let mut samples = collect_fragment_vertices(split_faces);
    samples.sort_by(|lhs, rhs| {
        lhs.1.x
            .total_cmp(&rhs.1.x)
            .then(lhs.1.y.total_cmp(&rhs.1.y))
            .then(lhs.1.z.total_cmp(&rhs.1.z))
            .then(lhs.0.cmp(&rhs.0))
    });

    bopds.clear_merged_vertices();

    let mut adjacency = vec![Vec::<usize>::new(); samples.len()];
    for i in 0..samples.len() {
        for j in (i + 1)..samples.len() {
            if samples[i].1.distance(samples[j].1) <= tolerance {
                adjacency[i].push(j);
                adjacency[j].push(i);
            }
        }
    }

    let mut equivalence = FxHashMap::default();
    let mut merged_vertices = Vec::new();
    let mut visited = vec![false; samples.len()];
    for (index, (vertex_id, _point)) in samples.iter().enumerate() {
        if equivalence.contains_key(vertex_id) {
            continue;
        }

        let mut stack = vec![index];
        let mut cluster = Vec::new();
        while let Some(current) = stack.pop() {
            if visited[current] {
                continue;
            }
            visited[current] = true;
            cluster.push(samples[current]);
            stack.extend(adjacency[current].iter().copied());
        }

        cluster.sort_by_key(|(id, _)| *id);
        let canonical = cluster[0].0;
        let merged_point = average_point(cluster.iter().map(|(_, cluster_point)| *cluster_point));
        let original_vertices = cluster.iter().map(|(id, _)| *id).collect::<Vec<_>>();
        for original in &original_vertices {
            equivalence.insert(*original, canonical);
        }
        merged_vertices.push(MergedVertex {
            id: canonical,
            original_vertices,
            point: merged_point,
        });
    }

    merged_vertices.sort_by_key(|merged| merged.id);
    for merged in merged_vertices {
        bopds.push_merged_vertex(merged);
    }

    equivalence
}

/// Reconnects fragment edges with matching merged endpoints into continuous oriented paths.
pub fn sew_fragment_edges(
    bopds: &mut BopDs,
    split_faces: &[SplitFace],
    merged_vertices: &FxHashMap<VertexId, VertexId>,
) -> Vec<SewnPath> {
    let edges = collect_sewn_edges(split_faces, merged_vertices);
    let mut paths = Vec::new();
    let mut used = vec![false; edges.len()];

    bopds.clear_sewn_paths();

    while let Some(start_index) = used.iter().position(|used_edge| !*used_edge) {
        used[start_index] = true;
        let mut path = vec![edges[start_index].clone()];
        let start_vertex = edges[start_index].start_vertex;
        let mut tail_vertex = edges[start_index].end_vertex;

        while let Some(next_index) = next_edge_index(&edges, &used, tail_vertex) {
            let mut next_edge = edges[next_index].clone();
            if next_edge.start_vertex != tail_vertex {
                std::mem::swap(&mut next_edge.start_vertex, &mut next_edge.end_vertex);
                next_edge.reversed = !next_edge.reversed;
            }
            tail_vertex = next_edge.end_vertex;
            used[next_index] = true;
            path.push(next_edge);
        }

        let sewn_path = SewnPath {
            is_closed: tail_vertex == start_vertex,
            edges: path,
        };
        bopds.push_sewn_path(sewn_path.clone());
        paths.push(sewn_path);
    }

    paths
}

/// Assembles oriented split faces into closed shell components.
pub fn assemble_shells<C, S>(
    split_faces: &[SplitFace],
    faces_by_id: &FxHashMap<FaceId, Face<Point3, C, S>>,
) -> Result<Vec<truck_topology::Shell<Point3, C, S>>, BopError>
where
    C: Clone + truck_geotrait::ParametricCurve<Point = Point3> + truck_geotrait::BoundedCurve + Invertible,
    S: Clone + truck_geotrait::ParametricSurface<Point = Point3> + Invertible + SearchParameter<D2, Point = Point3>,
    truck_modeling::Line<Point3>: truck_modeling::ToSameGeometry<C>,
{
    let rebuilt_faces = split_faces
        .iter()
        .map(|split_face| {
            let original_face = faces_by_id
                .get(&split_face.original_face)
                .ok_or(BopError::InternalInvariant("missing source face for split face"))?;
            let face = if split_face.trimming_loops.is_empty() {
                original_face.clone()
            } else {
                rebuild_face_from_split_face(split_face, original_face)?
            };
            Ok(face)
        })
        .collect::<Result<Vec<_>, BopError>>()?;
    let sewn_faces = sew_shell_faces(split_faces, rebuilt_faces)?;
    let mut shells = Vec::new();

    for shell_faces in sewn_faces {
        let shell: truck_topology::Shell<Point3, C, S> = shell_faces.into_iter().collect();
        if shell_faces_reused_original_boundaries(&shell, split_faces)
            && shell.len() > 1
            && shell.shell_condition() != ShellCondition::Closed
        {
            return Err(BopError::TopologyInvariantBroken);
        }
        shells.push(shell);
    }

    Ok(shells)
}

fn sew_shell_faces<C, S>(
    split_faces: &[SplitFace],
    rebuilt_faces: Vec<Face<Point3, C, S>>,
) -> Result<Vec<Vec<Face<Point3, C, S>>>, BopError>
where
    C: Clone,
    S: Clone,
{
    let component_groups = connected_face_components(split_faces);
    let mut shells = Vec::new();

    for component in component_groups {
        let mut remaining = component;
        let seed_index = remaining.swap_remove(0);
        let mut shell_faces = vec![rebuilt_faces[seed_index].clone()];

        while !remaining.is_empty() {
            let mut advanced = false;
            let mut index = 0;
            while index < remaining.len() {
                let split_face_index = remaining[index];
                let candidate = rebuilt_faces[split_face_index].clone();
                if let Some(oriented) = orient_face_against_shell(&shell_faces, &candidate) {
                    shell_faces.push(oriented);
                    remaining.swap_remove(index);
                    advanced = true;
                } else {
                    index += 1;
                }
            }

            if !advanced {
                return Err(BopError::TopologyInvariantBroken);
            }
        }

        shells.push(shell_faces);
    }

    Ok(shells)
}

fn connected_face_components(split_faces: &[SplitFace]) -> Vec<Vec<usize>> {
    let adjacency = component_adjacency(split_faces);
    let mut visited = vec![false; split_faces.len()];
    let mut components = Vec::new();

    for seed in 0..split_faces.len() {
        if visited[seed] {
            continue;
        }

        let mut stack = vec![seed];
        let mut component = Vec::new();
        visited[seed] = true;

        while let Some(index) = stack.pop() {
            component.push(index);
            for &neighbor in &adjacency[index] {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    stack.push(neighbor);
                }
            }
        }

        component.sort_unstable();
        components.push(component);
    }

    components.sort_by_key(|component| component[0]);
    components
}

fn component_adjacency(split_faces: &[SplitFace]) -> Vec<Vec<usize>> {
    let mut adjacency = vec![Vec::new(); split_faces.len()];

    for left in 0..split_faces.len() {
        for right in (left + 1)..split_faces.len() {
            if split_faces_share_component(&split_faces[left], &split_faces[right]) {
                adjacency[left].push(right);
                adjacency[right].push(left);
            }
        }
    }

    for neighbors in &mut adjacency {
        neighbors.sort_unstable();
        neighbors.dedup();
    }

    adjacency
}

fn split_faces_share_component(lhs: &SplitFace, rhs: &SplitFace) -> bool {
    lhs.trimming_loops.iter().any(|lhs_loop| {
        rhs.trimming_loops
            .iter()
            .any(|rhs_loop| loops_share_boundary(lhs_loop, rhs_loop))
    })
}

fn loops_share_boundary(lhs: &TrimmingLoop, rhs: &TrimmingLoop) -> bool {
    let rhs_edges = canonical_loop_edges(rhs);
    let lhs_edges = canonical_loop_edges(lhs);
    if lhs_edges.is_empty() || rhs_edges.is_empty() {
        return false;
    }

    lhs_edges.into_iter().any(|lhs_edge| rhs_edges.contains(&lhs_edge))
}

fn canonical_loop_edges(trimming_loop: &TrimmingLoop) -> Vec<CanonicalRebuiltEdge> {
    let mut canonical_edges = Vec::new();
    let vertices = open_loop_vertex_ids(trimming_loop);
    if vertices.len() < 2 {
        return canonical_edges;
    }

    for (edge_index, edge) in trimming_loop.edges.iter().enumerate() {
        let start = vertices[edge_index % vertices.len()];
        let end = vertices[(edge_index + 1) % vertices.len()];
        let canonical = canonical_rebuilt_edge(edge, start, end);
        if !canonical_edges.contains(&canonical) {
            canonical_edges.push(canonical);
        }
    }

    canonical_edges
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum CanonicalRebuiltEdge {
    Source(EdgeId),
    Boundary(VertexId, VertexId),
}

fn canonical_rebuilt_edge(
    edge: &TrimmingEdge,
    start: VertexId,
    end: VertexId,
) -> CanonicalRebuiltEdge {
    if let Some(section_curve_id) = edge.section_curve {
        return CanonicalRebuiltEdge::Source(edge_id_from_section_curve(section_curve_id));
    }

    let undirected = undirected_edge_key(start, end);
    CanonicalRebuiltEdge::Boundary(undirected.0, undirected.1)
}

fn shell_faces_reused_original_boundaries<C, S>(
    shell: &truck_topology::Shell<Point3, C, S>,
    split_faces: &[SplitFace],
) -> bool
where
    C: Clone,
    S: Clone,
{
    shell.face_iter().zip(split_faces.iter()).all(|(face, split_face)| {
        split_face.trimming_loops.iter().all(|trimming_loop| {
            face.boundaries()
                .iter()
                .flat_map(|wire| wire.vertex_iter())
                .count()
                == open_polygon_vertices(&trimming_loop.uv_points).len()
        })
    })
}

fn rebuild_face_from_split_face<C, S>(
    split_face: &SplitFace,
    original_face: &Face<Point3, C, S>,
) -> Result<Face<Point3, C, S>, BopError>
where
    C: Clone + truck_geotrait::ParametricCurve<Point = Point3> + truck_geotrait::BoundedCurve + Invertible,
    S: Clone + truck_geotrait::ParametricSurface<Point = Point3> + Invertible + SearchParameter<D2, Point = Point3>,
    truck_modeling::Line<Point3>: truck_modeling::ToSameGeometry<C>,
{
    let surface = original_face.oriented_surface();
    let boundaries = split_face
        .trimming_loops
        .iter()
        .map(|trimming_loop| rebuild_wire_from_trimming_loop(trimming_loop, &surface))
        .collect::<Result<Vec<_>, _>>()?;

    if boundaries.is_empty() {
        return Err(BopError::TopologyInvariantBroken);
    }

    Ok(Face::new(boundaries, original_face.surface().clone()))
}

fn rebuild_wire_from_trimming_loop<C, S>(
    trimming_loop: &TrimmingLoop,
    surface: &S,
) -> Result<truck_topology::Wire<Point3, C>, BopError>
where
    C: Clone + truck_geotrait::ParametricCurve<Point = Point3> + truck_geotrait::BoundedCurve + Invertible,
    S: truck_geotrait::ParametricSurface<Point = Point3> + SearchParameter<D2, Point = Point3> + Invertible,
    truck_modeling::Line<Point3>: truck_modeling::ToSameGeometry<C>,
{
    let vertices = open_polygon_vertices(&trimming_loop.uv_points);
    if vertices.len() < 3 {
        return Err(BopError::TopologyInvariantBroken);
    }

    let vertex_points = vertices
        .iter()
        .copied()
        .map(|uv| surface.subs(uv.x, uv.y))
        .collect::<Vec<_>>();
    let vertices = truck_modeling::builder::vertices(vertex_points);
    let mut edges = Vec::with_capacity(vertices.len());
    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        edges.push(truck_modeling::builder::line(start, end));
    }

    Ok(truck_topology::Wire::from(edges))
}

/// Builds solids from closed shell components and verifies topology validity.
pub fn build_solids_from_shells<C, S>(
    shells: Vec<truck_topology::Shell<Point3, C, S>>,
) -> Result<Vec<Solid<Point3, C, S>>, BopError>
where
    C: Clone,
    S: Clone,
{
    let mut solids = Vec::with_capacity(shells.len());
    for shell in shells {
        if shell.shell_condition() != ShellCondition::Closed {
            return Err(BopError::TopologyInvariantBroken);
        }

        let solid = Solid::new(vec![shell]);
        match solid.boundaries().as_slice() {
            [boundary] if boundary.shell_condition() == ShellCondition::Closed => solids.push(solid),
            _ => return Err(BopError::TopologyInvariantBroken),
        }
    }

    Ok(solids)
}

fn collect_sewn_edges(
    split_faces: &[SplitFace],
    merged_vertices: &FxHashMap<VertexId, VertexId>,
) -> Vec<SewnEdge> {
    let mut edges = Vec::new();
    let mut shared_sources = FxHashMap::<(VertexId, VertexId), Vec<SewnEdgeSource>>::default();

    for split_face in split_faces {
        for (loop_index, trimming_loop) in split_face.trimming_loops.iter().enumerate() {
            let vertices = open_loop_vertex_ids(trimming_loop);
            if vertices.len() < 2 {
                continue;
            }

            for (edge_index, edge) in trimming_loop.edges.iter().enumerate() {
                let edge_vertices = open_polygon_vertices(&edge.uv_points);
                let (raw_start, raw_end) = match (edge_vertices.first(), edge_vertices.last()) {
                    (Some(start), Some(end)) if !near_points(*start, *end) => (*start, *end),
                    _ => (
                        trimming_loop.uv_points[edge_index],
                        trimming_loop.uv_points[(edge_index + 1) % trimming_loop.uv_points.len()],
                    ),
                };
                let loop_start = trimming_loop.uv_points[edge_index];
                let loop_end = trimming_loop.uv_points[(edge_index + 1) % trimming_loop.uv_points.len()];
                let start = vertices[edge_index];
                let end = vertices[(edge_index + 1) % vertices.len()];
                let sewn_start = merged_vertices.get(&start).copied().unwrap_or(start);
                let sewn_end = merged_vertices.get(&end).copied().unwrap_or(end);
                let undirected = undirected_edge_key(sewn_start, sewn_end);
                let source = SewnEdgeSource {
                    face: split_face.original_face,
                    loop_index,
                    edge_index,
                    original_edge: edge.section_curve.map(edge_id_from_section_curve),
                };
                shared_sources.entry(undirected).or_default().push(source);
                let reversed = near_points(raw_start, loop_end) && near_points(raw_end, loop_start);
                edges.push(SewnEdge {
                    source,
                    face: split_face.original_face,
                    loop_index,
                    edge_index,
                    start_vertex: sewn_start,
                    end_vertex: sewn_end,
                    reversed,
                    section_curve: edge.section_curve,
                    sewn_pair: None,
                });
            }
        }
    }

    for edge in &mut edges {
        let undirected = undirected_edge_key(edge.start_vertex, edge.end_vertex);
        let Some(sources) = shared_sources.get(&undirected) else {
            continue;
        };

        if edge.section_curve.is_some() {
            edge.sewn_pair = sources
                .iter()
                .copied()
                .find(|source| *source != edge.source)
                .map(|other| SewnEdgePair::new(edge.source, other));
        } else if sources.len() > 1 {
            edge.sewn_pair = sources
                .iter()
                .copied()
                .find(|source| *source != edge.source)
                .map(|other| SewnEdgePair::new(edge.source, other));
        }
    }

    edges.retain(|edge| {
        edge.section_curve.is_some() || edge.sewn_pair.is_some()
    });

    edges.sort_by(|lhs, rhs| {
        lhs.start_vertex
            .cmp(&rhs.start_vertex)
            .then(lhs.end_vertex.cmp(&rhs.end_vertex))
            .then(lhs.face.cmp(&rhs.face))
            .then(lhs.loop_index.cmp(&rhs.loop_index))
            .then(lhs.edge_index.cmp(&rhs.edge_index))
    });
    edges
}

fn edge_id_from_section_curve(section_curve_id: SectionCurveId) -> EdgeId {
    EdgeId(section_curve_id.0)
}

fn next_edge_index(edges: &[SewnEdge], used: &[bool], vertex: VertexId) -> Option<usize> {
    edges
        .iter()
        .enumerate()
        .find(|(index, edge)| !used[*index] && (edge.start_vertex == vertex || edge.end_vertex == vertex))
        .map(|(index, _)| index)
}

fn undirected_edge_key(start: VertexId, end: VertexId) -> (VertexId, VertexId) {
    if start <= end {
        (start, end)
    } else {
        (end, start)
    }
}

fn open_loop_vertex_ids(trimming_loop: &TrimmingLoop) -> &[VertexId] {
    if trimming_loop.vertex_ids.len() >= 2
        && trimming_loop.uv_points.len() >= 2
        && trimming_loop.uv_points.first() == trimming_loop.uv_points.last()
        && trimming_loop.vertex_ids.first() == trimming_loop.vertex_ids.last()
    {
        &trimming_loop.vertex_ids[..trimming_loop.vertex_ids.len() - 1]
    } else {
        &trimming_loop.vertex_ids
    }
}

fn orient_face_against_shell<C, S>(
    shell_faces: &[Face<Point3, C, S>],
    candidate: &Face<Point3, C, S>,
) -> Option<Face<Point3, C, S>>
where
    C: Clone,
    S: Clone,
{
    for existing in shell_faces {
        if let Some(should_invert) = shared_edge_orientation(existing, candidate) {
            return Some(if should_invert {
                candidate.inverse()
            } else {
                candidate.clone()
            });
        }
    }
    None
}

fn shared_edge_orientation<C, S>(
    lhs: &Face<Point3, C, S>,
    rhs: &Face<Point3, C, S>,
) -> Option<bool>
where
    C: Clone,
    S: Clone,
{
    for lhs_edge in lhs.boundaries().into_iter().flatten() {
        for rhs_edge in rhs.boundaries().into_iter().flatten() {
            if lhs_edge.id() == rhs_edge.id() {
                return Some(lhs_edge.orientation() == rhs_edge.orientation());
            }
        }
    }
    None
}

fn near_points(lhs: Point2, rhs: Point2) -> bool {
    const UV_TOLERANCE: f64 = 1.0e-9;
    lhs.distance2(rhs) <= UV_TOLERANCE * UV_TOLERANCE
}

fn should_select_split_face(split_face: &SplitFace, operation: BooleanOp) -> bool {
    let Some(classification) = split_face.classification else {
        return false;
    };

    match operation {
        BooleanOp::Common => matches!(classification, PointClassification::Inside | PointClassification::OnBoundary),
        BooleanOp::Fuse => matches!(classification, PointClassification::Outside | PointClassification::OnBoundary),
        BooleanOp::Cut => {
            if split_face.operand_rank == 0 {
                matches!(classification, PointClassification::Outside | PointClassification::OnBoundary)
            } else {
                matches!(classification, PointClassification::OnBoundary)
            }
        }
        BooleanOp::Section => false,
    }
}

fn collect_fragment_vertices(split_faces: &[SplitFace]) -> Vec<(VertexId, Point3)> {
    let mut vertices = Vec::new();
    for split_face in split_faces {
        for trimming_loop in &split_face.trimming_loops {
            for (&vertex_id, &point) in trimming_loop.vertex_ids.iter().zip(open_polygon_vertices(&trimming_loop.uv_points).iter()) {
                vertices.push((vertex_id, uv_to_model_point(point)));
            }
        }
    }
    vertices.sort_by_key(|(id, _)| *id);
    vertices.dedup_by_key(|(id, _)| *id);
    vertices
}

fn vertex_ids_for_polyline(face_id: FaceId, seed: u32, polyline: &[Point2]) -> Vec<VertexId> {
    open_polygon_vertices(polyline)
        .iter()
        .enumerate()
        .map(|(index, _)| VertexId(face_id.0 * 10_000 + seed * 100 + index as u32))
        .collect()
}

fn average_point(points: impl Iterator<Item = Point3>) -> Point3 {
    let mut count = 0.0;
    let mut sum = Point3::new(0.0, 0.0, 0.0);
    for point in points {
        sum.x += point.x;
        sum.y += point.y;
        sum.z += point.z;
        count += 1.0;
    }
    Point3::new(sum.x / count, sum.y / count, sum.z / count)
}

fn build_loops_for_face<C, S>(
    face_id: FaceId,
    face: &Face<Point3, C, S>,
    section_curves: &[SectionCurve],
    tolerance: f64,
) -> Vec<TrimmingLoop>
where
    C: Clone,
    S: Clone + Invertible + SearchParameter<D2, Point = Point3>,
{
    let mut loops = boundary_loops(face_id, face, tolerance);

    for section_curve in section_curves {
        let Some(parameters) = section_curve
            .face_parameters
            .iter()
            .find(|(candidate_face_id, _)| *candidate_face_id == face_id)
            .map(|(_, parameters)| parameters.clone())
        else {
            continue;
        };

        let closed = close_polyline(parameters, tolerance);
        if closed.len() < 4 {
            continue;
        }

        loops.push(loop_from_polyline(
            face_id,
            vec![TrimmingEdge {
                section_curve: Some(section_curve.id),
                uv_points: closed.clone(),
            }],
            closed,
            tolerance,
        ));
    }

    classify_loops(&mut loops);
    loops
}

fn boundary_loops<C, S>(face_id: FaceId, face: &Face<Point3, C, S>, tolerance: f64) -> Vec<TrimmingLoop>
where
    C: Clone,
    S: Clone + Invertible + SearchParameter<D2, Point = Point3>,
{
    let surface = face.oriented_surface();
    let mut result = Vec::new();

    for wire in face.boundaries() {
        let mut hint = None;
        let mut uv_points = Vec::new();

        for vertex in wire.vertex_iter() {
            let point = vertex.point();
            let uv = surface
                .search_parameter(point, hint.map(Into::into), SEARCH_PARAMETER_TRIALS)
                .or_else(|| surface.search_parameter(point, None, SEARCH_PARAMETER_TRIALS));
            let Some(uv) = uv else {
                uv_points.clear();
                break;
            };
            let uv = Point2::new(uv.0, uv.1);
            hint = Some(uv);
            uv_points.push(uv);
        }

        if uv_points.len() < 3 {
            continue;
        }

        let closed = close_polyline(uv_points, tolerance);
        let edges = closed
            .windows(2)
            .map(|segment| TrimmingEdge {
                section_curve: None,
                uv_points: vec![segment[0], segment[1]],
            })
            .collect();
        result.push(loop_from_polyline(face_id, edges, closed, tolerance));
    }

    result
}

fn loop_from_polyline(
    face_id: FaceId,
    edges: Vec<TrimmingEdge>,
    uv_points: Vec<Point2>,
    tolerance: f64,
) -> TrimmingLoop {
    let mut closed = close_polyline(uv_points, tolerance);
    let area = signed_area(&closed);
    if area.abs() <= tolerance {
        closed = dedup_consecutive_points(closed, tolerance);
    }
    let vertex_ids = vertex_ids_for_polyline(face_id, 0, &closed);

    TrimmingLoop {
        face: face_id,
        vertex_ids,
        edges,
        signed_area: signed_area(&closed),
        uv_points: closed,
        is_outer: false,
    }
}

fn classify_loops(loops: &mut [TrimmingLoop]) {
    if loops.is_empty() {
        return;
    }

    let outer_index = loops
        .iter()
        .enumerate()
        .max_by(|(_, lhs), (_, rhs)| lhs.signed_area.abs().total_cmp(&rhs.signed_area.abs()))
        .map(|(index, _)| index)
        .unwrap();

    let outer_sign = loops[outer_index].signed_area.signum();
    for (index, trimming_loop) in loops.iter_mut().enumerate() {
        trimming_loop.is_outer = index == outer_index;
        if outer_sign != 0.0 && trimming_loop.signed_area.signum() == outer_sign {
            reverse_trimming_loop(trimming_loop, outer_index == index);
            trimming_loop.signed_area = -trimming_loop.signed_area;
        }
    }
}

fn reverse_trimming_loop(trimming_loop: &mut TrimmingLoop, is_closed: bool) {
    trimming_loop.uv_points.reverse();
    trimming_loop.edges.reverse();
    for edge in &mut trimming_loop.edges {
        edge.uv_points.reverse();
    }

    if is_closed {
        trimming_loop.uv_points = close_polyline(
            trimming_loop.uv_points.clone(),
            f64::EPSILON,
        );
    }
}

fn close_polyline(mut uv_points: Vec<Point2>, tolerance: f64) -> Vec<Point2> {
    uv_points = dedup_consecutive_points(uv_points, tolerance);
    if uv_points.len() >= 2 && uv_points.first().unwrap().distance2(*uv_points.last().unwrap()) > tolerance * tolerance {
        uv_points.push(*uv_points.first().unwrap());
    }
    uv_points
}

fn dedup_consecutive_points(uv_points: Vec<Point2>, tolerance: f64) -> Vec<Point2> {
    let mut deduped = Vec::with_capacity(uv_points.len());
    for point in uv_points {
        if deduped
            .last()
            .is_some_and(|previous: &Point2| previous.distance2(point) <= tolerance * tolerance)
        {
            continue;
        }
        deduped.push(point);
    }
    deduped
}

fn signed_area(polyline: &[Point2]) -> f64 {
    if polyline.len() < 3 {
        return 0.0;
    }

    let mut area = 0.0;
    let mut prev = *polyline.last().unwrap();
    for &curr in polyline {
        area += (curr.x + prev.x) * (curr.y - prev.y);
        prev = curr;
    }
    area / 2.0
}

fn collect_splitting_edges(trimming_loops: &[TrimmingLoop]) -> Vec<SectionCurveId> {
    let mut splitting_edges = Vec::new();
    for trimming_loop in trimming_loops {
        for edge in &trimming_loop.edges {
            let Some(section_curve) = edge.section_curve else {
                continue;
            };
            if !splitting_edges.contains(&section_curve) {
                splitting_edges.push(section_curve);
            }
        }
    }
    splitting_edges
}

fn representative_point(split_face: &SplitFace) -> Option<Point3> {
    let outer_loop = split_face.trimming_loops.iter().find(|loop_| loop_.is_outer)?;
    let inner_loops: Vec<_> = split_face.trimming_loops.iter().filter(|loop_| !loop_.is_outer).cloned().collect();

    loop_representative_point(outer_loop, &inner_loops).or_else(|| {
        inner_loops
            .iter()
            .find_map(|trimming_loop| loop_representative_point(trimming_loop, &[]))
    })
}

fn loop_representative_point(trimming_loop: &TrimmingLoop, fragment_loops: &[TrimmingLoop]) -> Option<Point3> {
    loop_centroid(&trimming_loop.uv_points)
        .filter(|point| point_in_fragment_region(fragment_loops, *point))
        .map(uv_to_model_point)
        .or_else(|| {
            open_polygon_vertices(&trimming_loop.uv_points)
                .iter()
                .copied()
                .find(|point| point_in_fragment_region(fragment_loops, *point))
                .map(uv_to_model_point)
        })
        .or_else(|| loop_interior_sample_point(trimming_loop, fragment_loops).map(uv_to_model_point))
}

fn loop_centroid(loop_points: &[Point2]) -> Option<Point2> {
    let vertices = open_polygon_vertices(loop_points);
    if vertices.len() < 3 {
        return None;
    }

    let mut area = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;

    for i in 0..vertices.len() {
        let curr = vertices[i];
        let next = vertices[(i + 1) % vertices.len()];
        let cross = curr.x * next.y - next.x * curr.y;
        area += cross;
        cx += (curr.x + next.x) * cross;
        cy += (curr.y + next.y) * cross;
    }

    if area.abs() < 1e-10 {
        return None;
    }

    area *= 0.5;
    Some(Point2::new(cx / (6.0 * area), cy / (6.0 * area)))
}

fn loop_interior_sample_point(trimming_loop: &TrimmingLoop, fragment_loops: &[TrimmingLoop]) -> Option<Point2> {
    let vertices = open_polygon_vertices(&trimming_loop.uv_points);
    if vertices.len() < 2 {
        return None;
    }
    for start in 0..vertices.len() {
        for end in (start + 1)..vertices.len() {
            let midpoint = Point2::new(
                (vertices[start].x + vertices[end].x) * 0.5,
                (vertices[start].y + vertices[end].y) * 0.5,
            );
            if point_in_fragment_region(fragment_loops, midpoint) {
                return Some(midpoint);
            }
        }
    }
    None
}

fn point_in_loop_region(trimming_loop: &TrimmingLoop, point: Point2) -> bool {
    !point_on_polygon_boundary(&trimming_loop.uv_points, point) && point_in_polygon(&trimming_loop.uv_points, point)
}

fn point_in_fragment_region(fragment_loops: &[TrimmingLoop], point: Point2) -> bool {
    fragment_loops.is_empty() || fragment_loops.iter().any(|trimming_loop| point_in_loop_region(trimming_loop, point))
}

fn uv_to_model_point(point: Point2) -> Point3 {
    Point3::new(point.x, point.y, 0.0)
}

fn loop_contained_by(inner: &TrimmingLoop, outer: &TrimmingLoop) -> bool {
    polygon_centroid(&inner.uv_points)
        .filter(|point| !point_on_polygon_boundary(&outer.uv_points, *point))
        .is_some_and(|point| point_in_polygon(&outer.uv_points, point))
}

fn contained_by_other_outer(
    inner_index: usize,
    owner_outer_index: usize,
    all_loops: &[TrimmingLoop],
) -> bool {
    let inner = &all_loops[inner_index];
    all_loops
        .iter()
        .enumerate()
        .filter(|(outer_index, candidate)| {
            candidate.is_outer
                && *outer_index != owner_outer_index
                && candidate.face == inner.face
                && loop_contained_by(inner, candidate)
        })
        .any(|(_, candidate)| {
            let owner = &all_loops[owner_outer_index];
            candidate.signed_area.abs() < owner.signed_area.abs()
        })
}

fn point_in_polygon(polygon: &[Point2], point: Point2) -> bool {
    let mut inside = false;
    let mut prev = *polygon.last().unwrap();
    for &curr in polygon {
        let intersects = (curr.y > point.y) != (prev.y > point.y)
            && point.x < (prev.x - curr.x) * (point.y - curr.y) / (prev.y - curr.y) + curr.x;
        if intersects {
            inside = !inside;
        }
        prev = curr;
    }
    inside
}

fn polygon_centroid(polygon: &[Point2]) -> Option<Point2> {
    let vertices = open_polygon_vertices(polygon);
    if vertices.len() < 3 {
        return None;
    }

    let mut area2 = 0.0;
    let mut centroid_x = 0.0;
    let mut centroid_y = 0.0;
    for window in vertices.windows(2) {
        let start = window[0];
        let end = window[1];
        let cross = start.x * end.y - end.x * start.y;
        area2 += cross;
        centroid_x += (start.x + end.x) * cross;
        centroid_y += (start.y + end.y) * cross;
    }

    if area2.abs() <= 1.0e-9 {
        return None;
    }

    Some(Point2::new(
        centroid_x / (3.0 * area2),
        centroid_y / (3.0 * area2),
    ))
}

fn open_polygon_vertices(polygon: &[Point2]) -> &[Point2] {
    if polygon.len() >= 2 && polygon.first() == polygon.last() {
        &polygon[..polygon.len() - 1]
    } else {
        polygon
    }
}

fn point_on_polygon_boundary(polygon: &[Point2], point: Point2) -> bool {
    polygon.windows(2).any(|segment| point_on_segment(segment[0], segment[1], point))
}

fn point_on_segment(start: Point2, end: Point2, point: Point2) -> bool {
    let edge = end - start;
    let offset = point - start;
    let cross = edge.x * offset.y - edge.y * offset.x;
    if cross.abs() > 1.0e-9 {
        return false;
    }

    let dot = edge.x * offset.x + edge.y * offset.y;
    if dot < -1.0e-9 {
        return false;
    }

    let length2 = edge.x * edge.x + edge.y * edge.y;
    dot <= length2 + 1.0e-9
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{bopds::SectionCurve, BopOptions, VertexId};
    use truck_base::bounding_box::BoundingBox;
    use truck_base::cgmath64::Vector3;
    use truck_modeling::{builder, primitive, Curve, Surface};
    use truck_topology::shell::ShellCondition;
    use truck_topology::{Wire, Solid};

    #[test]
    fn trimming_loop_construction_uses_face_boundary_as_outer_loop() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face = unit_square_face();

        let count = build_trimming_loops(&mut bopds, &[(FaceId(0), face)]);

        assert_eq!(count, 1);
        let loops = bopds.trimming_loops();
        assert_eq!(loops.len(), 1);
        assert!(loops[0].is_outer);
        assert_eq!(loops[0].face, FaceId(0));
        assert_eq!(loops[0].uv_points.first(), loops[0].uv_points.last());
        assert_eq!(loops[0].edges.len(), 4);
    }

    #[test]
    fn trimming_loop_construction_adds_closed_section_loop_as_hole() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let section_curve_id = bopds.next_section_curve_id();
        bopds.push_section_curve(SectionCurve {
            id: section_curve_id,
            faces: (FaceId(0), FaceId(1)),
            start: VertexId(10),
            end: VertexId(10),
            samples: vec![
                Point3::new(0.25, 0.25, 0.0),
                Point3::new(0.75, 0.25, 0.0),
                Point3::new(0.75, 0.75, 0.0),
                Point3::new(0.25, 0.75, 0.0),
            ],
            face_parameters: [
                (
                    FaceId(0),
                    vec![
                        Point2::new(0.25, 0.25),
                        Point2::new(0.75, 0.25),
                        Point2::new(0.75, 0.75),
                        Point2::new(0.25, 0.75),
                    ],
                ),
                (
                    FaceId(1),
                    vec![
                        Point2::new(0.25, 0.25),
                        Point2::new(0.75, 0.25),
                        Point2::new(0.75, 0.75),
                        Point2::new(0.25, 0.75),
                    ],
                ),
            ],
            face_projection_available: [(FaceId(0), true), (FaceId(1), true)],
        });

        let count = build_trimming_loops(&mut bopds, &[(FaceId(0), unit_square_face())]);

        assert_eq!(count, 2);
        let loops = bopds.trimming_loops();
        assert_eq!(loops.len(), 2);
        assert_eq!(loops.iter().filter(|loop_| loop_.is_outer).count(), 1);
        let hole = loops.iter().find(|loop_| !loop_.is_outer).unwrap();
        assert_eq!(hole.uv_points.first(), hole.uv_points.last());
        assert_eq!(hole.edges.len(), 1);
        assert_eq!(hole.edges[0].section_curve, Some(section_curve_id));
    }

    #[test]
    fn trimming_loop_construction_marks_existing_inner_boundary_as_hole() {
        let mut bopds = BopDs::with_options(BopOptions::default());

        let count = build_trimming_loops(&mut bopds, &[(FaceId(0), square_face_with_square_hole())]);

        assert_eq!(count, 2);
        let loops = bopds.trimming_loops();
        assert_eq!(loops.iter().filter(|loop_| loop_.is_outer).count(), 1);
        assert_eq!(loops.iter().filter(|loop_| !loop_.is_outer).count(), 1);
        assert!(loops.iter().all(|loop_| loop_.uv_points.first() == loop_.uv_points.last()));
        let outer = loops.iter().find(|loop_| loop_.is_outer).unwrap();
        let inner = loops.iter().find(|loop_| !loop_.is_outer).unwrap();
        assert!(outer.signed_area.abs() > inner.signed_area.abs());
        assert!(outer.edges.iter().all(|edge| edge.section_curve.is_none()));
        assert!(inner.edges.iter().all(|edge| edge.section_curve.is_none()));
    }

    #[test]
    fn trimming_loop_reversal_preserves_closed_polyline_and_edge_connectivity() {
        let mut loop_record = TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(0), VertexId(1), VertexId(2), VertexId(3)],
            edges: vec![
                TrimmingEdge {
                    section_curve: Some(SectionCurveId(1)),
                    uv_points: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
                },
                TrimmingEdge {
                    section_curve: Some(SectionCurveId(2)),
                    uv_points: vec![Point2::new(1.0, 0.0), Point2::new(1.0, 1.0)],
                },
                TrimmingEdge {
                    section_curve: Some(SectionCurveId(3)),
                    uv_points: vec![Point2::new(1.0, 1.0), Point2::new(0.0, 1.0)],
                },
                TrimmingEdge {
                    section_curve: Some(SectionCurveId(4)),
                    uv_points: vec![Point2::new(0.0, 1.0), Point2::new(0.0, 0.0)],
                },
            ],
            signed_area: 1.0,
            uv_points: vec![
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 0.0),
                Point2::new(1.0, 1.0),
                Point2::new(0.0, 1.0),
                Point2::new(0.0, 0.0),
            ],
            is_outer: false,
        };

        reverse_trimming_loop(&mut loop_record, true);

        assert_eq!(loop_record.uv_points.first(), loop_record.uv_points.last());
        assert_eq!(loop_record.edges[0].section_curve, Some(SectionCurveId(4)));
        assert_eq!(loop_record.edges[0].uv_points, vec![Point2::new(0.0, 0.0), Point2::new(0.0, 1.0)]);
        assert_eq!(loop_record.edges[1].uv_points[0], loop_record.edges[0].uv_points[1]);
        assert_eq!(loop_record.edges[2].uv_points[0], loop_record.edges[1].uv_points[1]);
        assert_eq!(loop_record.edges[3].uv_points[0], loop_record.edges[2].uv_points[1]);
        assert_eq!(loop_record.edges[3].uv_points[1], loop_record.edges[0].uv_points[0]);
    }

    #[test]
    fn split_face_records_group_loops_by_original_face() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let section_curve_id = bopds.next_section_curve_id();
        bopds.push_section_curve(section_curve(section_curve_id));

        let loops = build_trimming_loops(&mut bopds, &[(FaceId(0), unit_square_face())]);
        assert_eq!(loops, 2);

        let built = build_split_faces(&mut bopds);

        assert_eq!(built, 1);
        let split_faces = bopds.split_faces();
        assert_eq!(split_faces.len(), 1);
        assert_eq!(split_faces[0].original_face, FaceId(0));
        assert_eq!(split_faces[0].trimming_loops.len(), 2);
    }

    #[test]
    fn split_face_records_capture_splitting_edges_once() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let section_curve_id = bopds.next_section_curve_id();
        bopds.push_section_curve(section_curve(section_curve_id));

        build_trimming_loops(&mut bopds, &[(FaceId(0), unit_square_face())]);
        build_split_faces(&mut bopds);

        let split_face = &bopds.split_faces()[0];
        assert_eq!(split_face.splitting_edges, vec![section_curve_id]);
    }

    #[test]
    fn split_face_records_are_queryable_from_bopds() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        build_trimming_loops(
            &mut bopds,
            &[(FaceId(0), unit_square_face()), (FaceId(1), unit_square_face())],
        );

        let built = build_split_faces(&mut bopds);

        assert_eq!(built, 2);
        let faces: Vec<FaceId> = bopds.split_faces().iter().map(|split_face| split_face.original_face).collect();
        assert_eq!(faces, vec![FaceId(0), FaceId(1)]);
        assert!(bopds.split_faces().iter().all(|split_face| split_face.trimming_loops.len() == 1));
    }

    #[test]
    fn split_face_records_emit_distinct_fragments_for_multiple_outer_loops_on_same_face() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let first_section = SectionCurveId(11);
        let second_section = SectionCurveId(12);
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(0), VertexId(1), VertexId(2), VertexId(3)],
            edges: vec![TrimmingEdge {
                section_curve: Some(first_section),
                uv_points: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 0.0),
                Point2::new(1.0, 1.0),
                Point2::new(0.0, 1.0),
                Point2::new(0.0, 0.0),
            ],
            signed_area: -1.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(4), VertexId(5), VertexId(6), VertexId(7)],
            edges: vec![TrimmingEdge {
                section_curve: Some(second_section),
                uv_points: vec![Point2::new(2.0, 0.0), Point2::new(3.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(2.0, 0.0),
                Point2::new(3.0, 0.0),
                Point2::new(3.0, 1.0),
                Point2::new(2.0, 1.0),
                Point2::new(2.0, 0.0),
            ],
            signed_area: -1.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(8), VertexId(9), VertexId(10), VertexId(11)],
            edges: vec![TrimmingEdge {
                section_curve: Some(SectionCurveId(99)),
                uv_points: vec![Point2::new(0.25, 0.25), Point2::new(0.75, 0.25)],
            }],
            uv_points: vec![
                Point2::new(0.25, 0.25),
                Point2::new(0.75, 0.25),
                Point2::new(0.75, 0.75),
                Point2::new(0.25, 0.75),
                Point2::new(0.25, 0.25),
            ],
            signed_area: 0.25,
            is_outer: false,
        });

        let built = build_split_faces(&mut bopds);

        assert_eq!(built, 2);
        assert_eq!(bopds.split_faces().len(), 2);
        assert_eq!(bopds.split_faces()[0].trimming_loops.len(), 2);
        assert_eq!(bopds.split_faces()[0].splitting_edges, vec![first_section, SectionCurveId(99)]);
        assert_eq!(bopds.split_faces()[1].trimming_loops.len(), 1);
        assert_eq!(bopds.split_faces()[1].splitting_edges, vec![second_section]);
    }

    #[test]
    fn split_face_records_assign_inner_loop_to_containing_outer_fragment_only() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let outer_a = SectionCurveId(11);
        let outer_b = SectionCurveId(12);
        let hole = SectionCurveId(13);
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(12), VertexId(13), VertexId(14), VertexId(15)],
            edges: vec![TrimmingEdge {
                section_curve: Some(outer_a),
                uv_points: vec![Point2::new(0.0, 0.0), Point2::new(4.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(0.0, 0.0),
                Point2::new(4.0, 0.0),
                Point2::new(4.0, 4.0),
                Point2::new(0.0, 4.0),
                Point2::new(0.0, 0.0),
            ],
            signed_area: -16.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(16), VertexId(17), VertexId(18), VertexId(19)],
            edges: vec![TrimmingEdge {
                section_curve: Some(outer_b),
                uv_points: vec![Point2::new(5.0, 0.0), Point2::new(7.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(5.0, 0.0),
                Point2::new(7.0, 0.0),
                Point2::new(7.0, 2.0),
                Point2::new(5.0, 2.0),
                Point2::new(5.0, 0.0),
            ],
            signed_area: -4.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(20), VertexId(21), VertexId(22), VertexId(23)],
            edges: vec![TrimmingEdge {
                section_curve: Some(hole),
                uv_points: vec![Point2::new(1.0, 1.0), Point2::new(2.0, 1.0)],
            }],
            uv_points: vec![
                Point2::new(1.0, 1.0),
                Point2::new(2.0, 1.0),
                Point2::new(2.0, 2.0),
                Point2::new(1.0, 2.0),
                Point2::new(1.0, 1.0),
            ],
            signed_area: 1.0,
            is_outer: false,
        });

        let built = build_split_faces(&mut bopds);

        assert_eq!(built, 2);
        assert_eq!(bopds.split_faces()[0].trimming_loops.len(), 2);
        assert_eq!(bopds.split_faces()[0].splitting_edges, vec![outer_a, hole]);
        assert_eq!(bopds.split_faces()[1].trimming_loops.len(), 1);
        assert_eq!(bopds.split_faces()[1].splitting_edges, vec![outer_b]);
    }

    #[test]
    fn split_face_records_do_not_duplicate_inner_loop_across_nested_outer_fragments() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let outer = SectionCurveId(21);
        let nested_outer = SectionCurveId(22);
        let hole = SectionCurveId(23);
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(24), VertexId(25), VertexId(26), VertexId(27)],
            edges: vec![TrimmingEdge {
                section_curve: Some(outer),
                uv_points: vec![Point2::new(0.0, 0.0), Point2::new(6.0, 0.0)],
            }],
            uv_points: vec![
                Point2::new(0.0, 0.0),
                Point2::new(6.0, 0.0),
                Point2::new(6.0, 6.0),
                Point2::new(0.0, 6.0),
                Point2::new(0.0, 0.0),
            ],
            signed_area: -36.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(28), VertexId(29), VertexId(30), VertexId(31)],
            edges: vec![TrimmingEdge {
                section_curve: Some(nested_outer),
                uv_points: vec![Point2::new(1.0, 1.0), Point2::new(5.0, 1.0)],
            }],
            uv_points: vec![
                Point2::new(1.0, 1.0),
                Point2::new(5.0, 1.0),
                Point2::new(5.0, 5.0),
                Point2::new(1.0, 5.0),
                Point2::new(1.0, 1.0),
            ],
            signed_area: -16.0,
            is_outer: true,
        });
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(32), VertexId(33), VertexId(34), VertexId(35)],
            edges: vec![TrimmingEdge {
                section_curve: Some(hole),
                uv_points: vec![Point2::new(2.0, 2.0), Point2::new(3.0, 2.0)],
            }],
            uv_points: vec![
                Point2::new(2.0, 2.0),
                Point2::new(3.0, 2.0),
                Point2::new(3.0, 3.0),
                Point2::new(2.0, 3.0),
                Point2::new(2.0, 2.0),
            ],
            signed_area: 1.0,
            is_outer: false,
        });

        build_split_faces(&mut bopds);

        assert_eq!(bopds.split_faces()[0].trimming_loops.len(), 1);
        assert_eq!(bopds.split_faces()[0].splitting_edges, vec![outer]);
        assert_eq!(bopds.split_faces()[1].trimming_loops.len(), 2);
        assert_eq!(bopds.split_faces()[1].splitting_edges, vec![nested_outer, hole]);
    }

    #[test]
    fn split_face_records_skip_faces_without_outer_loops() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(40), VertexId(41), VertexId(42), VertexId(43)],
            edges: vec![TrimmingEdge {
                section_curve: Some(SectionCurveId(7)),
                uv_points: vec![Point2::new(0.25, 0.25), Point2::new(0.75, 0.25)],
            }],
            uv_points: vec![
                Point2::new(0.25, 0.25),
                Point2::new(0.75, 0.25),
                Point2::new(0.75, 0.75),
                Point2::new(0.25, 0.75),
                Point2::new(0.25, 0.25),
            ],
            signed_area: -0.25,
            is_outer: false,
        });

        let built = build_split_faces(&mut bopds);

        assert_eq!(built, 0);
        assert!(bopds.split_faces().is_empty());
    }

    #[test]
    fn fragment_classification_uses_loop_centroid_when_it_is_inside_fragment() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face_id = bopds.register_face_source(0);
        bopds.push_split_face(SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![outer_loop(face_id, square_loop(0.25, 0.75))],
            splitting_edges: vec![],
            representative_point: None,
            classification: None,
        });

        let opposite = operand_box(1, Point3::new(0.0, 0.0, -1.0), Point3::new(1.0, 1.0, 1.0));
        let classified = classify_split_faces_against_operand(&mut bopds, &[(1, opposite)]).unwrap();

        assert_eq!(classified, 1);
        let fragment = &bopds.split_faces()[0];
        assert_eq!(fragment.representative_point, Some(Point3::new(0.5, 0.5, 0.0)));
        assert_eq!(fragment.classification, Some(PointClassification::Inside));
    }

    #[test]
    fn fragment_classification_falls_back_to_sample_point_when_centroid_is_in_hole() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face_id = bopds.register_face_source(0);
        bopds.push_split_face(SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![
                outer_loop(face_id, square_loop(0.0, 2.0)),
                inner_loop(face_id, square_loop(0.5, 1.5)),
            ],
            splitting_edges: vec![],
            representative_point: None,
            classification: None,
        });

        let opposite = operand_box(1, Point3::new(-1.0, -1.0, -1.0), Point3::new(0.25, 0.25, 1.0));
        classify_split_faces_against_operand(&mut bopds, &[(1, opposite)]).unwrap();

        let fragment = &bopds.split_faces()[0];
        assert_eq!(fragment.representative_point, Some(Point3::new(1.0, 1.0, 0.0)));
        assert_eq!(fragment.classification, Some(PointClassification::Outside));
    }

    #[test]
    fn fragment_classification_skips_unpaired_operand_fragments() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face_id = bopds.register_face_source(2);
        bopds.push_split_face(SplitFace {
            original_face: face_id,
            operand_rank: 2,
            trimming_loops: vec![outer_loop(face_id, square_loop(0.0, 1.0))],
            splitting_edges: vec![],
            representative_point: None,
            classification: None,
        });

        let classified = classify_split_faces_against_operand(&mut bopds, &[(0, operand_box(0, Point3::new(0.0, 0.0, -1.0), Point3::new(1.0, 1.0, 1.0)))]).unwrap();

        assert_eq!(classified, 0);
        assert_eq!(bopds.split_faces()[0].representative_point, None);
        assert_eq!(bopds.split_faces()[0].classification, None);
    }

    #[test]
    fn boolean_selection_common_selects_inside_and_boundary_fragments() {
        let bopds = bopds_with_classified_fragments(vec![
            (0, PointClassification::Inside),
            (0, PointClassification::OnBoundary),
            (1, PointClassification::Outside),
        ]);

        let selected = select_split_faces_for_boolean_op(&bopds, BooleanOp::Common);

        assert_eq!(selected.len(), 2);
        assert_eq!(selected[0].classification, Some(PointClassification::Inside));
        assert_eq!(selected[1].classification, Some(PointClassification::OnBoundary));
    }

    #[test]
    fn boolean_selection_fuse_selects_outside_and_boundary_fragments_from_both_operands() {
        let bopds = bopds_with_classified_fragments(vec![
            (0, PointClassification::Outside),
            (0, PointClassification::Inside),
            (1, PointClassification::OnBoundary),
            (1, PointClassification::Outside),
        ]);

        let selected = select_split_faces_for_boolean_op(&bopds, BooleanOp::Fuse);

        assert_eq!(selected.len(), 3);
        assert_eq!(selected[0].operand_rank, 0);
        assert_eq!(selected[0].classification, Some(PointClassification::Outside));
        assert_eq!(selected[1].operand_rank, 1);
        assert_eq!(selected[1].classification, Some(PointClassification::OnBoundary));
        assert_eq!(selected[2].operand_rank, 1);
        assert_eq!(selected[2].classification, Some(PointClassification::Outside));
    }

    #[test]
    fn boolean_selection_cut_selects_only_operand_a_outside_and_boundary_fragments() {
        let bopds = bopds_with_classified_fragments(vec![
            (0, PointClassification::Outside),
            (0, PointClassification::OnBoundary),
            (0, PointClassification::Inside),
            (1, PointClassification::Outside),
            (1, PointClassification::OnBoundary),
        ]);

        let selected = select_split_faces_for_boolean_op(&bopds, BooleanOp::Cut);

        assert_eq!(selected.len(), 3);
        assert_eq!(selected[0].operand_rank, 0);
        assert_eq!(selected[0].classification, Some(PointClassification::Outside));
        assert_eq!(selected[1].operand_rank, 0);
        assert_eq!(selected[1].classification, Some(PointClassification::OnBoundary));
        assert_eq!(selected[2].operand_rank, 1);
        assert_eq!(selected[2].classification, Some(PointClassification::OnBoundary));
    }

    #[test]
    fn boolean_selection_skips_unclassified_fragments() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face_id = bopds.register_face_source(0);
        bopds.push_split_face(SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![outer_loop(face_id, square_loop(0.0, 1.0))],
            splitting_edges: vec![],
            representative_point: None,
            classification: None,
        });

        let selected = select_split_faces_for_boolean_op(&bopds, BooleanOp::Fuse);

        assert!(selected.is_empty());
    }

    #[test]
    fn vertex_merging_merges_vertices_within_geometric_tolerance() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-3,
            ..BopOptions::default()
        });
        let face_id = bopds.register_face_source(0);
        let selected = vec![SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![test_loop(
                face_id,
                0,
                vec![
                    Point2::new(0.0, 0.0),
                    Point2::new(1.0, 0.0),
                    Point2::new(1.0, 1.0),
                    Point2::new(0.0, 1.0),
                    Point2::new(0.0, 0.0),
                ],
                true,
            )],
            splitting_edges: vec![],
            representative_point: None,
            classification: Some(PointClassification::Inside),
        }, SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![test_loop(
                face_id,
                1,
                vec![
                    Point2::new(0.0005, 0.0004),
                    Point2::new(2.0, 0.0),
                    Point2::new(2.0, 1.0),
                    Point2::new(0.0004, 1.0003),
                    Point2::new(0.0005, 0.0004),
                ],
                true,
            )],
            splitting_edges: vec![],
            representative_point: None,
            classification: Some(PointClassification::Inside),
        }];

        let map = merge_equivalent_vertices(&mut bopds, &selected);

        let origin = selected[0].trimming_loops[0].vertex_ids[0];
        let near_origin = selected[1].trimming_loops[0].vertex_ids[0];
        let top_left = selected[0].trimming_loops[0].vertex_ids[3];
        let near_top_left = selected[1].trimming_loops[0].vertex_ids[3];
        assert_eq!(map.get(&origin), Some(&origin.min(near_origin)));
        assert_eq!(map.get(&near_origin), Some(&origin.min(near_origin)));
        assert_eq!(map.get(&top_left), Some(&top_left.min(near_top_left)));
        assert_eq!(map.get(&near_top_left), Some(&top_left.min(near_top_left)));
        assert_eq!(bopds.merged_vertices().len(), 6);
    }

    #[test]
    fn vertex_merging_preserves_distinct_vertices_outside_tolerance() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-4,
            ..BopOptions::default()
        });
        let face_id = bopds.register_face_source(0);
        let selected = vec![SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![test_loop(
                face_id,
                0,
                vec![
                    Point2::new(0.0, 0.0),
                    Point2::new(1.0, 0.0),
                    Point2::new(1.0, 1.0),
                    Point2::new(0.0, 1.0),
                    Point2::new(0.0, 0.0),
                ],
                true,
            )],
            splitting_edges: vec![],
            representative_point: None,
            classification: Some(PointClassification::Inside),
        }, SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![test_loop(
                face_id,
                1,
                vec![
                    Point2::new(0.001, 0.0),
                    Point2::new(2.0, 0.0),
                    Point2::new(2.0, 1.0),
                    Point2::new(0.001, 1.0),
                    Point2::new(0.001, 0.0),
                ],
                true,
            )],
            splitting_edges: vec![],
            representative_point: None,
            classification: Some(PointClassification::Inside),
        }];

        let map = merge_equivalent_vertices(&mut bopds, &selected);

        let first = selected[0].trimming_loops[0].vertex_ids[0];
        let second = selected[1].trimming_loops[0].vertex_ids[0];
        assert_eq!(map.get(&first), Some(&first));
        assert_eq!(map.get(&second), Some(&second));
        assert_ne!(map.get(&first), map.get(&second));
    }

    #[test]
    fn vertex_merging_rebuilds_equivalence_map_when_re_run() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-3,
            ..BopOptions::default()
        });
        let face_id = bopds.register_face_source(0);
        let first = vec![SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![test_loop(face_id, 0, square_loop(0.0, 1.0), true)],
            splitting_edges: vec![],
            representative_point: None,
            classification: Some(PointClassification::Inside),
        }];

        let first_map = merge_equivalent_vertices(&mut bopds, &first);
        assert_eq!(bopds.merged_vertices().len(), 4);
        assert_eq!(first_map.len(), 4);

        let second = vec![SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![test_loop(face_id, 1, square_loop(10.0, 11.0), true)],
            splitting_edges: vec![],
            representative_point: None,
            classification: Some(PointClassification::Outside),
        }];

        let second_map = merge_equivalent_vertices(&mut bopds, &second);
        assert_eq!(bopds.merged_vertices().len(), 4);
        assert_eq!(second_map.len(), 4);
        assert!(second_map.keys().all(|id| !first_map.contains_key(id)));
    }

    #[test]
    fn vertex_merging_merges_transitive_tolerance_chain() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-3,
            ..BopOptions::default()
        });
        let face_id = bopds.register_face_source(0);
        let selected = vec![SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![test_loop(
                face_id,
                0,
                vec![
                    Point2::new(0.0, 0.0),
                    Point2::new(0.0009, 0.0),
                    Point2::new(0.0018, 0.0),
                    Point2::new(0.0018, 1.0),
                    Point2::new(0.0, 0.0),
                ],
                true,
            )],
            splitting_edges: vec![],
            representative_point: None,
            classification: Some(PointClassification::Inside),
        }];

        let map = merge_equivalent_vertices(&mut bopds, &selected);

        let a = selected[0].trimming_loops[0].vertex_ids[0];
        let b = selected[0].trimming_loops[0].vertex_ids[1];
        let c = selected[0].trimming_loops[0].vertex_ids[2];
        let canonical = a.min(b).min(c);
        assert_eq!(map.get(&a), Some(&canonical));
        assert_eq!(map.get(&b), Some(&canonical));
        assert_eq!(map.get(&c), Some(&canonical));
        assert_eq!(bopds.merged_vertices().len(), 2);
    }

    #[test]
    fn edge_sewing_identifies_edges_with_same_endpoints_across_fragments() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face_a = bopds.register_face_source(0);
        let face_b = bopds.register_face_source(1);
        let split_faces = vec![
            SplitFace {
                original_face: face_a,
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: face_a,
                    vertex_ids: vec![VertexId(10), VertexId(11)],
                    edges: vec![line_edge(None, Point2::new(0.0, 0.0), Point2::new(1.0, 0.0))],
                    uv_points: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
                    signed_area: 0.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::Inside),
            },
            SplitFace {
                original_face: face_b,
                operand_rank: 1,
                trimming_loops: vec![TrimmingLoop {
                    face: face_b,
                    vertex_ids: vec![VertexId(20), VertexId(21)],
                    edges: vec![line_edge(None, Point2::new(1.0, 0.0), Point2::new(0.0, 0.0))],
                    uv_points: vec![Point2::new(1.0, 0.0), Point2::new(0.0, 0.0)],
                    signed_area: 0.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::Inside),
            },
        ];

        let merged = FxHashMap::from_iter([
            (VertexId(10), VertexId(1)),
            (VertexId(11), VertexId(2)),
            (VertexId(20), VertexId(2)),
            (VertexId(21), VertexId(1)),
        ]);

        let paths = sew_fragment_edges(&mut bopds, &split_faces, &merged);

        assert_eq!(paths.len(), 1);
        assert!(paths[0].is_closed);
        assert_eq!(paths[0].edges.len(), 2);
        let endpoints: Vec<_> = paths[0]
            .edges
            .iter()
            .map(|edge| (edge.start_vertex, edge.end_vertex))
            .collect();
        assert_eq!(endpoints, vec![(VertexId(1), VertexId(2)), (VertexId(2), VertexId(1))]);
        let first_source = paths[0].edges[0].source;
        let second_source = paths[0].edges[1].source;
        assert_eq!(first_source.face, face_a);
        assert_eq!(second_source.face, face_b);
        assert_eq!(paths[0].edges[0].sewn_pair, Some(SewnEdgePair::new(first_source, second_source)));
        assert_eq!(paths[0].edges[1].sewn_pair, Some(SewnEdgePair::new(first_source, second_source)));
        assert_eq!(bopds.sewn_paths(), paths.as_slice());
    }

    #[test]
    fn edge_sewing_handles_reverse_orientation_across_fragments() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face_a = bopds.register_face_source(0);
        let face_b = bopds.register_face_source(1);
        let split_faces = vec![
            SplitFace {
                original_face: face_a,
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: face_a,
                    vertex_ids: vec![VertexId(20), VertexId(21)],
                    edges: vec![line_edge(None, Point2::new(0.0, 0.0), Point2::new(1.0, 0.0))],
                    uv_points: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
                    signed_area: 0.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::Inside),
            },
            SplitFace {
                original_face: face_b,
                operand_rank: 1,
                trimming_loops: vec![TrimmingLoop {
                    face: face_b,
                    vertex_ids: vec![VertexId(22), VertexId(23)],
                    edges: vec![line_edge(None, Point2::new(0.0, 0.0), Point2::new(1.0, 0.0))],
                    uv_points: vec![Point2::new(1.0, 0.0), Point2::new(0.0, 0.0)],
                    signed_area: 0.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::Inside),
            },
        ];

        let merged = FxHashMap::from_iter([
            (VertexId(20), VertexId(1)),
            (VertexId(21), VertexId(2)),
            (VertexId(22), VertexId(1)),
            (VertexId(23), VertexId(2)),
        ]);

        let paths = sew_fragment_edges(&mut bopds, &split_faces, &merged);

        assert_eq!(paths.len(), 1);
        assert!(paths[0].is_closed);
        assert!(!paths[0].edges[1].reversed);
        assert_eq!(paths[0].edges[1].start_vertex, VertexId(2));
        assert_eq!(paths[0].edges[1].end_vertex, VertexId(1));
    }

    #[test]
    fn edge_sewing_forms_continuous_open_path() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face_id = bopds.register_face_source(0);
        let split_faces = vec![SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![TrimmingLoop {
                face: face_id,
                vertex_ids: vec![VertexId(30), VertexId(31), VertexId(32)],
                edges: vec![
                    line_edge(Some(SectionCurveId(7)), Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)),
                    line_edge(Some(SectionCurveId(8)), Point2::new(2.0, 0.0), Point2::new(1.0, 0.0)),
                ],
                uv_points: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0), Point2::new(2.0, 0.0)],
                signed_area: 0.0,
                is_outer: true,
            }],
            splitting_edges: vec![SectionCurveId(7), SectionCurveId(8)],
            representative_point: None,
            classification: Some(PointClassification::OnBoundary),
        }];

        let merged = FxHashMap::from_iter([
            (VertexId(30), VertexId(100)),
            (VertexId(31), VertexId(101)),
            (VertexId(32), VertexId(102)),
        ]);

        let paths = sew_fragment_edges(&mut bopds, &split_faces, &merged);

        assert_eq!(paths.len(), 1);
        assert!(!paths[0].is_closed);
        assert_eq!(paths[0].edges.len(), 2);
        assert_eq!(paths[0].edges[0].end_vertex, paths[0].edges[1].start_vertex);
        assert_eq!(paths[0].edges[0].section_curve, Some(SectionCurveId(7)));
        assert_eq!(paths[0].edges[1].section_curve, Some(SectionCurveId(8)));
        assert!(paths[0].edges[1].reversed);
    }

    #[test]
    fn edge_sewing_ignores_single_face_loop_adjacency() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face_id = bopds.register_face_source(0);
        let split_faces = vec![SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![TrimmingLoop {
                face: face_id,
                vertex_ids: vec![VertexId(40), VertexId(41), VertexId(42), VertexId(43)],
                edges: vec![
                    line_edge(None, Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)),
                    line_edge(None, Point2::new(1.0, 0.0), Point2::new(1.0, 1.0)),
                    line_edge(None, Point2::new(1.0, 1.0), Point2::new(0.0, 1.0)),
                    line_edge(None, Point2::new(0.0, 1.0), Point2::new(0.0, 0.0)),
                ],
                uv_points: square_loop(0.0, 1.0),
                signed_area: -1.0,
                is_outer: true,
            }],
            splitting_edges: vec![],
            representative_point: None,
            classification: Some(PointClassification::Inside),
        }];

        let merged = FxHashMap::from_iter([
            (VertexId(40), VertexId(1)),
            (VertexId(41), VertexId(2)),
            (VertexId(42), VertexId(3)),
            (VertexId(43), VertexId(4)),
        ]);

        let paths = sew_fragment_edges(&mut bopds, &split_faces, &merged);

        assert!(paths.is_empty());
        assert!(bopds.sewn_paths().is_empty());
    }

    #[test]
    fn edge_sewing_only_keeps_true_shared_fragment_edges() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face_a = bopds.register_face_source(0);
        let face_b = bopds.register_face_source(1);
        let split_faces = vec![
            SplitFace {
                original_face: face_a,
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: face_a,
                    vertex_ids: vec![VertexId(50), VertexId(51), VertexId(52), VertexId(53)],
                    edges: vec![
                        line_edge(None, Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)),
                        line_edge(None, Point2::new(1.0, 0.0), Point2::new(1.0, 1.0)),
                        line_edge(None, Point2::new(1.0, 1.0), Point2::new(0.0, 1.0)),
                        line_edge(None, Point2::new(0.0, 1.0), Point2::new(0.0, 0.0)),
                    ],
                    uv_points: square_loop(0.0, 1.0),
                    signed_area: -1.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::Inside),
            },
            SplitFace {
                original_face: face_b,
                operand_rank: 1,
                trimming_loops: vec![TrimmingLoop {
                    face: face_b,
                    vertex_ids: vec![VertexId(60), VertexId(61), VertexId(62), VertexId(63)],
                    edges: vec![
                        line_edge(None, Point2::new(1.0, 0.0), Point2::new(0.0, 0.0)),
                        line_edge(None, Point2::new(0.0, 0.0), Point2::new(0.0, -1.0)),
                        line_edge(None, Point2::new(0.0, -1.0), Point2::new(1.0, -1.0)),
                        line_edge(None, Point2::new(1.0, -1.0), Point2::new(1.0, 0.0)),
                    ],
                    uv_points: vec![
                        Point2::new(1.0, 0.0),
                        Point2::new(0.0, 0.0),
                        Point2::new(0.0, -1.0),
                        Point2::new(1.0, -1.0),
                        Point2::new(1.0, 0.0),
                    ],
                    signed_area: -1.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::Inside),
            },
        ];

        let merged = FxHashMap::from_iter([
            (VertexId(50), VertexId(1)),
            (VertexId(51), VertexId(2)),
            (VertexId(52), VertexId(3)),
            (VertexId(53), VertexId(4)),
            (VertexId(60), VertexId(2)),
            (VertexId(61), VertexId(1)),
            (VertexId(62), VertexId(5)),
            (VertexId(63), VertexId(6)),
        ]);

        let paths = sew_fragment_edges(&mut bopds, &split_faces, &merged);

        assert_eq!(paths.len(), 1);
        assert!(paths[0].is_closed);
        assert_eq!(paths[0].edges.len(), 2);
        let sewn_faces: Vec<_> = paths[0].edges.iter().map(|edge| edge.face).collect();
        assert_eq!(sewn_faces, vec![face_a, face_b]);
        let sewn_pairs: Vec<_> = paths[0]
            .edges
            .iter()
            .map(|edge| undirected_edge_key(edge.start_vertex, edge.end_vertex))
            .collect();
        assert_eq!(sewn_pairs, vec![(VertexId(1), VertexId(2)), (VertexId(1), VertexId(2))]);
        assert_eq!(paths[0].edges[0].source.original_edge, None);
        assert_eq!(paths[0].edges[1].source.original_edge, None);
        assert_eq!(paths[0].edges[0].sewn_pair, Some(SewnEdgePair::new(paths[0].edges[0].source, paths[0].edges[1].source)));
        assert_eq!(paths[0].edges[1].sewn_pair, Some(SewnEdgePair::new(paths[0].edges[0].source, paths[0].edges[1].source)));
    }

    #[test]
    fn edge_sewing_tracks_section_edge_identity_for_open_paths() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face_id = bopds.register_face_source(0);
        let split_faces = vec![SplitFace {
            original_face: face_id,
            operand_rank: 0,
            trimming_loops: vec![TrimmingLoop {
                face: face_id,
                vertex_ids: vec![VertexId(70), VertexId(71), VertexId(72)],
                edges: vec![
                    line_edge(Some(SectionCurveId(7)), Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)),
                    line_edge(Some(SectionCurveId(8)), Point2::new(1.0, 0.0), Point2::new(2.0, 0.0)),
                ],
                uv_points: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0), Point2::new(2.0, 0.0)],
                signed_area: 0.0,
                is_outer: true,
            }],
            splitting_edges: vec![SectionCurveId(7), SectionCurveId(8)],
            representative_point: None,
            classification: Some(PointClassification::OnBoundary),
        }];

        let merged = FxHashMap::from_iter([
            (VertexId(70), VertexId(100)),
            (VertexId(71), VertexId(101)),
            (VertexId(72), VertexId(102)),
        ]);

        let paths = sew_fragment_edges(&mut bopds, &split_faces, &merged);

        assert_eq!(paths.len(), 1);
        assert_eq!(paths[0].edges.len(), 2);
        assert_eq!(paths[0].edges[0].source.original_edge, Some(EdgeId(7)));
        assert_eq!(paths[0].edges[1].source.original_edge, Some(EdgeId(8)));
        assert!(paths[0].edges.iter().all(|edge| edge.sewn_pair.is_none()));
    }

    #[test]
    fn face_orientation_flips_adjacent_face_to_close_shell() {
        let shell = box_shell_with_inverted_top();
        let faces: Vec<_> = shell.face_iter().cloned().collect();
        let split_faces = split_faces_for_source_faces(&faces);
        let faces_by_id = source_face_map(&split_faces, &faces);

        let shells = assemble_shells(&split_faces, &faces_by_id).unwrap();

        assert_eq!(shells.len(), 1);
        assert_eq!(shells[0].shell_condition(), ShellCondition::Closed);
    }

    #[test]
    fn shell_closure_rejects_boundary_edge_component() {
        let shell = open_box_shell_missing_top();
        let faces: Vec<_> = shell.face_iter().cloned().collect();
        let split_faces = split_faces_for_source_faces(&faces);
        let faces_by_id = source_face_map(&split_faces, &faces);

        let err = assemble_shells(&split_faces, &faces_by_id).unwrap_err();

        assert!(matches!(err, BopError::TopologyInvariantBroken));
    }

    #[test]
    fn shell_assembly_separates_disconnected_closed_components() {
        let left = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]));
        let right: Solid<Point3, Curve, Surface> = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]))
            .mapped(
                &|point: &Point3| *point + Vector3::new(3.0, 0.0, 0.0),
                &|curve: &Curve| curve.clone(),
                &|surface: &Surface| surface.clone(),
            );
        let mut faces: Vec<_> = left.boundaries()[0].face_iter().cloned().collect();
        faces.extend(right.boundaries()[0].face_iter().cloned());
        let split_faces = split_faces_for_source_faces(&faces);
        let faces_by_id = source_face_map(&split_faces, &faces);

        let shells = assemble_shells(&split_faces, &faces_by_id).unwrap();

        assert_eq!(shells.len(), 2);
        assert!(shells.iter().all(|shell| shell.shell_condition() == ShellCondition::Closed));
    }

    #[test]
    fn shell_assembly_groups_faces_by_shared_topology_components() {
        let face_a = FaceId(0);
        let face_b = FaceId(1);
        let face_c = FaceId(2);
        let face_d = FaceId(3);
        let split_faces = vec![
            SplitFace {
                original_face: face_a,
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: face_a,
                    vertex_ids: vec![VertexId(0), VertexId(1), VertexId(2), VertexId(3)],
                    edges: vec![
                        line_edge(None, Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)),
                        line_edge(None, Point2::new(1.0, 0.0), Point2::new(1.0, 1.0)),
                        line_edge(None, Point2::new(1.0, 1.0), Point2::new(0.0, 1.0)),
                        line_edge(None, Point2::new(0.0, 1.0), Point2::new(0.0, 0.0)),
                    ],
                    uv_points: square_loop(0.0, 1.0),
                    signed_area: -1.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::OnBoundary),
            },
            SplitFace {
                original_face: face_b,
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: face_b,
                    vertex_ids: vec![VertexId(10), VertexId(1), VertexId(11), VertexId(12)],
                    edges: vec![
                        line_edge(None, Point2::new(2.0, 0.0), Point2::new(1.0, 0.0)),
                        line_edge(None, Point2::new(1.0, 0.0), Point2::new(1.0, -1.0)),
                        line_edge(None, Point2::new(1.0, -1.0), Point2::new(2.0, -1.0)),
                        line_edge(None, Point2::new(2.0, -1.0), Point2::new(2.0, 0.0)),
                    ],
                    uv_points: vec![
                        Point2::new(2.0, 0.0),
                        Point2::new(1.0, 0.0),
                        Point2::new(1.0, -1.0),
                        Point2::new(2.0, -1.0),
                        Point2::new(2.0, 0.0),
                    ],
                    signed_area: -1.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::OnBoundary),
            },
            SplitFace {
                original_face: face_c,
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: face_c,
                    vertex_ids: vec![VertexId(20), VertexId(21), VertexId(22), VertexId(23)],
                    edges: vec![
                        line_edge(None, Point2::new(5.0, 0.0), Point2::new(6.0, 0.0)),
                        line_edge(None, Point2::new(6.0, 0.0), Point2::new(6.0, 1.0)),
                        line_edge(None, Point2::new(6.0, 1.0), Point2::new(5.0, 1.0)),
                        line_edge(None, Point2::new(5.0, 1.0), Point2::new(5.0, 0.0)),
                    ],
                    uv_points: vec![
                        Point2::new(5.0, 0.0),
                        Point2::new(6.0, 0.0),
                        Point2::new(6.0, 1.0),
                        Point2::new(5.0, 1.0),
                        Point2::new(5.0, 0.0),
                    ],
                    signed_area: -1.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::OnBoundary),
            },
            SplitFace {
                original_face: face_d,
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: face_d,
                    vertex_ids: vec![VertexId(30), VertexId(21), VertexId(31), VertexId(32)],
                    edges: vec![
                        line_edge(None, Point2::new(6.0, 0.0), Point2::new(5.0, 0.0)),
                        line_edge(None, Point2::new(5.0, 0.0), Point2::new(5.0, -1.0)),
                        line_edge(None, Point2::new(5.0, -1.0), Point2::new(6.0, -1.0)),
                        line_edge(None, Point2::new(6.0, -1.0), Point2::new(6.0, 0.0)),
                    ],
                    uv_points: vec![
                        Point2::new(6.0, 0.0),
                        Point2::new(5.0, 0.0),
                        Point2::new(5.0, -1.0),
                        Point2::new(6.0, -1.0),
                        Point2::new(6.0, 0.0),
                    ],
                    signed_area: -1.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::OnBoundary),
            },
        ];

        let components = connected_face_components(&split_faces);

        assert_eq!(components, vec![vec![0, 1], vec![2, 3]]);
    }

    #[test]
    fn shell_assembly_groups_faces_by_section_curve_identity() {
        let shared = SectionCurveId(77);
        let split_faces = vec![
            SplitFace {
                original_face: FaceId(0),
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: FaceId(0),
                    vertex_ids: vec![VertexId(0), VertexId(1), VertexId(2), VertexId(3)],
                    edges: vec![
                        line_edge(Some(shared), Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)),
                        line_edge(None, Point2::new(1.0, 0.0), Point2::new(1.0, 1.0)),
                        line_edge(None, Point2::new(1.0, 1.0), Point2::new(0.0, 1.0)),
                        line_edge(None, Point2::new(0.0, 1.0), Point2::new(0.0, 0.0)),
                    ],
                    uv_points: square_loop(0.0, 1.0),
                    signed_area: -1.0,
                    is_outer: true,
                }],
                splitting_edges: vec![shared],
                representative_point: None,
                classification: Some(PointClassification::OnBoundary),
            },
            SplitFace {
                original_face: FaceId(1),
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: FaceId(1),
                    vertex_ids: vec![VertexId(10), VertexId(11), VertexId(12), VertexId(13)],
                    edges: vec![
                        line_edge(Some(shared), Point2::new(1.0, 0.0), Point2::new(0.0, 0.0)),
                        line_edge(None, Point2::new(0.0, 0.0), Point2::new(0.0, -1.0)),
                        line_edge(None, Point2::new(0.0, -1.0), Point2::new(1.0, -1.0)),
                        line_edge(None, Point2::new(1.0, -1.0), Point2::new(1.0, 0.0)),
                    ],
                    uv_points: vec![
                        Point2::new(1.0, 0.0),
                        Point2::new(0.0, 0.0),
                        Point2::new(0.0, -1.0),
                        Point2::new(1.0, -1.0),
                        Point2::new(1.0, 0.0),
                    ],
                    signed_area: -1.0,
                    is_outer: true,
                }],
                splitting_edges: vec![shared],
                representative_point: None,
                classification: Some(PointClassification::OnBoundary),
            },
        ];

        let components = connected_face_components(&split_faces);

        assert_eq!(components, vec![vec![0, 1]]);
    }

    #[test]
    fn shell_orientation_rejects_closed_shell_with_inverted_faces() {
        let shell = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]))
        .into_boundaries()
        .pop()
        .unwrap()
        .inverse();
        let shell: truck_topology::Shell<Point3, Curve, Surface> = shell.into();
        assert_eq!(shell.shell_condition(), ShellCondition::Closed);

        let faces: Vec<_> = shell.into_iter().collect();
        let split_faces = split_faces_for_source_faces(&faces);
        let faces_by_id = source_face_map(&split_faces, &faces);

        let err = assemble_shells(&split_faces, &faces_by_id).unwrap_err();

        assert!(matches!(err, BopError::TopologyInvariantBroken));
    }

    #[test]
    fn shell_assembly_rebuilds_distinct_faces_for_multiple_fragments_from_same_source_face() {
        let source_face = unit_square_face();
        let first = SplitFace {
            original_face: FaceId(0),
            operand_rank: 0,
            trimming_loops: vec![outer_loop(FaceId(0), square_loop(0.0, 0.5))],
            splitting_edges: vec![],
            representative_point: None,
            classification: Some(PointClassification::OnBoundary),
        };
        let second = SplitFace {
            original_face: FaceId(0),
            operand_rank: 0,
            trimming_loops: vec![outer_loop(FaceId(0), square_loop(0.5, 1.0))],
            splitting_edges: vec![],
            representative_point: None,
            classification: Some(PointClassification::OnBoundary),
        };

        let rebuilt_first = rebuild_face_from_split_face(&first, &source_face).unwrap();
        let rebuilt_second = rebuild_face_from_split_face(&second, &source_face).unwrap();

        assert_ne!(rebuilt_first.id(), rebuilt_second.id());
        let first_points: Vec<_> = rebuilt_first.boundaries()[0].vertex_iter().map(|v| v.point()).collect();
        let second_points: Vec<_> = rebuilt_second.boundaries()[0].vertex_iter().map(|v| v.point()).collect();
        assert_eq!(first_points[0], Point3::new(0.0, 0.0, 0.0));
        assert_eq!(first_points[1], Point3::new(0.5, 0.0, 0.0));
        assert_eq!(second_points[0], Point3::new(0.5, 0.5, 0.0));
        assert_eq!(second_points[1], Point3::new(1.0, 0.5, 0.0));
    }

    #[test]
    fn solid_construction_wraps_each_closed_shell_into_valid_solid() {
        let left = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]))
        .into_boundaries()
        .pop()
        .unwrap();
        let right = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]))
        .mapped(
            &|point: &Point3| *point + Vector3::new(3.0, 0.0, 0.0),
            &|curve: &Curve| curve.clone(),
            &|surface: &Surface| surface.clone(),
        )
        .into_boundaries()
        .pop()
        .unwrap();

        let solids = build_solids_from_shells(vec![left, right]).unwrap();

        assert_eq!(solids.len(), 2);
        assert!(solids.iter().all(|solid| solid.boundaries().len() == 1));
        assert!(solids.iter().all(|solid| solid.boundaries()[0].shell_condition() == ShellCondition::Closed));
    }

    #[test]
    fn solid_construction_rejects_non_closed_shells() {
        let shell = open_box_shell_missing_top();

        let err = build_solids_from_shells(vec![shell]).unwrap_err();

        assert!(matches!(err, BopError::TopologyInvariantBroken));
    }

    fn bopds_with_classified_fragments(classifications: Vec<(u8, PointClassification)>) -> BopDs {
        let mut bopds = BopDs::with_options(BopOptions::default());
        for (index, (operand_rank, classification)) in classifications.into_iter().enumerate() {
            let face_id = bopds.register_face_source(operand_rank);
            bopds.push_split_face(SplitFace {
                original_face: face_id,
                operand_rank,
                trimming_loops: vec![outer_loop(face_id, square_loop(index as f64, index as f64 + 1.0))],
                splitting_edges: vec![],
                representative_point: Some(Point3::new(index as f64 + 0.5, 0.5, 0.0)),
                classification: Some(classification),
            });
        }
        bopds
    }

    fn unit_square_face() -> Face<Point3, Curve, Surface> {
        square_face(
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            None,
        )
    }

    fn square_face_with_square_hole() -> Face<Point3, Curve, Surface> {
        square_face(
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            Some([
                Point3::new(0.25, 0.25, 0.0),
                Point3::new(0.75, 0.25, 0.0),
                Point3::new(0.75, 0.75, 0.0),
                Point3::new(0.25, 0.75, 0.0),
            ]),
        )
    }

    fn square_face(
        outer: [Point3; 4],
        hole: Option<[Point3; 4]>,
    ) -> Face<Point3, Curve, Surface> {
        let outer_vertices = builder::vertices(outer);
        let outer_wire = Wire::from(vec![
            builder::line(&outer_vertices[0], &outer_vertices[1]),
            builder::line(&outer_vertices[1], &outer_vertices[2]),
            builder::line(&outer_vertices[2], &outer_vertices[3]),
            builder::line(&outer_vertices[3], &outer_vertices[0]),
        ]);

        let mut boundaries = vec![outer_wire];
        if let Some(hole) = hole {
            let hole_vertices = builder::vertices(hole);
            let mut hole_wire = Wire::from(vec![
                builder::line(&hole_vertices[0], &hole_vertices[1]),
                builder::line(&hole_vertices[1], &hole_vertices[2]),
                builder::line(&hole_vertices[2], &hole_vertices[3]),
                builder::line(&hole_vertices[3], &hole_vertices[0]),
            ]);
            hole_wire.invert();
            boundaries.push(hole_wire);
        }

        Face::new(
            boundaries,
            Surface::Plane(truck_modeling::Plane::new(outer[0], outer[1], outer[3])),
        )
    }

    fn section_curve(section_curve_id: SectionCurveId) -> SectionCurve {
        SectionCurve {
            id: section_curve_id,
            faces: (FaceId(0), FaceId(1)),
            start: VertexId(10),
            end: VertexId(10),
            samples: vec![
                Point3::new(0.25, 0.25, 0.0),
                Point3::new(0.75, 0.25, 0.0),
                Point3::new(0.75, 0.75, 0.0),
                Point3::new(0.25, 0.75, 0.0),
            ],
            face_parameters: [
                (
                    FaceId(0),
                    vec![
                        Point2::new(0.25, 0.25),
                        Point2::new(0.75, 0.25),
                        Point2::new(0.75, 0.75),
                        Point2::new(0.25, 0.75),
                    ],
                ),
                (
                    FaceId(1),
                    vec![
                        Point2::new(0.25, 0.25),
                        Point2::new(0.75, 0.25),
                        Point2::new(0.75, 0.75),
                        Point2::new(0.25, 0.75),
                    ],
                ),
            ],
            face_projection_available: [(FaceId(0), true), (FaceId(1), true)],
        }
    }

    fn square_loop(min: f64, max: f64) -> Vec<Point2> {
        vec![
            Point2::new(min, min),
            Point2::new(max, min),
            Point2::new(max, max),
            Point2::new(min, max),
            Point2::new(min, min),
        ]
    }

    fn line_edge(section_curve: Option<SectionCurveId>, start: Point2, end: Point2) -> TrimmingEdge {
        TrimmingEdge {
            section_curve,
            uv_points: vec![start, end],
        }
    }

    fn outer_loop(face: FaceId, uv_points: Vec<Point2>) -> TrimmingLoop {
        test_loop(face, 0, uv_points, true)
    }

    fn inner_loop(face: FaceId, mut uv_points: Vec<Point2>) -> TrimmingLoop {
        uv_points.reverse();
        test_loop(face, 1, uv_points, false)
    }

    fn test_loop(face: FaceId, seed: u32, uv_points: Vec<Point2>, is_outer: bool) -> TrimmingLoop {
        TrimmingLoop {
            face,
            vertex_ids: vertex_ids_for_polyline(face, seed, &uv_points),
            edges: vec![],
            uv_points,
            signed_area: if is_outer { -1.0 } else { 1.0 },
            is_outer,
        }
    }

    fn operand_box(rank: u8, min: Point3, max: Point3) -> Solid<Point3, Curve, Surface> {
        let _ = rank;
        primitive::cuboid(BoundingBox::from_iter([min, max]))
    }

    fn split_faces_for_source_faces(faces: &[Face<Point3, Curve, Surface>]) -> Vec<SplitFace> {
        faces
            .iter()
            .enumerate()
            .map(|(index, face)| SplitFace {
                original_face: FaceId(index as u32),
                operand_rank: 0,
                trimming_loops: boundary_loops(FaceId(index as u32), face, 1.0e-9),
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::OnBoundary),
            })
            .collect()
    }

    fn source_face_map(
        split_faces: &[SplitFace],
        faces: &[Face<Point3, Curve, Surface>],
    ) -> FxHashMap<FaceId, Face<Point3, Curve, Surface>> {
        split_faces
            .iter()
            .zip(faces.iter().cloned())
            .map(|(split_face, face)| (split_face.original_face, face))
            .collect()
    }

    fn box_shell_with_inverted_top() -> truck_topology::Shell<Point3, Curve, Surface> {
        let mut shell = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]))
        .into_boundaries()
        .pop()
        .unwrap();
        shell[5].invert();
        shell
    }

    fn open_box_shell_missing_top() -> truck_topology::Shell<Point3, Curve, Surface> {
        let mut shell = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]))
        .into_boundaries()
        .pop()
        .unwrap();
        shell.pop();
        shell
    }
}
