//! Face trimming loop construction.

use crate::{
    bopds::{
        MergedVertex, SectionCurve, SewnEdge, SewnEdgePair, SewnEdgeSource, SewnPath, SplitFace,
        TrimmingEdge, TrimmingLoop,
    },
    classify_point_in_solid, geometry_utils, BooleanOp, BopDs, BopError, EdgeId, FaceId,
    PointClassification, SectionCurveId, VertexId,
};
use rustc_hash::FxHashMap;
use truck_base::cgmath64::{MetricSpace, Point2, Point3, Vector3};
use truck_geotrait::{Invertible, ParametricSurface, SearchNearestParameter, SearchParameter, D2};
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
    let tolerance = bopds
        .options()
        .parametric_tol
        .max(bopds.options().geometric_tol);
    let section_curves = bopds.section_curves().to_vec();
    let mut registry = std::mem::take(&mut bopds.boundary_edge_registry);
    let mut vertex_counter = bopds.next_generated_vertex_id;
    let mut built = 0;

    for &(face_id, ref face) in faces {
        let loops = build_loops_for_face(
            face_id,
            face,
            &section_curves,
            tolerance,
            &mut registry,
            &mut vertex_counter,
        );
        built += loops.len();
        for trimming_loop in loops {
            bopds.push_trimming_loop(trimming_loop);
        }
    }

    bopds.boundary_edge_registry = registry;
    bopds.next_generated_vertex_id = vertex_counter;
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
///
/// `faces` provides the original face geometry so that UV representative points
/// can be evaluated on the actual surface (not projected to Z=0).
pub fn classify_split_faces_against_operand<C, S>(
    bopds: &mut BopDs,
    solids_by_operand: &[(u8, Solid<Point3, C, S>)],
    faces: &[(FaceId, Face<Point3, C, S>)],
) -> Result<usize, BopError>
where
    C: Clone,
    S: Clone
        + Invertible
        + ParametricSurface<Point = Point3, Vector = Vector3>
        + SearchNearestParameter<D2, Point = Point3>,
{
    let surfaces: FxHashMap<FaceId, S> = faces
        .iter()
        .map(|(id, face)| (*id, face.oriented_surface()))
        .collect();
    let split_faces = bopds.split_faces().to_vec();
    let mut classified = 0;

    for (index, split_face) in split_faces.iter().enumerate() {
        let opposite_rank = match split_face.operand_rank {
            0 => 1,
            1 => 0,
            _ => continue,
        };

        let Some((_, solid)) = solids_by_operand
            .iter()
            .find(|(rank, _)| *rank == opposite_rank)
        else {
            continue;
        };

        let surface = surfaces.get(&split_face.original_face);
        let representative_point = representative_point_on_surface(split_face, surface)
            .ok_or(BopError::UnsupportedGeometry)?;
        let classification =
            classify_point_in_solid(solid, representative_point, bopds.options().geometric_tol)?;
        bopds.set_split_face_classification(index, representative_point, classification);
        classified += 1;
    }

    Ok(classified)
}

/// Selects classified split-face fragments according to the requested boolean operation.
pub fn select_split_faces_for_boolean_op(bopds: &BopDs, operation: BooleanOp) -> Vec<SplitFace> {
    bopds
        .split_faces()
        .iter()
        .filter(|split_face| should_select_split_face(split_face, operation))
        .cloned()
        .collect()
}

/// Merges equivalent vertices from the selected fragments and returns an original-to-canonical map.
///
/// `faces` provides the original face geometry so that UV vertex coordinates
/// can be evaluated on the actual surface (not projected to Z=0).
pub fn merge_equivalent_vertices<C, S>(
    bopds: &mut BopDs,
    split_faces: &[SplitFace],
    faces: &[(FaceId, Face<Point3, C, S>)],
) -> FxHashMap<VertexId, VertexId>
where
    C: Clone,
    S: Clone + ParametricSurface<Point = Point3> + Invertible,
{
    let surfaces: FxHashMap<FaceId, S> = faces
        .iter()
        .map(|(id, face)| (*id, face.oriented_surface()))
        .collect();
    let tolerance = bopds.options().geometric_tol;
    let mut samples = collect_fragment_vertices(split_faces, &surfaces);
    samples.sort_by(|lhs, rhs| {
        lhs.1
            .x
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
///
/// When `force_rebuild` is true, all faces are rebuilt through a shared
/// TopologyCache to ensure cross-operand edge sharing. Set to false when
/// testing with faces that already have valid shared topology.
pub fn assemble_shells<C, S>(
    bopds: &mut BopDs,
    split_faces: &[SplitFace],
    faces_by_id: &FxHashMap<FaceId, Face<Point3, C, S>>,
    merged_map: &FxHashMap<VertexId, VertexId>,
) -> Result<Vec<truck_topology::Shell<Point3, C, S>>, BopError>
where
    C: Clone
        + truck_geotrait::ParametricCurve<Point = Point3>
        + truck_geotrait::BoundedCurve
        + Invertible,
    S: Clone
        + Invertible
        + ParametricSurface<Point = Point3, Vector = Vector3>
        + SearchParameter<D2, Point = Point3>
        + SearchNearestParameter<D2, Point = Point3>,
    truck_modeling::Line<Point3>: truck_modeling::ToSameGeometry<C>,
{
    let any_sections = split_faces.iter().any(|sf|
        sf.trimming_loops.iter().any(|tl| tl.edges.iter().any(|e| e.section_curve.is_some()))
    );

    let merged = merged_map;
    let mut registry = std::mem::take(&mut bopds.boundary_edge_registry);
    let mut cache = TopologyCache::new();
    let rebuilt_faces = if any_sections {
        split_faces
            .iter()
            .map(|split_face| {
                let original_face = faces_by_id.get(&split_face.original_face).ok_or(
                    BopError::InternalInvariant("missing source face for split face"),
                )?;
                rebuild_face_from_split_face(split_face, original_face, &mut cache, &merged)
            })
            .collect::<Result<Vec<_>, BopError>>()?
    } else {
        split_faces
            .iter()
            .map(|split_face| {
                let original_face = faces_by_id.get(&split_face.original_face).ok_or(
                    BopError::InternalInvariant("missing source face for split face"),
                )?;
                if split_face_can_reuse_original_face(split_face, original_face, &mut registry) {
                    Ok(original_face.clone())
                } else {
                    rebuild_face_from_split_face(split_face, original_face, &mut cache, &merged)
                }
            })
            .collect::<Result<Vec<_>, BopError>>()?
    };
    bopds.boundary_edge_registry = registry;
    if any_sections {
        let shell: truck_topology::Shell<Point3, C, S> = rebuilt_faces.into_iter().collect();
        let condition = shell.shell_condition();
        if condition == ShellCondition::Closed || condition == ShellCondition::Regular {
            return Ok(vec![shell]);
        }
        return Err(BopError::TopologyInvariantBroken);
    }

    let sewn_faces = sew_shell_faces(split_faces, rebuilt_faces)?;
    let mut shells = Vec::new();
    for shell_faces in sewn_faces {
        let shell: truck_topology::Shell<Point3, C, S> = shell_faces.into_iter().collect();
        if shell.shell_condition() != ShellCondition::Closed {
            return Err(BopError::TopologyInvariantBroken);
        }
        validate_shell_orientation(&shell)?;
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
    let component_groups = shell_component_groups(split_faces, &rebuilt_faces);
    let orientation_neighbors = shell_orientation_adjacency(split_faces, &rebuilt_faces);
    let mut shells = Vec::new();

    for component in component_groups {
        let mut remaining = component;
        let mut shell_faces = Vec::new();

        while !remaining.is_empty() {
            let seed_index = remaining.swap_remove(0);
            let mut oriented_indices = vec![seed_index];
            let seed_face = rebuilt_faces[seed_index].clone();
            let mut oriented_faces = vec![seed_face.clone()];
            shell_faces.push(seed_face);

            loop {
                let mut advanced = false;
                let mut has_orientable_candidate = false;
                let mut index = 0;

                while index < remaining.len() {
                    let split_face_index = remaining[index];
                    if !orientation_neighbors[split_face_index]
                        .iter()
                        .any(|neighbor| oriented_indices.contains(neighbor))
                    {
                        index += 1;
                        continue;
                    }

                    has_orientable_candidate = true;
                    let candidate = rebuilt_faces[split_face_index].clone();
                    if let Some(oriented) = orient_face_against_shell(&oriented_faces, &candidate) {
                        oriented_indices.push(split_face_index);
                        oriented_faces.push(oriented.clone());
                        shell_faces.push(oriented);
                        remaining.swap_remove(index);
                        advanced = true;
                    } else {
                        index += 1;
                    }
                }

                if advanced {
                    continue;
                }
                if has_orientable_candidate {
                    return Err(BopError::TopologyInvariantBroken);
                }
                break;
            }
        }

        shells.push(shell_faces);
    }

    Ok(shells)
}

#[cfg(test)]
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

#[cfg(test)]
fn component_adjacency(split_faces: &[SplitFace]) -> Vec<Vec<usize>> {
    let topologies = split_faces
        .iter()
        .map(rebuilt_face_topology)
        .collect::<Vec<_>>();
    build_face_adjacency(split_faces, &topologies, split_faces_share_component)
}

#[cfg(test)]
fn orientation_adjacency(split_faces: &[SplitFace]) -> Vec<Vec<usize>> {
    let topologies = split_faces
        .iter()
        .map(rebuilt_face_topology)
        .collect::<Vec<_>>();
    build_face_adjacency(split_faces, &topologies, split_faces_share_orientable_edge)
}

#[cfg(test)]
fn build_face_adjacency(
    split_faces: &[SplitFace],
    topologies: &[RebuiltFaceTopology],
    predicate: fn(&SplitFace, &SplitFace, &RebuiltFaceTopology, &RebuiltFaceTopology) -> bool,
) -> Vec<Vec<usize>> {
    let mut adjacency = vec![Vec::new(); split_faces.len()];

    for left in 0..split_faces.len() {
        for right in (left + 1)..split_faces.len() {
            if predicate(
                &split_faces[left],
                &split_faces[right],
                &topologies[left],
                &topologies[right],
            ) {
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

fn split_faces_share_component(
    lhs: &SplitFace,
    rhs: &SplitFace,
    lhs_topology: &RebuiltFaceTopology,
    rhs_topology: &RebuiltFaceTopology,
) -> bool {
    split_faces_share_orientable_edge(lhs, rhs, lhs_topology, rhs_topology)
        || split_faces_share_vertex(lhs, rhs)
}

fn split_faces_share_orientable_edge(
    lhs: &SplitFace,
    rhs: &SplitFace,
    lhs_topology: &RebuiltFaceTopology,
    rhs_topology: &RebuiltFaceTopology,
) -> bool {
    if lhs.original_face == rhs.original_face {
        return false;
    }

    lhs_topology
        .shared_edges
        .iter()
        .any(|edge| rhs_topology.shared_edges.contains(edge))
}

fn split_faces_share_vertex(lhs: &SplitFace, rhs: &SplitFace) -> bool {
    if lhs.original_face == rhs.original_face {
        return false;
    }

    lhs.trimming_loops.iter().any(|lhs_loop| {
        let lhs_vertices = open_loop_vertex_ids(lhs_loop);
        rhs.trimming_loops.iter().any(|rhs_loop| {
            let rhs_vertices = open_loop_vertex_ids(rhs_loop);
            lhs_vertices
                .iter()
                .any(|vertex| rhs_vertices.contains(vertex))
        })
    })
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
        let canonical = canonical_rebuilt_edge(trimming_loop.face, edge, start, end);
        if !canonical_edges.contains(&canonical) {
            canonical_edges.push(canonical);
        }
    }

    canonical_edges
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum CanonicalRebuiltEdge {
    Source(EdgeId),
    SharedBoundary(VertexId, VertexId),
    OpenBoundary(FaceId, VertexId, VertexId),
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct RebuiltFaceTopology {
    shared_edges: Vec<CanonicalRebuiltEdge>,
    boundary_edges: Vec<CanonicalRebuiltEdge>,
}

fn canonical_rebuilt_edge(
    face: FaceId,
    edge: &TrimmingEdge,
    start: VertexId,
    end: VertexId,
) -> CanonicalRebuiltEdge {
    if let Some(section_curve_id) = edge.section_curve {
        return CanonicalRebuiltEdge::Source(edge_id_from_section_curve(section_curve_id));
    }
    if let Some(original_edge) = edge.original_edge {
        return CanonicalRebuiltEdge::Source(original_edge);
    }

    let undirected = undirected_edge_key(start, end);
    if edge_is_shared_non_section(edge, start, end) {
        CanonicalRebuiltEdge::SharedBoundary(undirected.0, undirected.1)
    } else {
        CanonicalRebuiltEdge::OpenBoundary(face, undirected.0, undirected.1)
    }
}

fn edge_is_shared_non_section(edge: &TrimmingEdge, start: VertexId, end: VertexId) -> bool {
    if edge.uv_points.len() <= 2 {
        return false;
    }

    let vertices = open_polygon_vertices(&edge.uv_points);
    vertices.len() > 2
        || start == end
        || edge
            .uv_points
            .first()
            .zip(edge.uv_points.last())
            .is_some_and(|(first, last)| near_points(*first, *last))
}

fn rebuilt_face_topology(split_face: &SplitFace) -> RebuiltFaceTopology {
    let mut shared_edges = Vec::new();
    let mut boundary_edges = Vec::new();

    for trimming_loop in &split_face.trimming_loops {
        for canonical in canonical_loop_edges(trimming_loop) {
            match canonical {
                CanonicalRebuiltEdge::Source(_) => {
                    if !shared_edges.contains(&canonical) {
                        shared_edges.push(canonical);
                    }
                }
                CanonicalRebuiltEdge::SharedBoundary(_, _) => {
                    if !shared_edges.contains(&canonical) {
                        shared_edges.push(canonical);
                    }
                }
                CanonicalRebuiltEdge::OpenBoundary(_, _, _) => {
                    if !boundary_edges.contains(&canonical) {
                        boundary_edges.push(canonical);
                    }
                }
            }
        }
    }

    RebuiltFaceTopology {
        shared_edges,
        boundary_edges,
    }
}

fn split_face_can_reuse_original_face<C, S>(
    split_face: &SplitFace,
    original_face: &Face<Point3, C, S>,
    registry: &mut crate::bopds::SourceBoundaryEdgeRegistry,
) -> bool
where
    C: Clone,
    S: Clone,
{
    if split_face.trimming_loops.is_empty() {
        return true;
    }

    let original_boundaries: Vec<Vec<EdgeId>> = original_face
        .boundaries()
        .iter()
        .map(|wire| {
            wire.edge_iter()
                .map(|edge| edge_id_from_source_boundary(edge, registry))
                .collect()
        })
        .collect();
    if original_boundaries.len() != split_face.trimming_loops.len() {
        return false;
    }

    let mut matched = vec![false; original_boundaries.len()];
    for trimming_loop in &split_face.trimming_loops {
        if trimming_loop
            .edges
            .iter()
            .any(|edge| edge.section_curve.is_some() || edge.original_edge.is_none())
        {
            return false;
        }

        let loop_edges: Vec<_> = trimming_loop
            .edges
            .iter()
            .filter_map(|edge| edge.original_edge)
            .collect();
        let Some((boundary_index, _)) =
            original_boundaries
                .iter()
                .enumerate()
                .find(|(index, boundary)| {
                    !matched[*index] && cyclic_edge_sequence_matches(boundary, &loop_edges)
                })
        else {
            return false;
        };
        matched[boundary_index] = true;
    }

    matched.into_iter().all(|matched| matched)
}

const ORIENTATION_TOLERANCE: f64 = 1.0e-6;

fn validate_shell_orientation<C, S>(
    shell: &truck_topology::Shell<Point3, C, S>,
) -> Result<(), BopError>
where
    C: Clone
        + truck_geotrait::ParametricCurve<Point = Point3>
        + truck_geotrait::BoundedCurve
        + Invertible,
    S: Clone
        + Invertible
        + ParametricSurface<Point = Point3, Vector = Vector3>
        + SearchNearestParameter<D2, Point = Point3>,
{
    let solid = Solid::new(vec![shell.clone()]);
    let probe_distance = shell_probe_distance(shell);

    for face in shell.face_iter() {
        let (sample_point, probe_vector) = face_orientation_probe(face, probe_distance)?;
        let outward =
            classify_point_in_solid(&solid, sample_point + probe_vector, ORIENTATION_TOLERANCE)?;
        let inward =
            classify_point_in_solid(&solid, sample_point - probe_vector, ORIENTATION_TOLERANCE)?;
        if outward != PointClassification::Outside || inward != PointClassification::Inside {
            return Err(BopError::TopologyInvariantBroken);
        }
    }

    Ok(())
}

fn shell_probe_distance<C, S>(shell: &truck_topology::Shell<Point3, C, S>) -> f64
where
    C: Clone,
    S: Clone, {
    let mut points = shell.vertex_iter().map(|vertex| vertex.point());
    let Some(first) = points.next() else {
        return ORIENTATION_TOLERANCE * 10.0;
    };

    let (mut min_x, mut max_x) = (first.x, first.x);
    let (mut min_y, mut max_y) = (first.y, first.y);
    let (mut min_z, mut max_z) = (first.z, first.z);
    for point in points {
        min_x = min_x.min(point.x);
        max_x = max_x.max(point.x);
        min_y = min_y.min(point.y);
        max_y = max_y.max(point.y);
        min_z = min_z.min(point.z);
        max_z = max_z.max(point.z);
    }

    let extent = (max_x - min_x)
        .abs()
        .max((max_y - min_y).abs())
        .max((max_z - min_z).abs())
        .max(1.0);
    (ORIENTATION_TOLERANCE * 10.0).max(extent * 1.0e-4)
}

fn face_orientation_probe<C, S>(
    face: &Face<Point3, C, S>,
    probe_distance: f64,
) -> Result<(Point3, Vector3), BopError>
where
    C: Clone
        + truck_geotrait::ParametricCurve<Point = Point3>
        + truck_geotrait::BoundedCurve
        + Invertible,
    S: Clone,
{
    let boundaries = collect_face_boundaries(face);
    let Some(outer_boundary) = boundaries.first() else {
        return Err(BopError::TopologyInvariantBroken);
    };
    if outer_boundary.len() < 3 {
        return Err(BopError::TopologyInvariantBroken);
    }

    let normal = oriented_face_normal(outer_boundary)?;
    let length = vector_length(normal);
    if length <= f64::EPSILON {
        return Err(BopError::TopologyInvariantBroken);
    }

    let sample_point = face_interior_sample_point(&boundaries, normal)?;

    Ok((sample_point, scale_vector(normal, probe_distance / length)))
}

fn collect_face_boundaries<C, S>(face: &Face<Point3, C, S>) -> Vec<Vec<Point3>>
where
    C: Clone
        + truck_geotrait::ParametricCurve<Point = Point3>
        + truck_geotrait::BoundedCurve
        + Invertible,
    S: Clone, {
    face.boundaries()
        .into_iter()
        .map(|boundary| {
            let mut vertices = Vec::new();
            for point in boundary.vertex_iter().map(|vertex| vertex.point()) {
                if vertices.last().is_some_and(|previous: &Point3| {
                    previous.distance2(point) <= ORIENTATION_TOLERANCE * ORIENTATION_TOLERANCE
                }) {
                    continue;
                }
                vertices.push(point);
            }
            if vertices.len() >= 2
                && vertices[0].distance2(*vertices.last().unwrap())
                    <= ORIENTATION_TOLERANCE * ORIENTATION_TOLERANCE
            {
                vertices.pop();
            }
            vertices
        })
        .filter(|boundary| boundary.len() >= 3)
        .collect()
}

fn face_interior_sample_point(
    boundaries: &[Vec<Point3>],
    normal: Vector3,
) -> Result<Point3, BopError> {
    let Some(outer_boundary) = boundaries.first() else {
        return Err(BopError::TopologyInvariantBroken);
    };
    let origin = outer_boundary[0];
    let unit_normal = scale_vector(normal, 1.0 / vector_length(normal));
    let (tangent, bitangent) = plane_basis(unit_normal)?;
    let projected_boundaries = boundaries
        .iter()
        .map(|boundary| {
            boundary
                .iter()
                .map(|point| project_point_to_plane(*point, origin, tangent, bitangent))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();

    if let Some(sample) = polygon_centroid(&projected_boundaries[0])
        .filter(|candidate| point_in_face_region(&projected_boundaries, *candidate))
    {
        return Ok(lift_point_from_plane(sample, origin, tangent, bitangent));
    }

    for index in 1..outer_boundary.len().saturating_sub(1) {
        let candidate = average_point(
            [
                outer_boundary[0],
                outer_boundary[index],
                outer_boundary[index + 1],
            ]
            .into_iter(),
        );
        let projected = project_point_to_plane(candidate, origin, tangent, bitangent);
        if point_in_face_region(&projected_boundaries, projected) {
            return Ok(candidate);
        }
    }

    Err(BopError::TopologyInvariantBroken)
}

fn point_in_face_region(boundaries: &[Vec<Point2>], point: Point2) -> bool {
    let Some(outer_boundary) = boundaries.first() else {
        return false;
    };
    if point_on_polygon_boundary(outer_boundary, point) || !point_in_polygon(outer_boundary, point)
    {
        return false;
    }

    boundaries
        .iter()
        .skip(1)
        .all(|hole| !point_on_polygon_boundary(hole, point) && !point_in_polygon(hole, point))
}

fn plane_basis(normal: Vector3) -> Result<(Vector3, Vector3), BopError> {
    let reference = if normal.x.abs() < 0.9 {
        Vector3::unit_x()
    } else {
        Vector3::unit_y()
    };
    let tangent = cross(normal, reference);
    let tangent_length = vector_length(tangent);
    if tangent_length <= f64::EPSILON {
        return Err(BopError::TopologyInvariantBroken);
    }
    let tangent = scale_vector(tangent, 1.0 / tangent_length);
    let bitangent = cross(normal, tangent);
    let bitangent_length = vector_length(bitangent);
    if bitangent_length <= f64::EPSILON {
        return Err(BopError::TopologyInvariantBroken);
    }
    Ok((tangent, scale_vector(bitangent, 1.0 / bitangent_length)))
}

fn project_point_to_plane(
    point: Point3,
    origin: Point3,
    tangent: Vector3,
    bitangent: Vector3,
) -> Point2 {
    let offset = point - origin;
    Point2::new(dot(offset, tangent), dot(offset, bitangent))
}

fn lift_point_from_plane(
    point: Point2,
    origin: Point3,
    tangent: Vector3,
    bitangent: Vector3,
) -> Point3 {
    Point3::new(
        origin.x + tangent.x * point.x + bitangent.x * point.y,
        origin.y + tangent.y * point.x + bitangent.y * point.y,
        origin.z + tangent.z * point.x + bitangent.z * point.y,
    )
}

fn oriented_face_normal(vertices: &[Point3]) -> Result<Vector3, BopError> {
    for index in 0..vertices.len() {
        let start = vertices[index];
        let middle = vertices[(index + 1) % vertices.len()];
        let end = vertices[(index + 2) % vertices.len()];
        let first = middle - start;
        let second = end - middle;
        let normal = cross(first, second);
        if vector_length(normal) > ORIENTATION_TOLERANCE {
            return Ok(normal);
        }
    }
    Err(BopError::TopologyInvariantBroken)
}

fn cross(lhs: Vector3, rhs: Vector3) -> Vector3 {
    Vector3::new(
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    )
}

fn dot(lhs: Vector3, rhs: Vector3) -> f64 { lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z }

fn vector_length(vector: Vector3) -> f64 {
    (vector.x * vector.x + vector.y * vector.y + vector.z * vector.z).sqrt()
}

fn scale_vector(vector: Vector3, scale: f64) -> Vector3 {
    Vector3::new(vector.x * scale, vector.y * scale, vector.z * scale)
}

// ── Topology cache for shared vertex/edge objects ───────────────────────────

struct TopologyCache<C> {
    vertices: FxHashMap<VertexId, truck_topology::Vertex<Point3>>,
    vertex_by_point: Vec<(Point3, VertexId)>,
    edges: FxHashMap<(VertexId, VertexId), truck_topology::Edge<Point3, C>>,
    tolerance: f64,
}

impl<C> TopologyCache<C>
where
    C: Clone + Invertible,
    truck_modeling::Line<Point3>: truck_modeling::ToSameGeometry<C>,
{
    fn new() -> Self {
        Self {
            vertices: FxHashMap::default(),
            vertex_by_point: Vec::new(),
            edges: FxHashMap::default(),
            tolerance: 0.1,
        }
    }

    /// Returns (canonical_vertex_id, vertex_object).
    fn get_or_create_vertex(&mut self, id: VertexId, point: Point3) -> (VertexId, truck_topology::Vertex<Point3>) {
        use truck_base::cgmath64::MetricSpace;

        if let Some(v) = self.vertices.get(&id) {
            return (id, v.clone());
        }

        let match_id = self.vertex_by_point
            .iter()
            .find(|(ep, _)| ep.distance(point) < self.tolerance)
            .map(|(_, eid)| *eid);

        if let Some(existing_id) = match_id {
            let v = self.vertices.get(&existing_id).unwrap().clone();
            self.vertices.insert(id, v.clone());
            return (existing_id, v);
        }

        let vertex = truck_modeling::builder::vertex(point);
        self.vertices.insert(id, vertex.clone());
        self.vertex_by_point.push((point, id));
        (id, vertex)
    }

    fn get_or_create_edge(
        &mut self,
        start_id: VertexId,
        end_id: VertexId,
        start: &truck_topology::Vertex<Point3>,
        end: &truck_topology::Vertex<Point3>,
    ) -> truck_topology::Edge<Point3, C> {
        if let Some(edge) = self.edges.get(&(start_id, end_id)) {
            return edge.clone();
        }
        if let Some(edge) = self.edges.get(&(end_id, start_id)) {
            return edge.inverse();
        }
        let edge = truck_modeling::builder::line(start, end);
        self.edges.insert((start_id, end_id), edge.clone());
        edge
    }
}

fn rebuild_face_from_split_face<C, S>(
    split_face: &SplitFace,
    original_face: &Face<Point3, C, S>,
    cache: &mut TopologyCache<C>,
    merged: &FxHashMap<VertexId, VertexId>,
) -> Result<Face<Point3, C, S>, BopError>
where
    C: Clone
        + truck_geotrait::ParametricCurve<Point = Point3>
        + truck_geotrait::BoundedCurve
        + Invertible,
    S: Clone
        + ParametricSurface<Point = Point3>
        + Invertible
        + SearchParameter<D2, Point = Point3>,
    truck_modeling::Line<Point3>: truck_modeling::ToSameGeometry<C>,
{
    let surface = original_face.oriented_surface();
    let boundaries = split_face
        .trimming_loops
        .iter()
        .map(|trimming_loop| rebuild_wire_from_trimming_loop(trimming_loop, &surface, cache, merged))
        .collect::<Result<Vec<_>, _>>()?;

    if boundaries.is_empty() {
        return Err(BopError::TopologyInvariantBroken);
    }

    Ok(Face::new(boundaries, original_face.surface().clone()))
}

fn rebuild_wire_from_trimming_loop<C, S>(
    trimming_loop: &TrimmingLoop,
    surface: &S,
    cache: &mut TopologyCache<C>,
    merged: &FxHashMap<VertexId, VertexId>,
) -> Result<truck_topology::Wire<Point3, C>, BopError>
where
    C: Clone
        + truck_geotrait::ParametricCurve<Point = Point3>
        + truck_geotrait::BoundedCurve
        + Invertible,
    S: ParametricSurface<Point = Point3>
        + SearchParameter<D2, Point = Point3>
        + Invertible,
    truck_modeling::Line<Point3>: truck_modeling::ToSameGeometry<C>,
{
    let uv_verts = open_polygon_vertices(&trimming_loop.uv_points);
    if uv_verts.len() < 3 {
        return Err(BopError::TopologyInvariantBroken);
    }

    let open_vertex_ids = if trimming_loop.vertex_ids.len() >= 2
        && trimming_loop.vertex_ids.first() == trimming_loop.vertex_ids.last()
    {
        &trimming_loop.vertex_ids[..trimming_loop.vertex_ids.len() - 1]
    } else {
        &trimming_loop.vertex_ids
    };

    let vid_count = open_vertex_ids.len().min(uv_verts.len());
    let mut topo_vertices = Vec::with_capacity(vid_count);
    for i in 0..vid_count {
        let point = surface.subs(uv_verts[i].x, uv_verts[i].y);
        let input_id = merged.get(&open_vertex_ids[i]).copied().unwrap_or(open_vertex_ids[i]);
        let (canonical_id, vertex) = cache.get_or_create_vertex(input_id, point);
        topo_vertices.push((canonical_id, vertex));
    }

    let mut edges = Vec::with_capacity(vid_count);
    for i in 0..vid_count {
        let next = (i + 1) % vid_count;
        let (sid, start) = &topo_vertices[i];
        let (eid, end) = &topo_vertices[next];
        edges.push(cache.get_or_create_edge(*sid, *eid, start, end));
    }

    Ok(truck_topology::Wire::from(edges))
}

/// Builds solids from closed shell components and verifies topology validity.
pub fn build_solids_from_shells<C, S>(
    shells: Vec<truck_topology::Shell<Point3, C, S>>,
) -> Result<Vec<Solid<Point3, C, S>>, BopError>
where
    C: Clone,
    S: Clone, {
    let mut solids = Vec::with_capacity(shells.len());
    for shell in shells {
        let condition = shell.shell_condition();
        if condition != ShellCondition::Closed && condition != ShellCondition::Regular {
            return Err(BopError::TopologyInvariantBroken);
        }

        let solid = Solid::new(vec![shell]);
        match solid.boundaries().as_slice() {
            [boundary] if {
                let c = boundary.shell_condition();
                c == ShellCondition::Closed || c == ShellCondition::Regular
            } => {
                solids.push(solid)
            }
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
                let loop_end =
                    trimming_loop.uv_points[(edge_index + 1) % trimming_loop.uv_points.len()];
                let start = vertices[edge_index];
                let end = vertices[(edge_index + 1) % vertices.len()];
                let sewn_start = merged_vertices.get(&start).copied().unwrap_or(start);
                let sewn_end = merged_vertices.get(&end).copied().unwrap_or(end);
                let undirected = undirected_edge_key(sewn_start, sewn_end);
                let source = SewnEdgeSource {
                    face: split_face.original_face,
                    loop_index,
                    edge_index,
                    original_edge: edge
                        .original_edge
                        .or(edge.section_curve.map(edge_id_from_section_curve)),
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

    edges.retain(|edge| edge.section_curve.is_some() || edge.sewn_pair.is_some());

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

fn edge_id_from_source_boundary<C>(
    edge: &truck_topology::Edge<Point3, C>,
    registry: &mut crate::bopds::SourceBoundaryEdgeRegistry,
) -> EdgeId {
    let key = source_boundary_edge_key(edge);
    registry.edge_id_for_key(key)
}

fn source_boundary_edge_key<C>(edge: &truck_topology::Edge<Point3, C>) -> String {
    let (start, end) =
        ordered_point_pair(edge.absolute_front().point(), edge.absolute_back().point());
    format!(
        "{:?}|{:.17e},{:.17e},{:.17e}|{:.17e},{:.17e},{:.17e}",
        edge.id(),
        start.x,
        start.y,
        start.z,
        end.x,
        end.y,
        end.z,
    )
}

fn ordered_point_pair(first: Point3, second: Point3) -> (Point3, Point3) {
    if compare_point3(first, second).is_le() {
        (first, second)
    } else {
        (second, first)
    }
}

fn compare_point3(lhs: Point3, rhs: Point3) -> std::cmp::Ordering {
    lhs.x
        .total_cmp(&rhs.x)
        .then(lhs.y.total_cmp(&rhs.y))
        .then(lhs.z.total_cmp(&rhs.z))
}

fn cyclic_edge_sequence_matches(expected: &[EdgeId], actual: &[EdgeId]) -> bool {
    expected.len() == actual.len()
        && (cyclically_aligned(expected, actual)
            || cyclically_aligned(expected, &actual.iter().rev().copied().collect::<Vec<_>>()))
}

fn cyclically_aligned(expected: &[EdgeId], actual: &[EdgeId]) -> bool {
    if expected.is_empty() {
        return true;
    }

    (0..actual.len()).any(|offset| {
        expected
            .iter()
            .enumerate()
            .all(|(index, edge_id)| *edge_id == actual[(index + offset) % actual.len()])
    })
}

fn next_edge_index(edges: &[SewnEdge], used: &[bool], vertex: VertexId) -> Option<usize> {
    edges
        .iter()
        .enumerate()
        .find(|(index, edge)| {
            !used[*index] && (edge.start_vertex == vertex || edge.end_vertex == vertex)
        })
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

fn shell_component_groups<C, S>(
    split_faces: &[SplitFace],
    rebuilt_faces: &[Face<Point3, C, S>],
) -> Vec<Vec<usize>>
where
    C: Clone,
    S: Clone,
{
    shell_face_groups(split_faces, rebuilt_faces, split_faces_share_component)
}

fn shell_orientation_adjacency<C, S>(
    split_faces: &[SplitFace],
    rebuilt_faces: &[Face<Point3, C, S>],
) -> Vec<Vec<usize>>
where
    C: Clone,
    S: Clone,
{
    shell_face_adjacency(
        split_faces,
        rebuilt_faces,
        split_faces_share_orientable_edge,
    )
}

fn shell_face_groups<C, S>(
    split_faces: &[SplitFace],
    rebuilt_faces: &[Face<Point3, C, S>],
    predicate: fn(&SplitFace, &SplitFace, &RebuiltFaceTopology, &RebuiltFaceTopology) -> bool,
) -> Vec<Vec<usize>>
where
    C: Clone,
    S: Clone,
{
    let adjacency = shell_face_adjacency(split_faces, rebuilt_faces, predicate);
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

fn shell_face_adjacency<C, S>(
    split_faces: &[SplitFace],
    rebuilt_faces: &[Face<Point3, C, S>],
    predicate: fn(&SplitFace, &SplitFace, &RebuiltFaceTopology, &RebuiltFaceTopology) -> bool,
) -> Vec<Vec<usize>>
where
    C: Clone,
    S: Clone,
{
    let topologies = split_faces
        .iter()
        .map(rebuilt_face_topology)
        .collect::<Vec<_>>();
    let mut adjacency = vec![Vec::new(); split_faces.len()];

    for left in 0..split_faces.len() {
        for right in (left + 1)..split_faces.len() {
            let topological = predicate(
                &split_faces[left],
                &split_faces[right],
                &topologies[left],
                &topologies[right],
            );
            let actual = faces_share_topology_edge(&rebuilt_faces[left], &rebuilt_faces[right]);
            if topological || actual {
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

fn faces_share_topology_edge<C, S>(lhs: &Face<Point3, C, S>, rhs: &Face<Point3, C, S>) -> bool
where
    C: Clone,
    S: Clone, {
    shared_edge_orientation(lhs, rhs).is_some()
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
        BooleanOp::Common => matches!(
            classification,
            PointClassification::Inside | PointClassification::OnBoundary
        ),
        BooleanOp::Fuse => matches!(
            classification,
            PointClassification::Outside | PointClassification::OnBoundary
        ),
        BooleanOp::Cut => {
            if split_face.operand_rank == 0 {
                matches!(
                    classification,
                    PointClassification::Outside | PointClassification::OnBoundary
                )
            } else {
                matches!(classification, PointClassification::OnBoundary)
            }
        }
        BooleanOp::Section => false,
    }
}

fn collect_fragment_vertices<S>(
    split_faces: &[SplitFace],
    surfaces: &FxHashMap<FaceId, S>,
) -> Vec<(VertexId, Point3)>
where
    S: ParametricSurface<Point = Point3>,
{
    let mut vertices = Vec::new();
    for split_face in split_faces {
        let surface = surfaces.get(&split_face.original_face);
        for trimming_loop in &split_face.trimming_loops {
            for (vertex_id, uv) in trimming_loop
                .vertex_ids
                .iter()
                .zip(open_polygon_vertices(&trimming_loop.uv_points).iter())
            {
                vertices.push((*vertex_id, eval_uv_on_surface(*uv, surface)));
            }
        }
    }
    vertices.sort_by_key(|(id, _)| *id);
    vertices.dedup_by_key(|(id, _)| *id);
    vertices
}

fn vertex_ids_for_polyline(counter: &mut u32, polyline: &[Point2]) -> Vec<VertexId> {
    open_polygon_vertices(polyline)
        .iter()
        .map(|_| {
            let id = VertexId(*counter);
            *counter += 1;
            id
        })
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
    registry: &mut crate::bopds::SourceBoundaryEdgeRegistry,
    vertex_counter: &mut u32,
) -> Vec<TrimmingLoop>
where
    C: Clone,
    S: Clone + Invertible + SearchParameter<D2, Point = Point3>,
{
    let all_sections: Vec<_> = section_curves
        .iter()
        .filter_map(|sc| {
            sc.face_parameters
                .iter()
                .find(|(fid, _)| *fid == face_id)
                .map(|(_, params)| (sc.id, params.clone()))
        })
        .filter(|(_, params)| params.len() >= 2)
        .collect();

    if all_sections.is_empty() {
        let mut loops = boundary_loops(face_id, face, tolerance, registry, vertex_counter);
        classify_loops(&mut loops);
        return loops;
    }

    let (open_sections, closed_sections): (Vec<_>, Vec<_>) = all_sections
        .into_iter()
        .partition(|(_, params)| {
            let closed = close_polyline(params.clone(), tolerance);
            let area = geometry_utils::signed_area(&closed).abs();
            area < tolerance * tolerance
        });

    let mut loops = if !open_sections.is_empty() {
        build_loops_via_edge_graph(
            face_id,
            face,
            &open_sections,
            tolerance,
            registry,
            vertex_counter,
        )
    } else {
        Vec::new()
    };

    if loops.is_empty() && !open_sections.is_empty() {
        loops = boundary_loops(face_id, face, tolerance, registry, vertex_counter);
        for (sc_id, parameters) in &open_sections {
            let closed = close_polyline(parameters.clone(), tolerance);
            if closed.len() < 4 {
                continue;
            }
            loops.push(loop_from_polyline(
                face_id,
                vec![TrimmingEdge {
                    section_curve: Some(*sc_id),
                    original_edge: None,
                    uv_points: closed.clone(),
                }],
                closed,
                tolerance,
                vertex_counter,
            ));
        }
    }

    if loops.is_empty() {
        loops = boundary_loops(face_id, face, tolerance, registry, vertex_counter);
    }

    for (sc_id, parameters) in &closed_sections {
        let closed = close_polyline(parameters.clone(), tolerance);
        if closed.len() < 4 {
            continue;
        }
        loops.push(loop_from_polyline(
            face_id,
            vec![TrimmingEdge {
                section_curve: Some(*sc_id),
                original_edge: None,
                uv_points: closed.clone(),
            }],
            closed,
            tolerance,
            vertex_counter,
        ));
    }

    classify_loops(&mut loops);
    loops
}

fn extract_boundary_uv_points<C, S>(
    face: &Face<Point3, C, S>,
    tolerance: f64,
) -> Vec<Point2>
where
    C: Clone,
    S: Clone + Invertible + SearchParameter<D2, Point = Point3>,
{
    let surface = face.oriented_surface();
    let mut points = Vec::new();
    for wire in face.boundaries() {
        for vertex in wire.vertex_iter() {
            let uv = surface
                .search_parameter(vertex.point(), None, SEARCH_PARAMETER_TRIALS)
                .map(|(u, v)| Point2::new(u, v));
            if let Some(uv) = uv {
                points.push(uv);
            }
        }
    }
    points
}

fn point_near_polygon_boundary(polygon: &[Point2], point: Point2, tolerance: f64) -> bool {
    geometry_utils::point_on_polygon_boundary(polygon, point, tolerance)
}

// ── Edge-graph based loop construction ──────────────────────────────────────

#[derive(Clone, Debug)]
struct UvEdge {
    start: Point2,
    end: Point2,
    points: Vec<Point2>,
    section_curve: Option<SectionCurveId>,
    original_edge: Option<EdgeId>,
}

fn build_loops_via_edge_graph<C, S>(
    face_id: FaceId,
    face: &Face<Point3, C, S>,
    relevant_sections: &[(SectionCurveId, Vec<Point2>)],
    tolerance: f64,
    registry: &mut crate::bopds::SourceBoundaryEdgeRegistry,
    vertex_counter: &mut u32,
) -> Vec<TrimmingLoop>
where
    C: Clone,
    S: Clone + Invertible + SearchParameter<D2, Point = Point3>,
{
    let surface = face.oriented_surface();
    let mut boundary_edges: Vec<UvEdge> = Vec::new();

    for wire in face.boundaries() {
        let mut hint = None;
        let mut uv_points = Vec::new();
        let mut edges_iter: Vec<_> = wire.edge_iter().collect();

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

        let closed = close_polyline(uv_points.clone(), tolerance);
        for (i, edge) in edges_iter.iter().enumerate() {
            let start = closed[i];
            let end = closed[(i + 1) % (closed.len().max(1))];
            if i + 1 >= closed.len() {
                break;
            }
            let end = closed[i + 1];
            boundary_edges.push(UvEdge {
                start,
                end,
                points: vec![start, end],
                section_curve: None,
                original_edge: Some(edge_id_from_source_boundary(edge.clone(), registry)),
            });
        }
    }

    if boundary_edges.is_empty() {
        return Vec::new();
    }

    let mut section_edges: Vec<UvEdge> = Vec::new();
    for (sc_id, params) in relevant_sections {
        if params.len() < 2 {
            continue;
        }
        let start = params[0];
        let end = *params.last().unwrap();
        section_edges.push(UvEdge {
            start,
            end,
            points: params.clone(),
            section_curve: Some(*sc_id),
            original_edge: None,
        });
    }

    let split_boundary = split_boundary_at_section_endpoints(
        &boundary_edges,
        &section_edges,
        tolerance,
    );

    let all_edges: Vec<UvEdge> = split_boundary
        .into_iter()
        .chain(section_edges)
        .collect();

    let mut extracted = extract_loops_from_graph(&all_edges, tolerance);

    if extracted.len() > 1 {
        let areas: Vec<f64> = extracted
            .iter()
            .map(|loop_edges| {
                let points: Vec<Point2> = loop_edges
                    .iter()
                    .flat_map(|e| e.points.iter().copied())
                    .collect();
                geometry_utils::signed_area(&points).abs()
            })
            .collect();
        if let Some(max_area) = areas.iter().cloned().reduce(f64::max) {
            let threshold = max_area * 0.99;
            let mut kept = Vec::new();
            for (i, loop_edges) in extracted.into_iter().enumerate() {
                if areas[i] < threshold {
                    kept.push(loop_edges);
                }
            }
            extracted = kept;
        }
    }

    extracted
        .into_iter()
        .map(|loop_edges| {
            let mut uv_points = Vec::new();
            let mut trimming_edges = Vec::new();

            for edge in &loop_edges {
                if uv_points.is_empty() {
                    uv_points.extend(edge.points.iter().copied());
                } else {
                    uv_points.extend(edge.points.iter().skip(1).copied());
                }
                trimming_edges.push(TrimmingEdge {
                    section_curve: edge.section_curve,
                    original_edge: edge.original_edge,
                    uv_points: edge.points.clone(),
                });
            }

            loop_from_polyline(face_id, trimming_edges, uv_points, tolerance, vertex_counter)
        })
        .collect()
}

fn split_boundary_at_section_endpoints(
    boundary_edges: &[UvEdge],
    section_edges: &[UvEdge],
    tolerance: f64,
) -> Vec<UvEdge> {
    use truck_base::cgmath64::MetricSpace;

    let mut endpoints: Vec<Point2> = Vec::new();
    for se in section_edges {
        endpoints.push(se.start);
        endpoints.push(se.end);
    }

    let mut result = Vec::new();
    for edge in boundary_edges {
        let mut split_ts: Vec<f64> = Vec::new();
        let dx = edge.end.x - edge.start.x;
        let dy = edge.end.y - edge.start.y;
        let len_sq = dx * dx + dy * dy;

        if len_sq < tolerance * tolerance {
            result.push(edge.clone());
            continue;
        }

        for ep in &endpoints {
            let t = ((ep.x - edge.start.x) * dx + (ep.y - edge.start.y) * dy) / len_sq;
            if t > tolerance && t < 1.0 - tolerance {
                let proj = Point2::new(edge.start.x + t * dx, edge.start.y + t * dy);
                if proj.distance(*ep) < tolerance * 100.0 {
                    split_ts.push(t);
                }
            }
        }

        if split_ts.is_empty() {
            result.push(edge.clone());
            continue;
        }

        split_ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        split_ts.dedup_by(|a, b| (*a - *b).abs() < tolerance);

        let mut prev_point = edge.start;
        for t in &split_ts {
            let on_edge = Point2::new(edge.start.x + t * dx, edge.start.y + t * dy);
            let snap_point = endpoints
                .iter()
                .find(|ep| ep.distance(on_edge) < tolerance * 100.0)
                .copied()
                .unwrap_or(on_edge);
            result.push(UvEdge {
                start: prev_point,
                end: snap_point,
                points: vec![prev_point, snap_point],
                section_curve: None,
                original_edge: edge.original_edge,
            });
            prev_point = snap_point;
        }
        result.push(UvEdge {
            start: prev_point,
            end: edge.end,
            points: vec![prev_point, edge.end],
            section_curve: None,
            original_edge: edge.original_edge,
        });
    }

    result
}

fn extract_loops_from_graph(edges: &[UvEdge], tolerance: f64) -> Vec<Vec<UvEdge>> {
    use truck_base::cgmath64::MetricSpace;

    let merge_tol = tolerance * 100.0;
    let mut vertices: Vec<Point2> = Vec::new();
    let mut vertex_index = |p: Point2| -> usize {
        for (i, v) in vertices.iter().enumerate() {
            if v.distance(p) < merge_tol {
                return i;
            }
        }
        vertices.push(p);
        vertices.len() - 1
    };

    let mut directed: Vec<(usize, usize, UvEdge)> = Vec::new();
    for edge in edges {
        let si = vertex_index(edge.start);
        let ei = vertex_index(edge.end);
        if si != ei {
            directed.push((si, ei, edge.clone()));
            let mut rev = edge.clone();
            rev.start = edge.end;
            rev.end = edge.start;
            rev.points = edge.points.iter().rev().copied().collect();
            directed.push((ei, si, rev));
        }
    }

    let mut used = vec![false; directed.len()];
    let mut loops = Vec::new();

    for start_idx in 0..directed.len() {
        if used[start_idx] {
            continue;
        }
        let start_vi = directed[start_idx].0;
        let mut chain = vec![start_idx];
        let mut current_vi = directed[start_idx].1;
        used[start_idx] = true;

        let max_steps = directed.len();
        for _ in 0..max_steps {
            if current_vi == start_vi {
                let loop_edges: Vec<UvEdge> = chain
                    .iter()
                    .map(|&idx| directed[idx].2.clone())
                    .collect();
                loops.push(loop_edges);
                break;
            }

            let prev_edge_idx = *chain.last().unwrap();
            let prev_dir = {
                let e = &directed[prev_edge_idx];
                let d = Point2::new(
                    vertices[e.1].x - vertices[e.0].x,
                    vertices[e.1].y - vertices[e.0].y,
                );
                f64::atan2(d.y, d.x)
            };

            let mut best: Option<(usize, f64)> = None;
            for (idx, (si, _ei, _)) in directed.iter().enumerate() {
                if used[idx] || *si != current_vi {
                    continue;
                }
                let d = Point2::new(
                    vertices[directed[idx].1].x - vertices[current_vi].x,
                    vertices[directed[idx].1].y - vertices[current_vi].y,
                );
                let angle = f64::atan2(d.y, d.x);
                let mut turn = prev_dir - angle + std::f64::consts::PI;
                while turn < 0.0 {
                    turn += std::f64::consts::TAU;
                }
                while turn >= std::f64::consts::TAU {
                    turn -= std::f64::consts::TAU;
                }

                if best.map_or(true, |(_, best_turn)| turn > best_turn) {
                    best = Some((idx, turn));
                }
            }

            match best {
                Some((next_idx, _)) => {
                    used[next_idx] = true;
                    chain.push(next_idx);
                    current_vi = directed[next_idx].1;
                }
                None => break,
            }
        }
    }

    loops
}

fn boundary_loops<C, S>(
    face_id: FaceId,
    face: &Face<Point3, C, S>,
    tolerance: f64,
    registry: &mut crate::bopds::SourceBoundaryEdgeRegistry,
    vertex_counter: &mut u32,
) -> Vec<TrimmingLoop>
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
        let edges = wire
            .edge_iter()
            .zip(closed.windows(2))
            .map(|(edge, segment)| TrimmingEdge {
                section_curve: None,
                original_edge: Some(edge_id_from_source_boundary(edge, registry)),
                uv_points: vec![segment[0], segment[1]],
            })
            .collect();
        result.push(loop_from_polyline(face_id, edges, closed, tolerance, vertex_counter));
    }

    result
}

fn loop_from_polyline(
    face_id: FaceId,
    edges: Vec<TrimmingEdge>,
    uv_points: Vec<Point2>,
    tolerance: f64,
    vertex_counter: &mut u32,
) -> TrimmingLoop {
    let mut closed = close_polyline(uv_points, tolerance);
    let area = signed_area(&closed);
    if area.abs() <= tolerance {
        closed = dedup_consecutive_points(closed, tolerance);
    }
    let vertex_ids = vertex_ids_for_polyline(vertex_counter, &closed);

    TrimmingLoop {
        face: face_id,
        vertex_ids,
        edges,
        signed_area: signed_area(&closed),
        uv_points: closed,
        is_outer: false,
    }
}

/// Classifies loops as outer (growth) or inner (hole) using a containment tree.
///
/// Algorithm:
/// 1. Normalize orientation so the face boundary (largest area) has canonical
///    sign (negative signed area = CW = outer in UV convention).
/// 2. Build a containment tree: for each loop, find its tightest containing
///    parent loop.
/// 3. Roots (depth 0) and even-depth loops are outer (growth); odd-depth loops
///    are holes. This correctly handles multi-level nesting (e.g., island
///    inside a hole inside the face boundary).
fn classify_loops(loops: &mut [TrimmingLoop]) {
    if loops.is_empty() {
        return;
    }

    let reference_index = loops
        .iter()
        .enumerate()
        .max_by(|(_, lhs), (_, rhs)| lhs.signed_area.abs().total_cmp(&rhs.signed_area.abs()))
        .map(|(index, _)| index)
        .unwrap();

    let n = loops.len();

    let reference_sign = loops[reference_index].signed_area.signum();
    let mut needs_reverse = vec![false; n];
    for (i, trimming_loop) in loops.iter().enumerate() {
        if reference_sign != 0.0 && trimming_loop.signed_area.signum() == reference_sign {
            needs_reverse[i] = true;
        }
    }

    if n <= 1 {
        if needs_reverse[0] {
            reverse_trimming_loop(&mut loops[0], true);
            loops[0].signed_area = -loops[0].signed_area;
        }
        loops[0].is_outer = true;
        return;
    }

    let mut parent: Vec<Option<usize>> = vec![None; n];
    for i in 0..n {
        let area_i = loops[i].signed_area.abs();
        let mut best: Option<(usize, f64)> = None;
        for j in 0..n {
            if i == j {
                continue;
            }
            let area_j = loops[j].signed_area.abs();
            if area_j <= area_i {
                continue;
            }
            if loop_contained_by(&loops[i], &loops[j]) {
                if best.map_or(true, |(_, best_area)| area_j < best_area) {
                    best = Some((j, area_j));
                }
            }
        }
        parent[i] = best.map(|(idx, _)| idx);
    }

    let depths = compute_loop_depths(&parent);

    for (i, trimming_loop) in loops.iter_mut().enumerate() {
        if needs_reverse[i] {
            let use_closed_reversal = depths[i] == 0;
            reverse_trimming_loop(trimming_loop, use_closed_reversal);
            trimming_loop.signed_area = -trimming_loop.signed_area;
        }
        trimming_loop.is_outer = depths[i] % 2 == 0;
    }
}

fn compute_loop_depths(parent: &[Option<usize>]) -> Vec<usize> {
    let n = parent.len();
    let mut depths = vec![usize::MAX; n];

    for start in 0..n {
        if depths[start] != usize::MAX {
            continue;
        }

        let mut chain = Vec::new();
        let mut cursor = Some(start);
        let anchor_depth;

        loop {
            match cursor {
                None => {
                    anchor_depth = 0;
                    break;
                }
                Some(c) if depths[c] != usize::MAX => {
                    anchor_depth = depths[c] + 1;
                    break;
                }
                Some(c) if chain.contains(&c) => {
                    anchor_depth = 0;
                    break;
                }
                Some(c) => {
                    chain.push(c);
                    cursor = parent[c];
                }
            }
        }

        for (i, &idx) in chain.iter().rev().enumerate() {
            if depths[idx] == usize::MAX {
                depths[idx] = anchor_depth + i;
            }
        }
    }

    for d in &mut depths {
        if *d == usize::MAX {
            *d = 0;
        }
    }
    depths
}

fn reverse_trimming_loop(trimming_loop: &mut TrimmingLoop, is_closed: bool) {
    trimming_loop.uv_points.reverse();
    trimming_loop.edges.reverse();
    for edge in &mut trimming_loop.edges {
        edge.uv_points.reverse();
    }
    if is_closed {
        let mut open_vertex_ids = trimming_loop.vertex_ids.clone();
        if open_vertex_ids.len() >= 2 && open_vertex_ids.first() == open_vertex_ids.last() {
            open_vertex_ids.pop();
        }
        trimming_loop.vertex_ids = match open_vertex_ids.split_first() {
            Some((first, rest)) => {
                let mut reversed = Vec::with_capacity(open_vertex_ids.len() + 1);
                reversed.push(*first);
                reversed.extend(rest.iter().rev().copied());
                reversed.push(*first);
                reversed
            }
            None => Vec::new(),
        };
    } else {
        trimming_loop.vertex_ids.reverse();
    }

    if is_closed {
        trimming_loop.uv_points = close_polyline(trimming_loop.uv_points.clone(), f64::EPSILON);
    }
}

fn close_polyline(mut uv_points: Vec<Point2>, tolerance: f64) -> Vec<Point2> {
    uv_points = dedup_consecutive_points(uv_points, tolerance);
    if uv_points.len() >= 2
        && uv_points
            .first()
            .unwrap()
            .distance2(*uv_points.last().unwrap())
            > tolerance * tolerance
    {
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

fn signed_area(polyline: &[Point2]) -> f64 { geometry_utils::signed_area(polyline) }

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

fn representative_point_on_surface<S>(
    split_face: &SplitFace,
    surface: Option<&S>,
) -> Option<Point3>
where
    S: ParametricSurface<Point = Point3>,
{
    let outer_loop = split_face
        .trimming_loops
        .iter()
        .find(|loop_| loop_.is_outer)?;
    let inner_loops: Vec<_> = split_face
        .trimming_loops
        .iter()
        .filter(|loop_| !loop_.is_outer)
        .cloned()
        .collect();

    loop_representative_point_on_surface(outer_loop, &inner_loops, surface).or_else(|| {
        inner_loops
            .iter()
            .find_map(|tl| loop_representative_point_on_surface(tl, &[], surface))
    })
}

/// Evaluates a UV point on a parametric surface to get a 3D point.
/// When surface is None (e.g., unit-test UV-only scenarios), falls back to
/// projecting onto the Z=0 plane. Production callers should always provide
/// the surface via the surfaces HashMap built from the face list.
fn eval_uv_on_surface<S>(uv: Point2, surface: Option<&S>) -> Point3
where
    S: ParametricSurface<Point = Point3>,
{
    match surface {
        Some(s) => s.subs(uv.x, uv.y),
        None => Point3::new(uv.x, uv.y, 0.0),
    }
}

fn loop_representative_point_on_surface<S>(
    trimming_loop: &TrimmingLoop,
    fragment_loops: &[TrimmingLoop],
    surface: Option<&S>,
) -> Option<Point3>
where
    S: ParametricSurface<Point = Point3>,
{
    loop_centroid(&trimming_loop.uv_points)
        .filter(|point| point_in_fragment_region(fragment_loops, *point))
        .map(|uv| eval_uv_on_surface(uv, surface))
        .or_else(|| {
            open_polygon_vertices(&trimming_loop.uv_points)
                .iter()
                .copied()
                .find(|point| point_in_fragment_region(fragment_loops, *point))
                .map(|uv| eval_uv_on_surface(uv, surface))
        })
        .or_else(|| {
            loop_interior_sample_point(trimming_loop, fragment_loops)
                .map(|uv| eval_uv_on_surface(uv, surface))
        })
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

fn loop_interior_sample_point(
    trimming_loop: &TrimmingLoop,
    fragment_loops: &[TrimmingLoop],
) -> Option<Point2> {
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
    !point_on_polygon_boundary(&trimming_loop.uv_points, point)
        && point_in_polygon(&trimming_loop.uv_points, point)
}

fn point_in_fragment_region(fragment_loops: &[TrimmingLoop], point: Point2) -> bool {
    fragment_loops.is_empty()
        || fragment_loops
            .iter()
            .any(|trimming_loop| point_in_loop_region(trimming_loop, point))
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
    geometry_utils::point_in_polygon(polygon, point)
}

fn polygon_centroid(polygon: &[Point2]) -> Option<Point2> {
    geometry_utils::polygon_centroid(polygon)
}

fn open_polygon_vertices(polygon: &[Point2]) -> &[Point2] {
    geometry_utils::open_polygon_vertices(polygon)
}

fn point_on_polygon_boundary(polygon: &[Point2], point: Point2) -> bool {
    geometry_utils::point_on_polygon_boundary(polygon, point, 1.0e-9)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{bopds::SectionCurve, BopOptions, VertexId};
    use truck_base::bounding_box::BoundingBox;
    use truck_base::cgmath64::Vector3;
    use truck_modeling::{builder, primitive, Curve, Surface};
    use truck_topology::shell::ShellCondition;
    use truck_topology::{Solid, Wire};

    const NO_FACES: &[(FaceId, Face<Point3, Curve, Surface>)] = &[];

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
    fn boundary_loops_preserve_original_edge_provenance() {
        let (left_face, right_face) = adjacent_shell_faces_with_shared_edge();

        let mut registry = crate::bopds::SourceBoundaryEdgeRegistry::default();
        let mut vtx_counter = 1_000_000u32;
        let left_loops = boundary_loops(FaceId(0), &left_face, 1.0e-9, &mut registry, &mut vtx_counter);
        let right_loops = boundary_loops(FaceId(1), &right_face, 1.0e-9, &mut registry, &mut vtx_counter);

        assert_eq!(left_loops.len(), 1);
        assert_eq!(right_loops.len(), 1);
        let left_sources: Vec<_> = canonical_loop_edges(&left_loops[0])
            .into_iter()
            .filter_map(|edge| match edge {
                CanonicalRebuiltEdge::Source(edge_id) => Some(edge_id),
                _ => None,
            })
            .collect();
        let right_sources: Vec<_> = canonical_loop_edges(&right_loops[0])
            .into_iter()
            .filter_map(|edge| match edge {
                CanonicalRebuiltEdge::Source(edge_id) => Some(edge_id),
                _ => None,
            })
            .collect();
        assert!(!left_sources.is_empty());
        assert!(left_sources
            .iter()
            .any(|edge_id| right_sources.contains(edge_id)));
    }

    #[test]
    fn edge_sewing_tracks_source_boundary_identity_for_shared_faces() {
        let (left_face, right_face) = adjacent_shell_faces_with_shared_edge();
        let split_faces = split_faces_for_source_faces(&[left_face, right_face]);
        let mut bopds = BopDs::with_options(BopOptions::default());

        let merged = merge_equivalent_vertices(&mut bopds, &split_faces, NO_FACES);
        let paths = sew_fragment_edges(&mut bopds, &split_faces, &merged);

        assert!(!paths.is_empty());
        let original_edges: Vec<_> = paths
            .iter()
            .flat_map(|path| path.edges.iter())
            .map(|edge| edge.source.original_edge)
            .collect();
        assert!(!original_edges.is_empty());
        assert!(original_edges.iter().all(|edge| edge.is_some()));
        assert!(original_edges.iter().enumerate().any(|(index, edge)| {
            original_edges
                .iter()
                .skip(index + 1)
                .any(|other| other == edge)
        }));
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

        let count =
            build_trimming_loops(&mut bopds, &[(FaceId(0), square_face_with_square_hole())]);

        assert_eq!(count, 2);
        let loops = bopds.trimming_loops();
        assert_eq!(loops.iter().filter(|loop_| loop_.is_outer).count(), 1);
        assert_eq!(loops.iter().filter(|loop_| !loop_.is_outer).count(), 1);
        assert!(loops
            .iter()
            .all(|loop_| loop_.uv_points.first() == loop_.uv_points.last()));
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
                    original_edge: None,
                    uv_points: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
                },
                TrimmingEdge {
                    section_curve: Some(SectionCurveId(2)),
                    original_edge: None,
                    uv_points: vec![Point2::new(1.0, 0.0), Point2::new(1.0, 1.0)],
                },
                TrimmingEdge {
                    section_curve: Some(SectionCurveId(3)),
                    original_edge: None,
                    uv_points: vec![Point2::new(1.0, 1.0), Point2::new(0.0, 1.0)],
                },
                TrimmingEdge {
                    section_curve: Some(SectionCurveId(4)),
                    original_edge: None,
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
        assert_eq!(
            loop_record.edges[0].uv_points,
            vec![Point2::new(0.0, 0.0), Point2::new(0.0, 1.0)]
        );
        assert_eq!(
            loop_record.edges[1].uv_points[0],
            loop_record.edges[0].uv_points[1]
        );
        assert_eq!(
            loop_record.edges[2].uv_points[0],
            loop_record.edges[1].uv_points[1]
        );
        assert_eq!(
            loop_record.edges[3].uv_points[0],
            loop_record.edges[2].uv_points[1]
        );
        assert_eq!(
            loop_record.edges[3].uv_points[1],
            loop_record.edges[0].uv_points[0]
        );
    }

    #[test]
    fn reverse_trimming_loop_preserves_vertex_alignment_for_closed_loop() {
        let mut loop_record = TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(10), VertexId(11), VertexId(12), VertexId(13)],
            edges: vec![
                line_edge(
                    Some(SectionCurveId(1)),
                    Point2::new(0.0, 0.0),
                    Point2::new(1.0, 0.0),
                ),
                line_edge(
                    Some(SectionCurveId(2)),
                    Point2::new(1.0, 0.0),
                    Point2::new(1.0, 1.0),
                ),
                line_edge(
                    Some(SectionCurveId(3)),
                    Point2::new(1.0, 1.0),
                    Point2::new(0.0, 1.0),
                ),
                line_edge(
                    Some(SectionCurveId(4)),
                    Point2::new(0.0, 1.0),
                    Point2::new(0.0, 0.0),
                ),
            ],
            uv_points: square_loop(0.0, 1.0),
            signed_area: -1.0,
            is_outer: true,
        };

        reverse_trimming_loop(&mut loop_record, true);

        assert_eq!(
            loop_record.vertex_ids,
            vec![
                VertexId(10),
                VertexId(13),
                VertexId(12),
                VertexId(11),
                VertexId(10),
            ]
        );
        assert_eq!(
            open_loop_vertex_ids(&loop_record),
            &[VertexId(10), VertexId(13), VertexId(12), VertexId(11)]
        );
        assert_eq!(
            loop_record.edges.len(),
            open_loop_vertex_ids(&loop_record).len()
        );
    }

    #[test]
    fn classify_loops_keeps_vertex_ids_aligned_after_orientation_fix() {
        let outer = TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(20), VertexId(21), VertexId(22), VertexId(23)],
            edges: vec![
                line_edge(
                    Some(SectionCurveId(10)),
                    Point2::new(0.0, 0.0),
                    Point2::new(4.0, 0.0),
                ),
                line_edge(
                    Some(SectionCurveId(11)),
                    Point2::new(4.0, 0.0),
                    Point2::new(4.0, 4.0),
                ),
                line_edge(
                    Some(SectionCurveId(12)),
                    Point2::new(4.0, 4.0),
                    Point2::new(0.0, 4.0),
                ),
                line_edge(
                    Some(SectionCurveId(13)),
                    Point2::new(0.0, 4.0),
                    Point2::new(0.0, 0.0),
                ),
            ],
            uv_points: vec![
                Point2::new(0.0, 0.0),
                Point2::new(4.0, 0.0),
                Point2::new(4.0, 4.0),
                Point2::new(0.0, 4.0),
                Point2::new(0.0, 0.0),
            ],
            signed_area: -16.0,
            is_outer: false,
        };
        let inner = TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(30), VertexId(31), VertexId(32), VertexId(33)],
            edges: vec![
                line_edge(
                    Some(SectionCurveId(20)),
                    Point2::new(1.0, 1.0),
                    Point2::new(3.0, 1.0),
                ),
                line_edge(
                    Some(SectionCurveId(21)),
                    Point2::new(3.0, 1.0),
                    Point2::new(3.0, 3.0),
                ),
                line_edge(
                    Some(SectionCurveId(22)),
                    Point2::new(3.0, 3.0),
                    Point2::new(1.0, 3.0),
                ),
                line_edge(
                    Some(SectionCurveId(23)),
                    Point2::new(1.0, 3.0),
                    Point2::new(1.0, 1.0),
                ),
            ],
            uv_points: vec![
                Point2::new(1.0, 1.0),
                Point2::new(3.0, 1.0),
                Point2::new(3.0, 3.0),
                Point2::new(1.0, 3.0),
                Point2::new(1.0, 1.0),
            ],
            signed_area: -4.0,
            is_outer: false,
        };
        let mut loops = vec![outer, inner];

        classify_loops(&mut loops);

        assert!(loops[0].is_outer);
        assert_eq!(
            loops[0].vertex_ids,
            vec![
                VertexId(20),
                VertexId(23),
                VertexId(22),
                VertexId(21),
                VertexId(20),
            ]
        );
        assert_eq!(
            open_loop_vertex_ids(&loops[0]),
            &[VertexId(20), VertexId(23), VertexId(22), VertexId(21)]
        );
        assert_eq!(
            loops[1].vertex_ids,
            vec![VertexId(33), VertexId(32), VertexId(31), VertexId(30)]
        );
        assert_eq!(
            open_loop_vertex_ids(&loops[1]),
            &[VertexId(33), VertexId(32), VertexId(31), VertexId(30)]
        );
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
    fn face_without_sections_remains_single_fragment() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        let face = unit_square_face();

        let loops = build_trimming_loops(&mut bopds, &[(FaceId(0), face)]);
        let fragments = build_split_faces(&mut bopds);

        assert_eq!(loops, 1);
        assert_eq!(fragments, 1);
        assert_eq!(bopds.split_faces().len(), 1);
        assert_eq!(bopds.split_faces()[0].trimming_loops.len(), 1);
        assert!(bopds.split_faces()[0].splitting_edges.is_empty());
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
            &[
                (FaceId(0), unit_square_face()),
                (FaceId(1), unit_square_face()),
            ],
        );

        let built = build_split_faces(&mut bopds);

        assert_eq!(built, 2);
        let faces: Vec<FaceId> = bopds
            .split_faces()
            .iter()
            .map(|split_face| split_face.original_face)
            .collect();
        assert_eq!(faces, vec![FaceId(0), FaceId(1)]);
        assert!(bopds
            .split_faces()
            .iter()
            .all(|split_face| split_face.trimming_loops.len() == 1));
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
                original_edge: None,
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
                original_edge: None,
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
                original_edge: None,
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
        assert_eq!(
            bopds.split_faces()[0].splitting_edges,
            vec![first_section, SectionCurveId(99)]
        );
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
                original_edge: None,
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
                original_edge: None,
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
                original_edge: None,
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
                original_edge: None,
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
                original_edge: None,
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
                original_edge: None,
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
        assert_eq!(
            bopds.split_faces()[1].splitting_edges,
            vec![nested_outer, hole]
        );
    }

    #[test]
    fn split_face_records_skip_faces_without_outer_loops() {
        let mut bopds = BopDs::with_options(BopOptions::default());
        bopds.push_trimming_loop(TrimmingLoop {
            face: FaceId(0),
            vertex_ids: vec![VertexId(40), VertexId(41), VertexId(42), VertexId(43)],
            edges: vec![TrimmingEdge {
                section_curve: Some(SectionCurveId(7)),
                original_edge: None,
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
        let classified =
            classify_split_faces_against_operand(&mut bopds, &[(1, opposite)], NO_FACES)
                .unwrap();

        assert_eq!(classified, 1);
        let fragment = &bopds.split_faces()[0];
        assert_eq!(
            fragment.representative_point,
            Some(Point3::new(0.5, 0.5, 0.0))
        );
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

        let opposite = operand_box(
            1,
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(0.25, 0.25, 1.0),
        );
        classify_split_faces_against_operand(&mut bopds, &[(1, opposite)], NO_FACES).unwrap();

        let fragment = &bopds.split_faces()[0];
        assert_eq!(
            fragment.representative_point,
            Some(Point3::new(1.0, 1.0, 0.0))
        );
        assert_eq!(fragment.classification, Some(PointClassification::Outside));
    }

    #[test]
    fn fragment_classification_marks_boundary_representative_points_as_on_boundary() {
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

        let opposite = operand_box(1, Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let classified =
            classify_split_faces_against_operand(&mut bopds, &[(1, opposite)], NO_FACES)
                .unwrap();

        assert_eq!(classified, 1);
        let fragment = &bopds.split_faces()[0];
        assert_eq!(
            fragment.representative_point,
            Some(Point3::new(0.5, 0.5, 0.0))
        );
        assert_eq!(fragment.classification, Some(PointClassification::OnBoundary));
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

        let classified = classify_split_faces_against_operand(
            &mut bopds,
            &[(
                0,
                operand_box(0, Point3::new(0.0, 0.0, -1.0), Point3::new(1.0, 1.0, 1.0)),
            )],
            NO_FACES,
        )
        .unwrap();

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
        assert_eq!(
            selected[0].classification,
            Some(PointClassification::Inside)
        );
        assert_eq!(
            selected[1].classification,
            Some(PointClassification::OnBoundary)
        );
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
        assert_eq!(
            selected[0].classification,
            Some(PointClassification::Outside)
        );
        assert_eq!(selected[1].operand_rank, 1);
        assert_eq!(
            selected[1].classification,
            Some(PointClassification::OnBoundary)
        );
        assert_eq!(selected[2].operand_rank, 1);
        assert_eq!(
            selected[2].classification,
            Some(PointClassification::Outside)
        );
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
        assert_eq!(
            selected[0].classification,
            Some(PointClassification::Outside)
        );
        assert_eq!(selected[1].operand_rank, 0);
        assert_eq!(
            selected[1].classification,
            Some(PointClassification::OnBoundary)
        );
        assert_eq!(selected[2].operand_rank, 1);
        assert_eq!(
            selected[2].classification,
            Some(PointClassification::OnBoundary)
        );
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
        let selected = vec![
            SplitFace {
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
            },
            SplitFace {
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
            },
        ];

        let map = merge_equivalent_vertices(&mut bopds, &selected, NO_FACES);

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
        let selected = vec![
            SplitFace {
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
            },
            SplitFace {
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
            },
        ];

        let map = merge_equivalent_vertices(&mut bopds, &selected, NO_FACES);

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

        let first_map = merge_equivalent_vertices(&mut bopds, &first, NO_FACES);
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

        let second_map = merge_equivalent_vertices(&mut bopds, &second, NO_FACES);
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

        let map = merge_equivalent_vertices(&mut bopds, &selected, NO_FACES);

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
    fn vertex_merging_and_edge_sewing_replace_derived_state_when_re_run() {
        let mut bopds = BopDs::with_options(BopOptions {
            geometric_tol: 1.0e-3,
            ..BopOptions::default()
        });
        let face_a = bopds.register_face_source(0);
        let face_b = bopds.register_face_source(1);
        let first = vec![
            SplitFace {
                original_face: face_a,
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: face_a,
                    vertex_ids: vec![VertexId(10), VertexId(11)],
                    edges: vec![line_edge(
                        None,
                        Point2::new(0.0, 0.0),
                        Point2::new(1.0, 0.0),
                    )],
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
                    edges: vec![line_edge(
                        None,
                        Point2::new(1.0, 0.0),
                        Point2::new(0.0, 0.0),
                    )],
                    uv_points: vec![Point2::new(1.0, 0.0), Point2::new(0.0, 0.0)],
                    signed_area: 0.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::Inside),
            },
        ];

        let first_map = merge_equivalent_vertices(&mut bopds, &first, NO_FACES);
        let first_paths = sew_fragment_edges(&mut bopds, &first, &first_map);
        assert_eq!(bopds.merged_vertices().len(), 2);
        assert_eq!(first_map.len(), 4);
        assert_eq!(bopds.sewn_paths().len(), 1);
        assert_eq!(first_paths.len(), 1);
        assert!(bopds.merged_vertices().iter().any(|merged| {
            merged.original_vertices == vec![VertexId(10), VertexId(21)]
        }));
        assert!(bopds.merged_vertices().iter().any(|merged| {
            merged.original_vertices == vec![VertexId(11), VertexId(20)]
        }));
        assert_eq!(bopds.sewn_paths()[0].edges.len(), 2);

        let second = vec![SplitFace {
            original_face: face_a,
            operand_rank: 0,
            trimming_loops: vec![TrimmingLoop {
                face: face_a,
                vertex_ids: vec![VertexId(30), VertexId(31), VertexId(32)],
                edges: vec![
                    line_edge(
                        Some(SectionCurveId(7)),
                        Point2::new(0.0, 0.0),
                        Point2::new(1.0, 0.0),
                    ),
                    line_edge(
                        Some(SectionCurveId(8)),
                        Point2::new(2.0, 0.0),
                        Point2::new(1.0, 0.0),
                    ),
                ],
                uv_points: vec![
                    Point2::new(0.0, 0.0),
                    Point2::new(1.0, 0.0),
                    Point2::new(2.0, 0.0),
                ],
                signed_area: 0.0,
                is_outer: true,
            }],
            splitting_edges: vec![SectionCurveId(7), SectionCurveId(8)],
            representative_point: None,
            classification: Some(PointClassification::OnBoundary),
        }];

        let second_map = merge_equivalent_vertices(&mut bopds, &second, NO_FACES);
        let second_paths = sew_fragment_edges(&mut bopds, &second, &second_map);
        assert_eq!(bopds.merged_vertices().len(), 3);
        assert_eq!(second_map.len(), 3);
        assert!(second_map.keys().all(|id| !first_map.contains_key(id)));
        assert_eq!(bopds.sewn_paths().len(), 1);
        assert_eq!(second_paths.len(), 1);
        assert_eq!(bopds.sewn_paths(), second_paths.as_slice());
        assert_eq!(bopds.sewn_paths()[0].edges.len(), 2);
        assert!(!bopds.sewn_paths()[0].is_closed);
        assert!(bopds.merged_vertices().iter().all(|merged| {
            merged
                .original_vertices
                .iter()
                .all(|vertex| matches!(*vertex, VertexId(30) | VertexId(31) | VertexId(32)))
        }));
        assert!(bopds.sewn_paths()[0].edges.iter().all(|edge| {
            matches!(edge.start_vertex, VertexId(30) | VertexId(31) | VertexId(32))
                && matches!(edge.end_vertex, VertexId(30) | VertexId(31) | VertexId(32))
        }));
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
                    edges: vec![line_edge(
                        None,
                        Point2::new(0.0, 0.0),
                        Point2::new(1.0, 0.0),
                    )],
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
                    edges: vec![line_edge(
                        None,
                        Point2::new(1.0, 0.0),
                        Point2::new(0.0, 0.0),
                    )],
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
        assert_eq!(
            endpoints,
            vec![(VertexId(1), VertexId(2)), (VertexId(2), VertexId(1))]
        );
        let first_source = paths[0].edges[0].source;
        let second_source = paths[0].edges[1].source;
        assert_eq!(first_source.face, face_a);
        assert_eq!(second_source.face, face_b);
        assert_eq!(
            paths[0].edges[0].sewn_pair,
            Some(SewnEdgePair::new(first_source, second_source))
        );
        assert_eq!(
            paths[0].edges[1].sewn_pair,
            Some(SewnEdgePair::new(first_source, second_source))
        );
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
                    edges: vec![line_edge(
                        None,
                        Point2::new(0.0, 0.0),
                        Point2::new(1.0, 0.0),
                    )],
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
                    edges: vec![line_edge(
                        None,
                        Point2::new(0.0, 0.0),
                        Point2::new(1.0, 0.0),
                    )],
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
                    line_edge(
                        Some(SectionCurveId(7)),
                        Point2::new(0.0, 0.0),
                        Point2::new(1.0, 0.0),
                    ),
                    line_edge(
                        Some(SectionCurveId(8)),
                        Point2::new(2.0, 0.0),
                        Point2::new(1.0, 0.0),
                    ),
                ],
                uv_points: vec![
                    Point2::new(0.0, 0.0),
                    Point2::new(1.0, 0.0),
                    Point2::new(2.0, 0.0),
                ],
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
        assert_eq!(
            sewn_pairs,
            vec![(VertexId(1), VertexId(2)), (VertexId(1), VertexId(2))]
        );
        assert_eq!(paths[0].edges[0].source.original_edge, None);
        assert_eq!(paths[0].edges[1].source.original_edge, None);
        assert_eq!(
            paths[0].edges[0].sewn_pair,
            Some(SewnEdgePair::new(
                paths[0].edges[0].source,
                paths[0].edges[1].source
            ))
        );
        assert_eq!(
            paths[0].edges[1].sewn_pair,
            Some(SewnEdgePair::new(
                paths[0].edges[0].source,
                paths[0].edges[1].source
            ))
        );
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
                    line_edge(
                        Some(SectionCurveId(7)),
                        Point2::new(0.0, 0.0),
                        Point2::new(1.0, 0.0),
                    ),
                    line_edge(
                        Some(SectionCurveId(8)),
                        Point2::new(1.0, 0.0),
                        Point2::new(2.0, 0.0),
                    ),
                ],
                uv_points: vec![
                    Point2::new(0.0, 0.0),
                    Point2::new(1.0, 0.0),
                    Point2::new(2.0, 0.0),
                ],
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
        let mut bopds = BopDs::with_options(BopOptions::default());

        let shells = assemble_shells(&mut bopds, &split_faces, &faces_by_id, &FxHashMap::default()).unwrap();

        assert_eq!(shells.len(), 1);
        assert_eq!(shells[0].shell_condition(), ShellCondition::Closed);
    }

    #[test]
    fn shell_closure_rejects_boundary_edge_component() {
        let shell = open_box_shell_missing_top();
        let faces: Vec<_> = shell.face_iter().cloned().collect();
        let split_faces = split_faces_for_source_faces(&faces);
        let faces_by_id = source_face_map(&split_faces, &faces);
        let mut bopds = BopDs::with_options(BopOptions::default());

        let err = assemble_shells(&mut bopds, &split_faces, &faces_by_id, &FxHashMap::default()).unwrap_err();

        assert!(matches!(err, BopError::TopologyInvariantBroken));
    }

    #[test]
    fn disconnected_closed_shell_source_faces_can_reuse_original_topology() {
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

        let mut registry = crate::bopds::SourceBoundaryEdgeRegistry::default();
        assert!(split_faces.iter().all(|split_face| {
            let original_face = faces_by_id.get(&split_face.original_face).unwrap();
            split_face_can_reuse_original_face(split_face, original_face, &mut registry)
        }));
    }

    #[test]
    fn sew_shell_faces_separates_disconnected_closed_components() {
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
        let mut registry = crate::bopds::SourceBoundaryEdgeRegistry::default();
        let rebuilt_faces = split_faces
            .iter()
            .map(|split_face| {
                let original_face = faces_by_id.get(&split_face.original_face).unwrap();
                if split_face_can_reuse_original_face(split_face, original_face, &mut registry) {
                    original_face.clone()
                } else {
                    let mut cache = TopologyCache::new();
                    let empty_merged = FxHashMap::default();
                    rebuild_face_from_split_face(split_face, original_face, &mut cache, &empty_merged).unwrap()
                }
            })
            .collect::<Vec<_>>();

        let shells = sew_shell_faces(&split_faces, rebuilt_faces).unwrap();

        assert_eq!(shells.len(), 2);
        assert!(shells.iter().all(|shell| shell.len() == 6));
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
        let mut bopds = BopDs::with_options(BopOptions::default());

        let shells = assemble_shells(&mut bopds, &split_faces, &faces_by_id, &FxHashMap::default()).unwrap();

        assert_eq!(shells.len(), 2);
        assert!(shells
            .iter()
            .all(|shell| shell.shell_condition() == ShellCondition::Closed));
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
    fn shared_vertex_only_faces_still_form_one_component() {
        let split_faces = vec![
            SplitFace {
                original_face: FaceId(0),
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: FaceId(0),
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
                original_face: FaceId(1),
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: FaceId(1),
                    vertex_ids: vec![VertexId(3), VertexId(10), VertexId(11), VertexId(12)],
                    edges: vec![
                        line_edge(None, Point2::new(1.0, 1.0), Point2::new(2.0, 1.0)),
                        line_edge(None, Point2::new(2.0, 1.0), Point2::new(2.0, 2.0)),
                        line_edge(None, Point2::new(2.0, 2.0), Point2::new(1.0, 2.0)),
                        line_edge(None, Point2::new(1.0, 2.0), Point2::new(1.0, 1.0)),
                    ],
                    uv_points: vec![
                        Point2::new(1.0, 1.0),
                        Point2::new(2.0, 1.0),
                        Point2::new(2.0, 2.0),
                        Point2::new(1.0, 2.0),
                        Point2::new(1.0, 1.0),
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

        assert_eq!(components, vec![vec![0, 1]]);
    }

    #[test]
    fn shared_vertex_only_faces_do_not_create_orientation_adjacency() {
        let split_faces = vec![
            SplitFace {
                original_face: FaceId(0),
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: FaceId(0),
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
                original_face: FaceId(1),
                operand_rank: 0,
                trimming_loops: vec![TrimmingLoop {
                    face: FaceId(1),
                    vertex_ids: vec![VertexId(3), VertexId(10), VertexId(11), VertexId(12)],
                    edges: vec![
                        line_edge(None, Point2::new(1.0, 1.0), Point2::new(2.0, 1.0)),
                        line_edge(None, Point2::new(2.0, 1.0), Point2::new(2.0, 2.0)),
                        line_edge(None, Point2::new(2.0, 2.0), Point2::new(1.0, 2.0)),
                        line_edge(None, Point2::new(1.0, 2.0), Point2::new(1.0, 1.0)),
                    ],
                    uv_points: vec![
                        Point2::new(1.0, 1.0),
                        Point2::new(2.0, 1.0),
                        Point2::new(2.0, 2.0),
                        Point2::new(1.0, 2.0),
                        Point2::new(1.0, 1.0),
                    ],
                    signed_area: -1.0,
                    is_outer: true,
                }],
                splitting_edges: vec![],
                representative_point: None,
                classification: Some(PointClassification::OnBoundary),
            },
        ];

        let adjacency = orientation_adjacency(&split_faces);

        assert!(adjacency.iter().all(|neighbors| neighbors.is_empty()));
    }

    #[test]
    fn shared_edge_faces_create_orientation_adjacency() {
        let shared = SectionCurveId(501);
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

        let adjacency = orientation_adjacency(&split_faces);

        assert_eq!(adjacency, vec![vec![1], vec![0]]);
    }

    #[test]
    fn canonical_rebuilt_edge_ids_distinguish_shared_and_open_boundaries() {
        let shared = SectionCurveId(91);
        let left = SplitFace {
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
        };
        let right = SplitFace {
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
        };
        let open = SplitFace {
            original_face: FaceId(2),
            operand_rank: 0,
            trimming_loops: vec![TrimmingLoop {
                face: FaceId(2),
                vertex_ids: vec![VertexId(0), VertexId(1), VertexId(4), VertexId(5)],
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
        };

        let left_topology = rebuilt_face_topology(&left);
        let right_topology = rebuilt_face_topology(&right);
        let open_topology = rebuilt_face_topology(&open);

        assert_eq!(
            left_topology.shared_edges,
            vec![CanonicalRebuiltEdge::Source(EdgeId(shared.0))]
        );
        assert_eq!(
            right_topology.shared_edges,
            vec![CanonicalRebuiltEdge::Source(EdgeId(shared.0))]
        );
        assert!(open_topology.shared_edges.is_empty());
        assert!(left_topology
            .boundary_edges
            .contains(&CanonicalRebuiltEdge::OpenBoundary(
                FaceId(0),
                VertexId(1),
                VertexId(2)
            )));
        assert!(!split_faces_share_orientable_edge(
            &left,
            &open,
            &left_topology,
            &open_topology
        ));
        assert!(split_faces_share_component(
            &left,
            &open,
            &left_topology,
            &open_topology
        ));
        assert!(split_faces_share_component(
            &left,
            &right,
            &left_topology,
            &right_topology
        ));
    }

    #[test]
    fn canonical_rebuilt_edge_ids_share_true_non_section_trim_edges_only() {
        let shared_left = SplitFace {
            original_face: FaceId(10),
            operand_rank: 0,
            trimming_loops: vec![TrimmingLoop {
                face: FaceId(10),
                vertex_ids: vec![VertexId(100), VertexId(101), VertexId(102), VertexId(103)],
                edges: vec![
                    TrimmingEdge {
                        section_curve: None,
                        original_edge: None,
                        uv_points: vec![
                            Point2::new(0.0, 0.0),
                            Point2::new(1.0, 0.0),
                            Point2::new(0.0, 0.0),
                        ],
                    },
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
        };
        let shared_right = SplitFace {
            original_face: FaceId(11),
            operand_rank: 0,
            trimming_loops: vec![TrimmingLoop {
                face: FaceId(11),
                vertex_ids: vec![VertexId(100), VertexId(101), VertexId(111), VertexId(112)],
                edges: vec![
                    TrimmingEdge {
                        section_curve: None,
                        original_edge: None,
                        uv_points: vec![
                            Point2::new(1.0, 0.0),
                            Point2::new(0.0, 0.0),
                            Point2::new(1.0, 0.0),
                        ],
                    },
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
            classification: Some(PointClassification::OnBoundary),
        };
        let open = SplitFace {
            original_face: FaceId(12),
            operand_rank: 0,
            trimming_loops: vec![TrimmingLoop {
                face: FaceId(12),
                vertex_ids: vec![VertexId(100), VertexId(101), VertexId(122), VertexId(123)],
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
        };

        let left_topology = rebuilt_face_topology(&shared_left);
        let right_topology = rebuilt_face_topology(&shared_right);
        let open_topology = rebuilt_face_topology(&open);

        assert!(left_topology
            .shared_edges
            .contains(&CanonicalRebuiltEdge::SharedBoundary(
                VertexId(100),
                VertexId(101)
            )));
        assert!(right_topology
            .shared_edges
            .contains(&CanonicalRebuiltEdge::SharedBoundary(
                VertexId(100),
                VertexId(101)
            )));
        assert!(open_topology.shared_edges.is_empty());
        assert!(split_faces_share_component(
            &shared_left,
            &shared_right,
            &left_topology,
            &right_topology,
        ));
        assert!(!split_faces_share_orientable_edge(
            &shared_left,
            &open,
            &left_topology,
            &open_topology,
        ));
        assert!(split_faces_share_component(
            &shared_left,
            &open,
            &left_topology,
            &open_topology,
        ));
    }

    #[test]
    fn shell_orientation_rejects_closed_shell_with_inverted_faces() {
        let mut shell = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]))
        .into_boundaries()
        .pop()
        .unwrap();
        for face in shell.iter_mut() {
            face.invert();
        }
        let shell: truck_topology::Shell<Point3, Curve, Surface> = shell.into();
        assert_eq!(shell.shell_condition(), ShellCondition::Closed);

        let faces: Vec<_> = shell.into_iter().collect();
        let split_faces = split_faces_for_source_faces(&faces);
        let faces_by_id = source_face_map(&split_faces, &faces);
        let mut bopds = BopDs::with_options(BopOptions::default());

        let err = assemble_shells(&mut bopds, &split_faces, &faces_by_id, &FxHashMap::default()).unwrap_err();

        assert!(matches!(err, BopError::TopologyInvariantBroken));
    }

    #[test]
    fn shell_orientation_accepts_outward_closed_shell() {
        let shell = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]))
        .into_boundaries()
        .pop()
        .unwrap();
        let shell: truck_topology::Shell<Point3, Curve, Surface> = shell.into();
        assert_eq!(shell.shell_condition(), ShellCondition::Closed);

        let faces: Vec<_> = shell.into_iter().collect();
        let split_faces = split_faces_for_source_faces(&faces);
        let faces_by_id = source_face_map(&split_faces, &faces);
        let mut bopds = BopDs::with_options(BopOptions::default());

        let shells = assemble_shells(&mut bopds, &split_faces, &faces_by_id, &FxHashMap::default()).unwrap();

        assert_eq!(shells.len(), 1);
        assert_eq!(shells[0].shell_condition(), ShellCondition::Closed);
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

        let mut cache1 = TopologyCache::new();
        let mut cache2 = TopologyCache::new();
        let empty_merged = FxHashMap::default();
        let rebuilt_first = rebuild_face_from_split_face(&first, &source_face, &mut cache1, &empty_merged).unwrap();
        let rebuilt_second = rebuild_face_from_split_face(&second, &source_face, &mut cache2, &empty_merged).unwrap();

        assert_ne!(rebuilt_first.id(), rebuilt_second.id());
        let first_points: Vec<_> = rebuilt_first.boundaries()[0]
            .vertex_iter()
            .map(|v| v.point())
            .collect();
        let second_points: Vec<_> = rebuilt_second.boundaries()[0]
            .vertex_iter()
            .map(|v| v.point())
            .collect();
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
        assert!(solids
            .iter()
            .all(|solid| solid.boundaries()[0].shell_condition() == ShellCondition::Closed));
    }

    #[test]
    fn minimal_closed_fragment_pipeline_builds_one_solid() {
        let shell = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]))
        .into_boundaries()
        .pop()
        .unwrap();

        let faces: Vec<_> = shell.face_iter().cloned().collect();
        let split_faces = split_faces_for_source_faces(&faces);
        let face_map = source_face_map(&split_faces, &faces);
        let mut bopds = BopDs::with_options(BopOptions::default());

        let shells = assemble_shells(&mut bopds, &split_faces, &face_map, &FxHashMap::default()).unwrap();
        assert_eq!(shells.len(), 1);
        assert_eq!(shells[0].shell_condition(), ShellCondition::Closed);

        let solids = build_solids_from_shells(shells).unwrap();
        assert_eq!(solids.len(), 1);
        assert_eq!(solids[0].boundaries().len(), 1);
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
                trimming_loops: vec![outer_loop(
                    face_id,
                    square_loop(index as f64, index as f64 + 1.0),
                )],
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

    fn square_face(outer: [Point3; 4], hole: Option<[Point3; 4]>) -> Face<Point3, Curve, Surface> {
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

    fn adjacent_shell_faces_with_shared_edge(
    ) -> (Face<Point3, Curve, Surface>, Face<Point3, Curve, Surface>) {
        let shell = primitive::cuboid(BoundingBox::from_iter([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ]))
        .into_boundaries()
        .pop()
        .unwrap();
        let faces: Vec<_> = shell.face_iter().cloned().collect();

        for left in 0..faces.len() {
            for right in (left + 1)..faces.len() {
                for lhs_edge in faces[left].boundaries().iter().flatten() {
                    for rhs_edge in faces[right].boundaries().iter().flatten() {
                        if lhs_edge.id() == rhs_edge.id() {
                            return (faces[left].clone(), faces[right].clone());
                        }
                    }
                }
            }
        }

        panic!("expected adjacent cuboid faces with a shared edge");
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

    fn line_edge(
        section_curve: Option<SectionCurveId>,
        start: Point2,
        end: Point2,
    ) -> TrimmingEdge {
        TrimmingEdge {
            section_curve,
            original_edge: None,
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
        let mut counter = face.0 * 10_000 + seed * 100;
        TrimmingLoop {
            face,
            vertex_ids: vertex_ids_for_polyline(&mut counter, &uv_points),
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
        let mut registry = crate::bopds::SourceBoundaryEdgeRegistry::default();
        let mut vtx_counter = 1_000_000u32;
        faces
            .iter()
            .enumerate()
            .map(|(index, face)| SplitFace {
                original_face: FaceId(index as u32),
                operand_rank: 0,
                trimming_loops: boundary_loops(
                    FaceId(index as u32),
                    face,
                    1.0e-9,
                    &mut registry,
                    &mut vtx_counter,
                ),
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
