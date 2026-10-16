"""Validation helpers for MuJoCo's packed compiled convex-mesh graph."""

MAX_COMPILED_MESH_QUERY_VERTICES = 100_000
MAX_COMPILED_MESH_QUERY_FACES = 2 * MAX_COMPILED_MESH_QUERY_VERTICES - 4
MAX_COMPILED_MESH_GRAPH_ENTRIES = 1_500_000
SUPPORTED_MUJOCO_MESH_GRAPH_VERSION = "3.7.0"


class MeshHullTooLarge(ValueError):
    """The compiled collision hull exceeds the bounded query representation."""


def require_supported_mesh_graph_version(version: str) -> None:
    """Refuse packed graph layouts that have not been independently verified."""
    if version != SUPPORTED_MUJOCO_MESH_GRAPH_VERSION:
        raise ValueError(
            "only MuJoCo 3.7.0 is verified for compiled mesh graph parsing; "
            f"received {version}"
        )


def compiled_mesh_hull_vertex_ids(
    graph: list[int],
    graph_start: int,
    graph_end: int,
    mesh_vertex_count: int,
) -> list[int]:
    """Return local compiled-vertex ids after validating one packed graph record.

    The graph record stores counts, per-vertex edge offsets, vertex ids,
    per-edge local ids, and triangulated face vertex ids.  `graph_end` is the
    start of the next record (or the end of the packed array), so malformed
    counts cannot consume a neighboring mesh's record.
    """
    if graph_start < 0 or graph_start + 2 > graph_end or graph_end > len(graph):
        raise ValueError("invalid mesh graph record bounds")

    hull_count = int(graph[graph_start])
    face_count = int(graph[graph_start + 1])
    if hull_count < 4 or face_count < 4:
        raise ValueError("compiled mesh hull is incomplete")

    if hull_count > MAX_COMPILED_MESH_QUERY_VERTICES:
        raise MeshHullTooLarge(
            f"compiled hull has {hull_count} vertices; limit is "
            f"{MAX_COMPILED_MESH_QUERY_VERTICES}"
        )
    if (
        face_count > MAX_COMPILED_MESH_QUERY_FACES
        or face_count > 2 * hull_count - 4
    ):
        raise ValueError("compiled mesh face count exceeds convex hull bound")

    record_entries = 2 + 3 * hull_count + 6 * face_count
    if record_entries > MAX_COMPILED_MESH_GRAPH_ENTRIES:
        raise ValueError("packed graph record exceeds entry limit")
    record_end = graph_start + record_entries
    if record_end > graph_end:
        raise ValueError("compiled mesh hull exceeds its local graph record")

    edge_count = hull_count + 3 * face_count

    edge_offsets_start = graph_start + 2
    vertex_ids_start = edge_offsets_start + hull_count
    edge_ids_start = vertex_ids_start + hull_count
    face_ids_start = edge_ids_start + edge_count
    vertex_ids = [int(value) for value in graph[vertex_ids_start:edge_ids_start]]
    edge_offsets = [int(value) for value in graph[edge_offsets_start:vertex_ids_start]]
    edge_ids = [int(value) for value in graph[edge_ids_start:face_ids_start]]
    face_ids = [int(value) for value in graph[face_ids_start:record_end]]

    if len(vertex_ids) != hull_count or len(set(vertex_ids)) != hull_count:
        raise ValueError("compiled mesh hull has duplicate or missing vertex ids")
    if any(value < 0 or value >= mesh_vertex_count for value in vertex_ids):
        raise ValueError("compiled mesh hull references an invalid vertex")
    if any(value < 0 or value > edge_count for value in edge_offsets):
        raise ValueError("compiled mesh hull has an invalid vertex edge offset")
    if any(value < -1 or value >= hull_count for value in edge_ids):
        raise ValueError("compiled mesh hull has an invalid local edge vertex id")
    if len(face_ids) != 3 * face_count:
        raise ValueError("compiled mesh hull has incomplete face indices")
    if any(value < 0 or value >= mesh_vertex_count for value in face_ids):
        raise ValueError("compiled mesh hull face references an invalid vertex")
    if set(face_ids) != set(vertex_ids):
        raise ValueError("compiled mesh hull vertex ids do not match packed faces")

    return vertex_ids
