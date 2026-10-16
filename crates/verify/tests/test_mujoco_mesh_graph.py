import unittest
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mujoco_mesh_graph import (
    MeshHullTooLarge,
    MAX_COMPILED_MESH_QUERY_VERTICES,
    compiled_mesh_hull_vertex_ids,
)
import mujoco_mesh_graph


class HeaderOnlyGraph:
    def __init__(self, hull_count, face_count, length):
        self.header = [hull_count, face_count]
        self.length = length

    def __len__(self):
        return self.length

    def __getitem__(self, index):
        if isinstance(index, int) and 0 <= index < len(self.header):
            return self.header[index]
        raise AssertionError("packed graph data was sliced before count validation")


def tetrahedron_graph(vertex_count=4, face_ids=None):
    if face_ids is None:
        face_ids = [0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3]
    vertex_ids = list(range(4))
    edge_count = len(vertex_ids) + len(face_ids)
    return [
        len(vertex_ids),
        len(face_ids) // 3,
        *([0] * len(vertex_ids)),
        *vertex_ids,
        *([0] * edge_count),
        *face_ids,
    ]


class CompiledMeshGraphTests(unittest.TestCase):
    def test_only_accepts_the_mujoco_version_whose_graph_layout_is_verified(self):
        version_check = getattr(
            mujoco_mesh_graph, "require_supported_mesh_graph_version", None
        )
        self.assertIsNotNone(version_check)
        self.assertIsNone(version_check("3.7.0"))
        with self.assertRaisesRegex(ValueError, "only MuJoCo 3.7.0 is verified"):
            version_check("3.7.1")

    def test_wx250s_extracted_vertices_match_mujoco_compiled_hull_faces(self):
        repo_root = Path(__file__).resolve().parents[3]
        model_path = repo_root / "robots/external/wx250s/model/wx250s.xml"
        model = mujoco.MjModel.from_xml_path(str(model_path))
        graph = model.mesh_graph
        graph_addresses = [int(value) for value in model.mesh_graphadr if int(value) >= 0]
        checked_meshes = 0

        for mesh_id, graph_start_value in enumerate(model.mesh_graphadr):
            graph_start = int(graph_start_value)
            if graph_start < 0:
                continue

            graph_end = min(
                (address for address in graph_addresses if address > graph_start),
                default=len(graph),
            )
            selected_ids = compiled_mesh_hull_vertex_ids(
                graph,
                graph_start,
                graph_end,
                int(model.mesh_vertnum[mesh_id]),
            )
            hull_count = int(graph[graph_start])
            face_count = int(graph[graph_start + 1])
            face_start = graph_start + 2 + 3 * hull_count + 3 * face_count
            face_ids = {
                int(value) for value in graph[face_start : face_start + 3 * face_count]
            }
            vertex_start = int(model.mesh_vertadr[mesh_id])
            selected_vertices = sorted(
                tuple(float(value) for value in model.mesh_vert[vertex_start + index])
                for index in selected_ids
            )
            face_vertices = sorted(
                tuple(float(value) for value in model.mesh_vert[vertex_start + index])
                for index in face_ids
            )

            self.assertEqual(selected_vertices, face_vertices, f"mesh {mesh_id}")
            checked_meshes += 1

        self.assertGreater(checked_meshes, 0, "WX250S should contain compiled convex mesh hulls")

    def test_extracted_hull_matches_known_tetrahedron_and_excludes_interior_vertex(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            (root / "known.obj").write_text(
                """v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
v 0.1 0.1 0.1
f 1 3 2
f 1 2 4
f 1 4 3
f 2 3 4
""",
                encoding="utf-8",
            )
            (root / "model.xml").write_text(
                """<mujoco>
  <compiler meshdir="."/>
  <asset><mesh name="known" file="known.obj"/></asset>
  <worldbody><body><geom name="g" type="mesh" mesh="known"/></body></worldbody>
</mujoco>
""",
                encoding="utf-8",
            )
            model = mujoco.MjModel.from_xml_path(str(root / "model.xml"))

        mesh_id = int(model.geom_dataid[0])
        graph_start = int(model.mesh_graphadr[mesh_id])
        graph_starts = [int(value) for value in model.mesh_graphadr if int(value) >= 0]
        graph_end = min(
            (value for value in graph_starts if value > graph_start),
            default=len(model.mesh_graph),
        )
        self.assertEqual(int(model.mesh_vertnum[mesh_id]), 5)
        selected_ids = compiled_mesh_hull_vertex_ids(
            model.mesh_graph,
            graph_start,
            graph_end,
            int(model.mesh_vertnum[mesh_id]),
        )

        # The authored OBJ defines vertices 0..3 as a tetrahedron and vertex 4
        # as its interior point. This expected hull is independent of packed
        # graph offsets and the parser's face/index consistency checks.
        self.assertEqual(set(selected_ids), {0, 1, 2, 3})
        self.assertNotIn(4, selected_ids)

    def test_reads_hull_ids_from_one_complete_packed_record(self):
        graph = tetrahedron_graph()
        self.assertEqual(
            compiled_mesh_hull_vertex_ids(graph, 0, len(graph), 4),
            [0, 1, 2, 3],
        )

    def test_rejects_record_that_would_consume_following_mesh(self):
        graph = tetrahedron_graph() + tetrahedron_graph()
        with self.assertRaisesRegex(ValueError, "local graph record"):
            compiled_mesh_hull_vertex_ids(graph, 0, 20, 4)

    def test_rejects_face_vertex_mismatch(self):
        graph = tetrahedron_graph(face_ids=[0, 2, 1, 0, 1, 2, 1, 2, 2, 2, 0, 2])
        with self.assertRaisesRegex(ValueError, "do not match packed faces"):
            compiled_mesh_hull_vertex_ids(graph, 0, len(graph), 4)

    def test_bounds_hull_size_before_materializing_vertex_indices(self):
        hull_count = MAX_COMPILED_MESH_QUERY_VERTICES + 1
        face_count = 4
        edge_count = hull_count + 3 * face_count
        graph = [hull_count, face_count] + [0] * (
            3 * hull_count + 6 * face_count
        )
        with self.assertRaises(MeshHullTooLarge):
            compiled_mesh_hull_vertex_ids(graph, 0, len(graph), hull_count)

    def test_rejects_face_count_above_convex_hull_bound_before_slicing(self):
        graph = HeaderOnlyGraph(hull_count=4, face_count=5, length=100)
        with self.assertRaisesRegex(ValueError, "face count exceeds convex hull bound"):
            compiled_mesh_hull_vertex_ids(graph, 0, 100, 4)

    def test_rejects_total_graph_entry_limit_before_slicing(self):
        graph = HeaderOnlyGraph(hull_count=4, face_count=4, length=100)
        with patch.object(
            mujoco_mesh_graph,
            "MAX_COMPILED_MESH_GRAPH_ENTRIES",
            37,
            create=True,
        ):
            with self.assertRaisesRegex(ValueError, "packed graph record exceeds entry limit"):
                compiled_mesh_hull_vertex_ids(graph, 0, 100, 4)


if __name__ == "__main__":
    unittest.main()
