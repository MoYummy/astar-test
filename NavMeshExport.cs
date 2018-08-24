using UnityEditor;
using UnityEngine;
using UnityEngine.AI;

public class NavMeshExport {
	[MenuItem("NavMesh/Unity")]
	static void Unity()
	{
		var unityNavMesh = NavMesh.CalculateTriangulation();
		Debug.Log("Unity NavMesh: " + unityNavMesh.vertices.Length + " verts, " + (unityNavMesh.indices.Length / 3) + " tris");
	}

	[MenuItem("NavMesh/Astar")]
	static void Astar() {
		var astarNavMesh = AstarPath.active.data;
		var nverts = 0;
		var ntris = 0;
		if (astarNavMesh != null) {
			if (astarNavMesh.recastGraph != null) {
				foreach (var tile in astarNavMesh.recastGraph.GetTiles()) {
					nverts += tile.verts.Length;
					ntris += tile.tris.Length;
				}
			} else if (astarNavMesh.graphs != null && astarNavMesh.graphs.Length > 0) {
				for (int i = 0; i < astarNavMesh.graphs.Length; i++) {
					var graph = astarNavMesh.graphs[i] as Pathfinding.RecastGraph;
					foreach (var tile in graph.GetTiles()) {
						nverts += tile.verts.Length;
						ntris += tile.tris.Length;
					}
				}
			}
		}
		Debug.Log("Astar NavMesh: " + nverts + " verts, " + (ntris / 3) + " tris");
	}
}

