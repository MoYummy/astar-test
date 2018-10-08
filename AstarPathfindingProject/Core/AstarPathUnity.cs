using System.Collections.Generic;
using Pathfinding;
using Pathfinding.Util;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.Profiling;

public partial class AstarPath : VersionedMonoBehaviour {
	public void ClearUnity()
	{
		var surface = GetNavMeshSurface();
		if (surface == null)
		{
			Debug.LogError("NavMesh Surface invalid.");
			return;
		}
		surface.RemoveData();
	}

	public void ScanFromUnity()
	{
		Profiler.BeginSample("Scan");

		var recast = GetFirstActiveRecastGraph(); // multi recast graph not supported
		if (recast == null)
		{
			Debug.LogError("Recast graph not found.");
			return;
		}

		var surface = GetNavMeshSurface();
		if (surface == null)
		{
			Debug.LogError("NavMesh Surface invalid.");
			return;
		}

		if (!ConfigNavMeshSurface(recast, ref surface))
		{
			Debug.LogError("Agent setting not found.");
			return;
		}
		surface.RemoveData();
		surface.BuildNavMesh();

		var unityNavMesh = NavMesh.CalculateTriangulation();
		SerializeUnityNavMesh(unityNavMesh, ref recast);

		surface.RemoveData();
		Profiler.EndSample();
	}

	public RecastGraph GetFirstActiveRecastGraph()
	{
		if (active == null || active.data == null)
		{
			return null;
		}

		if (active.data.recastGraph != null)
		{
			return active.data.recastGraph;
		}

		if (active.data.graphs != null)
		{
			foreach (var graph in active.data.graphs)
			{
				var isRecast = (graph.GetType() == typeof(RecastGraph) || graph.GetType().IsSubclassOf(typeof(RecastGraph)));
				if (isRecast)
				{
					return graph as RecastGraph;
				}
			}
		}

		return null;
	}

	NavMeshSurface GetNavMeshSurface()
	{
		var astar = FindObjectOfType<AstarPath>();
		var surface = astar.GetComponent<NavMeshSurface>();
		if (surface == null)
		{
			surface = astar.gameObject.AddComponent<NavMeshSurface>();
		}

		return surface;
	}

	bool ConfigNavMeshSurface(RecastGraph recast, ref NavMeshSurface surface)
	{
		#region set agent
		var agentTypeID = int.MinValue;
		var count = NavMesh.GetSettingsCount();
		for (var i = 0; i < count; i++)
		{
			var settings = NavMesh.GetSettingsByIndex(i);
			if (Mathf.Abs(settings.agentRadius - recast.characterRadius) > Mathf.Epsilon)
			{
				continue;
			}
			if (Mathf.Abs(settings.agentHeight - recast.walkableHeight) > Mathf.Epsilon)
			{
				continue;
			}
			if (Mathf.Abs(settings.agentClimb - recast.walkableClimb) > Mathf.Epsilon)
			{
				continue;
			}
			if (Mathf.Abs(settings.agentSlope - recast.maxSlope) > Mathf.Epsilon)
			{
				continue;
			}
			agentTypeID = settings.agentTypeID;
		}
		if (agentTypeID == int.MinValue)
		{
			return false;
		}
		surface.agentTypeID = agentTypeID;
		#endregion

		surface.collectObjects = CollectObjects.Volume;
		surface.size = recast.forcedBoundsSize;
		surface.center = recast.forcedBoundsCenter;
		surface.layerMask = recast.mask;
		surface.useGeometry = NavMeshCollectGeometry.RenderMeshes;
		if (recast.rasterizeColliders)
		{
			surface.useGeometry = NavMeshCollectGeometry.PhysicsColliders;
		}

		#region TODO: set area
		// RecastMeshObj?
		#endregion

		surface.overrideVoxelSize = true;
		surface.voxelSize = recast.cellSize;
		surface.overrideTileSize = true;
		surface.tileSize = recast.editorTileSize;

		return true;
	}

	void SerializeUnityNavMesh(NavMeshTriangulation unityNavMesh, ref RecastGraph recast)
	{
		if (active == null || active.data == null)
		{
			return;
		}

		var vertMap = ObjectPoolSimple<Dictionary<int, int>>.Claim();

		var totalVoxelWidth = (int)(recast.forcedBoundsSize.x / recast.cellSize + 0.5f);
		var totalVoxelDepth = (int)(recast.forcedBoundsSize.z / recast.cellSize + 0.5f);
		var tileSizeX = recast.editorTileSize;
		var tileSizeZ = recast.editorTileSize;
		var tileXCount = (totalVoxelWidth + tileSizeX - 1) / tileSizeX;
		var tileZCount = (totalVoxelDepth + tileSizeZ - 1) / tileSizeZ;
		var tileWorldSize = recast.TileWorldSizeX;
		var bucket = ArrayPool<List<int>>.Claim((tileXCount + 1) * (tileZCount + 1));
		for (int i = 0; i < unityNavMesh.vertices.Length; i++)
		{
			var v = unityNavMesh.vertices[i];
			var tileIndex = vertexOnTile(
				v, recast.forcedBoundsCenter, recast.forcedBoundsSize, tileWorldSize, tileXCount, tileZCount);
			tileIndex = 0;
			if (bucket[tileIndex] == null)
			{
				bucket[tileIndex] = ListPool<int>.Claim();
			}
			bucket[tileIndex].Add(i);
		}
		foreach (var b in bucket)
		{
			if (b == null)
			{
				continue;
			}
			for (int i = 0; i < b.Count; i++)
			{
				for (int j = 0; j < i; j++)
				{
					if (b[i] >= unityNavMesh.vertices.Length || b[j] >= unityNavMesh.vertices.Length)
					{
						continue;
					}
					if (Vector3.Distance(unityNavMesh.vertices[b[i]], unityNavMesh.vertices[b[j]]) < 1e-3)
					{
						vertMap[b[i]] = b[j];
						break;
					}
				}
			}
		}
		ArrayPool<List<int>>.Release(ref bucket, true);

		// only one tile
		recast.transform = recast.CalculateTransform();
		recast.tileXCount = 1;
		recast.tileZCount = 1;
		recast.tileSizeX = totalVoxelWidth + 1;
		recast.tileSizeZ = totalVoxelDepth + 1;
		recast.ResetTiles(recast.tileXCount * recast.tileZCount);
		TriangleMeshNode.SetNavmeshHolder((int)recast.graphIndex, recast);
		var graphUpdateLock = active.PausePathfinding();
		for (int z = 0; z < recast.tileZCount; z++)
		{
			for (int x = 0; x < recast.tileXCount; x++)
			{
				var tileOffset = recast.forcedBoundsCenter - recast.forcedBoundsSize * 0.5f + new Vector3(
					x * tileWorldSize,
					0,
					z * tileWorldSize
				);
				var trisClaim = ArrayPool<int>.Claim(unityNavMesh.indices.Length);
				var tris = Memory.ShrinkArray(trisClaim, unityNavMesh.indices.Length);
				ArrayPool<int>.Release(ref trisClaim, true);
				for (int i = 0; i < tris.Length; i++)
				{
					var tri = unityNavMesh.indices[i];
					if (vertMap.ContainsKey(tri))
					{
						tri = vertMap[tri];
					}
					tris[i] = tri;
				}
				var vertsClaim = ArrayPool<Int3>.Claim(unityNavMesh.vertices.Length);
				var verts = Memory.ShrinkArray(vertsClaim, unityNavMesh.vertices.Length);
				ArrayPool<Int3>.Release(ref vertsClaim, true);
				for (int i = 0; i < verts.Length; i++)
				{
					var vertInWorld = unityNavMesh.vertices[i];
					var vertInTile = vertInWorld - tileOffset;
					verts[i] = new Int3(vertInTile);
				}
				recast.ReplaceTile(x, z, 1, 1, verts, tris);
			}
		}
		graphUpdateLock.Release();

		ObjectPoolSimple<Dictionary<int, int>>.Release(ref vertMap);
	}

	int vertexOnTile(Vector3 v, Vector3 center, Vector3 size, float tileWorldSize, int tileXCount, int _tileZCount)
	{
		var min = center - size * 0.5f;
		var max = center + size * 0.5f;
		var offsetx = Mathf.FloorToInt(min.x / tileWorldSize);
		var offsetz = Mathf.FloorToInt(min.z / tileWorldSize);
		var x = Mathf.FloorToInt(v.x / tileWorldSize) - offsetx;
		var z = Mathf.FloorToInt(v.z / tileWorldSize) - offsetz;
		return x + z * tileXCount;
	}
}
