using UnityEditor;
using UnityEngine;
using UnityEngine.AI;

namespace Pathfinding
{
	public partial class AstarPathEditor : Editor
	{
		void MenuClear()
		{
			if (AstarPath.active == null) return;

			AstarPath.active.ClearUnity();
		}
		void MenuScanFromUnity()
		{
			if (AstarPath.active == null) return;

			AstarPath.active.ScanFromUnity();
		}
		void MenuScanUnity()
		{
			if (AstarPath.active == null) return;
			var recast = AstarPath.active.GetFirstActiveRecastGraph(); // multi recast graph not supported
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

			ResetNavMeshSurface(recast, ref surface);
			surface.BuildNavMesh();
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

		void ResetNavMeshSurface(RecastGraph recast, ref NavMeshSurface surface)
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
				var settings = NavMesh.CreateSettings();
				settings.agentRadius = recast.characterRadius;
				settings.agentHeight = recast.walkableHeight;
				settings.agentClimb = recast.walkableClimb;
				settings.agentSlope = recast.maxSlope;
				agentTypeID = settings.agentTypeID;
				Debug.LogWarning("Creating new agent setting");
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
			//surface.tileSize = Mathf.FloorToInt(recast.editorTileSize / surface.voxelSize);
			surface.tileSize = recast.editorTileSize;
		}
	}
}
