
			VoxelMesh pmesh;
			vox.BuildPolyMesh(cset, 3, out pmesh);

			VoxelMesh mesh;
			if (buildPolyMeshDetail)
			{
				var sampleDist = cellSize * 6.0f;
				var sampleMaxError = CellHeight * 1.0f;;
				vox.BuildPolyMeshDetail(sampleDist, sampleMaxError, pmesh, out mesh);
			}
			else
			{
				mesh = pmesh;
			}
