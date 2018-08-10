
			VoxelMesh pmesh;
			vox.BuildPolyMesh(cset, 3, out pmesh);

			VoxelMesh mesh;
			if (buildPolyMeshDetail)
			{
				var sampleDist = 6; // cellSize * 6.0f
				var sampleMaxError = 1; // CellHeight * 1.0f
				vox.BuildPolyMeshDetail(sampleDist, sampleMaxError, pmesh, out mesh);
			}
			else
			{
				mesh = pmesh;
			}
