...

			// Put all tiles in the queue
			for (int z = 0; z < tileZCount; z++)
			{
				for (int x = 0; x < tileXCount; x++)
				{
					if ((x + z) % 2 == 0)
						tileQueue.Enqueue(new Int2(x, z));
				}
			}
			for (int z = 0; z < tileZCount; z++)
			{
				for (int x = 0; x < tileXCount; x++)
				{
					if ((x + z) % 2 == 1)
					tileQueue.Enqueue(new Int2(x, z));
				}
			}

...

			VoxelMesh pmesh;
			vox.BuildPolyMesh(cset, 3, out pmesh);

			var tileBorder = new Voxelize.TileBorder();
			tileBorder.Init(vox.width, vox.depth, vox.borderSize);
			if ((x + z) % 2 == 1) FindTileBorder(x, z, ref tileBorder);

			VoxelMesh mesh;
			if (buildPolyMeshDetail)
			{
				vox.BuildPolyMeshDetail(sampleDist, sampleMaxError, pmesh, out mesh, ref tileBorder, detailMeshHeightQueryRoundMax);
			}
			else
			{
				mesh = pmesh;
			}

			AstarProfiler.StartProfile("Build Nodes");

			// Position the vertices correctly in graph space (all tiles are laid out on the xz plane with the (0,0) tile at the origin)
			for (int i = 0; i < mesh.verts.Length; i++) {
				mesh.verts[i] *= Int3.Precision;
			}
			vox.transformVoxel2Graph.Transform(mesh.verts);

			NavmeshTile tile = CreateTile(vox, mesh, x, z, threadIndex);
			tile.tileBorder = tileBorder;

			AstarProfiler.EndProfile("Build Nodes");

...

		public void FindTileBorder(int x, int z, ref Voxelize.TileBorder tileBorder)
		{
			var neighborOffset = new Int2[] { new Int2(0, -1), new Int2(-1, 0), new Int2(0, 1), new Int2(1, 0) };
			for (int i = 0; i < 4; i++)
			{
				if (
					x + neighborOffset[i].x < 0 || x + neighborOffset[i].x > tileXCount - 1 ||
					z + neighborOffset[i].y < 0 || z + neighborOffset[i].y > tileZCount - 1
				)
				{
					continue;
				}

				var t = GetTile(x + neighborOffset[i].x, z + neighborOffset[i].y);
				if (t == null)
				{
					continue;
				}

				tileBorder.CopyToSide((Voxelize.TileBorder.Side)i, t.tileBorder);
			}
		}
