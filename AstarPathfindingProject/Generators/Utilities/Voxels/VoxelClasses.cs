
	/** VoxelMesh used for recast graphs.
	 * \astarpro
	 */
	public struct VoxelMesh {
		/** Vertices of the mesh */
		public Int3[] verts;

		/** Triangles of the mesh.
		 * Each element points to a vertex in the #verts array
		 */
		public int[] tris;

		/** Area index for each triangle */
		public int[] areas;

		/** Region index for each triangle */
		public int[] regs;
	}
