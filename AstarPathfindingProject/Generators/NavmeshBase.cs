...
    public void ResetTiles(int length = 0)
		{
			if (tiles != null) ArrayPool<NavmeshTile>.Release(ref tiles, true);
			tiles = new NavmeshTile[length];
		}

		/** Clear all tiles within the rectangle with one corner at (x,z), width w and depth d */
		protected void ClearTiles (int x, int z, int w, int d) {
...
