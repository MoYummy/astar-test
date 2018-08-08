using UnityEngine;

namespace Pathfinding.Voxels
{
	using System;
	using System.Collections.Generic;
	using Pathfinding.Util;

	public partial class Voxelize
	{
		public struct VoxelHeightPatch {
			public int[] data;
			public int xmin;
			public int zmin; // ymin
			public int width;
			public int depth; // height,
			public float cs;
			public float ch;
		}

		ushort RC_UNSET_HEIGHT = 0xffff;
		int UNDEF = -1;

		float vdot2(Vector3 p, Vector3 q) {
			return p.x * q.x + p.z * q.z;
		}

		float vdistSq2(Vector3 p, Vector3 q)
		{
			var dx = p.x - q.x;
			var dz = p.z - q.z;
			return dx * dx + dz * dz;
		}

		float vdist2(Vector3 p, Vector3 q)
		{
			return Mathf.Sqrt(vdistSq2(p, q));
		}

		float vdist3(Vector3 p, Vector3 q)
		{
			var dx = p.x - q.x;
			var dy = p.y - q.y;
			var dz = p.z - q.z;
			return Mathf.Sqrt(dx * dx + dy * dy + dz * dz);
		}

		float vcross2(Vector3 p1, Vector3 p2, Vector3 p3)
		{
			var u1 = p2.x - p1.x;
			var v1 = p2.z - p1.z;
			var u2 = p3.x - p1.x;
			var v2 = p3.z - p1.z;
			return u1* v2 - v1* u2;
		}

		bool circumCircle(Vector4 p1, Vector4 p2, Vector4 p3, ref Vector3 c, ref float r)
		{
			var v1 = new Vector3(0f, 0f, 0f);
			var v2 = new Vector3(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
			var v3 = new Vector3(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

			var cp = vcross2(v1, v2, v3);
			if (Mathf.Abs(cp) > Mathf.Epsilon) {
				var v1Sq = vdot2(v1, v1);
				var v2Sq = vdot2(v2, v2);
				var v3Sq = vdot2(v3, v3);
				var _c = new Vector3(
					(v1Sq * (v2.z - v3.z) + v2Sq * (v3.z - v1.z) + v3Sq * (v1.z - v2.z)) / (2 * cp),
					 0f,
					(v1Sq * (v3.x - v2.x) + v2Sq * (v1.x - v3.x) + v3Sq * (v2.x - v1.x)) / (2 * cp)
				);
				r = vdist2(_c, v1);
				c = new Vector3(
					_c.x + p1.x,
					_c.y + p1.y,
					_c.z + p1.z
				);
				return true;
			}
			c = new Vector3(p1.x, p1.y, p1.z);
			r = 0f;
			return false;
		}

		float distPtTri(Vector3 p, Vector3 a, Vector3 b, Vector3 c) {
			var v0 = new Vector3(c.x - a.x, c.y - a.y, c.z - a.z);
			var v1 = new Vector3(b.x - a.x, b.y - a.y, b.z - a.z);
			var v2 = new Vector3(p.x - a.x, p.y - a.y, p.z - a.z);

			var dot00 = vdot2(v0, v0);
			var dot01 = vdot2(v0, v1);
			var dot02 = vdot2(v0, v2);
			var dot11 = vdot2(v1, v1);
			var dot12 = vdot2(v1, v2);

			// Compute barycentric coordinates
			var invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
			var u = (dot11 * dot02 - dot01 * dot12) * invDenom;
			var v = (dot00 * dot12 - dot01 * dot02) * invDenom;

			// If point lies inside the triangle, return interpolated y-coord.
			if (u >= -Mathf.Epsilon && v >= -Mathf.Epsilon && (u + v) < 1 + Mathf.Epsilon) {
				var y = a.y + v0.y * u + v1.y * v;
				return Mathf.Abs(y - p.y);
			}
			return Mathf.Infinity;
		}

		float distancePtSeg(Vector3 pt, Vector3 p, Vector3 q)
		{
			float pqx = q.x - p.x;
			float pqy = q.y - p.y;
			float pqz = q.z - p.z;
			float dx = pt.x - p.x;
			float dy = pt.y - p.y;
			float dz = pt.z - p.z;
			float d = pqx * pqx + pqy * pqy + pqz * pqz;
			float t = pqx * dx + pqy * dy + pqz * dz;
			if (d > 0)
				t /= d;
			if (t < 0)
				t = 0;
			else if (t > 1)
				t = 1;
			dx = p.x + t * pqx - pt.x;
			dy = p.y + t * pqy - pt.y;
			dz = p.z + t * pqz - pt.z;

			return dx * dx + dy * dy + dz * dz;
		}

		float distancePtSeg2d(Vector3 pt, Vector3 p, Vector3 q)
		{
			float pqx = q.x - p.x;
			float pqz = q.z - p.z;
			float dx = pt.x - p.x;
			float dz = pt.z - p.z;
			float d = pqx * pqx + pqz * pqz;
			float t = pqx * dx + pqz * dz;
			if (d > 0)
				t /= d;
			if (t < 0)
				t = 0;
			else if (t > 1)
				t = 1;

			dx = p.x + t * pqx - pt.x;
			dz = p.z + t * pqz - pt.z;

			return dx * dx + dz * dz;
		}

		float distToTriMesh(Vector3 p, List<Vector4> verts, int nvert, List<int> tris) {
			var dmin = Mathf.Infinity;
			var dminSet = false;
			for (int i = 0; i < tris.Count / 4; i++) {
				var d = distPtTri(p, 
					verts[tris[i * 4 + 0]], 
					verts[tris[i * 4 + 1]], 
					verts[tris[i * 4 + 2]]);
				if (d < dmin) {
					dmin = d;
					dminSet = true;
				}
			}
			if (!dminSet) return -1;
			return dmin;
		}

		float distToPoly(int nvert, float[] verts, Vector3 p)
		{
			var dmin = Mathf.Infinity;
			var i = 0;
			var j = nvert - 1;
			var c = false;
			for (;i < nvert ;j = i++) {
				var vi = new Vector3(verts[i * 3 + 0], verts[i * 3 + 1], verts[i * 3 + 2]);
				var vj = new Vector3(verts[j * 3 + 0], verts[j * 3 + 1], verts[j * 3 + 2]);
				if (((vi.z > p.z) != (vj.z > p.z)) && (p.x < (vj.x - vi.x) * (p.z - vi.z) / (vj.z - vi.z) + vi.x))
					c = !c;
				dmin = Mathf.Min(dmin, distancePtSeg2d(p, vj, vi));
			}
			return c ? -dmin : dmin;
		}

		int getHeight(float fx, float fy, float fz, float cs, float ics, float ch, VoxelHeightPatch hp)
		{
			var ix = (int)(fx * ics + 0.01f);
			var iz = (int)(fz * ics + 0.01f);
			ix = Mathf.Clamp(ix - hp.xmin, 0, hp.width - 1);
			iz = Mathf.Clamp(iz - hp.zmin, 0, hp.depth - 1);
			var h = hp.data[ix + iz * hp.width];

			var hpList = new List<int>() { ix + iz * hp.width };
			for (int j = 0; h == RC_UNSET_HEIGHT; j++) {
				var hx = hpList[j] % hp.width;
				var hz = (hpList[j] - hx) / hp.width;
				// Special case when data might be bad.
				// Find nearest neighbour pixel which has valid height.
				var off = new int[] { -1, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1 };
				var dmin = Mathf.Infinity;
				for (int i = 0; i < 8; i++)
				{
					var nx = hx + off[i * 2 + 0];
					var nz = hz + off[i * 2 + 1];
					if (nx < 0 || nz < 0 || nx >= hp.width || nz >= hp.depth) continue;
					var nh = hp.data[nx + nz * hp.width];
					if (nh == RC_UNSET_HEIGHT) {
						if (!hpList.Contains(nx + nz * hp.width)) hpList.Add(nx + nz * hp.width);
						continue;
					}
					var d = Mathf.Abs(nh * hp.ch - fy);
					if (d < dmin) {
						h = nh;
						dmin = d;
					}
				}
			}
			return h;
		}

		int findEdge(List<int> edges, int nedges, int s, int t)
		{
			for (int i = 0; i < nedges; i++)
			{
				if ((edges[i * 4 + 0] == s && edges[i * 4 + 1] == t) || (edges[i * 4 + 0] == t && edges[i * 4 + 1] == s))
					return i;
			}
			return -1;
		}

		int addEdge(List<int> edges, ref int nedges, int maxEdges, int s, int t, int l, int r)
		{
			if (nedges >= maxEdges) {
				return UNDEF;
			}

			// Add edge if not already in the triangulation.
			int e = findEdge(edges, nedges, s, t);
			if (e == -1) {
				edges[nedges * 4 + 0] = s;
				edges[nedges * 4 + 1] = t;
				edges[nedges * 4 + 2] = l;
				edges[nedges * 4 + 3] = r;
				nedges++;
				return nedges;
			} else {
				return UNDEF;
			}
		}


		void updateLeftFace(ref List<int> edges, int e, int s, int t, int f)
		{
			if (edges[e + 0] == s && edges[e + 1] == t && edges[e + 2] == UNDEF)
				edges[e + 2] = f;
			else if (edges[e + 1] == s && edges[e + 0] == t && edges[e + 3] == UNDEF)
				edges[e + 3] = f;
		}

		int overlapSegSeg2d(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
		{
			var a1 = vcross2(a, b, d);
			var a2 = vcross2(a, b, c);
			if (a1* a2 < 0.0f)
			{
				var a3 = vcross2(c, d, a);
				var a4 = a3 + a2 - a1;
				if (a3* a4 < 0.0f)
					return 1;
			}
			return 0;
		}

		bool overlapEdges(Vector4[] pts, List<int> edges, int nedges, int s1, int t1)
		{
			for (int i = 0; i < nedges; ++i) {
				var s0 = edges[i * 4 + 0];
				var t0 = edges[i * 4 + 1];
				// Same or connected edges do not overlap.
				if (s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1)
					continue;
				if (overlapSegSeg2d(pts[s0], pts[t0], pts[s1], pts[t1]) != 0)
					return true;
			}
			return false;
		}

		void completeFacet(Vector4[] pts, int npts, List<int> edges, ref int nedges, int maxEdges, ref int nfaces, int e)
		{
			var edgeId = e * 4;

			// Cache s and t.
			var s = -1;
			var t = -1;
			if (edges[edgeId + 2] == UNDEF) {
				s = edges[edgeId + 0];
				t = edges[edgeId + 1];
			} else if (edges[edgeId + 3] == UNDEF) {
				s = edges[edgeId + 1];
				t = edges[edgeId + 0];
			} else {
				// Edge already completed.
				return;
			}

			// Find best point on left of edge.
			int pt = npts;
			var c = new Vector3(0f, 0f, 0f);
			var r = -1f;
			for (int u = 0; u < npts; u++)
			{
				if (u == s || u == t) continue;
				if (vcross2(pts[s], pts[t], pts[u]) > Mathf.Epsilon)
				{
					if (r < 0)
					{
						// The circle is not updated yet, do it now.
						pt = u;
						circumCircle(pts[s], pts[t], pts[u], ref c, ref r);
						continue;
					}
					var d = vdist2(c, pts[u]);
					var tol = 0.001f;
					if (d > r * (1 + tol))
					{
						// Outside current circumcircle, skip.
						continue;
					}
					else if (d < r * (1 - tol))
					{
						// Inside safe circumcircle, update circle.
						pt = u;
						circumCircle(pts[s], pts[t], pts[u], ref c, ref r);
					}
					else {
						// Inside epsilon circum circle, do extra tests to make sure the edge is valid.
						// s-u and t-u cannot overlap with s-pt nor t-pt if they exists.
						if (overlapEdges(pts, edges, nedges, s, u))
							continue;
						if (overlapEdges(pts, edges, nedges, s, u))
							continue;
						// Edge is valid.
						pt = u;
						circumCircle(pts[s], pts[t], pts[u], ref c, ref r);
					}
				}
			}

			// Add new triangle or update edge info if s-t is on hull.
			if (pt < npts) {
				// Update face information of edge being completed.
				updateLeftFace(ref edges, e * 4, s, t, nfaces);

				// Add new edge or update face info of old edge.
				e = findEdge(edges, nedges, pt, s);
				if (e == UNDEF)
					addEdge(edges, ref nedges, maxEdges, pt, s, nfaces, UNDEF);
				else
					updateLeftFace(ref edges, e * 4, t, pt, nfaces);

				// Add new edge or update face info of old edge.
				e = findEdge(edges, nedges, t, pt);
				if (e == UNDEF)
					addEdge(edges, ref nedges, maxEdges, t, pt, nfaces, UNDEF);
				else
					updateLeftFace(ref edges, e * 4, t, pt, nfaces);
				nfaces++;
			} else {
				updateLeftFace(ref edges, e * 4, t, pt, nfaces);
			}
		}

		void delaunayHull(int npts, Vector4[] pts, int nhull, int[] hull, ref List<int> tris, ref List<int> edges)
		{
			var nfaces = 0;
			var nedges = 0;
			var maxEdges = npts * 10;
			var UNSET_TRI_VERT = -1;
			var HULL = -2;
			edges.Clear();
			for (int i = 0; i < maxEdges * 4; i++)
				edges.Add(UNDEF);

			for (int i = 0, j = nhull - 1; i < nhull; j = i++) {
				addEdge(edges, ref nedges, maxEdges, hull[j], hull[i], HULL, UNDEF);
			}

			var currentEdge = 0;
			while (currentEdge < nedges) {
				if (edges[currentEdge * 4 + 2] == UNDEF)
					completeFacet(pts, npts, edges, ref nedges, maxEdges, ref nfaces, currentEdge);
				if (edges[currentEdge * 4 + 3] == UNDEF)
					completeFacet(pts, npts, edges, ref nedges, maxEdges, ref nfaces, currentEdge);
				currentEdge++;
			}

			// Create tris
			tris.Clear();
			for (int i = 0; i < nfaces * 4; i++)
				tris.Add(UNSET_TRI_VERT);
			
			for (int i = 0; i < nedges; i++) {
				var e = new Vector3Int(
					edges[i * 4 + 0],
					edges[i * 4 + 1],
					edges[i * 4 + 2]
				);
				var e_w = edges[i * 4 + 3];
				if (e_w >= 0) {
					// Left face
					if (tris[e_w * 4 + 0] == UNSET_TRI_VERT) {
						tris[e_w * 4 + 0] = e.x;
						tris[e_w * 4 + 1] = e.y;
					} else if (tris[e_w * 4 + 0] == e.y) {
						tris[e_w * 4 + 2] = e.x;
					} else if (tris[e_w * 4 + 1] == e.x) {
						tris[e_w * 4 + 2] = e.y;
					}
				}
				if (e.z >= 0) {
					// Right
					if (tris[e.z * 4 + 0] == UNSET_TRI_VERT)
					{
						tris[e.z * 4 + 0] = e.y;
						tris[e.z * 4 + 1] = e.x;
					}
					else if (tris[e.z * 4 + 0] == e.x)
					{
						tris[e.z * 4 + 2] = e.y;
					}
					else if (tris[e.z * 4 + 1] == e.y)
					{
						tris[e.z * 4 + 2] = e.x;
					}
				}
			}

			for (int i = tris.Count - 4; i > 0 ; i -= 4) {
				if (tris[i + 0] == UNSET_TRI_VERT || tris[i + 1] == UNSET_TRI_VERT || tris[i + 2] == UNSET_TRI_VERT) {
					Debug.LogWarning("delaunayHull: Removing dangling face");
					tris.RemoveRange(i, 4);
				}
			}
		}

		float triMinExtent(Vector4[] verts)
		{
			Debug.Assert(verts.Length == 3);

			var d = distancePtSeg2d(verts[2], verts[0], verts[1]);
			var maxEdgeDist = Mathf.Max(0f, d);

			return Mathf.Sqrt(Mathf.Min(Mathf.Infinity, maxEdgeDist));
		}

		void triangulateHull(int nverts, Vector4[] verts, int nhull, int[] hull, ref List<int> tris)
		{
			var start = 0;
			var left = 1;
			var right = nhull - 1;

			// Start from an ear with shortest perimeter.
			// This tends to favor well formed triangles as starting point.
			var dmin = Mathf.Infinity;
			for (int i = 0, pi = nhull - 1; i < nhull; pi = i++)
			{
				var ni = (i + 1) % nhull;
				var pv = verts[hull[pi]];
				var cv = verts[hull[i]];
				var nv = verts[hull[ni]];
				var d = vdist2(pv, cv) + vdist2(cv, nv) + vdist2(nv, pv);
				if (d < dmin)
				{
					start = i;
					left = ni;
					right = pi;
					dmin = d;
				}
			}

			// Add first triangle
			tris.Add(hull[start]);
			tris.Add(hull[left]);
			tris.Add(hull[right]);
			tris.Add(0);

			// Triangulate the polygon by moving left or right,
			// depending on which triangle has shorter perimeter.
			// This heuristic was chose emprically, since it seems
			// handle tesselated straight edges well.
			while (((left + 1) % nhull) != right)
			{
				var nleft = (left + 1) % nhull;
				var nright = (right - 1 + nhull) % nhull;

				var cvleft = verts[hull[left]];
				var nvleft = verts[hull[nleft]];
				var cvright = verts[hull[right]];
				var nvright = verts[hull[nright]];
				var dleft = vdist2(cvleft, nvleft) + vdist2(nvleft, cvright);
				var dright = vdist2(cvright, nvright) + vdist2(cvleft, nvright);

				if (dleft < dright)
				{
					tris.Add(hull[left]);
					tris.Add(hull[nleft]);
					tris.Add(hull[right]);
					tris.Add(0);
					left = nleft;
				}
				else
				{
					tris.Add(hull[left]);
					tris.Add(hull[nright]);
					tris.Add(hull[right]);
					tris.Add(0);
					right = nright;
				}
			}

			Debug.Assert((tris.Count / 4) == nverts - 2);
		}

		float getJitterX(int i)
		{
			return (((i* 0x8da6b343) & 0xffff) / 65535.0f * 2.0f) - 1.0f;
		}

		float getJitterZ(int i)
		{
			return (((i * 0xd8163841) & 0xffff) / 65535.0f * 2.0f) - 1.0f;
		}

		void buildPolyDetail(
			float[] _in, int nin,
			float sampleDist, float sampleMaxError, VoxelArea chf, VoxelHeightPatch hp,
			ref List<Vector4> verts, ref int nverts,
			ref List<int> tris, ref List<int> edges, ref List<int> samples) {

			var MAX_VERTS = 127;
			var MAX_TRIS = 255; // Max tris for delaunay is 2n-2-k (n=num verts, k=num hull verts).
			var MAX_VERTS_PER_EDGE = 32;
			var vedge = ArrayPool<Vector3>.Claim(MAX_VERTS_PER_EDGE + 1);
			var hull = ArrayPool<int>.Claim(MAX_VERTS);
			var nhull = 0;

			nverts = 0;

			verts.Clear();
			tris.Clear();

			for (int i = 0; i < nin; i++) {
				verts.Add(new Vector4(_in[i * 3 + 0], _in[i * 3 + 1], _in[i * 3 + 2], 1f));
			}
			nverts = nin;

			edges.Clear();
			tris.Clear();

			var cs = cellSize;
			var ics = 1.0f / cellSize;
			var ch = cellHeight;

			// Calculate minimum extents of the polygon based on input data.
			var minExtent = triMinExtent(verts.ToArray());

			// Tessellate outlines.
			// This is done in separate pass in order to ensure
			// seamless height values across the ply boundaries.
			if (sampleDist > 0) {
				for (int i = 0, j = nin - 1; i < nin; j = i++) {
					var vj = new Vector3(_in[j * 3 + 0], _in[j * 3 + 1], _in[j * 3 + 2]);
					var vi = new Vector3(_in[i * 3 + 0], _in[i * 3 + 1], _in[i * 3 + 2]);
					var swapped = false;

					// Make sure the segments are always handled in same order
					// using lexological sort or else there will be seams.
					if (Mathf.Abs(vj.x - vi.x) < Mathf.Epsilon) {
						if (vj.z > vi.z) {
							MemorySwap(ref vj, ref vi);
							swapped = true;
						}
					} else {
						if (vj.x > vi.x) {
							MemorySwap(ref vj, ref vi);
							swapped = true;
						}
					}
					// Create samples along the edge.
					var dx = vi.x - vj.x;
					var dy = vi.y - vj.y;
					var dz = vi.z - vj.z;
					var d = Mathf.Sqrt(dx * dx + dz * dz);
					var nn = 1 + Mathf.FloorToInt(d / sampleDist);
					if (nn >= MAX_VERTS_PER_EDGE) nn = MAX_VERTS_PER_EDGE - 1;
					if (nverts + nn >= MAX_VERTS)
						nn = MAX_VERTS - 1 - nverts;

					for (int k = 0; k <= nn; k++) {
						var u = (k * 1f) / (nn * 1f);
						vedge[k] = new Vector3(vj.x + dx * u, vj.y + dy * u, vj.z + dz * u);
						var edgeY = ch * getHeight(
							vedge[k].x, vedge[k].y, vedge[k].z, cs, ics, ch, hp);
						vedge[k] = new Vector3(vedge[k].x, edgeY, vedge[k].z);
					}

					var idx = new List<int>();
					idx.Add(0);
					idx.Add(nn);

					for (int k = 0; k < idx.Count - 1;)
					{
						var a = idx[k];
						var b = idx[k + 1];
						var va = vedge[a];
						var vb = vedge[b];
						// Find maximum deviation along the segment.
						var maxd = 0f;
						var maxi = -1;
						for (int m = a + 1; m < b; m++)
						{
							var dev = distancePtSeg(vedge[m], va, vb);
							if (dev > maxd)
							{
								maxd = dev;
								maxi = m;
							}
						}

						// If the max deviation is larger than accepted error,
						// add new point, else continue to next segment.
						if (maxi != -1 && maxd > sampleMaxError * sampleMaxError)
						{
							idx.Insert(k + 1, maxi);
						}
						else
						{
							k++;
						}
					}

					hull[nhull] = j;
					nhull++;
					// Add new vertices.
					if (swapped) {
						for (int k = idx.Count - 2; k > 0; k--) {
							verts.Add(new Vector4(
								vedge[idx[k]].x,
								vedge[idx[k]].y,
								vedge[idx[k]].z,
								0f
							));
							hull[nhull] = nverts;
							nhull++;
							nverts++;
						}
					} else {
						for (int k = 1; k < idx.Count - 1; k++) {
							verts.Add(new Vector4(
								vedge[idx[k]].x,
								vedge[idx[k]].y,
								vedge[idx[k]].z,
								0f
							));
							hull[nhull] = nverts;
							nhull++;
							nverts++;
						}
					}
				}
			}

			// If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
			if (minExtent < sampleDist * 2) {
				triangulateHull(nverts, verts.ToArray(), nhull, hull, ref tris);
				return;
			}

			// Tessellate the base mesh.
			// We're using the triangulateHull instead of delaunayHull as it tends to
			// create a bit better triangulation for long thing triangles when there
			// are no internal points.
			triangulateHull(nverts, verts.ToArray(), nhull, hull, ref tris);

			if (tris.Count == 0) {
				// Could not triangulate the poly, make sure there is some valid data there.
				Debug.Log("buildPolyDetail: Could not triangulate polygon.");
				return;
			}

			if (sampleDist > 0) {
				// Create sample locations in a grid.
				var bmin = new Vector3(Mathf.Infinity, Mathf.Infinity, Mathf.Infinity);
				var bmax = new Vector3(-Mathf.Infinity, -Mathf.Infinity, -Mathf.Infinity);
				for (int i = 0; i < nin; i++) {
					bmin.x = Mathf.Min(bmin.x, _in[i * 3 + 0]);
					bmin.y = Mathf.Min(bmin.y, _in[i * 3 + 1]);
					bmin.z = Mathf.Min(bmin.z, _in[i * 3 + 2]);
					bmax.x = Mathf.Max(bmax.x, _in[i * 3 + 0]);
					bmax.y = Mathf.Max(bmax.y, _in[i * 3 + 1]);
					bmax.z = Mathf.Max(bmax.z, _in[i * 3 + 2]);
				}

				var x0 = Mathf.FloorToInt(bmin.x / sampleDist);
				var x1 = Mathf.FloorToInt(bmax.x / sampleDist);
				var z0 = Mathf.FloorToInt(bmin.z / sampleDist);
				var z1 = Mathf.FloorToInt(bmax.z / sampleDist);
				samples.Clear();
				for (int z = z0; z < z1; z++) {
					for (int x = x0; x < x1; x++) {
						var pt = new Vector3(x * sampleDist, (bmax.y + bmin.y) * 0.5f, z * sampleDist);
						// Make sure the samples are not too close to the edges.
						if (distToPoly(nin, _in, pt) > -sampleDist / 2) continue;
						samples.Add(x);
						samples.Add(getHeight(pt.x, pt.y, pt.z, cs, ics, ch, hp));
						samples.Add(z);
						samples.Add(0); // Not added
					}
				}

				// Add the samples starting from the one that has the most
				// error. The procedure stops when all samples are added
				// or when the max error is within treshold.
				var nsamples = samples.Count / 4;
				for (int iter = 0; iter < nsamples; iter++)
				{
					if (nverts >= MAX_VERTS)
						break;

					var bestpt = new Vector3(0f, 0f, 0f);
					var bestd = 0f;
					var besti = -1;
					for (int i = 0; i < nsamples; i++) {
						if (samples[i * 4 + 3] != 0) continue; // skip added.
						var s = new Vector3Int(
							samples[i * 4 + 0], samples[i * 4 + 1], samples[i * 4 + 2]
						);
						var pt = new Vector3(
							s.x * sampleDist + getJitterX(i) * cs * 0.1f,
							s.y * ch,
							s.z * sampleDist + getJitterZ(i) * cs * 0.1f
						);
						// The sample location is jittered to get rid of some bad triangulations
						// which are cause by symmetrical data from the grid structure.
						var d = distToTriMesh(pt, verts, nverts, tris);
						if (d < 0) continue;
						if (d > bestd) {
							bestd = d;
							besti = i;
							bestpt = pt;
						}
					}
					// If the max error is within accepted threshold, stop tesselating.
					if (bestd < sampleMaxError || besti == -1)
						break;
					// Mark sample as added.
					samples[besti * 4 + 3] = 1;
					// Add the new sample point.
					verts.Add(bestpt);

					// Create new triangulation.
					// TODO: Incremental add instead of full rebuild.
					edges.Clear();
					tris.Clear();
					delaunayHull(nverts, verts.ToArray(), nhull, hull, ref tris, ref edges);
					//triangulateHull(nverts, verts.ToArray(), nhull, hull, ref tris);
				}
			}

			if (tris.Count > MAX_TRIS * 4) {
				Debug.LogError("Shrinking triangle count");
				tris.RemoveRange(MAX_TRIS * 4, tris.Count - MAX_TRIS * 4);
			}
		}

		void getHeightDataSeedsFromVertices(
			VoxelArea chf, int[] poly, Int3[] verts, int bs,
			ref VoxelHeightPatch hp, ref List<Vector3Int> stack, int regId, int areaId) {

			stack.Clear();

			var offset = new int[] {
				0, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0 };

			// Use poly vertices as seed points for the flood fill.
			for (int j = 0; j < poly.Length; j++) {
				var cx = 0;
				var cz = 0;
				var ci = -1;
				var dmin = Mathf.Infinity;

				for (int k = 0; k < 9; k++) {
					var av = verts[poly[j]];
					var ax = av.x + offset[k * 2 + 0];
					var ay = av.y;
					var az = av.z + offset[k * 2 + 1];
					if (
						ax < hp.xmin || ax >= hp.xmin + hp.width ||
						az < hp.zmin || az >= hp.zmin + hp.depth)
						continue;

					var c = chf.compactCells[(ax + bs) + (az + bs) * chf.width];
					for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; i++) {
						var s = chf.compactSpans[i];
						var d = Mathf.Abs(ay - (int)s.y);
						if (d < dmin) {
							cx = ax;
							cz = az;
							ci = i;
							dmin = d;
						}
					}
				}
				if (ci != -1) {
					stack.Add(new Vector3Int(cx, cz, ci));
				}
			}

			var pcx = 0;
			var pcz = 0;
			for (int j = 0; j < poly.Length; j++) {
				pcx += verts[poly[j]].x;
				pcz += verts[poly[j]].z;
			}
			pcx /= poly.Length;
			pcz /= poly.Length;

			for (int i = 0; i < stack.Count; i++) {
				var cx = stack[i].x;
				var cz = stack[i].y;
				var idx = cx - hp.xmin + (cz - hp.zmin) * hp.width;
				hp.data[idx] = 1;
			}

			while (stack.Count > 0) {
				var ci = stack[stack.Count - 1].z;
				var cz = stack[stack.Count - 1].y;
				var cx = stack[stack.Count - 1].x;
				stack.RemoveAt(stack.Count - 1);

				// Check if close to center of the polygon.
				if (Mathf.Abs(cx-pcx) <= 1 && Mathf.Abs(cz-pcz) <= 1) {
					stack.Clear();
					stack.Add(new Vector3Int(cx, cz, ci));
					break;
				}

				var cs = chf.compactSpans[ci];

				for (int dir = 0; dir < 4; dir++) {
					if (cs.GetConnection(dir) == NotConnected) continue;

					var ax = cx + chf.DirectionX[dir];
					var az = cz + chf.DirectionZ[dir] / chf.width;

					if (
						ax < hp.xmin || ax >= (hp.xmin + hp.width) ||
						az < hp.zmin || az >= (hp.zmin + hp.depth)) continue;

					if (hp.data[ax - hp.xmin + (az - hp.zmin) * hp.width] != 0)
						continue;

					var ai = (int)chf.compactCells[(ax + bs) + (az + bs) * chf.width].index + cs.GetConnection(dir);

					var idx = ax - hp.xmin + (az - hp.zmin) * hp.width;
					hp.data[idx] = 1;

					stack.Add(new Vector3Int(ax, az, ai));
				}
			}

			for (int i = 0; i < stack.Count; i++) {
				var cx = stack[i].x;
				var cz = stack[i].y;
				var ci = stack[i].z;
				var idx = cx - hp.xmin + (cz - hp.zmin) * hp.width;
				var cs = chf.compactSpans[ci];
				hp.data[idx] = cs.y;

				stack[i] = new Vector3Int(stack[i].x + bs, stack[i].y + bs, stack[i].z);
			}
		}

		void getHeightData(VoxelArea chf, int[] poly, Int3[] verts, int bs, ref VoxelHeightPatch hp, ref List<Vector3Int> stack, int regId, int areaId)
		{
			stack.Clear();
			for (int i = 0; i < hp.width * hp.depth; i++) hp.data[i] = RC_UNSET_HEIGHT;

			var empty = true;
			// Copy the height from the same region, and mark region borders
			// as seed points to fill the rest.
			for (int hz = 0; hz < hp.depth; hz++)
			{
				var z = hp.zmin + hz/* + bs*/;
				for (int hx = 0; hx < hp.width; hx++)
				{
					var x = hp.xmin + hx/* + bs*/;
					var c = chf.compactCells[x + z * chf.width];
					for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; i++)
					{
						var s = chf.compactSpans[i];

						if (s.reg == regId)
						{
							//Debug.Assert(chf.areaTypes[i] == areaId);
							// Store height
							hp.data[hx + hz * hp.width] = s.y;
							empty = false;

							// If any of the neighbours is not in same region,
							// add the current location as flood fill start
							var border = false;
							for (int dir = 0; dir < 4; dir++)
							{
								if (s.GetConnection(dir) != NotConnected)
								{
									var ax = x + chf.DirectionX[dir];
									var az = z + chf.DirectionZ[dir] / chf.width;
									var ai = (int)chf.compactCells[ax + az * chf.width].index + s.GetConnection(dir);
									var _as = chf.compactSpans[ai];
									if (_as.reg != regId)
									{
										border = true;
										break;
									}
								}
							}
							if (border)
							{
								stack.Add(new Vector3Int(x, z, i));
							}
						}
					}
				}
			}
			if (empty)
			{
				getHeightDataSeedsFromVertices(chf, poly, verts, bs, ref hp, ref stack, regId, areaId);
			}

			var RETRACT_SIZE = 256;
			var head = 0;

			while (head * 3 > stack.Count)
			{
				var cx = stack[head].x;
				var cz = stack[head].y;
				var ci = stack[head].z;
				head++;
				if (head > RETRACT_SIZE)
				{
					head = 0;
					if (stack.Count > RETRACT_SIZE * 3)
					{
						Debug.LogError("Add more code");
					}
				}

				var cs = chf.compactSpans[ci];
				for (int dir = 0; dir < 4; dir++)
				{
					if (cs.GetConnection(dir) == NotConnected) continue;

					var ax = cx + chf.DirectionX[dir];
					var az = cz + chf.DirectionZ[dir] / chf.width;
					var hx = ax - hp.xmin/* - bs*/;
					var hz = az - hp.zmin/* - bs*/;

					if (hx < 0 || hx >= hp.width || hz < 0 || hz >= hp.depth) continue;

					var ai = (int)chf.compactCells[ax + az * chf.width].index + cs.GetConnection(dir);
					var _as = chf.compactSpans[ai];

					hp.data[hx + hz * hp.width] = _as.y;

					stack.Add(new Vector3Int(ax, az, ai));
				}
			}
		}

		public void BuildPolyMeshDetail(
			float sampleDist, float sampleMaxError,
			VoxelMesh mesh, out VoxelMesh dmesh)
		{

			var nvp = 3; // all original polygons are triangles
			var cs = cellSize;
			var ch = cellHeight;
			var bs = borderSize;

			var edges = ListPool<int>.Claim(64);
			var tris = ListPool<int>.Claim(256);
			var stack = ListPool<Vector3Int>.Claim(512);
			var samples = ListPool<int>.Claim(512);
			var verts = ListPool<Vector4>.Claim(256); // a flag for original vertex
			var hp = new VoxelHeightPatch();
			var nPolyVerts = 0;
			var maxhw = 0;
			var maxhd = 0;

			var meshNpolys = mesh.tris.Length / nvp; // triangle count
			var bounds = ArrayPool<int>.Claim(meshNpolys * 4);
			var poly = ArrayPool<float>.Claim(nvp * 3);

			// Find max size for a polygon area.
			for (int i = 0; i < meshNpolys; i++)
			{
				var xmin = voxelArea.width;
				var xmax = 0;
				var zmin = voxelArea.depth; // ymin
				var zmax = 0; // ymax
				for (int j = 0; j < nvp; j++)
				{
					var polyVertId = mesh.tris[i * nvp + j];
					xmin = Mathf.Min(xmin, mesh.verts[polyVertId].x);
					xmax = Mathf.Max(xmax, mesh.verts[polyVertId].x);
					zmin = Mathf.Min(zmin, mesh.verts[polyVertId].z);
					zmax = Mathf.Max(zmax, mesh.verts[polyVertId].z);
					nPolyVerts++;
				}

				xmin = Mathf.Max(0, xmin - 1);
				xmax = Mathf.Min(voxelArea.width, xmax + 1);
				zmin = Mathf.Max(0, zmin - 1);
				zmax = Mathf.Min(voxelArea.depth, zmax + 1);

				bounds[i * 4 + 0] = xmin;
				bounds[i * 4 + 1] = xmax;
				bounds[i * 4 + 2] = zmin;
				bounds[i * 4 + 3] = zmax;

				if (xmin >= xmax || zmin >= zmax) continue;
				maxhw = Mathf.Max(maxhw, xmax - xmin);
				maxhd = Mathf.Max(maxhd, zmax - zmin);
			}

			hp.data = ArrayPool<int>.Claim(maxhw * maxhd);
			for (int i = 0; i < maxhw * maxhd; i++) hp.data[i] = RC_UNSET_HEIGHT;
			hp.cs = cs;
			hp.ch = ch;

			var dmeshVerts = new List<Int3>();
			var dmeshTris = new List<int>();
			var dmeshAreas = new List<int>();
			var dmeshRegs = new List<int>();

			for (int i = 0; i < meshNpolys; i++)
			{
				// Store polygon vertices for processing.
				var npoly = 0;
				for (int j = 0; j < nvp; j++)
				{
					var polyVertId = mesh.tris[i * nvp + j];
					poly[j * 3 + 0] = mesh.verts[polyVertId].x * cs;
					poly[j * 3 + 1] = mesh.verts[polyVertId].y * ch;
					poly[j * 3 + 2] = mesh.verts[polyVertId].z * cs;
					npoly++;
				}

				// Get the height data from the area of the polygon.
				hp.xmin = bounds[i * 4 + 0];
				hp.zmin = bounds[i * 4 + 2];
				hp.width = bounds[i * 4 + 1] - bounds[i * 4 + 0];
				hp.depth = bounds[i * 4 + 3] - bounds[i * 4 + 2];
				getHeightData(voxelArea, mesh.tris, mesh.verts, bs, ref hp, ref stack, mesh.regs[i], mesh.areas[i]);
				//verifyHeightData(mesh, i, hp);

				var nverts = 0;
				buildPolyDetail(
					poly, npoly, sampleDist, sampleMaxError, voxelArea, hp,
					ref verts, ref nverts, ref tris, ref edges, ref samples);

				var startIndex = dmeshVerts.Count;
				for (int vi = 0; vi < verts.Count; vi++)
				{
					var dvx = (int)(verts[vi].x / cs);
					var dvy = (int)(verts[vi].y / ch);
					var dvz = (int)(verts[vi].z / cs);
					var dv = new Int3(dvx, dvy, dvz);
					var isOrig = verts[vi].w > 0.5f;
					if (isOrig)
					{
						// find nearest in original mesh
						var vDistMin = new Int3();
						var distMin = Mathf.Infinity;
						foreach (var ov in mesh.verts)
						{
							var distSq = vdist3(
								new Vector3(ov.x, ov.y, ov.z),
								new Vector3(dv.x, dv.y, dv.z));
							if (distSq < distMin)
							{
								vDistMin = ov;
								distMin = distSq;
							}
						}
						vDistMin.y = dvy;
						dmeshVerts.Add(vDistMin);
					}
					else
					{
						dmeshVerts.Add(dv);
					}
				}
				for (int ti = 0; ti < tris.Count; ti += 4)
				{
					dmeshTris.Add(startIndex + tris[ti + 0]);
					dmeshTris.Add(startIndex + tris[ti + 1]);
					dmeshTris.Add(startIndex + tris[ti + 2]);
					dmeshAreas.Add(mesh.areas[i]);
					dmeshRegs.Add(mesh.regs[i]);
				}
			}

			RemoveDuplicateVertices(ref dmeshVerts, ref dmeshTris);

			dmesh = new VoxelMesh
			{
				verts = dmeshVerts.ToArray(),
				tris = dmeshTris.ToArray(),
				areas = dmeshAreas.ToArray(),
				regs = dmeshRegs.ToArray()
			};
			//PrintMesh(dmesh, "dmesh");
		}

		void MemorySwap<T>(ref T a, ref T b)
		{
			T tmp = a;

			a = b;
			b = tmp;
		}

		void verifyHeightData(VoxelMesh mesh, int i, VoxelHeightPatch hp)
		{
			var verts = new Int3[] {
				mesh.verts[mesh.tris[i * 3 + 0]],
				mesh.verts[mesh.tris[i * 3 + 1]],
				mesh.verts[mesh.tris[i * 3 + 2]]
			};
			var error = "";
			foreach (var v in verts)
			{
				var hx = v.x - hp.xmin;
				var hz = v.z - hp.zmin;
				var hy = hp.data[hx + hz * hp.width];
				if (Mathf.Abs(v.y - hy) > hy * 0.05f && Mathf.Abs(v.y - hy) > 2)
				{
					error += v + "=>(" + hx + "," + hz + "),hy:" + hy + "\n";
				}
			}
			if (!string.IsNullOrEmpty(error)) {
				var hpData = "";
				for (int hx = 0; hx < hp.width; hx++)
					for (int hz = 0; hz < hp.depth; hz++)
						hpData += hx + "," + hz + ":" + hp.data[hx + hz * hp.width] + "\n";
				error = hp.width + "," + hp.depth + "\n" + error + hpData;
				Debug.LogError(error);
			}
		}

		public void RemoveDuplicateVertices(ref List<Int3> verts, ref List<int> tris)
		{
			var vertMap = new Dictionary<Int3, int>();
			for (int i = verts.Count - 1; i > 0; i--)
			{
				vertMap[verts[i]] = i;
			}

			var removeVertMap = new Dictionary<int, int>();
			for (int i = 0; i < verts.Count; i++)
			{
				if (vertMap.ContainsKey(verts[i])) {
					var j = vertMap[verts[i]];
					if (j != i)
					{
						removeVertMap[i] = j;
					}
				}
			}

			// removeId is hole, find moveId from tail to fill
			var moveVertMap = new Dictionary<int, int>();
			var removeId = 0;
			var moveId = verts.Count - 1;
			for (int i = 0; i < removeVertMap.Keys.Count; i++)
			{
				while (!removeVertMap.ContainsKey(removeId))
				{
					removeId++;
				}
				while (removeVertMap.ContainsKey(moveId))
				{
					moveId--;
				}
				if (moveId <= removeId) break;

				moveVertMap[moveId] = removeId;
				verts[removeId] = verts[moveId];
				removeId++;
				moveId--;
			}

			verts.RemoveRange(verts.Count - removeVertMap.Count, removeVertMap.Count);

			for (int i = 0; i < tris.Count; i++)
			{
				if (removeVertMap.ContainsKey(tris[i]))
				{
					tris[i] = removeVertMap[tris[i]];
				}
				if (moveVertMap.ContainsKey(tris[i]))
				{
					tris[i] = moveVertMap[tris[i]];
				}
			}
		}

		void PrintMesh(VoxelMesh vm, string title)
		{
			var area = "";
			foreach (var a in vm.areas)
			{
				area += a + ",";
			}
			var reg = "";
			foreach (var r in vm.regs)
			{
				reg += r + ",";
                                			}
			Debug.Log(
				title + ":v:" + vm.verts.Length + ",tri:" + vm.tris.Length + "," +
				"area(" + vm.areas.Length + "):" + area +
				"reg(" + vm.regs.Length + "):" + reg
			);
		}
	}
}
