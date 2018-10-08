using UnityEngine;

namespace Pathfinding.Voxels
{
	using System.Collections.Generic;
	using Pathfinding.Util;

	public partial class Voxelize
	{
		public struct VoxelHeightPatch {
			public int[] data;
			public int xmin;
			public int zmin; // ymin
			public int width;
			public int depth; // height
			public int querymax;
		}

		public class TileBorder: IAstarPooledObject {
			public enum Side {
				ZMIN = 0,
				XMIN = 1,
				ZMAX = 2,
				XMAX = 3,
				UNKNOWN = 4
			}

			public struct Contour {
				public List<Vector3Int> verts;
			}

			Dictionary<Side, List<Contour>> sides;
			int w;
			int d;
			int bs;

			public void Init(int _w, int _d, int _bs)
			{
				sides = ObjectPoolSimple<Dictionary<Side, List<Contour>>>.Claim();
				sides[Side.ZMIN] = ListPool<Contour>.Claim();
				sides[Side.XMIN] = ListPool<Contour>.Claim();
				sides[Side.ZMAX] = ListPool<Contour>.Claim();
				sides[Side.XMAX] = ListPool<Contour>.Claim();
				w = _w;
				d = _d;
				bs = _bs;
			}

			public Side IsOnSide(Vector3Int vj, Vector3Int vi)
			{
				if (vj.z <= bs && vi.z <= bs) return Side.ZMIN;
				if (vj.x <= bs && vi.x <= bs) return Side.XMIN;
				if (vj.z >= d - bs && vi.z >= d - bs) return Side.ZMAX;
				if (vj.x >= w - bs && vi.x >= w - bs) return Side.XMAX;
				return Side.UNKNOWN;
			}

			public System.Collections.ObjectModel.ReadOnlyCollection<Contour> GetSide(Side side)
			{
				if (side == Side.UNKNOWN)
					return null;
				return sides[side].AsReadOnly();
			}

			public void AddContour(List<Vector3Int> verts, Side side = Side.UNKNOWN)
			{
				if (side == Side.UNKNOWN) return;

				#region update existing
				var headOverlap = -1;
				var headToPrepend = -1;
				var tailOverlap = -1;
				var tailToAppend = -1;
				for (int i = 0; i < sides[side].Count; i++)
				{
					var sc = sides[side][i];
					if (sc.verts[0] == verts[0])
					{
						headOverlap = i;
					}
					else if (sc.verts[sc.verts.Count - 1] == verts[verts.Count - 1])
					{
						tailOverlap = i;
					}
					else if (sc.verts[0] == verts[verts.Count - 1])
					{
						headToPrepend = i;
					}
					else if (sc.verts[sc.verts.Count - 1] == verts[0])
					{
						tailToAppend = i;
					}
					if ((headOverlap != -1) && (tailOverlap != -1))
					{
						// overlap
						return;
					}
				}
				if (tailToAppend != -1 && headToPrepend != -1)
				{
					for (int i = 1; i < verts.Count; i++)
					{
						sides[side][tailToAppend].verts.Add(verts[i]);
					}
					var vs = sides[side][headToPrepend].verts;
					for (int i = 1; i < vs.Count; i++)
					{
						sides[side][tailToAppend].verts.Add(vs[i]);
					}
					sides[side].RemoveAt(headToPrepend);
					return;
				}
				if (tailToAppend != -1)
				{
					for (int i = 1; i < verts.Count; i++)
					{
						sides[side][tailToAppend].verts.Add(verts[i]);
					}
					return;
				}
				if (headToPrepend != -1)
				{
					for (int i = 0; i < verts.Count - 1; i++)
					{
						sides[side][headToPrepend].verts.Insert(i, verts[i]);
					}
					return;
				}
				if (headOverlap != -1)
				{
					var vs = sides[side][headOverlap].verts;
					if (side == Side.XMIN || side == Side.XMAX)
					{
						if (vs[vs.Count - 1].z >= verts[verts.Count - 1].z)
						{
							return;
						}
					}
					else if (side == Side.ZMIN || side == Side.ZMAX)
					{
						if (vs[vs.Count - 1].x >= verts[verts.Count - 1].x)
						{
							return;
						}
					}
				}
				if (tailOverlap != -1)
				{
					var vs = sides[side][tailOverlap].verts;
					if (side == Side.XMIN || side == Side.XMAX)
					{
						if (vs[0].z <= verts[0].z)
						{
							return;
						}
					}
					else if (side == Side.ZMIN || side == Side.ZMAX)
					{
						if (vs[0].x <= verts[0].x)
						{
							return;
						}
					}
				}
				#endregion

				var c = new Contour
				{
					verts = ListPool<Vector3Int>.Claim()
				};
				c.verts.Clear();
				foreach (var v in verts)
				{
					c.verts.Add(v);
				}
				sides[side].Add(c);
			}

			public void CopyToSide(Side side, TileBorder other)
			{
				var sideOther = (Side)((((int)side) + 2) % 4);
				var ocs = other.GetSide(sideOther);
				if (ocs.Count == 0) return;
				foreach (var oc in ocs)
				{
					var c = new Contour
					{
						verts = ListPool<Vector3Int>.Claim()
					};
					c.verts.Clear();
					foreach ( var v in oc.verts)
					{
						var vx = v.x;
						var vz = v.z;
						switch (side)
						{
							case Side.ZMIN: vz = bs; break;
							case Side.XMIN: vx = bs; break;
							case Side.ZMAX: vz = d - bs; break;
							case Side.XMAX: vx = w - bs; break;
						}
						c.verts.Add(new Vector3Int(vx, v.y, vz));
					}
					sides[side].Add(c);
				}
			}

			public bool HasData()
			{
				foreach (var side in sides)
				{
					if (side.Value.Count > 0)
					{
						return true;
					}
				}
				return false;
			}

			public int FixByContour(ref Vector3Int v, Side side, IList<Contour> contours)
			{
				if (contours == null)
				{
					contours = sides[side];
				}
				var besti = -1;
				var bestd = int.MaxValue;
				var besty = int.MinValue;
				for (int i = 0; i < contours.Count; i++)
				{
					var vs = contours[i].verts;
					if (side == Side.XMIN || side == Side.XMAX)
					{
						if (vs[0].z <= v.z && v.z <= vs[vs.Count - 1].z)
						{
							for (int j = 0; j < vs.Count - 1; j++)
							{
								if (vs[j].z <= v.z && v.z <= vs[j + 1].z) 
								{
									var vy = vs[j].y;
									if (vs[j].y != vs[j + 1].y)
										vy = Mathf.RoundToInt(1f * (vs[j + 1].y - vs[j].y) / (vs[j + 1].z - vs[j].z) * (v.z - vs[j].z) + vs[j].y);
									var dy = Mathf.Abs(vy - v.y);
									if (dy < bestd)
									{
										bestd = dy;
										besti = i;
										besty = vy;
									}
									break;
								}
							}
						}
					}
					else if (side == Side.ZMIN || side == Side.ZMAX)
					{
						if (vs[0].x <= v.x && v.x <= vs[vs.Count - 1].x)
						{
							for (int j = 0; j < vs.Count - 1; j++)
							{
								if (vs[j].x <= v.x && v.x <= vs[j + 1].x)
								{
									var vy = vs[j].y;
									if (vs[j].y != vs[j + 1].y)
										vy = Mathf.RoundToInt(1f * (vs[j + 1].y - vs[j].y) / (vs[j + 1].x - vs[j].x) * (v.x - vs[j].x) + vs[j].y);
									var dy = Mathf.Abs(vy - v.y);
									if (dy < bestd)
									{
										bestd = dy;
										besti = i;
										besty = vy;
									}
									break;
								}
							}
						}
					}
				}
				if (besti != -1)
				{
					v.y = besty;
				}
				return besti;
			}

			public void Print(string print = "")
			{
				var valid = false;
				var s = new Side[] { Side.ZMIN, Side.XMIN, Side.ZMAX, Side.XMAX };
				foreach (var side in s)
				{
					print += side.ToString() + ":\n";
					foreach (var c in sides[side])
					{
						foreach (var v in c.verts)
						{
							print += v + "=>";
							valid = true;
						}
						print += ";\n";
					}
					print += ";\n\n";
				}
				if (valid) Debug.Log(print);
			}

			void IAstarPooledObject.OnEnterPool()
			{
				Debug.LogWarning("OnEnterPool");
				foreach (var side in sides)
				{
					foreach (var contour in side.Value)
					{
						contour.verts.Clear();
						ListPool<Vector3Int>.Release(contour.verts);
					}
					side.Value.Clear();
					ListPool<Contour>.Release(side.Value);
				}
				sides.Clear();
				ObjectPoolSimple<Dictionary<Side, List<Contour>>>.Release(ref sides);
			}
		}

		enum HULL {
			VALID = 0,
			UNCOVERED = 1,
			OVERLAPPED = 2,
			MISSING_TRIANGLE = 3,
			DUPLICATE_TRIANGLE = 4,
			MISSING_EDGE = 5,
			DUPLICATE_EDGE = 6
		}

		enum SAMPLE {
			NOT_ADDED = 0,
			ADDED = 1
		}

		ushort RC_UNSET_HEIGHT = 0xffff;
		int UNDEF = -1;

		int vdot2(Vector3Int p, Vector3Int q) {
			return p.x * q.x + p.z * q.z;
		}

		float vdistSq2(Vector3Int p, Vector3Int q)
		{
			var dx = p.x - q.x;
			var dz = p.z - q.z;
			return dx * dx + dz * dz;
		}

		float vdist2(Vector3Int p, Vector3Int q)
		{
			return Mathf.Sqrt(vdistSq2(p, q));
		}

		int vdist3Sqr(Int3 p, Int3 q)
		{
			var dx = p.x - q.x;
			var dy = p.y - q.y;
			var dz = p.z - q.z;
			return dx * dx + dy * dy + dz * dz;
		}

		int vcross2(Vector3Int p1, Vector3Int p2, Vector3Int p3)
		{
			var u1 = p2.x - p1.x;
			var v1 = p2.z - p1.z;
			var u2 = p3.x - p1.x;
			var v2 = p3.z - p1.z;
			return u1* v2 - v1* u2;
		}

		float distancePtSeg(Vector3 pt, Vector3 p, Vector3 q)
		{
			var pqx = q.x - p.x;
			var pqy = q.y - p.y;
			var pqz = q.z - p.z;
			var dx = pt.x - p.x;
			var dy = pt.y - p.y;
			var dz = pt.z - p.z;
			var d = pqx * pqx + pqy * pqy + pqz * pqz;
			var t = pqx * dx + pqy * dy + pqz * dz;
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
			var pqx = q.x - p.x;
			var pqz = q.z - p.z;
			var dx = pt.x - p.x;
			var dz = pt.z - p.z;
			var d = pqx * pqx + pqz * pqz;
			var t = pqx * dx + pqz * dz;
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

		int getHeight(int fx, int fy, int fz, VoxelHeightPatch hp)
		{
			var ix = fx;
			var iz = fz;
			ix = Mathf.Clamp(ix - hp.xmin, 0, hp.width - 1);
			iz = Mathf.Clamp(iz - hp.zmin, 0, hp.depth - 1);
			var h = hp.data[ix + iz * hp.width];
			if (h != RC_UNSET_HEIGHT) return h;

			var hpList = ListPool<int>.Claim();
			hpList.Add(ix + iz * hp.width);
			var off = new int[] { -1, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1 };
			for (int j = 0; j < hp.querymax; j++) {
				var hx = hpList[j] % hp.width;
				var hz = (hpList[j] - hx) / hp.width;
				// Special case when data might be bad.
				// Find nearest neighbour pixel which has valid height.
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
					var d = Mathf.Abs(nh - fy);
					if (d < dmin) {
						h = nh;
						dmin = d;
					}
				}
				hp.data[ix + iz * hp.width] = h;
				if (h != RC_UNSET_HEIGHT) break;
			}
			if (h == RC_UNSET_HEIGHT) Debug.LogError("Height not found after " + hp.querymax + " trials");
			ListPool<int>.Release(hpList);
			ArrayPool<int>.Release(ref off, true);
			return h;
		}
 
		float triMinExtent(Vector3Int[] verts)
		{
			Debug.Assert(verts.Length == 3);

			var d = distancePtSeg2d(verts[2], verts[0], verts[1]);
			var maxEdgeDist = Mathf.Max(0f, d);

			return Mathf.Sqrt(Mathf.Min(Mathf.Infinity, maxEdgeDist));
		}

		int prev(int i, int n) { return i - 1 >= 0 ? i - 1 : n - 1; }
		void triangulateHull(int nverts, Vector3Int[] verts, int nhull, int[] hull, int nin, ref List<int> tris)
		{
			var start = 0;
			var left = 1;
			var right = nhull - 1;

			// Start from an ear with shortest perimeter.
			// This tends to favor well formed triangles as starting point.
			var dmin = int.MaxValue;
			for (int i = 0; i < nhull; i++)
			{
				if (hull[i] >= 3) continue; // only use vertice from original triangles as center

				var pi = prev(i, nhull);
				var ni = (i + 1) % nhull;
				var pv = verts[hull[pi]];
				var cv = verts[hull[i]];
				var nv = verts[hull[ni]];
				var d = Mathf.FloorToInt(Int3.FloatPrecision * (vdist2(pv, cv) + vdist2(cv, nv) + vdist2(nv, pv)));
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
			while ((left + 1) % nhull != right) {
				// Check to see if se should advance left or right.
				var nleft = (left + 1) % nhull;
				var nright = prev(right, nhull);

				var cvleft = verts[hull[left]];
				var nvleft = verts[hull[nleft]];
				var cvright = verts[hull[right]];
				var nvright = verts[hull[nright]];
				var dleft = Mathf.FloorToInt(Int3.FloatPrecision * (vdist2(cvleft, nvleft) + vdist2(nvleft, cvright)));
				var dright = Mathf.FloorToInt(Int3.FloatPrecision * (vdist2(cvright, nvright) + vdist2(cvleft, nvright)));

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
			Vector3Int[] _in, int nin,
			int sampleDist, int sampleMaxError, VoxelHeightPatch[] hps, bool useTileBorder,
			ref List<Vector3Int> verts, ref List<Vector3Int> vertsi, ref List<Vector3Int> vertso, ref int nverts,
			ref List<int> tris, ref List<int> edges, ref List<int> samples,
			ref List<Int3> dverts, ref Int3[] mverts, int[] origVerts, ref TileBorder tileBorder) {

			var MAX_VERTS = 127;
			var MAX_TRIS = 255; // Max tris for delaunay is 2n-2-k (n=num verts, k=num hull verts).
			var MAX_VERTS_PER_EDGE = 32;
			var vedge = ArrayPool<Vector3Int>.Claim(MAX_VERTS_PER_EDGE + 1); // use round to avoid duplicates
			var vedgei = ArrayPool<Vector3Int>.Claim(MAX_VERTS_PER_EDGE + 1); // make sure vertices inside polygon
			var vedgeo = ArrayPool<Vector3Int>.Claim(MAX_VERTS_PER_EDGE + 1); // make sure vertices outside polygon
			var hull = ArrayPool<int>.Claim(MAX_VERTS);
			var contour = ListPool<Vector3Int>.Claim();
			var contourIds = ArrayPool<int>.Claim(2);
			var nhull = 0;

			nverts = 0;

			verts.Clear();
			vertsi.Clear();
			vertso.Clear();

			for (int i = 0; i < nin; i++) {
				verts.Add(_in[i]);
				vertsi.Add(_in[i]);
				vertso.Add(_in[i]);
			}
			nverts = nin;

			edges.Clear();
			tris.Clear();

			// Calculate minimum extents of the polygon based on input data.
			var minExtent = triMinExtent(verts.ToArray());

			// Tessellate outlines.
			// This is done in separate pass in order to ensure
			// seamless height values across the ply boundaries.
			if (sampleDist > 0) {
				for (int i = 0, j = nin - 1, p = nin - 2; i < nin; p = j, j = i++) {
					var vj = _in[j];
					var vi = _in[i];

					#region fix origin with neighbor
					vj.y = Mathf.RoundToInt(0.5f * (
						getHeight(vj.x, vj.y, vj.z, hps[3]) +
						getHeight(vj.x, vj.y, vj.z, hps[i])
					));
					vi.y = Mathf.RoundToInt(0.5f * (
						getHeight(vi.x, vi.y, vi.z, hps[3]) +
						getHeight(vi.x, vi.y, vi.z, hps[i])
					));
					dverts[origVerts[j]] = new Int3(vj.x, vj.y, vj.z);
					mverts[origVerts[j]] = new Int3(vj.x, vj.y, vj.z);
					dverts[origVerts[i]] = new Int3(vi.x, vi.y, vi.z);
					mverts[origVerts[i]] = new Int3(vi.x, vi.y, vi.z);
					#endregion

					var swapped = false;
					// Make sure the segments are always handled in same order
					// using lexological sort or else there will be seams.
					if (vj.x == vi.x) {
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
					contour.Clear();
					var borderSide = tileBorder.IsOnSide(vj, vi);
					var contours = tileBorder.GetSide(borderSide);

					#region fix origin with tile border
					if (useTileBorder && (borderSide != TileBorder.Side.UNKNOWN))
					{
						contourIds[0] = tileBorder.FixByContour(ref vj, borderSide, contours);
						contourIds[1] = tileBorder.FixByContour(ref vi, borderSide, contours);
						if (swapped)
						{
							dverts[origVerts[i]] = new Int3(vj.x, vj.y, vj.z);
							mverts[origVerts[i]] = new Int3(vj.x, vj.y, vj.z);
							dverts[origVerts[j]] = new Int3(vi.x, vi.y, vi.z);
							mverts[origVerts[j]] = new Int3(vi.x, vi.y, vi.z);
						}
						else
						{
							dverts[origVerts[j]] = new Int3(vj.x, vj.y, vj.z);
							mverts[origVerts[j]] = new Int3(vj.x, vj.y, vj.z);
							dverts[origVerts[i]] = new Int3(vi.x, vi.y, vi.z);
							mverts[origVerts[i]] = new Int3(vi.x, vi.y, vi.z);
						}
						if (contourIds[0] == contourIds[1])
						{
							contourIds[1] = -1;
						}
					}
					#endregion

					var newVj = vj;
					var newVi = vi;
					var kstart = 0;
					var kend = 0;
					var kendCount = 0;

					#region skip sample for tile border contour
					if (useTileBorder && (borderSide != TileBorder.Side.UNKNOWN))
					{
						if (contourIds[0] != -1)
						{
							var contv = contours[contourIds[0]].verts;
							for (int contvi = 0; contvi < contv.Count; contvi++)
							{
								if (borderSide == TileBorder.Side.ZMIN || borderSide == TileBorder.Side.ZMAX)
								{
									if (contv[contvi].x <= newVj.x) continue;
									if (contv[contvi].x > newVi.x) break;
									vedge[kstart] = newVj;
									vedgei[kstart] = newVj;
									vedgeo[kstart] = newVj;
									newVj = new Vector3Int(contv[contvi].x, contv[contvi].y, contv[contvi].z);
									kstart++;
								}
								else if (borderSide == TileBorder.Side.XMIN || borderSide == TileBorder.Side.XMAX)
								{
									if (contv[contvi].z <= newVj.z) continue;
									if (contv[contvi].z > newVi.z) break;
									vedge[kstart] = newVj;
									vedgei[kstart] = newVj;
									vedgeo[kstart] = newVj;
									newVj = new Vector3Int(contv[contvi].x, contv[contvi].y, contv[contvi].z);
									kstart++;
								}
							}
						}
						if (contourIds[1] != -1)
						{
							var contv = contours[contourIds[1]].verts;
							for (int contvi = contv.Count - 1; contvi >=0; contvi--)
							{
								if (borderSide == TileBorder.Side.ZMIN || borderSide == TileBorder.Side.ZMAX)
								{
									if (contv[contvi].x >= newVi.x) continue;
									if (contv[contvi].x < newVj.x) break;
									newVi = new Vector3Int(contv[contvi].x, contv[contvi].y, contv[contvi].z);
									kend = contvi;
									kendCount++;
								}
								else if (borderSide == TileBorder.Side.XMIN || borderSide == TileBorder.Side.XMAX)
								{
									if (contv[contvi].z >= newVi.z) continue;
									if (contv[contvi].z < newVj.z) break;
									newVi = new Vector3Int(contv[contvi].x, contv[contvi].y, contv[contvi].z);
									kend = contvi;
									kendCount++;
								}
							}
						}
					}
					#endregion

					var vgDir = vcross2(_in[p], newVj, newVi) > 0;

					// Create samples along thbuildPolye edge.
					var dx = newVi.x - newVj.x;
					var dy = newVi.y - newVj.y;
					var dz = newVi.z - newVj.z;
					var nn = 1 + Mathf.Max(dx, dy, dz) / sampleDist;
					if (nn >= MAX_VERTS_PER_EDGE - kstart) nn = MAX_VERTS_PER_EDGE - 1 - kstart;
					if (nverts + nn >= MAX_VERTS - kstart)
						nn = MAX_VERTS - 1 - nverts - kstart;

					if (dx == 0 && dz == 0)
					{
						#region special handling for start equals end
						var v = new Vector3Int(newVj.x, newVj.y, newVj.z);
						vedge[kstart] = v;
						vedgei[kstart] = v;
						vedgeo[kstart] = v;

						if (kendCount > 0)
						{
							var contv = contours[contourIds[1]].verts;
							for (int k = 1; k < kendCount; k++)
							{
								v = new Vector3Int(contv[kend + k].x, contv[kend + k].y, contv[kend + k].z);
								vedge[kstart + 1 + k] = v;
								vedgei[kstart + 1 + k] = v;
								vedgeo[kstart + 1 + k] = v;
							}
						}
						#endregion
					}
					else
					{
						for (int ki = 0; ki <= nn; ki++)
						{
							var k = ki + kstart;
							var u = (ki * 1f) / (nn * 1f);
							var edgeX = newVj.x + Mathf.RoundToInt(dx * u);
							var edgeY = newVj.y + Mathf.RoundToInt(dy * u);
							var edgeZ = newVj.z + Mathf.RoundToInt(dz * u);
							vedge[k] = new Vector3Int(edgeX, edgeY, edgeZ);

							edgeX = Mathf.FloorToInt(newVj.x + dx * u);
							edgeY = Mathf.FloorToInt(newVj.y + dy * u);
							edgeZ = Mathf.FloorToInt(newVj.z + dz * u);
							vedgei[k] = new Vector3Int(edgeX, edgeY, edgeZ);
							vedgeo[k] = new Vector3Int(edgeX, edgeY, edgeZ);

							#region make sure position, compared with another vertex
							if (vgDir != (vcross2(vedgei[k], newVj, newVi) > 0))
								vedgei[k].x = Mathf.CeilToInt(newVj.x + dx * u);
							if (vgDir != (vcross2(vedgei[k], newVj, newVi) > 0))
								vedgei[k].z = Mathf.CeilToInt(newVj.z + dz * u);
							if (vgDir != (vcross2(vedgei[k], newVj, newVi) > 0))
								vedgei[k].x = Mathf.FloorToInt(newVj.x + dx * u);

							if (vgDir == (vcross2(vedgeo[k], newVj, newVi) > 0))
								vedgeo[k].x = Mathf.CeilToInt(newVj.x + dx * u);
							if (vgDir == (vcross2(vedgeo[k], newVj, newVi) > 0))
								vedgeo[k].z = Mathf.CeilToInt(newVj.z + dz * u);
							if (vgDir == (vcross2(vedgeo[k], newVj, newVi) > 0))
								vedgeo[k].x = Mathf.FloorToInt(newVj.x + dx * u);
							#endregion

							vedgei[k].y = Mathf.RoundToInt(0.5f * (
								getHeight(vedgei[k].x, vedgei[k].y, vedgei[k].z, hps[3]) +
								getHeight(vedgeo[k].x, vedgeo[k].y, vedgeo[k].z, hps[i])
							));
							if (useTileBorder && (borderSide != TileBorder.Side.UNKNOWN))
							{
								tileBorder.FixByContour(ref vedgei[k], borderSide, contours);
							}
							vedge[k].y = vedgei[k].y;
							vedgeo[k].y = vedgei[k].y;
						}

						if (kendCount > 0)
						{
							var contv = contours[contourIds[1]].verts;
							for (int k = 0; k < kendCount; k++)
							{
								var v = new Vector3Int(contv[kend + k].x, contv[kend + k].y, contv[kend + k].z);
								vedge[kstart + 1 + k] = v;
								vedgei[kstart + 1 + k] = v;
								vedgeo[kstart + 1 + k] = v;
							}
						}
					}

					// Simplify samples.
					var idx = ListPool<int>.Claim();
					var nidx = 0;
					for (int k = 0; k < kstart; k++, nidx++)
					{
						idx.Add(k);
					}
					if (dx == 0 && dz == 0)
					{
						idx.Add(kstart);
						nidx++;
					}
					else
					{
						idx.Add(kstart);
						nidx++;
						idx.Add(kstart + nn);
						nidx++;
					}

					for (int k = kstart; k < nidx - 1;)
					{
						var a = idx[k];
						var b = idx[k + 1];
						var va = vedge[a];
						var vb = vedge[b];
						// Find maximum deviation along the segment.
						var maxd = 0;
						var maxi = UNDEF;
						for (int m = a + 1; m < b; m++)
						{
							var dev = Mathf.FloorToInt(Int3.FloatPrecision * distancePtSeg(vedge[m], va, vb));
							if (dev > maxd)
							{
								maxd = dev;
								maxi = m;
							}
						}

						// If the max deviation is larger than accepted error,
						// add new point, else continue to next segment.
						if (maxi != UNDEF && maxd > sampleMaxError * sampleMaxError * Int3.Precision)
						{
							idx.Insert(k + 1, maxi);
							nidx++;
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
						for (int k = nidx - 2; k > 0; k--) {
							var kid = idx[k];
							verts.Add(new Vector3Int(
								vedge[kid].x,
								vedge[kid].y,
								vedge[kid].z
							));
							vertsi.Add(new Vector3Int(
								vedgei[kid].x,
								vedgei[kid].y,
								vedgei[kid].z
							));
							vertso.Add(new Vector3Int(
								vedgeo[kid].x,
								vedgeo[kid].y,
								vedgeo[kid].z
							));
							hull[nhull] = nverts;
							nhull++;
							nverts++;
						}
					} else {
						for (int k = 1; k < nidx - 1; k++) {
							var kid = idx[k];
							verts.Add(new Vector3Int(
								vedge[kid].x,
								vedge[kid].y,
								vedge[kid].z
							));
							vertsi.Add(new Vector3Int(
								vedgei[kid].x,
								vedgei[kid].y,
								vedgei[kid].z
							));
							vertso.Add(new Vector3Int(
								vedgeo[kid].x,
								vedgeo[kid].y,
								vedgeo[kid].z
							));
							hull[nhull] = nverts;
							nhull++;
							nverts++;
						}
					}

					//if (borderSide != TileBorder.Side.UNKNOWN)
					if (!useTileBorder)
					{
						for (int k = 0; k < nidx; k++)
						{
							var kid = idx[k];
							contour.Add(new Vector3Int(
								vedge[kid].x,
								vedge[kid].y,
								vedge[kid].z
							));
						}
						tileBorder.AddContour(contour, borderSide);
					}
					ListPool<int>.Release(idx);
				}
			}

			// If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
			if (minExtent < sampleDist * 2) {
				triangulateHull(nverts, vertso.ToArray(), nhull, hull, nin, ref tris);
				//verifyHull(nverts, vertso.ToArray(), nhull, hull, tris.ToArray(), _in);
				return;
			}

			// Tessellate the base mesh.
			// We're using the triangulateHull instead of delaunayHull as it tends to
			// create a bit better triangulation for long thing triangles when there
			// are no internal points.
			triangulateHull(nverts, vertso.ToArray(), nhull, hull, nin, ref tris);
			//verifyHull(nverts, vertso.ToArray(), nhull, hull, tris.ToArray(), _in);
				
			if (tris.Count == 0) {
				// Could not triangulate the poly, make sure there is some valid data there.
				//Debug.Log("buildPolyDetail: Could not triangulate polygon.");
				return;
			}

			if (sampleDist > 0) {
			}

			if (tris.Count > MAX_TRIS * 4) {
				//Debug.LogError("Shrinking triangle count");
				tris.RemoveRange(MAX_TRIS * 4, tris.Count - MAX_TRIS * 4);
			}
			ArrayPool<int>.Release(ref hull, true);
			ArrayPool<Vector3Int>.Release(ref vedge, true);
			ArrayPool<Vector3Int>.Release(ref vedgei, true);
			ArrayPool<Vector3Int>.Release(ref vedgeo, true);
			ListPool<Vector3Int>.Release(contour);
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
				var ci = UNDEF;
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
				if (ci != UNDEF) {
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
			ArrayPool<int>.Release(ref offset, true);
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

			while (head < stack.Count)
			{
				var cx = stack[head].x;
				var cz = stack[head].y;
				var ci = stack[head].z;
				head++;
				if (head > RETRACT_SIZE)
				{
					head = 0;
					if (stack.Count > RETRACT_SIZE)
					{
						stack.RemoveRange(0, RETRACT_SIZE + 1);
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

					if (hp.data[hx + hz * hp.width] != RC_UNSET_HEIGHT) continue;

					var ai = (int)chf.compactCells[ax + az * chf.width].index + cs.GetConnection(dir);
					var _as = chf.compactSpans[ai];

					hp.data[hx + hz * hp.width] = _as.y;

					stack.Add(new Vector3Int(ax, az, ai));
				}
			}
		}

		public void BuildPolyMeshDetail(
			int sampleDist, int sampleMaxError,
			VoxelMesh mesh, out VoxelMesh dmesh, ref TileBorder tileBorder,
			int heightSearchRadius = 1)
		{
			
			var nvp = 3; // all original polygons are triangles
			var bs = borderSize;

			var edges = ListPool<int>.Claim(64);
			var tris = ListPool<int>.Claim(256);
			var stack = ListPool<Vector3Int>.Claim(512);
			var samples = ListPool<int>.Claim(512);
			var verts = ListPool<Vector3Int>.Claim(256);
			var vertsi = ListPool<Vector3Int>.Claim(256);
			var vertso = ListPool<Vector3Int>.Claim(256);
			var hps = ArrayPool<VoxelHeightPatch>.Claim(4);
			var nPolyVerts = 0;
			var maxhw = 0;
			var maxhd = 0;

			var ntris = mesh.tris.Length / nvp; // triangle count
			var bounds = ArrayPool<int>.Claim(ntris * 4);
			var poly = ArrayPool<Vector3Int>.Claim(nvp);
			var origVerts = ArrayPool<int>.Claim(nvp);
			var neighbors = ArrayPool<int>.Claim(ntris * nvp);

			var useTileBorder = tileBorder.HasData();

			#region sort triangle verts
			for (int i = 0; i < ntris; i++)
			{
				if (mesh.tris[i * nvp + 0] > mesh.tris[i * nvp + 2])
					MemorySwap(ref mesh.tris[i * nvp + 0], ref mesh.tris[i * nvp + 2]);
				if (mesh.tris[i * nvp + 0] > mesh.tris[i * nvp + 1])
					MemorySwap(ref mesh.tris[i * nvp + 0], ref mesh.tris[i * nvp + 1]);
				else if (mesh.tris[i * nvp + 1] > mesh.tris[i * nvp + 2])
					MemorySwap(ref mesh.tris[i * nvp + 1], ref mesh.tris[i * nvp + 2]);
			}
			#endregion

			#region find neighbors
			for (int i = 0; i < ntris * nvp; i++) neighbors[i] = UNDEF;
			for (int i = 0; i < ntris; i++)
			{
				for (int j = i + 1; j < ntris; j++)
				{
					if (mesh.tris[i * nvp + 0] >= mesh.tris[j * nvp + 2]) continue;
					if (mesh.tris[j * nvp + 0] >= mesh.tris[i * nvp + 2]) continue;

					var s0 = UNDEF;
					var t0 = UNDEF;
					for (int s = 0; s < nvp && s0 == UNDEF; s++)
					{
						for (int t = 0; t < nvp && t0 == UNDEF; t++)
						{
							if (mesh.tris[i * nvp + s] == mesh.tris[j * nvp + t])
							{
								s0 = s;
								t0 = t;
							}
						}
					}
					if (s0 == UNDEF) continue;

					var s1 = UNDEF;
					var t1 = UNDEF;
					for (int s = s0 + 1; s < nvp && s1 == UNDEF; s++)
					{
						for (int t = t0 + 1; t < nvp && t1 == UNDEF; t++)
						{
							if (mesh.tris[i * nvp + s] == mesh.tris[j * nvp + t])
							{
								s1 = s;
								t1 = t;
							}
						}
					}
					if (s1 == UNDEF) continue;

					if (s0 + 1 == s1)
						neighbors[i * nvp + s1] = j;
					else
						neighbors[i * nvp + 0] = j;
					
					if (t0 + 1 == t1)
						neighbors[j * nvp + t1] = i;
					else
						neighbors[j * nvp + 0] = i;
				}
			}
			#endregion

			// Find max size for a polygon area.
			for (int i = 0; i < ntris; i++)
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

			for (int i = 0; i < 4; i++)
			{
				hps[i] = new VoxelHeightPatch();
				hps[i].querymax = 1 + 2 * heightSearchRadius * (heightSearchRadius + 1);
				hps[i].data = ArrayPool<int>.Claim(maxhw * maxhd);
				for (int j = 0; j < maxhw * maxhd; j++) hps[i].data[j] = RC_UNSET_HEIGHT;
			}

			var dmeshVerts = ListPool<Int3>.Claim();
			var dmeshTris = ListPool<int>.Claim();
			var dmeshAreas = ListPool<int>.Claim();
			var dmeshRegs = ListPool<int>.Claim();

			var vertexIndex = 0;
			dmeshVerts.Clear();
			for (int i = 0; i < mesh.verts.Length; i++, vertexIndex++) {
				dmeshVerts.Add(new Int3(
					mesh.verts[i].x,
					mesh.verts[i].y,
					mesh.verts[i].z
				));
			}

			for (int i = 0; i < ntris; i++)
			{
				// Store polygon vertices for processing.
				var npoly = 0;
				for (int j = 0; j < nvp; j++)
				{
					origVerts[j] = mesh.tris[i * nvp + j];
					poly[j].x = mesh.verts[origVerts[j]].x;
					poly[j].y = mesh.verts[origVerts[j]].y;
					poly[j].z = mesh.verts[origVerts[j]].z;
					npoly++;
				}

				// Get the height data from the area of the polygon.
				hps[3].xmin = bounds[i * 4 + 0];
				hps[3].zmin = bounds[i * 4 + 2];
				hps[3].width = bounds[i * 4 + 1] - bounds[i * 4 + 0];
				hps[3].depth = bounds[i * 4 + 3] - bounds[i * 4 + 2];
				getHeightData(voxelArea, mesh.tris, mesh.verts, bs, ref hps[3], ref stack, mesh.regs[i], mesh.areas[i]);
				//verifyHeightData(mesh, i, hps[3]);

				for (int j = 0; j < nvp; j++)
				{
					var trij = neighbors[i * 3 + j];
					if (trij == UNDEF) trij = i;

					hps[j].xmin = bounds[trij * 4 + 0];
					hps[j].zmin = bounds[trij * 4 + 2];
					hps[j].width = bounds[trij * 4 + 1] - bounds[trij * 4 + 0];
					hps[j].depth = bounds[trij * 4 + 3] - bounds[trij * 4 + 2];
					getHeightData(voxelArea, mesh.tris, mesh.verts, bs, ref hps[j], ref stack, mesh.regs[trij], mesh.areas[trij]);
					//verifyHeightData(mesh, trij, hps[j]);
				}

				var nverts = 0;
				buildPolyDetail(
					poly, npoly, sampleDist, sampleMaxError, hps, useTileBorder,
					ref verts, ref vertsi, ref vertso, ref nverts, ref tris, ref edges, ref samples,
					ref dmeshVerts, ref mesh.verts, origVerts, ref tileBorder);

				var startIndex = vertexIndex;
				for (int vi = nvp; vi < nverts; vi++, vertexIndex++)
				{
					dmeshVerts.Add(new Int3(
						verts[vi].x,
						verts[vi].y,
						verts[vi].z
					));
				}

				for (int ti = 0; ti < tris.Count; ti += 4)
				{
					if (tris[ti + 0] < nvp) dmeshTris.Add(origVerts[tris[ti + 0]]);
					else dmeshTris.Add(startIndex + tris[ti + 0] - nvp);
					if (tris[ti + 1] < nvp) dmeshTris.Add(origVerts[tris[ti + 1]]);
					else dmeshTris.Add(startIndex + tris[ti + 1] - nvp);
					if (tris[ti + 2] < nvp) dmeshTris.Add(origVerts[tris[ti + 2]]);
					else dmeshTris.Add(startIndex + tris[ti + 2] - nvp);

					dmeshAreas.Add(mesh.areas[i]);
					dmeshRegs.Add(mesh.regs[i]);
				}
			}

			dmesh = new VoxelMesh
			{
				verts = dmeshVerts.ToArray(),
				tris = dmeshTris.ToArray(),
				areas = dmeshAreas.ToArray(),
				regs = dmeshRegs.ToArray()
			};

			ArrayPool<int>.Release(ref bounds, true);
			ArrayPool<Vector3Int>.Release(ref poly, true);

			ListPool<int>.Release(edges);
			ListPool<int>.Release(tris);
			ListPool<Vector3Int>.Release(stack);
			ListPool<int>.Release(samples);
			ListPool<Vector3Int>.Release(verts);

			ListPool<Vector3Int>.Release(vertsi);
			ListPool<Vector3Int>.Release(vertso);
			ArrayPool<int>.Release(ref origVerts, true);
			ArrayPool<VoxelHeightPatch>.Release(ref hps, true);
			ArrayPool<int>.Release(ref neighbors, true);
		}

		bool VertexEqual(Int3 a, Vector3Int b)
		{
			return a.x == b.x && a.y == b.y && a.z == b.z;
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

		bool verifyHull(int nverts, Vector3Int[] verts, int nhull, int[] hull, int[] tris, Vector3Int[] orig, int[] edges = null)
		{
			if (tris.Length == 4){
				// only original triangle
				return true;
			}

			var nvp = 3;
			var valid = HULL.VALID;

			var print = "poly(" + (tris.Length / 4) + "):\n";
			for (int i = 0; i < nvp; i++) print += orig[i] + ";";
			print += "\nverts:\n";
			for (int i = 0; i < nverts; i++) print += verts[i] + ";";
			print += "\nhull:\n";
			for (int i = 0; i < nhull; i++) print += verts[hull[i]] + ";";
			print += "\ninternal:\n";
			for (int i = nhull; i < nverts; i++) print += verts[i] + ";";
			print += "\n\n";

			var hullTris = new List<Vector3Int>();
			for (int i = 0; i < tris.Length; i += 4) hullTris.Add(new Vector3Int(tris[i + 0], tris[i + 1], tris[i + 2]));
			for (int i = 0; i < hullTris.Count; i++) {
				print += verts[hullTris[i].x] + ";" + verts[hullTris[i].y] + ";" + verts[hullTris[i].z] + "\n";
			}

			// verify edge
			if (valid == HULL.VALID) {
				var nvborder = nhull;
				var nvinternal = nverts - nvborder;
				var neborder = nvborder;
				var nedges = nvborder * 2 + nvinternal * 3 - 3;
				var ntris = nvborder + 2 * nvinternal - 2;
				print += "\n" + nvborder + " border vertices, " + nvinternal + " internal vertices\n";
				print += "expected:" + nedges + " edges, " + ntris + " triangules\n";
				if (edges != null) {
					print += (edges.Length / 4) + " edges, ";
					if (edges.Length < nedges * 4) valid = HULL.MISSING_EDGE;
					else if (edges.Length > nedges * 4) valid = HULL.DUPLICATE_EDGE;
				}
				print += (tris.Length / 4) + " triangles\n";
				if (tris.Length > ntris * 4) valid = HULL.DUPLICATE_TRIANGLE;
				else if (tris.Length < ntris * 4) valid = HULL.MISSING_TRIANGLE;
			}

			// verify area
			if (valid == HULL.VALID) {
				var area = varea2d(orig, nvp);
				print += "\narea:" + area + "\n";
				var areaSum = 0f;
				for (int i = 0; i < hullTris.Count; i++)
				{
					var tri = new Vector3Int[] { verts[hullTris[i].x], verts[hullTris[i].y], verts[hullTris[i].z] };
					areaSum += varea2d(tri);
				}
				print += "area:" + areaSum + "\n\n";
				if (area > areaSum) valid = HULL.UNCOVERED;
			}

			// verify triangle
			if (valid == HULL.VALID) {
				var sameVerts = new List<Vector3Int>();
				var tri1Verts = new List<Vector3Int>();
				var tri2Verts = new List<Vector3Int>();
				for (int i = 0; i < hullTris.Count && valid == HULL.VALID; i++)
				{
					for (int j = i + 1; j < hullTris.Count && valid == HULL.VALID; j++)
					{
						var title =
							"Verifying:" + verts[hullTris[i].x] + ";" + verts[hullTris[i].y] + ";" + verts[hullTris[i].z] +
							"<=>" + verts[hullTris[j].x] + ";" + verts[hullTris[j].y] + ";" + verts[hullTris[j].z] + "\n";

						print += title;
						tri1Verts.Clear();
						tri2Verts.Clear();
						sameVerts.Clear();
						tri1Verts.Add(new Vector3Int(verts[hullTris[i].x].x, verts[hullTris[i].x].y, verts[hullTris[i].x].z));
						tri1Verts.Add(new Vector3Int(verts[hullTris[i].y].x, verts[hullTris[i].y].y, verts[hullTris[i].y].z));
						tri1Verts.Add(new Vector3Int(verts[hullTris[i].z].x, verts[hullTris[i].z].y, verts[hullTris[i].z].z));
						tri2Verts.Add(new Vector3Int(verts[hullTris[j].x].x, verts[hullTris[j].x].y, verts[hullTris[j].x].z));
						tri2Verts.Add(new Vector3Int(verts[hullTris[j].y].x, verts[hullTris[j].y].y, verts[hullTris[j].y].z));
						tri2Verts.Add(new Vector3Int(verts[hullTris[j].z].x, verts[hullTris[j].z].y, verts[hullTris[j].z].z));

						for (int k = 0; k < tri1Verts.Count; k++)
						{
							for (int l = 0; l < tri2Verts.Count; l++)
							{
								if (tri1Verts[k].x == tri2Verts[l].x && tri1Verts[k].y == tri2Verts[l].y && tri1Verts[k].z == tri2Verts[l].z)
									sameVerts.Add(tri2Verts[l]);
							}
						}
						for (int k = 0; k < sameVerts.Count; k++)
						{
							for (int l = tri1Verts.Count - 1; l >= 0; l--)
							{
								if (sameVerts[k].x == tri1Verts[l].x && sameVerts[k].y == tri1Verts[l].y && sameVerts[k].z == tri1Verts[l].z)
									tri1Verts.RemoveAt(l);
							}
							for (int l = tri2Verts.Count - 1; l >= 0; l--)
							{
								if (sameVerts[k].x == tri2Verts[l].x && sameVerts[k].y == tri2Verts[l].y && sameVerts[k].z == tri2Verts[l].z)
								{
									tri2Verts.RemoveAt(l);
								}
							}
						}
						print += "same:";
						for (int k = 0; k < sameVerts.Count; k++) print += sameVerts[k] + ";";
						print += "\ntri1:";
						for (int k = 0; k < tri1Verts.Count; k++) print += tri1Verts[k] + ";";
						print += "\ntri2:";
						for (int k = 0; k < tri2Verts.Count; k++) print += tri2Verts[k] + ";";
						print += "\n";

						if (sameVerts.Count == 3)
						{
							valid = HULL.OVERLAPPED;
						}
						else if (sameVerts.Count == 2)
						{
							if (vcross2(tri1Verts[0], sameVerts[0], sameVerts[1]) * vcross2(tri2Verts[0], sameVerts[0], sameVerts[1]) > 0)
								valid = HULL.OVERLAPPED;
						}
						else if (sameVerts.Count == 1)
						{
							valid = HULL.VALID;
						}
						else
						{
							valid = HULL.VALID;
						}
					}
				}
			}

			print = valid + ":\n" + print;
			if (valid == HULL.VALID) {
				//Debug.LogWarning(print);
			}
			else Debug.LogError(print);

			return valid == HULL.VALID;
		}

		int varea2d(Vector3Int[] verts, int n = int.MaxValue) {
			n = Mathf.Min(n, verts.Length);
			var a = 0;
			for (int i = 0, j = n - 1; i < n; j = i++) {
				a += verts[i].x * verts[j].z - verts[i].z * verts[j].x;
			}
			return Mathf.Abs(a);
		}
	}
}
