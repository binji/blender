/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contributor(s): Nicholas Bishop
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "MEM_guardedalloc.h"

#include "BLI_array.h"
#include "BLI_math.h"
#include "BLI_utildefines.h"

#include "bmesh.h"

// TODO: really fix all these mesh->bmesh comments

typedef struct {
	BMesh *bm;
	BMOperator *op;

	int axis;
	BMO_SymmDirection direction;

	/* Maps from indices in the input vertex array (src_mvert) to
	 * indices in the output vertex array (dst_mvert). Input vertices
	 * that do not havea corresponding output are set to
	 * SYMM_KILLED_VERTEX. */
	GHash *vert_copy_map;
	GHash *vert_symm_map;

	/* Edges that cross the symmetry plane and are asymmetric get
	 * split. This array maps from input edges (src_medge) to output
	 * splits (vertex indices in the dst_mvert array). Edges that
	 * aren't split are set to SYMM_EDGE_NOT_SPLIT. */
	GHash *edge_split_map;
} Symm;

/* Note: don't think there's much need to having these user-adjustable? */
#define SYMM_AXIS_THRESHOLD 0.001f
#define SYMM_VERT_THRESHOLD 0.001f

/* See Symm.vert_map */
#define SYMM_KILLED_VERTEX -1
/* See Symm.edge_splits */
#define SYMM_EDGE_NOT_SPLIT -1

typedef enum {
	/* Coordinate lies on the side being copied from */
	SYMM_SIDE_KEEP,
	/* Coordinate lies on the side being copied from and within the
	 * axis threshold */
	SYMM_SIDE_AXIS,
	/* Coordinate lies on the side being copied to */
	SYMM_SIDE_KILL
} SymmSide;

/* Return which side the coordinate lies on */
static SymmSide symm_co_side(const Symm *symm,
							 const float *co)
{
	float comp = co[symm->axis];
	if (ELEM3(symm->direction,
			  BMO_SYMMETRIZE_NEGATIVE_X,
			  BMO_SYMMETRIZE_NEGATIVE_Y,
			  BMO_SYMMETRIZE_NEGATIVE_Z))
	{
		comp = -comp;
	}

	if (comp >= 0) {
		if (comp < SYMM_AXIS_THRESHOLD)
			return SYMM_SIDE_AXIS;
		else
			return SYMM_SIDE_KEEP;
	}
	else
		return SYMM_SIDE_KILL;
}

/* Output vertices and the vert_map array */
static void symm_verts_mirror(Symm *symm)
{
	BMOIter oiter;
	BMVert *src_v;

	symm->vert_symm_map = BLI_ghash_ptr_new(AT);

	BMO_ITER (src_v, &oiter, symm->bm, symm->op, "input", BM_VERT) {
		SymmSide side = symm_co_side(symm, src_v->co);

		/* TODO: should this test allow for verts that are almost on
		 * the axis but in the opposite direction? */

		if (ELEM(side, SYMM_SIDE_KEEP, SYMM_SIDE_AXIS)) {

			/* The vertex lies in the half-space being copied from;
			 * add to output */
			//dst_v = BM_vert_create(symm->bm, src_v->co, src_v);
			//BLI_ghash_insert(symm->vert_map, src_v, dst_v);

			if (side == SYMM_SIDE_KEEP) {
				BMVert *dst_v;
				float co[3];

				/* The vertex is outside the axis area; output its mirror */

				copy_v3_v3(co, src_v->co);
				co[symm->axis] = -co[symm->axis];

				dst_v = BM_vert_create(symm->bm, co, src_v);
				BLI_ghash_insert(symm->vert_symm_map, src_v, dst_v);
			}
			else {
				/* The vertex is within the axis area, snap to center */
				src_v->co[symm->axis] = 0;
			}
		}
		else {
			/* The vertex does not lie in the half-space being copied
			   from */
			//symm->vert_map[i] = SYMM_KILLED_VERTEX;
		}
	}
}

/* Output edge split vertices for asymmetric edges and the edge_splits
 * mapping array */
static void symm_split_asymmetric_edges(Symm *symm)
{
	BMOIter oiter;
	BMEdge *e;

	symm->edge_split_map = BLI_ghash_ptr_new(AT);

	BMO_ITER (e, &oiter, symm->bm, symm->op, "input", BM_EDGE) {
		const int sides[2] = {symm_co_side(symm, e->v1->co),
							  symm_co_side(symm, e->v2->co)};
		float flipped[3];

		copy_v3_v3(flipped, e->v1->co);
		flipped[symm->axis] = -flipped[symm->axis];

		if ((sides[0] != SYMM_SIDE_AXIS) &&
			(sides[1] != SYMM_SIDE_AXIS) &&
			(sides[0] != sides[1]) &&
			(!compare_v3v3(e->v2->co, flipped, SYMM_VERT_THRESHOLD)))
		{
			/* Endpoints lie on opposite sides and are asymmetric */

			BMVert *v;
			float lambda, edge_dir[3], co[3];
			float plane_co[3][3][3] = {
				/* axis == 0 */
				{{0, 0, 0}, {0, 1, 0}, {0, 0, 1}},
				/* axis == 1 */
				{{0, 0, 0}, {1, 0, 0}, {0, 0, 1}},
				/* axis == 2 */
				{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
			};
			int r;

			/* Find intersection of edge with symmetry plane */
			sub_v3_v3v3(edge_dir, e->v2->co, e->v1->co);
			normalize_v3(edge_dir); /* XXX: not sure if needed or not */
			r = isect_ray_plane_v3(e->v1->co,
								   edge_dir,
								   plane_co[symm->axis][0],
								   plane_co[symm->axis][1],
								   plane_co[symm->axis][2],
								   &lambda, TRUE);
			BLI_assert(r);

			madd_v3_v3v3fl(co, e->v1->co, edge_dir, lambda);
			co[symm->axis] = 0;

			/* Edge is asymmetric, split it with a new vertex */
			v = BM_vert_create(symm->bm, co, e->v1);
			BLI_ghash_insert(symm->edge_split_map, e, v);



			/* Mark the edge */
			/* TODO */
			//e->flag = 1;
		}
		else {
			//symm->edge_splits[i] = SYMM_EDGE_NOT_SPLIT;

			/* Unmark the edge */
			//e->flag = 0;
		}
	}
}

/****************************** SymmPoly ******************************/

typedef struct {
	/* Indices into the source mvert array (or -1 if not in that array) */
	BMVert **src_verts;
	/* Indices into the destination mvert array, these are vertices
	 * created by an edge split (-1 for vertices not created by edge
	 * split) */
	BMVert **edge_verts;

	/* Number of vertices in the polygon */
	int len;

	/* True only if none of the polygon's edges were split */
	int already_symmetric;
} SymmPoly;

static void symm_poly_with_splits(const Symm *symm,
								  BMFace *f,
								  SymmPoly *out)
{
	BMIter iter;
	BMLoop *l;
	int i;

	/* Count vertices and check for edge splits */
	/* TODO, can mostly remove */
	out->len = f->len;
	out->already_symmetric = TRUE;
	BM_ITER_ELEM (l, &iter, f, BM_LOOPS_OF_FACE) {
		if (BLI_ghash_haskey(symm->edge_split_map, l->e)) {
			out->len++;
			out->already_symmetric = FALSE;
		}
	}

	/* TODO: unify allocs */
	//out->src_verts = MEM_mallocN(sizeof(*out->src_verts) * out->len, AT);
	//out->edge_verts = MEM_mallocN(sizeof(*out->edge_verts) * out->len, AT);

	i = 0;
	BM_ITER_ELEM (l, &iter, f, BM_LOOPS_OF_FACE) {
		BMVert *split = BLI_ghash_lookup(symm->edge_split_map, l->e);

		out->src_verts[i] = l->v;
		out->edge_verts[i] = NULL;
		i++;

		if (split) {
			out->src_verts[i] = NULL;
			out->edge_verts[i] = split;
			i++;
		}
	}
}

static const float *symm_poly_co(const SymmPoly *sp, int v)
{
	if (sp->src_verts[v])
		return sp->src_verts[v]->co;
	else if (sp->edge_verts[v])
		return sp->edge_verts[v]->co;
	else
		return NULL;
}

static SymmSide symm_poly_co_side(const Symm *symm,
								  const SymmPoly *sp,
								  int v)
{
	return symm_co_side(symm, symm_poly_co(sp, v));
}

/* Return the index of the vertex in the destination array at corner
 * 'v' of the polygon, or -1 if not in that array */
static BMVert *symm_poly_dst(const SymmPoly *sp, int v)
{
	if (sp->edge_verts[v])
		return sp->edge_verts[v];
	else if (sp->src_verts[v])
		return sp->src_verts[v];
	else
		return NULL;
}

/* Same as above, but returns the index of the mirror if available, or
 * the same index if on the axis, or -1 otherwise */
static BMVert *symm_poly_mirror_dst(const Symm *symm,
									const SymmPoly *sp,
									int v)
{
	if (sp->edge_verts[v])
		return sp->edge_verts[v];
	else if (sp->src_verts[v]) {
		if (BLI_ghash_haskey(symm->vert_symm_map, sp->src_verts[v]))
			return BLI_ghash_lookup(symm->vert_symm_map, sp->src_verts[v]);
		else
			return sp->src_verts[v];
	}
	else
		return NULL;
}

static int symm_poly_next_crossing(const Symm *symm,
								   const SymmPoly *sp,
								   int start,
								   int *l1,
								   int *l2)
{
	int i;

	for (i = 0; i < sp->len; i++) {
		(*l1) = (start + i) % sp->len;
		(*l2) = ((*l1) + 1) % sp->len;

		if ((symm_poly_co_side(symm, sp, *l1) == SYMM_SIDE_KILL) ^
			(symm_poly_co_side(symm, sp, *l2) == SYMM_SIDE_KILL))
		{
			return TRUE;
		}
	}

	BLI_assert(!"symm_poly_next_crossing failed");
	return FALSE;
}

/* XXX: is there an existing function for this? */
static void BM_face_create_v(BMesh *bm, BMVert **fv, BMEdge **fe, int len)
{
	int i;

	if (len >= 3) {
		/* TODO: check this further, not sure if there should even be
		 * cases where this gets called with len of < 3 */

		for (i = 0; i < len; i++) {
			int j = (i + 1) % len;
			fe[i] = BM_edge_create(bm, fv[i], fv[j], NULL, TRUE);
		}
		BM_face_create(bm, fv, fe, len, TRUE);
	}
}

static void symm_mesh_output_poly_zero_splits(Symm *symm,
											  SymmPoly *sp,
											  BMVert **fv,
											  BMEdge **fe,
											  int segment_len,
											  int start)
{
	int i, j;

	j = 0;

	/* Output the keep side of the input polygon */
	for (i = 0; i < segment_len; i++) {
		const int offset = (start + i) % sp->len;
		BLI_assert(sp->src_verts[offset]);
		fv[j++] = sp->src_verts[offset];
	}

	/* Output the kill side of the polygon */
	for (i = segment_len - 1; i >=0; i--) {
		const int offset = (start + i) % sp->len;

		if (symm_poly_co_side(symm, sp, offset) == SYMM_SIDE_KEEP) {
			BLI_assert(sp->src_verts[offset]);
			fv[j++] = BLI_ghash_lookup(symm->vert_symm_map,
									   sp->src_verts[offset]);
		}
	}

	BM_face_create_v(symm->bm, fv, fe, j);
}

static void symm_mesh_output_poly_with_splits(Symm *symm,
											  SymmPoly *sp,
											  BMVert **fv,
											  BMEdge **fe,
											  int segment_len,
											  int start)
{
	int i;

	/* Output the keep side of the input polygon */

	for (i = 0; i < segment_len; i++) {
		const int offset = (start + i) % sp->len;
		BMVert *v = symm_poly_dst(sp, offset);

		BLI_assert(v);

		fv[i] = v;
	}

	BM_face_create_v(symm->bm, fv, fe, segment_len);

	/* Output the kill side of the input polygon */

	for (i = 0; i < segment_len; i++) {
		const int offset = (start + i) % sp->len;
		BMVert *v = symm_poly_mirror_dst(symm, sp, offset);

		fv[segment_len - i - 1] = v;

	}

	BM_face_create_v(symm->bm, fv, fe, segment_len);
}

static void symm_mirror_polygons(Symm *symm)
{
	BMOIter oiter;
	BMFace *f;
	BMVert **pv = NULL;
	BMVert **fv = NULL;
	BMEdge **fe = NULL;
	BLI_array_declare(pv);
	BLI_array_declare(fv);
	BLI_array_declare(fe);

	BMO_ITER (f, &oiter, symm->bm, symm->op, "input", BM_FACE) {
		BMIter iter;
		BMLoop *l;
		int mirror_all = TRUE, ignore_all = TRUE;

		/* Check if entire polygon can be mirrored or ignored */
		BM_ITER_ELEM (l, &iter, f, BM_LOOPS_OF_FACE) {
			if (symm_co_side(symm, l->v->co) == SYMM_SIDE_KILL)
				mirror_all = FALSE;
			else
				ignore_all = FALSE;
		}

		if (mirror_all) {
			int i;

			/* Make a mirrored copy of the polygon */

			BLI_array_empty(fv);
			BLI_array_empty(fe);
			BLI_array_grow_items(fv, f->len);
			BLI_array_grow_items(fe, f->len);

			i = f->len;
			BM_ITER_ELEM (l, &iter, f, BM_LOOPS_OF_FACE) {
				i--;

				if (symm_co_side(symm, l->v->co) == SYMM_SIDE_KEEP)
					fv[i] = BLI_ghash_lookup(symm->vert_symm_map, l->v);
				else
					fv[i] = l->v;
			}

			BM_face_create_v(symm->bm, fv, fe, f->len);
		}
		else if (ignore_all) {
			BM_face_kill(symm->bm, f);
		}
		else {
			SymmPoly sp;
			int l1, l2, l3, l4;
			int double_l2, double_l3;
			int segment_len;

			BLI_array_empty(pv);
			BLI_array_grow_items(pv, f->len * 4);
			sp.src_verts = pv;
			sp.edge_verts = pv + f->len * 2;
			symm_poly_with_splits(symm, f, &sp);

			/* Find first loop edge crossing the axis */
			symm_poly_next_crossing(symm, &sp, 0, &l1, &l2);

			/* If crossing isn't kill to keep, find the next one */
			if (symm_poly_co_side(symm, &sp, l1) != SYMM_SIDE_KILL) {
				symm_poly_next_crossing(symm, &sp, l2, &l1, &l2);
			}

			/* Find next crossing (keep to kill) */
			symm_poly_next_crossing(symm, &sp, l2, &l3, &l4);

			if (l2 == l3)
				segment_len = 0;
			else if (l2 < l3)
				segment_len = l3 - l2 + 1;
			else
				segment_len = (sp.len - l2 + 1) + l3;

			double_l2 = symm_poly_co_side(symm, &sp, l2) == SYMM_SIDE_KEEP;
			double_l3 = symm_poly_co_side(symm, &sp, l3) == SYMM_SIDE_KEEP;

			/* Calculate number of new polygons/loops */
			if (segment_len == 0) {
			}
			else if (sp.already_symmetric) {
				int new_loops;

				if (double_l2 && double_l3)
					new_loops = segment_len * 2;
				else if (!double_l2 && !double_l3)
					new_loops = segment_len * 2 - 2;
				else
					new_loops = segment_len * 2 - 1;

				BLI_array_empty(fv);
				BLI_array_empty(fe);
				BLI_array_grow_items(fv, new_loops);
				BLI_array_grow_items(fe, new_loops);

				symm_mesh_output_poly_zero_splits(symm, &sp,
												  fv, fe,
												  segment_len, l2);
				BM_face_kill(symm->bm, f);
			}
			else if (!double_l2 && !double_l3) {
				BLI_array_empty(fv);
				BLI_array_empty(fe);
				BLI_array_grow_items(fv, segment_len);
				BLI_array_grow_items(fe, segment_len);

				symm_mesh_output_poly_with_splits(symm, &sp,
												  fv, fe,
												  segment_len,
												  l2);

				BM_face_kill(symm->bm, f);
			}
			else {
				BLI_array_empty(fv);
				BLI_array_empty(fe);
				BLI_array_grow_items(fv, segment_len);
				BLI_array_grow_items(fe, segment_len);

				symm_mesh_output_poly_with_splits(symm, &sp,
												  fv, fe,
												  segment_len,
												  l2);

				BM_face_kill(symm->bm, f);

				/* Output bridge triangle */

				BLI_array_empty(fv);
				BLI_array_empty(fe);
				BLI_array_grow_items(fv, 3);
				BLI_array_grow_items(fe, 3);

				if (double_l2) {
					fv[0] = symm_poly_dst(&sp, l2);
					fv[1] = symm_poly_mirror_dst(symm, &sp, l2);
					fv[2] = symm_poly_dst(&sp, l3);
				}
				else if (double_l3) {
					fv[0] = symm_poly_dst(&sp, l3);
					fv[1] = symm_poly_mirror_dst(symm, &sp, l3);
					fv[2] = symm_poly_dst(&sp, l2);
				}

				BLI_assert(fv[0] && fv[1] && fv[2]);

				BM_face_create_v(symm->bm, fv, fe, 3);
			}
		}
	}

	BLI_array_free(pv);
	BLI_array_free(fv);
	BLI_array_free(fe);
}

/* Remove unused edges and vertices from the side being copied to */
static void symm_kill_unused(Symm *symm)
{
	BMOIter oiter;
	BMEdge *e;
	BMVert *v;

	/* Kill unused edges */
	BMO_ITER (e, &oiter, symm->bm, symm->op, "input", BM_EDGE) {
		if ((symm_co_side(symm, e->v1->co) == SYMM_SIDE_KILL) ||
			(symm_co_side(symm, e->v2->co) == SYMM_SIDE_KILL))
		{
			/* TODO */
			//BLI_assert(BM_edge_face_count(e) == 0);
			if (BM_edge_face_count(e) == 0)
				BM_edge_kill(symm->bm, e);
		}
	}

	/* Kill unused vertices */
	BMO_ITER (v, &oiter, symm->bm, symm->op, "input", BM_VERT) {
		if (symm_co_side(symm, v->co) == SYMM_SIDE_KILL) {
			/* TODO */
			//BLI_assert(BM_vert_face_count(v) == 0);
			if (BM_vert_face_count(v) == 0)
				BM_vert_kill(symm->bm, v);
		}
	}
}

void bmo_symmetrize_exec(BMesh *bm, BMOperator *op)
{
	Symm symm;
	BMO_SymmDirection direction = BMO_slot_int_get(op, "direction");

	symm.bm = bm;
	symm.op = op;
	symm.axis = (ELEM(direction,
					  BMO_SYMMETRIZE_NEGATIVE_X,
					  BMO_SYMMETRIZE_POSITIVE_X) ? 0 :
				 ELEM(direction,
					  BMO_SYMMETRIZE_NEGATIVE_Y,
					  BMO_SYMMETRIZE_POSITIVE_Y) ? 1 :
				 ELEM(direction,
					  BMO_SYMMETRIZE_NEGATIVE_Z,
					  BMO_SYMMETRIZE_POSITIVE_Z) ? 2 : 0);
	symm.direction = direction;

	symm_verts_mirror(&symm);
	symm_split_asymmetric_edges(&symm);
	symm_mirror_polygons(&symm);
	symm_kill_unused(&symm);

	BLI_ghash_free(symm.vert_symm_map, NULL, NULL);
	BLI_ghash_free(symm.edge_split_map, NULL, NULL);
}
