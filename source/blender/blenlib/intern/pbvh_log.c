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
 * ***** END GPL LICENSE BLOCK *****
 */

#include "MEM_guardedalloc.h"

#include "BKE_ccg.h"
#include "BKE_DerivedMesh.h"

#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_pbvh.h"

#include "GPU_buffers.h"

#include "bmesh.h"
#include "pbvh_intern.h"
#include "range_tree_c_api.h"

#include <assert.h>

struct PBVHBMeshLog {
	/* Tree of free IDs */
	struct RangeTreeUInt *unused_ids;

	/* Mapping from unique ID's to vertices and faces
	 *
	 * More description TODO
	 */
	GHash *id_to_elem;

	/* All PBVHBMeshLogEntrys, ordered from earliest to most recent */
	ListBase entries;

	/* The current log entry from entries list
	 *
	 * If null, then the original mesh from before any of the log
	 * entries is current (i.e. there is nothing left to undo.)
	 *
	 * If equal to the last entry in the entries list, then all log
	 * entries have been applied (i.e. there is nothing left to redo.)
	 */
	PBVHBMeshLogEntry *current_entry;
};

struct PBVHBMeshLogVert {
	float co[3];
};

struct PBVHBMeshLogFace {
	unsigned int v_ids[3];
};

struct PBVHBMeshLogEntry {
	PBVHBMeshLogEntry *next, *prev;

	/* The following GHashes map from an element ID to one of the log
	 * types above */

	/* Elements that were in the previous entry, but have been
	 * deleted */
	GHash *deleted_verts;
	GHash *deleted_faces;
	/* Elements that were not in the previous entry, but are in the
	 * result of this entry */
	GHash *added_verts;
	GHash *added_faces;

	/* Vertices whose coordinates have changed v  */
	GHash *moved_verts;

	BLI_mempool *pool_verts;
	BLI_mempool *pool_faces;
};

/************************* Get/set element IDs ************************/

/* Get the vertex's unique ID from CustomData */
unsigned int pbvh_bmesh_log_vert_id_get(PBVH *bvh, BMVert *v)
{
	return *((unsigned int *)CustomData_bmesh_get(&bvh->bm->vdata,
												  v->head.data,
												  CD_ELEM_IDS));
}

static void pbvh_bmesh_log_vert_id_set(PBVH *bvh, BMVert *v, unsigned int id)
{
	void *vid = SET_INT_IN_POINTER(id);
	unsigned int *dst = CustomData_bmesh_get(&bvh->bm->vdata,
										   v->head.data,
										   CD_ELEM_IDS);
	(*dst) = id;
	BLI_ghash_remove(bvh->bm_log->id_to_elem, vid, NULL, NULL);
	BLI_ghash_insert(bvh->bm_log->id_to_elem, vid, v);
}

BMVert *pbvh_bmesh_log_vert_from_id(PBVH *bvh, unsigned int id)
{
	void *key = SET_INT_IN_POINTER(id);
	assert(BLI_ghash_haskey(bvh->bm_log->id_to_elem, key));
	return BLI_ghash_lookup(bvh->bm_log->id_to_elem, key);
}

/* Get the face's unique ID from CustomData */
unsigned int pbvh_bmesh_log_face_id_get(PBVH *bvh, BMFace *f)
{
	return *((unsigned int *)CustomData_bmesh_get(&bvh->bm->pdata,
												  f->head.data,
												  CD_ELEM_IDS));
}

static void pbvh_bmesh_log_face_id_set(PBVH *bvh, BMFace *f, unsigned int id)
{
	void *vid = SET_INT_IN_POINTER(id);
	unsigned int *dst = CustomData_bmesh_get(&bvh->bm->pdata,
										   f->head.data,
										   CD_ELEM_IDS);
	(*dst) = id;
	BLI_ghash_remove(bvh->bm_log->id_to_elem, vid, NULL, NULL);
	BLI_ghash_insert(bvh->bm_log->id_to_elem, vid, f);
}

BMFace *pbvh_bmesh_log_face_from_id(PBVH *bvh, unsigned int id)
{
	void *key = SET_INT_IN_POINTER(id);
	assert(BLI_ghash_haskey(bvh->bm_log->id_to_elem, key));
	return BLI_ghash_lookup(bvh->bm_log->id_to_elem, key);
}

/**********************************************************************/

static void pbvh_bmesh_log_assign_ids(PBVH *bvh)
{
	BMesh *bm = bvh->bm;
	BMIter iter;

	/* Generate vertex IDs */
	if (!CustomData_has_layer(&bm->vdata, CD_ELEM_IDS)) {
		BMVert *v;

		BM_data_layer_add(bm, &bm->vdata, CD_ELEM_IDS);
		BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
			unsigned int id = range_tree_uint_take_any(bvh->bm_log->unused_ids);
			pbvh_bmesh_log_vert_id_set(bvh, v, id);
		}
	}

	/* Generate face IDs */
	if (!CustomData_has_layer(&bm->pdata, CD_ELEM_IDS)) {
		BMFace *f;

		BM_data_layer_add(bm, &bm->pdata, CD_ELEM_IDS);
		BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
			unsigned int id = range_tree_uint_take_any(bvh->bm_log->unused_ids);
			pbvh_bmesh_log_face_id_set(bvh, f, id);
		}
	}
}

static PBVHBMeshLogEntry *pbvh_bmesh_log_entry_create(void)
{
	PBVHBMeshLogEntry *entry = MEM_callocN(sizeof(PBVHBMeshLogEntry), AT);

	entry->deleted_verts = BLI_ghash_ptr_new(AT);
	entry->deleted_faces = BLI_ghash_ptr_new(AT);
	entry->added_verts = BLI_ghash_ptr_new(AT);
	entry->added_faces = BLI_ghash_ptr_new(AT);
	entry->moved_verts = BLI_ghash_ptr_new(AT);

	entry->pool_verts = BLI_mempool_create(sizeof(PBVHBMeshLogVert), 1, 64, 0);
	entry->pool_faces = BLI_mempool_create(sizeof(PBVHBMeshLogFace), 1, 64, 0);

	return entry;
}

/* Free the data in a log entry; does not free the log entry itself */
static void pbvh_bmesh_log_entry_free(PBVHBMeshLogEntry *entry)
{
	BLI_ghash_free(entry->deleted_verts, NULL, NULL);
	BLI_ghash_free(entry->deleted_faces, NULL, NULL);
	BLI_ghash_free(entry->added_verts, NULL, NULL);
	BLI_ghash_free(entry->added_faces, NULL, NULL);
	BLI_ghash_free(entry->moved_verts, NULL, NULL);

	BLI_mempool_destroy(entry->pool_verts);
	BLI_mempool_destroy(entry->pool_faces);
}

static void pbvh_bmesh_log_elem_maps_init(PBVH *bvh)
{
	BMVert *v;
	BMFace *f;
	BMIter iter;

	BM_ITER_MESH (v, &iter, bvh->bm, BM_VERTS_OF_MESH) {
		unsigned int id = pbvh_bmesh_log_vert_id_get(bvh, v);
		void *vid = SET_INT_IN_POINTER(id);
		BLI_ghash_insert(bvh->bm_log->id_to_elem, vid, v);
	}

	BM_ITER_MESH (f, &iter, bvh->bm, BM_FACES_OF_MESH) {
		unsigned int id = pbvh_bmesh_log_face_id_get(bvh, f);
		void *vid = SET_INT_IN_POINTER(id);
		BLI_ghash_insert(bvh->bm_log->id_to_elem, vid, f);
	}
}

static PBVHBMeshLogVert *pbvh_bmesh_log_vert_alloc(PBVH *bvh,
												   BMVert *v)
{
	PBVHBMeshLogEntry *entry = bvh->bm_log->current_entry;
	PBVHBMeshLogVert *lv = BLI_mempool_alloc(entry->pool_verts);

	copy_v3_v3(lv->co, v->co);

	return lv;
}

static PBVHBMeshLogFace *pbvh_bmesh_log_face_alloc(PBVH *bvh,
												   BMFace *f)
{
	PBVHBMeshLogEntry *entry = bvh->bm_log->current_entry;
	PBVHBMeshLogFace *lf = BLI_mempool_alloc(entry->pool_faces);
	BMVert *v[3];

	assert(f->len == 3);

	BM_iter_as_array(NULL, BM_VERTS_OF_FACE, f, (void **)v, 3);

	lf->v_ids[0] = pbvh_bmesh_log_vert_id_get(bvh, v[0]);
	lf->v_ids[1] = pbvh_bmesh_log_vert_id_get(bvh, v[1]);
	lf->v_ids[2] = pbvh_bmesh_log_vert_id_get(bvh, v[2]);

	return lf;
}

/* Log a change to a vertex's coordinate
 *
 * Handles two separate cases:
 *
 * If the vertex was added in the current log entry, update the
 * coordinate in the map of added vertices.
 *
 * If the vertex already existed prior to the current log entry, a
 * seperate key/value map of moved vertices is used (using the
 * vertex's ID as the key). The coordinates stored in that case are
 * the vertex's original coordinates so that an undo can restore the
 * previous coordinates.
 *
 * On undo, the current coordinates will be swapped with the stored
 * coordinates so that a subsequent redo operation will restore the
 * newer vertex coordinates.
 */
void BLI_pbvh_bmesh_log_vert_moved(PBVH *bvh, BMVert *v)
{
	PBVHBMeshLogEntry *entry = bvh->bm_log->current_entry;
	PBVHBMeshLogVert *lv;
	unsigned int v_id = pbvh_bmesh_log_vert_id_get(bvh, v);
	void *key = SET_INT_IN_POINTER(v_id);

	/* Find or create the PBVHBMeshLogVert entry */
	if ((lv = BLI_ghash_lookup(entry->added_verts, key))) {
		copy_v3_v3(lv->co, v->co);
	}
	else if (!BLI_ghash_haskey(entry->moved_verts, key)) {
		lv = pbvh_bmesh_log_vert_alloc(bvh, v);
		BLI_ghash_insert(entry->moved_verts, key, lv);
	}
}

/* Log a new vertex as added to the BMesh
 *
 * The new vertex gets a unique ID assigned. (TODO: elaborate on
 * that?) It is then added to a map of added vertices, with the key
 * being its ID and the value containing everything needed to
 * reconstruct that vertex.
 */
void pbvh_bmesh_log_vert_added(PBVH *bvh, BMVert *v)
{
	PBVHBMeshLogVert *lv;
	unsigned int v_id = range_tree_uint_take_any(bvh->bm_log->unused_ids);

	pbvh_bmesh_log_vert_id_set(bvh, v, v_id);
	lv = pbvh_bmesh_log_vert_alloc(bvh, v);
	BLI_ghash_insert(bvh->bm_log->current_entry->added_verts,
					 SET_INT_IN_POINTER(v_id), lv);
}

/* Log a new face as added to the BMesh
 *
 * The new face gets a unique ID assigned. (TODO: elaborate on that?)
 * It is then added to a map of added faces, with the key being its ID
 * and the value containing everything needed to reconstruct that
 * face.
 */
void pbvh_bmesh_log_face_added(PBVH *bvh, BMFace *f)
{
	PBVHBMeshLogFace *lf;
	unsigned int f_id = range_tree_uint_take_any(bvh->bm_log->unused_ids);

	/* Only triangles are supported for now */
	assert(f->len == 3);

	pbvh_bmesh_log_face_id_set(bvh, f, f_id);
	lf = pbvh_bmesh_log_face_alloc(bvh, f);
	BLI_ghash_insert(bvh->bm_log->current_entry->added_faces,
					 SET_INT_IN_POINTER(f_id), lf);
}

/* Log a vertex as removed from the BMesh
 *
 * A couple things can happen here:
 * 
 * If the vertex was added as part of the current log entry, then it's
 * deleted and forgotten about entirely. Its unique ID is returned to
 * the unused pool.
 *
 * If the vertex was already part of the BMesh before the current log
 * entry, it is added to a map of deleted vertices, with the key being
 * its ID and the value containing everything needed to reconstruct
 * that vertex.
 *
 * If there's a move record for the vertex, that's used as the
 * vertices original location, then the move record is deleted.
 */
void pbvh_bmesh_log_vert_removed(PBVH *bvh, BMVert *v)
{
	PBVHBMeshLogEntry *entry = bvh->bm_log->current_entry;
	unsigned int v_id = pbvh_bmesh_log_vert_id_get(bvh, v);
	void *key = SET_INT_IN_POINTER(v_id);

	if (BLI_ghash_lookup(entry->added_verts, key)) {
		BLI_ghash_remove(entry->added_verts, key, NULL, NULL);
		range_tree_uint_release(bvh->bm_log->unused_ids, v_id);
	}
	else {
		PBVHBMeshLogVert *lv, *lv_co;

		lv = pbvh_bmesh_log_vert_alloc(bvh, v);
		BLI_ghash_insert(entry->deleted_verts, key, lv);

		/* If the vertex was moved before deletion, ensure that the
		 * original coordinates are stored */
		if ((lv_co = BLI_ghash_lookup(entry->moved_verts, key))) {
			copy_v3_v3(lv->co, lv_co->co);
			BLI_ghash_remove(entry->moved_verts, key, NULL, NULL);
		}
	}
}

/* Log a face as removed from the BMesh
 *
 * A couple things can happen here:
 * 
 * If the face was added as part of the current log entry, then it's
 * deleted and forgotten about entirely. Its unique ID is returned to
 * the unused pool.
 *
 * If the face was already part of the BMesh before the current log
 * entry, it is added to a map of deleted faces, with the key being
 * its ID and the value containing everything needed to reconstruct
 * that face.
 */
void pbvh_bmesh_log_face_removed(PBVH *bvh, BMFace *f)
{
	PBVHBMeshLogEntry *entry = bvh->bm_log->current_entry;
	unsigned int f_id = pbvh_bmesh_log_face_id_get(bvh, f);
	void *key = SET_INT_IN_POINTER(f_id);

	if (BLI_ghash_lookup(entry->added_faces, key)) {
		BLI_ghash_remove(entry->added_faces, key, NULL, NULL);
		range_tree_uint_release(bvh->bm_log->unused_ids, f_id);
	}
	else {
		PBVHBMeshLogFace *lf;

		lf = pbvh_bmesh_log_face_alloc(bvh, f);
		BLI_ghash_insert(entry->deleted_faces, key, lf);
	}
}


/************************ Helpers for undo/redo ***********************/

static void pbvh_bmesh_log_verts_unmake(PBVH *bvh, GHash *verts)
{
	GHashIterator gh_iter;
	GHASH_ITER (gh_iter, verts) {
		void *key = BLI_ghashIterator_getKey(&gh_iter);
		unsigned int id = GET_INT_FROM_POINTER(key);
		BMVert *v = pbvh_bmesh_log_vert_from_id(bvh, id);

		BM_vert_kill(bvh->bm, v);
	}
}

static void pbvh_bmesh_log_faces_unmake(PBVH *bvh, GHash *faces)
{
	GHashIterator gh_iter;
	GHASH_ITER (gh_iter, faces) {
		void *key = BLI_ghashIterator_getKey(&gh_iter);
		unsigned int id = GET_INT_FROM_POINTER(key);
		BMFace *f = pbvh_bmesh_log_face_from_id(bvh, id);

		BM_face_kill(bvh->bm, f);
	}
}

static void pbvh_bmesh_log_verts_restore(PBVH *bvh, GHash *verts)
{
	GHashIterator gh_iter;
	GHASH_ITER (gh_iter, verts) {
		void *key = BLI_ghashIterator_getKey(&gh_iter);
		PBVHBMeshLogVert *lv = BLI_ghashIterator_getValue(&gh_iter);
		BMVert *v = BM_vert_create(bvh->bm, lv->co, NULL);
		pbvh_bmesh_log_vert_id_set(bvh, v, GET_INT_FROM_POINTER(key));
	}
}

static void pbvh_bmesh_log_faces_restore(PBVH *bvh, GHash *faces)
{
	GHashIterator gh_iter;
	GHASH_ITER (gh_iter, faces) {
		void *key = BLI_ghashIterator_getKey(&gh_iter);
		PBVHBMeshLogFace *lf = BLI_ghashIterator_getValue(&gh_iter);
		BMVert *v[3] = {pbvh_bmesh_log_vert_from_id(bvh, lf->v_ids[0]),
						pbvh_bmesh_log_vert_from_id(bvh, lf->v_ids[1]),
						pbvh_bmesh_log_vert_from_id(bvh, lf->v_ids[2])};
		BMFace *f;

		f = BM_face_create_quad_tri_v(bvh->bm, v, 3, NULL, FALSE);
		pbvh_bmesh_log_face_id_set(bvh, f, GET_INT_FROM_POINTER(key));
	}
}

static void pbvh_bmesh_log_coords_swap(PBVH *bvh, GHash *verts)
{
	GHashIterator gh_iter;
	GHASH_ITER (gh_iter, verts) {
		void *key = BLI_ghashIterator_getKey(&gh_iter);
		PBVHBMeshLogVert *lv = BLI_ghashIterator_getValue(&gh_iter);
		unsigned int id = GET_INT_FROM_POINTER(key);
		BMVert *v = pbvh_bmesh_log_vert_from_id(bvh, id);

		swap_v3_v3(v->co, lv->co);
	}
}

/* Print the list of entries, marking the current one */
void pbvh_bmesh_log_print(const PBVHBMeshLog *log,
								 const char *description)
{
	const PBVHBMeshLogEntry *entry;
	const char *current = " <-- current";
	int i;

	printf("%s:\n", description);
	printf("    % 2d: [ initial ]%s\n", 0,
		   (!log->current_entry) ? current : "");
	for (entry = log->entries.first, i = 1; entry; entry = entry->next, i++) {
		printf("    % 2d: [%p]%s\n", i, entry,
			   (entry == log->current_entry) ? current : "");
	}
}


/***************************** Public API *****************************/

/* Allocate, initialize, and assign a new PBVHBMeshLog */
void BLI_pbvh_bmesh_log_create(PBVH *bvh)
{
	PBVHBMeshLog *log = MEM_callocN(sizeof(*log), AT);

	log->unused_ids = range_tree_uint_alloc(0, (unsigned)-1);
	log->id_to_elem = BLI_ghash_ptr_new(AT);

	bvh->bm_log = log;

	/* Assign IDs to all existing vertices and faces */
	pbvh_bmesh_log_assign_ids(bvh);

	pbvh_bmesh_log_elem_maps_init(bvh);
}

/* Free all the data in a PBVHBMeshLog including the log itself */
void BLI_pbvh_bmesh_log_free(PBVHBMeshLog *log)
{
	PBVHBMeshLogEntry *entry;

	if (log->unused_ids)
		range_tree_uint_free(log->unused_ids);

	if (log->id_to_elem)
		BLI_ghash_free(log->id_to_elem, NULL, NULL);

	for (entry = log->entries.first; entry; entry = entry->next)
		pbvh_bmesh_log_entry_free(entry);

	BLI_freelistN(&log->entries);

	MEM_freeN(log);
}

/* Get the PBVH's PBVHBMeshLog */
PBVHBMeshLog *BLI_pbvh_bmesh_log_get(PBVH *pbvh)
{
	return pbvh->bm_log;
}

/* Set the PBVH's PBVHBMeshLog */
void BLI_pbvh_bmesh_log_set(PBVH *pbvh, PBVHBMeshLog *log)
{
	pbvh->bm_log = log;
}

/* Start a new log entry and update the log entry list
 *
 * If the log entry list is empty, or if the current log entry is the
 * last entry, the new entry is simply appended to the end.
 *
 * Otherwise, the new entry is added after the current entry and all
 * following entries are deleted.
 *
 * In either case, the new entry is set as the current log entry.
 */
void BLI_pbvh_bmesh_log_entry_add(PBVH *bvh)
{
	PBVHBMeshLogEntry *entry, *next;

	/* Delete any entries after the current one */
	entry = bvh->bm_log->current_entry;
	if (entry) {
		for (entry = entry->next; entry; entry = next) {
			next = entry->next;
			pbvh_bmesh_log_entry_free(entry);
			BLI_freelinkN(&bvh->bm_log->entries, entry);
		}
	}

	/* Create and append the new entry */
	entry = pbvh_bmesh_log_entry_create();
	BLI_addtail(&bvh->bm_log->entries, entry);
	bvh->bm_log->current_entry = entry;
	
}

void BLI_pbvh_bmesh_log_undo(PBVH *bvh)
{
	PBVHBMeshLogEntry *entry = bvh->bm_log->current_entry;

	//pbvh_bmesh_log_print(bvh->bm_log, "Before undo");

	if (entry) {
		bvh->bm_log->current_entry = entry->prev;

		/* TODO: what do we need to with IDs? */

		/* Delete added faces and verts */
		pbvh_bmesh_log_faces_unmake(bvh, entry->added_faces);
		pbvh_bmesh_log_verts_unmake(bvh, entry->added_verts);

		/* Restore deleted verts and faces */
		pbvh_bmesh_log_verts_restore(bvh, entry->deleted_verts);
		pbvh_bmesh_log_faces_restore(bvh, entry->deleted_faces);

		/* Restore vertex coordinates */
		pbvh_bmesh_log_coords_swap(bvh, entry->moved_verts);
	}

	//pbvh_bmesh_log_print(bvh->bm_log, "After undo");
}

void BLI_pbvh_bmesh_log_redo(PBVH *bvh)
{
	PBVHBMeshLogEntry *entry = bvh->bm_log->current_entry;

	//pbvh_bmesh_log_print(bvh->bm_log, "Before redo");

	if (entry && entry->next)
		entry = entry->next;
	else
		entry = bvh->bm_log->entries.first;

	bvh->bm_log->current_entry = entry;

	if (entry) {
		/* TODO: what do we need to with IDs? */

		/* Re-delete previously deleted faces and verts */
		pbvh_bmesh_log_faces_unmake(bvh, entry->deleted_faces);
		pbvh_bmesh_log_verts_unmake(bvh, entry->deleted_verts);

		/* Restore previously added verts and faces */
		pbvh_bmesh_log_verts_restore(bvh, entry->added_verts);
		pbvh_bmesh_log_faces_restore(bvh, entry->added_faces);

		/* Restore vertex coordinates */
		pbvh_bmesh_log_coords_swap(bvh, entry->moved_verts);
	}

	//pbvh_bmesh_log_print(bvh->bm_log, "After redo");
}
