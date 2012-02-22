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
 * The Original Code is Copyright (C) 2011 Blender Foundation.
 * All rights reserved.
 *
 * Contributor(s): Campbell Barton
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/python/generic/blf_py_api.c
 *  \ingroup pygen
 *
 * This file defines the 'bme' bmesh main module.
 */

#include <Python.h>

#include "bmesh.h"

#include "bmesh_py_types.h"

#include "BLI_utildefines.h"

#include "BKE_tessmesh.h"

#include "DNA_mesh_types.h"

#include "../generic/py_capi_utils.h"

#include "bmesh_py_api.h" /* own include */

PyDoc_STRVAR(bpy_bm_from_mesh_doc,
".. method:: from_mesh(mesh)\n"
"\n"
"   todo.\n"
);

static PyObject *bpy_bm_from_mesh(PyObject *UNUSED(self), PyObject *value)
{
	Mesh *me = PyC_RNA_AsPointer(value, "Mesh");

	/* temp! */
	if (!me->edit_btmesh) {
		PyErr_SetString(PyExc_ValueError,
		                "Mesh is not in editmode");
		return NULL;
	}

	return BPy_BMesh_CreatePyObject(me->edit_btmesh->bm);
}

static struct PyMethodDef BPy_BM_methods[] = {
	{"from_mesh", (PyCFunction)bpy_bm_from_mesh, METH_O, bpy_bm_from_mesh_doc},
	{NULL, NULL, 0, NULL}
};

PyDoc_STRVAR(BPy_BM_doc,
"This module provides access to blenders bmesh data structures."
);
static struct PyModuleDef BPy_BM_module_def = {
	PyModuleDef_HEAD_INIT,
	"bme",  /* m_name */
	BPy_BM_doc,  /* m_doc */
	0,  /* m_size */
	BPy_BM_methods,  /* m_methods */
	NULL,  /* m_reload */
	NULL,  /* m_traverse */
	NULL,  /* m_clear */
	NULL,  /* m_free */
};

PyObject *BPyInit_bmesh(void)
{
	PyObject *submodule;

	BPy_BM_init_types();

	submodule = PyModule_Create(&BPy_BM_module_def);

	return submodule;
}
