/*
 * Copyright 2011, Blender Foundation.
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
 */

#include "camera.h"
#include "scene.h"

#include "blender_sync.h"
#include "blender_util.h"

CCL_NAMESPACE_BEGIN

/* Blender Camera Intermediate: we first convert both the offline and 3d view
 * render camera to this, and from there convert to our native camera format. */

struct BlenderCamera {
	float nearclip;
	float farclip;

	CameraType type;
	float ortho_scale;

	float lens;
	float shuttertime;

	float aperturesize;
	uint apertureblades;
	float aperturerotation;
	float focaldistance;

	float2 shift;
	float2 offset;
	float zoom;

	float2 pixelaspect;

	PanoramaType panorama_type;
	float fisheye_fov;
	float fisheye_lens;

	enum { AUTO, HORIZONTAL, VERTICAL } sensor_fit;
	float sensor_width;
	float sensor_height;

	float border_left;
	float border_right;
	float border_bottom;
	float border_top;

	Transform matrix;
};

static void blender_camera_init(BlenderCamera *bcam)
{
	memset(bcam, 0, sizeof(BlenderCamera));

	bcam->type = CAMERA_PERSPECTIVE;
	bcam->zoom = 1.0f;
	bcam->pixelaspect = make_float2(1.0f, 1.0f);
	bcam->sensor_width = 32.0f;
	bcam->sensor_height = 18.0f;
	bcam->sensor_fit = BlenderCamera::AUTO;
	bcam->shuttertime = 1.0f;
	bcam->border_right = 1.0f;
	bcam->border_top = 1.0f;
}

static float blender_camera_focal_distance(BL::Object b_ob, BL::Camera b_camera)
{
	BL::Object b_dof_object = b_camera.dof_object();

	if(!b_dof_object)
		return b_camera.dof_distance();
	
	/* for dof object, return distance along camera Z direction */
	Transform obmat = transform_clear_scale(get_transform(b_ob.matrix_world()));
	Transform dofmat = get_transform(b_dof_object.matrix_world());
	Transform mat = transform_inverse(obmat) * dofmat;

	return fabsf(transform_get_column(&mat, 3).z);
}

static void blender_camera_from_object(BlenderCamera *bcam, BL::Object b_ob)
{
	BL::ID b_ob_data = b_ob.data();

	if(b_ob_data.is_a(&RNA_Camera)) {
		BL::Camera b_camera(b_ob_data);
		PointerRNA ccamera = RNA_pointer_get(&b_camera.ptr, "cycles");

		bcam->nearclip = b_camera.clip_start();
		bcam->farclip = b_camera.clip_end();

		switch(b_camera.type())
		{
			case BL::Camera::type_ORTHO:
				bcam->type = CAMERA_ORTHOGRAPHIC;
				break;
			case BL::Camera::type_PANO:
				bcam->type = CAMERA_PANORAMA;
				break;
			case BL::Camera::type_PERSP:
			default:
				bcam->type = CAMERA_PERSPECTIVE;
				break;
		}	

		switch(RNA_enum_get(&ccamera, "panorama_type"))
		{
			case 1:
				bcam->panorama_type = PANORAMA_FISHEYE_EQUIDISTANT;
				break;
			case 2:
				bcam->panorama_type = PANORAMA_FISHEYE_EQUISOLID;
				break;
			case 0:
			default:
				bcam->panorama_type = PANORAMA_EQUIRECTANGULAR;
				break;
		}	

		bcam->fisheye_fov = RNA_float_get(&ccamera, "fisheye_fov");
		bcam->fisheye_lens = RNA_float_get(&ccamera, "fisheye_lens");

		bcam->ortho_scale = b_camera.ortho_scale();

		bcam->lens = b_camera.lens();

		/* allow f/stop number to change aperture_size but still
		 * give manual control over aperture radius */
		int aperture_type = RNA_enum_get(&ccamera, "aperture_type");

		if(aperture_type == 1) {
			float fstop = RNA_float_get(&ccamera, "aperture_fstop");
			bcam->aperturesize = (bcam->lens*1e-3f)/(2.0f*max(fstop, 1e-5f));
		}
		else
			bcam->aperturesize = RNA_float_get(&ccamera, "aperture_size");

		bcam->apertureblades = RNA_int_get(&ccamera, "aperture_blades");
		bcam->aperturerotation = RNA_float_get(&ccamera, "aperture_rotation");
		bcam->focaldistance = blender_camera_focal_distance(b_ob, b_camera);

		bcam->shift.x = b_camera.shift_x();
		bcam->shift.y = b_camera.shift_y();

		bcam->sensor_width = b_camera.sensor_width();
		bcam->sensor_height = b_camera.sensor_height();

		if(b_camera.sensor_fit() == BL::Camera::sensor_fit_AUTO)
			bcam->sensor_fit = BlenderCamera::AUTO;
		else if(b_camera.sensor_fit() == BL::Camera::sensor_fit_HORIZONTAL)
			bcam->sensor_fit = BlenderCamera::HORIZONTAL;
		else
			bcam->sensor_fit = BlenderCamera::VERTICAL;
	}
	else {
		/* from lamp not implemented yet */
	}
}

static Transform blender_camera_matrix(const Transform& tfm, CameraType type)
{
	Transform result;

	if(type == CAMERA_PANORAMA) {
		/* make it so environment camera needs to be pointed in the direction
		 * of the positive x-axis to match an environment texture, this way
		 * it is looking at the center of the texture */
		result = tfm *
			make_transform( 0.0f, -1.0f, 0.0f, 0.0f,
			                0.0f,  0.0f, 1.0f, 0.0f,
			               -1.0f,  0.0f, 0.0f, 0.0f,
			                0.0f,  0.0f, 0.0f, 1.0f);
	}
	else {
		/* note the blender camera points along the negative z-axis */
		result = tfm * transform_scale(1.0f, 1.0f, -1.0f);
	}

	return transform_clear_scale(result);
}

static void blender_camera_viewplane(BlenderCamera *bcam, int width, int height,
	float *left, float *right, float *bottom, float *top, float *aspectratio, float *sensor_size)
{
	/* dimensions */
	float xratio = width*bcam->pixelaspect.x;
	float yratio = height*bcam->pixelaspect.y;

	/* compute x/y aspect and ratio */
	float xaspect, yaspect;

	/* sensor fitting */
	bool horizontal_fit;

	if(bcam->sensor_fit == BlenderCamera::AUTO) {
		horizontal_fit = (xratio > yratio);
		*sensor_size = bcam->sensor_width;
	}
	else if(bcam->sensor_fit == BlenderCamera::HORIZONTAL) {
		horizontal_fit = true;
		*sensor_size = bcam->sensor_width;
	}
	else {
		horizontal_fit = false;
		*sensor_size = bcam->sensor_height;
	}

	if(horizontal_fit) {
		*aspectratio = xratio/yratio;
		xaspect = *aspectratio;
		yaspect = 1.0f;
	}
	else {
		*aspectratio = yratio/xratio;
		xaspect = 1.0f;
		yaspect = *aspectratio;
	}

	/* modify aspect for orthographic scale */
	if(bcam->type == CAMERA_ORTHOGRAPHIC) {
		xaspect = xaspect*bcam->ortho_scale/(*aspectratio*2.0f);
		yaspect = yaspect*bcam->ortho_scale/(*aspectratio*2.0f);
		*aspectratio = bcam->ortho_scale/2.0f;
	}

	if(bcam->type == CAMERA_PANORAMA) {
		/* set viewplane */
		*left = 0.0f;
		*right = 1.0f;
		*bottom = 0.0f;
		*top = 1.0f;
	}
	else {
		/* set viewplane */
		*left = -xaspect;
		*right = xaspect;
		*bottom = -yaspect;
		*top = yaspect;

		/* zoom for 3d camera view */
		*left *= bcam->zoom;
		*right *= bcam->zoom;
		*bottom *= bcam->zoom;
		*top *= bcam->zoom;

		/* modify viewplane with camera shift and 3d camera view offset */
		float dx = 2.0f*(*aspectratio*bcam->shift.x + bcam->offset.x*xaspect*2.0f);
		float dy = 2.0f*(*aspectratio*bcam->shift.y + bcam->offset.y*yaspect*2.0f);

		*left += dx;
		*right += dx;
		*bottom += dy;
		*top += dy;
	}
}

static void blender_camera_sync(Camera *cam, BlenderCamera *bcam, int width, int height)
{
	/* copy camera to compare later */
	Camera prevcam = *cam;
	float aspectratio, sensor_size;

	/* viewplane */
	blender_camera_viewplane(bcam, width, height,
		&cam->left, &cam->right, &cam->bottom, &cam->top, &aspectratio, &sensor_size);

	/* sensor */
	cam->sensorwidth = bcam->sensor_width;
	cam->sensorheight = bcam->sensor_height;

	/* clipping distances */
	cam->nearclip = bcam->nearclip;
	cam->farclip = bcam->farclip;

	/* type */
	cam->type = bcam->type;

	/* panorama */
	cam->panorama_type = bcam->panorama_type;
	cam->fisheye_fov = bcam->fisheye_fov;
	cam->fisheye_lens = bcam->fisheye_lens;

	/* perspective */
	cam->fov = 2.0f * atanf((0.5f * sensor_size) / bcam->lens / aspectratio);
	cam->focaldistance = bcam->focaldistance;
	cam->aperturesize = bcam->aperturesize;
	cam->blades = bcam->apertureblades;
	cam->bladesrotation = bcam->aperturerotation;

	/* transform */
	cam->matrix = blender_camera_matrix(bcam->matrix, bcam->type);
	cam->motion.pre = cam->matrix;
	cam->motion.post = cam->matrix;
	cam->use_motion = false;
	cam->shuttertime = bcam->shuttertime;

	/* border */
	cam->border_left = bcam->border_left;
	cam->border_right = bcam->border_right;
	cam->border_bottom = bcam->border_bottom;
	cam->border_top = bcam->border_top;

	/* set update flag */
	if(cam->modified(prevcam))
		cam->tag_update();
}

/* Sync Render Camera */

void BlenderSync::sync_camera(BL::Object b_override, int width, int height)
{
	BlenderCamera bcam;
	blender_camera_init(&bcam);

	/* pixel aspect */
	BL::RenderSettings r = b_scene.render();

	bcam.pixelaspect.x = r.pixel_aspect_x();
	bcam.pixelaspect.y = r.pixel_aspect_y();
	bcam.shuttertime = r.motion_blur_shutter();

	/* border */
	if(r.use_border()) {
		bcam.border_left = r.border_min_x();
		bcam.border_right = r.border_max_x();
		bcam.border_bottom = r.border_min_y();
		bcam.border_top = r.border_max_y();
	}

	/* camera object */
	BL::Object b_ob = b_scene.camera();

	if(b_override)
		b_ob = b_override;

	if(b_ob) {
		blender_camera_from_object(&bcam, b_ob);
		bcam.matrix = get_transform(b_ob.matrix_world());
	}

	/* sync */
	Camera *cam = scene->camera;
	blender_camera_sync(cam, &bcam, width, height);
}

void BlenderSync::sync_camera_motion(BL::Object b_ob, int motion)
{
	Camera *cam = scene->camera;

	Transform tfm = get_transform(b_ob.matrix_world());
	tfm = blender_camera_matrix(tfm, cam->type);

	if(tfm != cam->matrix) {
		if(motion == -1)
			cam->motion.pre = tfm;
		else
			cam->motion.post = tfm;

		cam->use_motion = true;
	}
}

/* Sync 3D View Camera */

static void blender_camera_from_view(BlenderCamera *bcam, BL::Scene b_scene, BL::SpaceView3D b_v3d, BL::RegionView3D b_rv3d, int width, int height)
{
	/* 3d view parameters */
	bcam->nearclip = b_v3d.clip_start();
	bcam->farclip = b_v3d.clip_end();
	bcam->lens = b_v3d.lens();
	bcam->shuttertime = b_scene.render().motion_blur_shutter();

	if(b_rv3d.view_perspective() == BL::RegionView3D::view_perspective_CAMERA) {
		/* camera view */
		BL::Object b_ob = (b_v3d.lock_camera_and_layers())? b_scene.camera(): b_v3d.camera();

		if(b_ob) {
			blender_camera_from_object(bcam, b_ob);

			/* magic zoom formula */
			bcam->zoom = (float)b_rv3d.view_camera_zoom();
			bcam->zoom = (1.41421f + bcam->zoom/50.0f);
			bcam->zoom *= bcam->zoom;
			bcam->zoom = 2.0f/bcam->zoom;

			/* offset */
			bcam->offset = get_float2(b_rv3d.view_camera_offset());
		}
	}
	else if(b_rv3d.view_perspective() == BL::RegionView3D::view_perspective_ORTHO) {
		/* orthographic view */
		bcam->farclip *= 0.5f;
		bcam->nearclip = -bcam->farclip;

		bcam->type = CAMERA_ORTHOGRAPHIC;
		bcam->ortho_scale = b_rv3d.view_distance();
	}

	bcam->zoom *= 2.0f;

	/* 3d view transform */
	bcam->matrix = transform_inverse(get_transform(b_rv3d.view_matrix()));
}

static void blender_camera_border(BlenderCamera *bcam, BL::Scene b_scene, BL::SpaceView3D b_v3d,
	BL::RegionView3D b_rv3d, int width, int height)
{
	BL::RenderSettings r = b_scene.render();

	if(!r.use_border())
		return;

	/* camera view? */
	if(!(b_rv3d && b_rv3d.view_perspective() == BL::RegionView3D::view_perspective_CAMERA))
		return;

	BL::Object b_ob = (b_v3d.lock_camera_and_layers())? b_scene.camera(): b_v3d.camera();

	if(!b_ob)
		return;

	bcam->border_left = r.border_min_x();
	bcam->border_right = r.border_max_x();
	bcam->border_bottom = r.border_min_y();
	bcam->border_top = r.border_max_y();

	float cam_left, cam_right, cam_bottom, cam_top;
	float view_left, view_right, view_bottom, view_top;
	float view_aspect, cam_aspect, sensor_size;

	/* get viewport viewplane */
	BlenderCamera view_bcam;
	blender_camera_init(&view_bcam);
	blender_camera_from_view(&view_bcam, b_scene, b_v3d, b_rv3d, width, height);

	blender_camera_viewplane(&view_bcam, width, height,
		&view_left, &view_right, &view_bottom, &view_top, &view_aspect, &sensor_size);

	view_left /= view_aspect;
	view_right /= view_aspect;
	view_bottom /= view_aspect;
	view_top /= view_aspect;

	/* get camera viewplane */
	BlenderCamera cam_bcam;
	blender_camera_init(&cam_bcam);
	blender_camera_from_object(&cam_bcam, b_ob);

	width = (int)(r.resolution_x()*r.resolution_percentage()/100);
	height = (int)(r.resolution_y()*r.resolution_percentage()/100);

	blender_camera_viewplane(&cam_bcam, width, height,
		&cam_left, &cam_right, &cam_bottom, &cam_top, &cam_aspect, &sensor_size);

	cam_left /= cam_aspect;
	cam_right /= cam_aspect;
	cam_bottom /= cam_aspect;
	cam_top /= cam_aspect;

	/* determine viewport subset matching camera border */
	float tmp_left = ((cam_left - view_left) / (view_right - view_left));
	float tmp_right = ((cam_right - view_left) / (view_right - view_left));
	float tmp_bottom = ((cam_bottom - view_bottom) / (view_top - view_bottom));
	float tmp_top = ((cam_top - view_bottom) / (view_top - view_bottom));

	bcam->border_left = tmp_left + bcam->border_left*(tmp_right - tmp_left);
	bcam->border_right = tmp_left + bcam->border_right*(tmp_right - tmp_left);
	bcam->border_bottom = tmp_bottom + bcam->border_bottom*(tmp_top - tmp_bottom);
	bcam->border_top = tmp_bottom + bcam->border_top*(tmp_top - tmp_bottom);

	/* clamp */
	bcam->border_left = clamp(bcam->border_left, 0.0f, 1.0f);
	bcam->border_right = clamp(bcam->border_right, 0.0f, 1.0f);
	bcam->border_bottom = clamp(bcam->border_bottom, 0.0f, 1.0f);
	bcam->border_top = clamp(bcam->border_top, 0.0f, 1.0f);
}

void BlenderSync::sync_view(BL::SpaceView3D b_v3d, BL::RegionView3D b_rv3d, int width, int height)
{
	BlenderCamera bcam;
	blender_camera_init(&bcam);
	blender_camera_from_view(&bcam, b_scene, b_v3d, b_rv3d, width, height);
	blender_camera_border(&bcam, b_scene, b_v3d, b_rv3d, width, height);

	blender_camera_sync(scene->camera, &bcam, width, height);
}

BufferParams BlenderSync::get_buffer_params(BL::Scene b_scene, Camera *cam, int width, int height)
{
	BufferParams params;

	params.full_width = width;
	params.full_height = height;

	if(b_scene.render().use_border()) {
		/* border render */
		params.full_x = cam->border_left*width;
		params.full_y = cam->border_bottom*height;
		params.width = (int)(cam->border_right*width) - params.full_x;
		params.height = (int)(cam->border_top*height) - params.full_y;

		/* survive in case border goes out of view or becomes too small */
		params.width = max(params.width, 1);
		params.height = max(params.height, 1);
	}
	else {
		params.width = width;
		params.height = height;
	}

	return params;
}

CCL_NAMESPACE_END

