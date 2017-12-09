#include "disp_device.h"

static LIST_HEAD(device_list);

s32 disp_device_set_manager(struct disp_device* dispdev, struct disp_manager *mgr)
{
	if((NULL == dispdev)) {
		DE_WRN("NULL hdl!\n");
		return DIS_FAIL;
	}
	DE_INF("device %d, mgr %d\n", dispdev->disp, mgr->disp);

	dispdev->manager = mgr;
	mgr->device = dispdev;

	return DIS_SUCCESS;
}

s32 disp_device_unset_manager(struct disp_device* dispdev)
{
	if((NULL == dispdev)) {
		DE_WRN("NULL hdl!\n");
		return DIS_FAIL;
	}
	DE_INF("\n");

	dispdev->manager->device = NULL;
	dispdev->manager = NULL;

	return DIS_SUCCESS;
}

s32 disp_device_get_resolution(struct disp_device* dispdev, u32 *xres, u32 *yres)
{
	if((NULL == dispdev)) {
		DE_WRN("NULL hdl!\n");
		return DIS_FAIL;
	}

	*xres = dispdev->timings.x_res;
	*yres = dispdev->timings.y_res;

	return 0;
}

s32 disp_device_get_timings(struct disp_device* dispdev, disp_video_timings *timings)
{
	if((NULL == dispdev)) {
		DE_WRN("NULL hdl!\n");
		return DIS_FAIL;
	}

	if(timings)
		memcpy(timings, &dispdev->timings, sizeof(disp_video_timings));

	return 0;
}

struct disp_device* disp_device_get(int disp, disp_output_type output_type)
{
	struct disp_device* dispdev = NULL;

	DE_INF("disp=%d, type=%d\n", disp, output_type);

	list_for_each_entry(dispdev, &device_list, list) {
		DE_INF("item disp=%d, type=%d\n", dispdev->disp, dispdev->type);
		if ((dispdev->type == output_type) && (dispdev->disp == disp)
			&& (NULL == dispdev->manager)) {
			return dispdev;
		}
	}

	return NULL;
}

s32 disp_device_register(struct disp_device *dispdev)
{
	list_add_tail(&dispdev->list, &device_list);
	return 0;
}

s32 disp_device_unregister(struct disp_device *dispdev)
{
	list_del(&dispdev->list);
	return 0;
}

