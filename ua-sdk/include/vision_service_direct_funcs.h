#pragma once
#include <vector>
#include <string>
#include <string.h>
#include "vision_service_direct_types.h"

//I.########################Others For Vsion########################

//1.1 Camera device
int vision_create_device_stream();
int vision_destroy_device_stream();

//int vision_mmap(ImageFrame *phy, ImageFrame *vir);
//int vision_unmap(ImageFrame *phy, ImageFrame *vir);

int vision_get_stereo_camera_distorted_frame(DeviceStream stream, MemoryType memory, ImageFrame* dist1, ImageFrame* dist2);
int vision_get_stereo_camera_rectified_frame(DeviceStream stream, MemoryType memory, ImageFrame* rect1, ImageFrame* rect2);
int vision_get_stereo_camera_disparity_frame(DeviceStream stream, MemoryType memory, ImageFrame* disp);
int vision_get_stereo_camera_distorted_rectified_frame(DeviceStream stream, MemoryType memory, ImageFrame* dist1, ImageFrame* dist2, ImageFrame* rect1, ImageFrame* rect2);
int vision_get_stereo_camera_distorted_rectified_disparity_frame(DeviceStream stream, MemoryType memory, ImageFrame* dist1, ImageFrame* dist2, ImageFrame* rect1, ImageFrame* rect2, ImageFrame* disp);

void *vision_stream_open(DeviceStream stream);
int vision_stream_get_raw_frame(void *handle, MemoryType memory, ImageFrame *raw1, ImageFrame *raw2);
int vision_stream_get_raw_rectify_frame(void *handle, MemoryType memory, ImageFrame *raw1, ImageFrame *raw2, ImageFrame* rect1, ImageFrame* rect2);
int vision_stream_get_raw_rectify_disparity_frame(void *handle, MemoryType memory, ImageFrame *raw1, ImageFrame *raw2, ImageFrame* rect1, ImageFrame* rect2, ImageFrame* disp);
int vision_stream_close(void *handle);

int vision_time_conversion_local2fcs_time(uint64_t local_time, uint64_t *fcs_time);
int vision_time_conversion_fcs2local_time(uint64_t fcs_time, uint64_t *local_time);
int64_t vision_get_system_time(TimeSystem sys);

// 1.2 History device
int vision_get_history_device_frame(DeviceStream stream, int64_t ts, int64_t radius, void* result);
int vision_get_history_device_frames(DeviceStream stream, int64_t ts_start, int64_t ts_end, void* results, int& count);

int vision_get_current_device_frame(DeviceStream stream, char* frame, int size);

// 1.3 send msg to fcs
int vision_send_msg(char* name, char* data, int len);
