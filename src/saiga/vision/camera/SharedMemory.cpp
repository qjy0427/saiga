//
// Created by jingye on 24-10-31.
//

#include "SharedMemory.h"

#include "saiga/core/util/FileSystem.h"
#include "saiga/core/util/file.h"
#include "saiga/core/util/yaml.h"

#include "../ua-sdk/include/vision_service_direct_funcs.h"
#include "../ua-sdk/include/vision_service_direct_types.h"

namespace Saiga
{

const std::string non_arm_error_msg = "This build is not for ARM, please use the ARM build to run this code.\n";

SharedMemory::SharedMemory(const DatasetParameters& _params, Sequence sequence) : EuRoCDataset(_params, sequence, false)
{
    params.preload = false;
    camera_type_ = CameraInputType::Stereo;
    Load();
#ifdef ARM
    // open msg receiver
    int ret = vision_create_device_stream();  // 0: success; -1: failed
    if (ret) {
        printf("vision_create_device_stream failed.\n");
        exit(1);
    }
#else
    std::cerr << non_arm_error_msg;
    exit(2);
#endif
}

void SharedMemory::Load() {
    SAIGA_ASSERT(this->camera_type_ != CameraInputType::Unknown);

    int num_images = LoadMetaData();
    SAIGA_ASSERT((int)frames.size() == num_images);
    ResetTime();
}

int SharedMemory::LoadMetaData()
{
    LoadMostMetaData();
    return 0;
}

std::vector<Imu::Data> SharedMemory::GetImuSample(const double curr_time)
{
#ifdef ARM
    int imu_num = 1024;
    AttFrame imu_datas[1024];
    // Increase std::cout precision
    std::cout << std::setprecision(10);
    if (last_time < curr_time - 1)
    {
        std::cout << "Too large time gap detected, last_time: " << last_time << " , curr_time: " << curr_time << std::endl;
    }
    int tried = 0;
    while (tried++ < 50) {
        std::cout << "Getting IMU from last_time: " << last_time << " to curr_time: " << curr_time << std::endl;
        int ret = vision_get_history_device_frames(DeviceStream::DEVICE_ATT,
            static_cast<int64_t>(last_time * 1e9), static_cast<int64_t>(curr_time * 1e9),
            imu_datas, imu_num);
        if (ret == 0)
        {
            break;
        }
        std::cout << "Failed to load IMU from shared memory, retrying! Returned value = " << ret << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "Got " << imu_num << " IMU samples.\n";
    std::vector<Imu::Data> vec_imu_data;
    for (int i = 0; i < imu_num; ++i)
    {
        Imu::Data result;
        result.omega = Vec3(imu_datas[i].imu_gyro[0], imu_datas[i].imu_gyro[1], imu_datas[i].imu_gyro[2]);
        result.acceleration = Vec3(imu_datas[i].imu_accel[0], imu_datas[i].imu_accel[1], imu_datas[i].imu_accel[2]);
        result.timestamp = imu_datas[i].timestamp / 1e9;
        vec_imu_data.emplace_back(result);
    }
    return vec_imu_data;
#else
    std::cerr << non_arm_error_msg;
    exit(4);
#endif
}

bool SharedMemory::getImageSync(FrameData& data)
{
#ifdef ARM
    data.id = currentId++;

    SAIGA_ASSERT(data.image.rows == 0);
    DeviceStream device = DeviceStream::DEVICE_STEREO_CAMERA_BOTTOM;
    ImageFrame img1, img2;
    int tried = 0;
    while (tried++ < 50)
    {
        int ret = vision_get_stereo_camera_distorted_frame(device, MemoryType::MEMORY_VIRT, &img1, &img2);
        if (ret == 0)
        {
            break;
        }
        std::cout << "Failed to getImageSync from shared memory, retrying!\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    data.image.load(img1);
    data.right_image.load(img2);

    data.timeStamp = img1.timestamp / 1e9;

    std::vector<Imu::Data> imu_data = GetImuSample(data.timeStamp);
    std::cout << "imu data size: " << imu_data.size() << std::endl;

    if (!imu_data.empty()) {
        data.timeStamp = findNearestElement(getTimestamps(imu_data), data.timeStamp);
    }


    // Add last 2 samples at the front
    if (last_imu_data.size() >= 2 && last_imu_data[last_imu_data.size() - 1].timestamp >= last_time)
    {
        imu_data.insert(imu_data.begin(), last_imu_data.end() - 2, last_imu_data.end());
    }
    data.imu_data.data       = imu_data;
    data.imu_data.time_begin = last_time;
    data.imu_data.time_end   = data.timeStamp;
    data.imu_data.FixBorder();
    SAIGA_ASSERT(data.imu_data.Valid());

    last_imu_data = imu_data;
    last_time     = data.timeStamp;
    std::cout << "Successfully loaded imgs and IMUs.\n";
    return true;
#else
    std::cerr << non_arm_error_msg;
    exit(5);
#endif
}

}  // namespace Saiga
