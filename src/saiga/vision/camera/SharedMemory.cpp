//
// Created by jingye on 24-10-31.
//

#include "SharedMemory.h"

#include "saiga/core/util/FileSystem.h"
#include "saiga/core/util/file.h"
#include "saiga/core/util/yaml.h"

#include "glog/logging.h"

#include "../ua-sdk/include/vision_service_direct_funcs.h"
#include "../ua-sdk/include/vision_service_direct_types.h"

namespace Saiga
{

const std::string non_arm_error_msg = "This build is not for ARM, please use the ARM build to run this code.\n";

SharedMemory::SharedMemory(const DatasetParameters& _params, Sequence sequence) : EuRoCDataset(_params, sequence, false)
{
    params.preload = false;
    camera_type = CameraInputType::Stereo;
    Load();
}

void SharedMemory::Load() {
    SAIGA_ASSERT(this->camera_type != CameraInputType::Unknown);

    int num_images = LoadMetaData();
    SAIGA_ASSERT((int)frames.size() == num_images);
    ResetTime();
}

void SharedMemory::LoadImageData(FrameData& data)
{
#ifdef ARM
    SAIGA_ASSERT(data.image.rows == 0);
    DeviceStream device = DeviceStream::DEVICE_STEREO_CAMERA_BOTTOM;
    ImageFrame img1, img2;
    while (true)
    {
        int ret = vision_get_stereo_camera_distorted_frame(device, MemoryType::MEMORY_VIRT, &img1, &img2);
        if (ret == 0)
        {
            break;
        }
        std::cout << "Failed to load img from shared memory, retrying!\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    data.image.load(img1);
    data.right_image.load(img2);
#else
    std::cerr << non_arm_error_msg;
    exit(1);
#endif
}

int SharedMemory::LoadMetaData()
{
    std::cout << "Loading EuRoCDataset Stereo Dataset: " << params.dir << std::endl;

    auto leftImageSensor  = params.dir + "/cam0/sensor.yaml";
    auto rightImageSensor = params.dir + "/cam1/sensor.yaml";
    auto imuSensor        = params.dir + "/imu0/sensor.yaml";

    SAIGA_ASSERT(std::filesystem::exists(leftImageSensor));
    SAIGA_ASSERT(std::filesystem::exists(rightImageSensor));
    SAIGA_ASSERT(std::filesystem::exists(imuSensor));

    FindSequence();

    {
        // == Cam 0 ==
        // Load camera meta data
        YAML::Node config = YAML::LoadFile(leftImageSensor);
        SAIGA_ASSERT(config);
        SAIGA_ASSERT(!config.IsNull());

        VLOG(1) << config["comment"].as<std::string>();
        SAIGA_ASSERT(config["camera_model"].as<std::string>() == "pinhole");
        intrinsics.fps = config["rate_hz"].as<double>();

        // in the euroc config only the first 4 elements are used
        Vec4 kparams = readYamlMatrix<Vec4>(config["intrinsics"]);
        Vec5 kparamsfull = Vec5::Zero();
        kparamsfull.head<4>() = kparams;
        intrinsics.model.K.coeffs(kparamsfull);
        auto res               = readYamlMatrix<ivec2>(config["resolution"]);
        intrinsics.imageSize.w = res(0);
        intrinsics.imageSize.h = res(1);
        // 4 parameter rad-tan model
        Vec4 d                  = readYamlMatrix<Vec4>(config["distortion_coefficients"]);
        intrinsics.model.dis.k1 = d(0);
        intrinsics.model.dis.k2 = d(1);
        intrinsics.model.dis.p1 = d(2);
        intrinsics.model.dis.p2 = d(3);
        Mat4 m                  = readYamlMatrix<Mat4>(config["T_BS"]["data"]);
        extrinsics_cam0         = SE3::fitToSE3(m);
    }

    {
        // == Cam 1 ==
        // Load camera meta data
        YAML::Node config = YAML::LoadFile(rightImageSensor);
        VLOG(1) << config["comment"].as<std::string>();
        SAIGA_ASSERT(config["camera_model"].as<std::string>() == "pinhole");

        // in the euroc config only the first 4 elements are used
        Vec4 kparams = readYamlMatrix<Vec4>(config["intrinsics"]);
        Vec5 kparamsfull = Vec5::Zero();
        kparamsfull.head<4>() = kparams;
        intrinsics.rightModel.K.coeffs(kparamsfull);
        /// intrinsics.rightModel.K.coeffs(readYamlMatrix<Vec5>(config["intrinsics"]));


        auto res                    = readYamlMatrix<ivec2>(config["resolution"]);
        intrinsics.rightImageSize.w = res(0);
        intrinsics.rightImageSize.h = res(1);
        // 4 parameter rad-tan model
        Vec4 d                       = readYamlMatrix<Vec4>(config["distortion_coefficients"]);
        intrinsics.rightModel.dis.k1 = d(0);
        intrinsics.rightModel.dis.k2 = d(1);
        intrinsics.rightModel.dis.p1 = d(2);
        intrinsics.rightModel.dis.p2 = d(3);
        Mat4 m                       = readYamlMatrix<Mat4>(config["T_BS"]["data"]);
        extrinsics_cam1              = SE3::fitToSE3(m);
    }

    {
        // == IMU ==
        // Load camera meta data
        YAML::Node config = YAML::LoadFile(imuSensor);
        VLOG(1) << config["comment"].as<std::string>();
        Mat4 m = readYamlMatrix<Mat4>(config["T_BS"]["data"]);


        imu                 = Imu::Sensor();
        imu->sensor_to_body = SE3::fitToSE3(m);

        imu->frequency                = config["rate_hz"].as<double>();
        imu->frequency_sqrt           = sqrt(imu->frequency);
        imu->omega_sigma              = config["gyroscope_noise_density"].as<double>();
        imu->omega_random_walk        = config["gyroscope_random_walk"].as<double>();
        imu->acceleration_sigma       = config["accelerometer_noise_density"].as<double>();
        imu->acceleration_random_walk = config["accelerometer_random_walk"].as<double>();

        VLOG(1) << *imu;
    }

    auto vicon0_file = params.dir + "/" + "vicon0/sensor.yaml";
    auto leica0_file = params.dir + "/" + "leica0/sensor.yaml";

    if (use_raw_gt_data && std::filesystem::exists(vicon0_file))
    {
        YAML::Node config = YAML::LoadFile(vicon0_file);
        Mat4 m            = readYamlMatrix<Mat4>(config["T_BS"]["data"]);
        extrinsics_gt     = SE3::fitToSE3(m);


        auto data_file = params.dir + "/" + "vicon0/data.csv";

        auto lines = File::loadFileStringArray(data_file);
        StringViewParser csvParser(", ");
        for (auto&& l : lines)
        {
            if (l.empty()) continue;
            if (l[0] == '#') continue;
            csvParser.set(l);

            auto svTime = csvParser.next();
            if (svTime.empty()) continue;

            Vec3 data;
            for (int i = 0; i < 3; ++i)
            {
                auto sv = csvParser.next();
                SAIGA_ASSERT(!sv.empty());
                data(i) = to_double(sv);
            }

            Vec4 dataq;
            for (int i = 0; i < 4; ++i)
            {
                auto sv = csvParser.next();
                SAIGA_ASSERT(!sv.empty());
                dataq(i) = to_double(sv);
            }

            Quat q;
            q.x() = dataq(0);
            q.y() = dataq(1);
            q.z() = dataq(2);
            q.w() = dataq(3);


            auto time = to_double(svTime) / 1e9;

            time += params.ground_truth_time_offset;
            ground_truth.emplace_back(time, SE3(q, data));
        }
    }
    else if (use_raw_gt_data && std::filesystem::exists(leica0_file))
    {
        YAML::Node config = YAML::LoadFile(leica0_file);
        Mat4 m            = readYamlMatrix<Mat4>(config["T_BS"]["data"]);
        extrinsics_gt     = SE3::fitToSE3(m);

        auto data_file = params.dir + "/" + "leica0/data.csv";

        auto lines = File::loadFileStringArray(data_file);
        StringViewParser csvParser(", ");
        for (auto&& l : lines)
        {
            if (l.empty()) continue;
            if (l[0] == '#') continue;
            csvParser.set(l);

            auto svTime = csvParser.next();
            if (svTime.empty()) continue;

            Vec3 data;
            for (int i = 0; i < 3; ++i)
            {
                auto sv = csvParser.next();
                SAIGA_ASSERT(!sv.empty());
                data(i) = to_double(sv);
            }
            auto time = to_double(svTime) / 1e9;

            time += params.ground_truth_time_offset;
            ground_truth.emplace_back(time, SE3(Quat::Identity(), data));

            //            std::cout << "gt " << ground_truth.back().second << std::endl;
        }
        // extrinsics_gt = SE3();
        std::cout << "extr " << extrinsics_gt << std::endl;
    }
    else
    {
        //        SAIGA_EXIT_ERROR("no gt file");
        // == Ground truth position ==
        auto sensorFile = params.dir + "/" + "state_groundtruth_estimate0/data.csv";


        auto lines = File::loadFileStringArray(sensorFile);
        StringViewParser csvParser(", ");
        for (auto&& l : lines)
        {
            if (l.empty()) continue;
            if (l[0] == '#') continue;
            csvParser.set(l);

            auto svTime = csvParser.next();
            if (svTime.empty()) continue;

            Vec3 data;
            for (int i = 0; i < 3; ++i)
            {
                auto sv = csvParser.next();
                // std::cout<<"sv p: "<<sv<<std::endl;
                SAIGA_ASSERT(!sv.empty());
                data(i) = to_double(sv);
            }

            Vec4 dataq;
            for (int i = 0; i < 4; ++i)
            {
                auto sv = csvParser.next();
                // std::cout<<"sv q: "<<sv<<std::endl;
                SAIGA_ASSERT(!sv.empty());
                dataq(i) = to_double(sv);
            }

            Quat q;
            q.x() = dataq(0);
            q.y() = dataq(1);
            q.z() = dataq(2);
            q.w() = dataq(3);


            auto time = to_double(svTime) / 1e9;

            time += params.ground_truth_time_offset;
            ground_truth.emplace_back(time, SE3(q, data));
        }

        // YAML::Node config = YAML::LoadFile(params.dir + "/state_groundtruth_estimate0/sensor.yaml");
        // Mat4 m            = readYamlMatrix<Mat4>(config["T_BS"]["data"]);
        // extrinsics_gt     = SE3::fitToSE3(m);
    }


    std::sort(ground_truth.begin(), ground_truth.end(), [](auto a, auto b) { return a.first < b.first; });

    // SE3 groundTruthToCamera = extrinsics_gt.inverse() * extrinsics_cam0;
    intrinsics.camera_to_gt   = extrinsics_cam0.inverse() * extrinsics_gt;
    intrinsics.camera_to_body = extrinsics_cam0.inverse();

    std::cout << "Camera -> GT  : " << intrinsics.camera_to_gt << std::endl;
    std::cout << "Camera -> Body: " << intrinsics.camera_to_body << std::endl;



    intrinsics.left_to_right = extrinsics_cam1.inverse() * extrinsics_cam0;
    intrinsics.maxDepth      = 35;
    intrinsics.bf            = intrinsics.left_to_right.translation().norm() * intrinsics.model.K.fx;
    //    std::cout << "Left->Right: " << intrinsics.left_to_right << std::endl;

    {
        // == Imu Data ==
        // Format:
        //   timestamp [ns],
        //   w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],
        //   a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
        auto sensorFile = params.dir + "/" + "imu0/data.csv";
        auto lines      = File::loadFileStringArray(sensorFile);
        StringViewParser csvParser(", ");

        for (auto&& l : lines)
        {
            if (l.empty()) continue;
            if (l[0] == '#') continue;
            csvParser.set(l);

            auto svTime = csvParser.next();
            if (svTime.empty()) continue;
            // time is given in nano seconds
            auto time = to_double(svTime) / 1e9;

            Vec3 omega;
            for (int i = 0; i < 3; ++i)
            {
                auto sv = csvParser.next();
                SAIGA_ASSERT(!sv.empty());
                omega(i) = to_double(sv);
            }

            Vec3 acceleration;
            for (int i = 0; i < 3; ++i)
            {
                auto sv = csvParser.next();
                SAIGA_ASSERT(!sv.empty());
                acceleration(i) = to_double(sv);
            }
            imuData.emplace_back(omega, acceleration, time);
        }
    }

    //    std::cout << std::setprecision(20);
    //    std::cout << "First image at " << cam0_images.front().first << " " << cam1_images.front().first << " First IMU
    //    at "
    //              << imuData.front().timestamp << " First GT at " << ground_truth.front().first << std::endl;


    std::cout << "Found " << cam1_images.size() << " images and " << ground_truth.size()
              << " ground truth meassurements and " << imuData.size() << " IMU meassurements." << std::endl;

    SAIGA_ASSERT(intrinsics.imageSize == intrinsics.rightImageSize);
    VLOG(1) << intrinsics;

    return 0;
}

std::vector<Imu::Data> SharedMemory::GetImuSample(const double curr_time)
{
#ifdef ARM
    int imu_num = 0;
    AttFrame imu_datas[1024];
    while (true) {
        int ret = vision_get_history_device_frames(DeviceStream::DEVICE_ATT,
            last_imu_time * 1e9, (curr_time + 0.01) * 1e9, imu_datas, imu_num);
        if (ret == 0)
        {
            break;
        }
        std::cout << "Failed to load IMU from shared memory, retrying!\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::vector<Imu::Data> vec_imu_data;
    for (int i = 0; i < imu_num; ++i)
    {
        Imu::Data result;
        result.omega = Vec3(imu_datas[i].imu_gyro[0], imu_datas[i].imu_gyro[1], imu_datas[i].imu_gyro[2]);
        result.acceleration = Vec3(imu_datas[i].imu_accel[0], imu_datas[i].imu_accel[1], imu_datas[i].imu_accel[2]);
        result.timestamp = imu_datas[i].timestamp / 1e9;
        last_imu_time = result.timestamp;
        vec_imu_data.emplace_back(result);
    }
    return vec_imu_data;
#else
    std::cerr << non_arm_error_msg;
    exit(1);
#endif
}

bool SharedMemory::getImageSync(FrameData& data)
{
#ifdef ARM
    data.id = currentId++;

    SAIGA_ASSERT(data.image.rows == 0);
    DeviceStream device = DeviceStream::DEVICE_STEREO_CAMERA_BOTTOM;
    ImageFrame img1, img2;
    while (true)
    {
        int ret = vision_get_stereo_camera_distorted_frame(device, MemoryType::MEMORY_VIRT, &img1, &img2);
        if (ret == 0)
        {
            break;
        }
        std::cout << "Failed to load img from shared memory, retrying!\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    data.image.load(img1);
    data.right_image.load(img2);

    data.timeStamp = img1.timestamp / 1e9;

    std::vector<Imu::Data> imu_data = GetImuSample(data.timeStamp);
    //    std::cout << "imu data size: " << imu_data.size() << " Imu frequency: " << imu_data.size() * 30 << std::endl;


    // Add last 2 samples at the front
    if (last_imu_data.size() >= 2)
    {
        imu_data.insert(imu_data.begin(), last_imu_data.end() - 2, last_imu_data.end());
        data.imu_data.data       = imu_data;
        data.imu_data.time_begin = last_time;
        data.imu_data.time_end   = data.timeStamp;

        data.imu_data.FixBorder();
        SAIGA_ASSERT(data.imu_data.Valid());
        SAIGA_ASSERT(data.imu_data.complete());
    }

    last_imu_data = imu_data;
    last_time     = data.timeStamp;
    std::cout << "Successfully loaded imgs and IMUs.\n";
    return true;
#else
    std::cerr << non_arm_error_msg;
    exit(1);
#endif
}

}  // namespace Saiga
