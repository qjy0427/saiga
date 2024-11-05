//
// Created by jingye on 24-11-1.
//

#include "RosMsg.h"

#include "saiga/core/util/FileSystem.h"
#include "saiga/core/util/file.h"
#include "saiga/core/util/yaml.h"

#include "glog/logging.h"
#include "nav_msgs/Odometry.h"

namespace Saiga
{

constexpr int kImgQueueSizeThreshold = 0;

std::queue<sensor_msgs::ImageConstPtr> image_queue0;
std::queue<sensor_msgs::ImageConstPtr> image_queue1;
std::queue<sensor_msgs::Imu> imuQueue;

std::mutex mtx0;
std::mutex mtx1;
std::mutex mtx_imu;
std::mutex yaw_mtx;
std::condition_variable cond_var0;
std::condition_variable cond_var1;
std::condition_variable cond_var_imu;
std::condition_variable cond_var_yaw;
double latest_imu_time = 0;
double init_global_yaw = 0.0;
bool yawReceived = false;

void cam0Callback(const sensor_msgs::ImageConstPtr& msg) {
    std::lock_guard lock(mtx0);
    if (image_queue0.size() > kImgQueueSizeThreshold)
    {
        // LOG(WARNING) << "image queue size too large, omitting!";
        image_queue0.pop();
    }
    if (msg->height == 0 || msg->width == 0) {
        LOG(ERROR) << "Image size is 0!";
        return;
    }
    image_queue0.push(msg);
    cond_var0.notify_one();
}

void cam1Callback(const sensor_msgs::ImageConstPtr& msg) {
    std::lock_guard lock(mtx1);
    if (image_queue1.size() > kImgQueueSizeThreshold)
    {
        // LOG(WARNING) << "image queue size too large, omitting!";
        image_queue1.pop();
    }
    if (msg->height == 0 || msg->width == 0) {
        LOG(ERROR) << "Image size is 0!";
        return;
    }
    image_queue1.push(msg);
    cond_var1.notify_one();
}

void imuCallback(const sensor_msgs::Imu& msg) {
    std::lock_guard lock(mtx_imu);
    imuQueue.push(msg);
    latest_imu_time = msg.header.stamp.toSec();
    cond_var_imu.notify_one();
}

//Quat --> Euler(Z-Y-X) (pitch range is limited to [-pi/2, pi/2])
Eigen::Vector3d Quaterniond2EulerAngles(const Eigen::Quaterniond& q) {
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(2) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(0) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

void mavrosOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(yaw_mtx);
    const auto& q = msg->pose.pose.orientation;
    const Eigen::Quaterniond enu_quat(q.w, q.x, q.y, q.z);
    const Eigen::Vector3d eulerAngles =
        Quaterniond2EulerAngles(enu_quat);  // ZYX order
    const double globalYaw = eulerAngles[0] * 180.0 / M_PI;
    init_global_yaw = globalYaw;
    LOG(INFO) << "parameter globalYaw set to " << globalYaw;
    yawReceived = true;
    cond_var_yaw.notify_one();
}

RosMsg::RosMsg(const DatasetParameters& _params, Sequence sequence, CameraInputType camera_type,
    const Snake::Settings& settings)
    :
EuRoCDataset(_params, sequence, false, camera_type), imageTransport(nodeHandle), spinner(3),
settings_(settings)
{
    params.preload = false;
    Load();

    const std::string mavrosLocalOdometryTopicName = settings_.mavros_odom_ros_topic;
    if (settings_.enable_mavros_yaw_init && !mavrosLocalOdometryTopicName.empty()) {
        ros::Subscriber sub = nodeHandle.subscribe(mavrosLocalOdometryTopicName,
            10, &mavrosOdometryCallback);
        std::unique_lock<std::mutex> lock(yaw_mtx);
        ros::AsyncSpinner spinner(1);
        spinner.start();
        LOG(INFO) << "Waiting for yaw from " << mavrosLocalOdometryTopicName.c_str();
        cond_var_yaw.wait(lock, [] {
            return yawReceived;
        });
        init_global_yaw_ = init_global_yaw;
        sub.shutdown();
        spinner.stop();
    }

    cam0Subscriber = imageTransport.subscribe(settings_.left_cam_ros_topic, 1, &cam0Callback);
    LOG(INFO) << "Cam0 subscribed " << settings_.left_cam_ros_topic;
    if (camera_type_ == CameraInputType::Stereo)
    {
        LOG(INFO) << "Using stereo mode!";
        use_stereo_ = true;
        cam1Subscriber = imageTransport.subscribe(settings_.right_cam_ros_topic, 1, &cam1Callback);
        LOG(INFO) << "Cam1 subscribed " << settings_.right_cam_ros_topic;
    }
    imuSubscriber = nodeHandle.subscribe(settings_.imu_ros_topic, 10, &imuCallback);
    LOG(INFO) << "IMU subscribed " << settings_.imu_ros_topic;

    LOG(INFO) << "SPINNING";
    spinner.start();
}

void RosMsg::Load() {
    SAIGA_ASSERT(this->camera_type_ != CameraInputType::Unknown);

    int num_images = LoadMetaData();
    SAIGA_ASSERT((int)frames.size() == num_images);
    ResetTime();
}

int RosMsg::LoadMetaData()
{
    LoadMostMetaData();
    return 0;
}

std::vector<Imu::Data> RosMsg::GetImuSample(const double curr_time)
{
    // Increase std::cout precision
    std::cout << std::setprecision(10);
    if (last_time < curr_time - 1)
    {
        last_time = curr_time - 1;
        std::cout << "Too large time gap detected, last_time: " << last_time << std::endl;
    }
    std::vector<Imu::Data> vec_imu_data;
    while (vec_imu_data.size() < 2 || vec_imu_data.back().timestamp < curr_time) {
        if (latest_imu_time < curr_time || imuQueue.size() < 2) {
            std::unique_lock lock(mtx_imu);
            cond_var_imu.wait(lock, [curr_time]{ return latest_imu_time >= curr_time && imuQueue.size() >= 2; });
        }
        const sensor_msgs::Imu& imu = imuQueue.front();
        const double imu_time = imu.header.stamp.toSec();
        if (imu_time < last_time) {
            std::unique_lock lock(mtx_imu);
            imuQueue.pop();
            continue;
        }
        Imu::Data result;
        result.omega = Vec3(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
        result.acceleration = Vec3(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
        result.timestamp = imu_time;
        vec_imu_data.emplace_back(result);
        std::unique_lock lock(mtx_imu);
        imuQueue.pop();
        if (imu_time >= curr_time && vec_imu_data.size() >= 2) {
            break;
        }
    }
    return vec_imu_data;
}

bool RosMsg::getImageSync(FrameData& data)
{
    data.id = currentId++;

    SAIGA_ASSERT(data.image.rows == 0);
    sensor_msgs::ImageConstPtr image0;
    sensor_msgs::ImageConstPtr image1;
    std::unique_lock lock0(mtx0);
    cond_var0.wait(lock0, []{ return !image_queue0.empty(); });
    image0 = image_queue0.front();
    image_queue0.pop();
    lock0.unlock();  // Don't forget to unlock so image_queue0 can continue being filled!
    if (use_stereo_)
    {
        std::unique_lock lock1(mtx1);
        cond_var1.wait(lock1, []
        {
            return !image_queue1.empty();
        });
        image1 = image_queue1.front();
        image_queue1.pop();
    }

    const double img0_time = image0->header.stamp.toSec();
    if (use_stereo_)
    {
        const double img1_time = image1->header.stamp.toSec();
        if (fabs(img0_time - img1_time) < 1e-9) {
            data.image.load(image0);
            data.right_image.load(image1);
            data.timeStamp = img0_time;
        } else
        {
            return false;
        }
    } else
    {
        data.image.load(image0);
        data.timeStamp = img0_time;
    }

    std::vector<Imu::Data> imu_data = GetImuSample(data.timeStamp);
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
    return true;
}

}  // namespace Saiga

