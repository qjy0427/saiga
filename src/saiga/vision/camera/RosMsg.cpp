//
// Created by jingye on 24-11-1.
//

#include "RosMsg.h"

#include "saiga/core/util/FileSystem.h"
#include "saiga/core/util/file.h"
#include "saiga/core/util/yaml.h"

#include "glog/logging.h"

namespace Saiga
{

std::queue<sensor_msgs::ImageConstPtr> image_queue0;
std::queue<sensor_msgs::ImageConstPtr> image_queue1;
std::queue<sensor_msgs::Imu> imuQueue;

std::mutex mtx0;
std::mutex mtx1;
std::mutex mtx_imu;
std::condition_variable cond_var0;
std::condition_variable cond_var1;
std::condition_variable cond_var_imu;
double latest_imu_time = 0;

void cam0Callback(const sensor_msgs::ImageConstPtr& msg) {
    if (msg->height == 0 || msg->width == 0) {
        LOG(ERROR) << "Image size is 0!";
        return;
    }
    std::lock_guard lock(mtx0);
    image_queue0.push(msg);
    cond_var0.notify_one();
}

void cam1Callback(const sensor_msgs::ImageConstPtr& msg) {
    if (msg->height == 0 || msg->width == 0) {
        LOG(ERROR) << "Image size is 0!";
        return;
    }
    std::lock_guard lock(mtx1);
    image_queue1.push(msg);
    cond_var1.notify_one();
}

void imuCallback(const sensor_msgs::Imu& msg) {
    std::lock_guard lock(mtx_imu);
    imuQueue.push(msg);
    latest_imu_time = msg.header.stamp.toSec();
    cond_var_imu.notify_one();
}

RosMsg::RosMsg(const DatasetParameters& _params, Sequence sequence, CameraInputType camera_type)
    :
EuRoCDataset(_params, sequence, false, camera_type), imageTransport(nodeHandle), spinner(3)
{
    params.preload = false;
    Load();

    cam0Subscriber = imageTransport.subscribe("/zhz/driver/cam4/image_raw", 1, &cam0Callback);
    if (camera_type_ == CameraInputType::Stereo)
    {
        LOG(INFO) << "Using stereo mode!";
        use_stereo_ = true;
        cam1Subscriber = imageTransport.subscribe("/zhz/driver/cam5/image_raw", 1, &cam1Callback);
    }
    imuSubscriber = nodeHandle.subscribe("/mavros/imu/ned_data", 10, &imuCallback);

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

double findNearestElement(const std::vector<double>& sorted_vec, double target) {
    if (sorted_vec.empty()) {
        LOG(ERROR) << "sorted_vec is empty!";
        return target;
    }
    auto lower = std::lower_bound(
        sorted_vec.begin(), sorted_vec.end(), target);

    if (lower == sorted_vec.end()) {
        return sorted_vec.back();
    }
    if (lower == sorted_vec.begin()) {
        return *lower;
    }

    auto prev = lower - 1;
    if (std::abs(*prev - target) <= std::abs(*lower - target)) {
        return *prev;
    }
    return *lower;
}

std::vector<double> getTimestamps(const std::vector<Imu::Data>& imu_data) {
    std::vector<double> timestamps;
    for (const auto& data : imu_data) {
        timestamps.push_back(data.timestamp);
    }
    return timestamps;
}

bool RosMsg::getImageSync(FrameData& data)
{
    data.id = currentId++;

    SAIGA_ASSERT(data.image.rows == 0);
    if (image_queue0.empty() || (use_stereo_ && image_queue1.empty())) {
        std::unique_lock lock0(mtx0);
        cond_var0.wait(lock0, []{ return !image_queue0.empty(); });
        lock0.unlock();  // Don't forget to unlock so image_queue0 can continue being filled!
        if (use_stereo_)
        {
            std::unique_lock lock1(mtx1);
            cond_var1.wait(lock1, []
            {
                if (image_queue1.empty())
                {
                    return false;
                }
                while (!image_queue0.empty() &&
                    image_queue0.front()->header.stamp.toSec() < image_queue1.front()->header.stamp.toSec())
                {
                    image_queue0.pop();
                }
                if (image_queue0.empty())
                {
                    return false;
                }

                while (!image_queue1.empty() &&
                    image_queue1.front()->header.stamp.toSec() < image_queue0.front()->header.stamp.toSec())
                {
                    image_queue1.pop();
                }
                return !image_queue1.empty();
            });
        }
    }
    while (!image_queue0.empty() && (!use_stereo_ || !image_queue1.empty())) {
        const sensor_msgs::ImageConstPtr& img0 = image_queue0.front();
        const double img0_time = img0->header.stamp.toSec();
        if (use_stereo_)
        {
            const sensor_msgs::ImageConstPtr& img1 = image_queue1.front();
            const double img1_time = img1->header.stamp.toSec();
            if (img0_time < img1_time) {
                std::cout << "img0_time: " << img0_time << " < img1_time: " << img1_time << "\n";
                image_queue0.pop();
            } else if (img0_time > img1_time) {
                std::cout << "img0_time: " << img0_time << " > img1_time: " << img1_time << "\n";
                image_queue1.pop();
            } else {
                data.image.load(img0);
                data.right_image.load(img1);
                data.timeStamp = img0_time;
                image_queue0.pop();
                image_queue1.pop();
                break;
            }
        } else
        {
            data.image.load(img0);
            data.timeStamp = img0_time;
            image_queue0.pop();
            break;
        }
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

