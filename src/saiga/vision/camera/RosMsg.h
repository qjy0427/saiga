//
// Created by jingye on 24-11-1.
//

#ifndef ROS_H
#define ROS_H

#include "saiga/vision/camera/EuRoCDataset.h"

#include <queue>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include "Snake/System/Settings.h"

namespace Saiga
{

class SAIGA_VISION_API RosMsg : public EuRoCDataset {
    bool use_stereo_ = false;
    std::vector<Imu::Data> last_imu_data;
    double last_time = 0;
    double last_imu_time = 0;
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber cam0Subscriber, cam1Subscriber;
    ros::Subscriber imuSubscriber;
    ros::AsyncSpinner spinner;
    std::string cam0_topic, cam1_topic, imu_topic;
    const Snake::Settings& settings_;

public:
    double init_global_yaw_ = 0.0;

    explicit RosMsg(const DatasetParameters& params, Sequence sequence,
        CameraInputType camera_type, const Snake::Settings& settings);

    void Load();

    int LoadMetaData() override;

    bool getImageSync(FrameData& data) override;

    std::vector<Imu::Data> GetImuSample(double curr_time);
};

}

#endif //ROS_H
