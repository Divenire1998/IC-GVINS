#pragma once

#include "common/types.h"
#include "ic_gvins.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>

class FusionROS {

public:
    FusionROS() = default;

    void run();

    void setFinished();

private:
    void imuCallback(const sensor_msgs::ImuConstPtr &imumsg);

    void gnssCallback(const sensor_msgs::NavSatFixConstPtr &gnssmsg);

    void imageCallback(const sensor_msgs::ImageConstPtr &imagemsg);

private:
    std::shared_ptr<GVINS> gvins_;

    IMU imu_{.time = 0}, imu_pre_{.time = 0};
    Frame::Ptr frame_;
    GNSS gnss_;

    bool isusegnssoutage_{false};
    double gnssoutagetime_{0};
    double gnssthreshold_{20.0};

    std::queue<IMU> imu_buffer_;
    std::queue<Frame::Ptr> frame_buffer_;
};
