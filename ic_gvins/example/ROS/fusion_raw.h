#pragma once

#include "common/types.h"
#include "ic_gvins.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>

class FusionRaw {

public:
    FusionRaw() = default;

    void run();

    void setFinished();

private:
    // 读取图像
    void loadImage(sensor_msgs::Image &imagemsg, size_t index);
    void inputImage(size_t index);

    // 读取IMU原始数据
    void loadImuData(sensor_msgs::Imu &imumsg);
    void inputIMU();

    // 读取Gnss数据
    void LoadGnssData(sensor_msgs::NavSatFix &gnssmsg);
    void inputGnss();

private:
    std::shared_ptr<GVINS> gvins_;

    IMU imu_{.time = 0}, imu_pre_{.time = 0};
    Frame::Ptr frame_;
    double imageTimeGps_ = 0;
    GNSS gnss_;

    bool isusegnssoutage_{false};
    double gnssoutagetime_{0};
    double gnssthreshold_{20.0};
    double imu_freq_{100};
    double image_freq_{100};
    double gnss_freq_{100};

    std::queue<IMU> imu_buffer_;
    std::queue<Frame::Ptr> frame_buffer_;

    std::string strPathToSequence;
    std::vector<string> vstrImageFilenames0;
    std::vector<string> vstrImageFilenames1;
    std::vector<uint64_t> vTimestamps;

    // 文件接口
    std::ifstream imufile;
    std::ifstream vrsGpsFile;

    ros::Publisher pubRawLeftImage;
    ros::Publisher pubRawImu;
    ros::Publisher pubRawVrsGps;
};
