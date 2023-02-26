/*
 * main.cc
 *
 *  Created on: 22/7/8
 *      Author: hailiang
 */

#include <glog/logging.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <absl/strings/numbers.h>
#include <absl/strings/str_format.h>
#include <absl/strings/str_split.h>
#include <opencv2/opencv.hpp>

#include <irp_sen_msgs/vrs.h>

#include "gpstime.h"

using namespace cv;
using namespace std;

int main(int argc, char *argv[]) {
    // glog初始化
    FLAGS_logtostderr = true;
    FLAGS_logtostderr = true;
    google::InitGoogleLogging(argv[0]);

    if (argc != 3) {
        LOG(ERROR) << "Usage: kaist_bag_fix rdbag wrbag gnsssfile";
        return -1;
    }

    rosbag::Bag rdbag(argv[1], rosbag::bagmode::Read);
    rosbag::Bag wrbag(argv[2], rosbag::bagmode::Write);

    vector<string> splits = absl::StrSplit(argv[2], ".");
    string image_stampfile_path = splits[0] + "_image.txt";
    string vlp_stampfile_path   = splits[0] + "_vlp.txt";

    ofstream image_stampfile(image_stampfile_path);
    ofstream vlp_stampfile(vlp_stampfile_path);

    // 订阅消息
    vector<string> topics;
    topics.emplace_back("/stereo/left/image_raw");
    topics.emplace_back("/imu/data_raw");
    topics.emplace_back("/vrs_gps_data");
    topics.emplace_back("/ns2/velodyne_points");
    rosbag::View view(rdbag, rosbag::TopicQuery(topics));

    int index         = 0;
    double last_stamp = 0;
    for (rosbag::MessageInstance const msg : view) {
        sensor_msgs::Image::ConstPtr image_ptr = msg.instantiate<sensor_msgs::Image>();
        if (image_ptr != nullptr) {
            LOG_EVERY_N(INFO, 100) << "Process " << index << " images";
            index++;

            // 检查时标
            if (image_ptr->header.stamp.toSec() - last_stamp < 0.01) {
                last_stamp = image_ptr->header.stamp.toSec();
                LOG(WARNING) << "Repeated stamp " << std::setprecision(16) << last_stamp;
                continue;
            }
            last_stamp = image_ptr->header.stamp.toSec();

            Mat raw(static_cast<int>(image_ptr->height), static_cast<int>(image_ptr->width), CV_8UC1,
                    (void *) image_ptr->data.data());
            Mat gray;
            cv::cvtColor(raw, gray, cv::COLOR_BayerBG2GRAY);

            // 彩色转为灰度
            sensor_msgs::Image imagemsg;
            imagemsg.header   = image_ptr->header;
            imagemsg.height   = gray.rows;
            imagemsg.width    = gray.cols;
            imagemsg.encoding = sensor_msgs::image_encodings::MONO8;
            imagemsg.step     = imagemsg.width;
            size_t size       = imagemsg.height * imagemsg.width;
            imagemsg.data.resize(size);
            memcpy(imagemsg.data.data(), gray.data, size);

            wrbag.write("/cam0", image_ptr->header.stamp, imagemsg);

            uint64_t unixsecond;
            unixsecond = static_cast<uint64_t>(image_ptr->header.stamp.sec * 1e9 + image_ptr->header.stamp.nsec);
            auto line  = absl::StrFormat("%-15.9lf %llu.png\n", image_ptr->header.stamp.toSec(), unixsecond);
            image_stampfile << line;
        }

        // IMU
        sensor_msgs::ImuConstPtr imu_ptr = msg.instantiate<sensor_msgs::Imu>();
        if (imu_ptr != nullptr) {
            sensor_msgs::Imu imumsg = *imu_ptr;

            imumsg.angular_velocity.y *= -1;
            imumsg.angular_velocity.z *= -1;
            imumsg.linear_acceleration.y *= -1;
            imumsg.linear_acceleration.z *= -1;

            wrbag.write("/imu0", imumsg.header.stamp, imumsg);
        }

        // GNSS
        irp_sen_msgs::vrsConstPtr vrs_ptr = msg.instantiate<irp_sen_msgs::vrs>();
        if (vrs_ptr != nullptr) {
            sensor_msgs::NavSatFix navsatfix;
            sensor_msgs::NavSatStatus navsatstate;
            navsatstate.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
            navsatstate.service = sensor_msgs::NavSatStatus::SERVICE_GPS + sensor_msgs::NavSatStatus::SERVICE_GLONASS;

            navsatfix.header = vrs_ptr->header;

            navsatfix.status    = navsatstate;
            navsatfix.latitude  = vrs_ptr->latitude;
            navsatfix.longitude = vrs_ptr->longitude;
            navsatfix.altitude  = vrs_ptr->altitude;

            navsatfix.position_covariance_type = sensor_msgs::NavSatFix ::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            // ENU
            navsatfix.position_covariance[0] = vrs_ptr->lon_std * vrs_ptr->lon_std;
            navsatfix.position_covariance[4] = vrs_ptr->lat_std * vrs_ptr->lat_std;
            navsatfix.position_covariance[8] = vrs_ptr->altitude_std * vrs_ptr->altitude_std;

            wrbag.write("/gnss0", navsatfix.header.stamp, navsatfix);
        }

        sensor_msgs::PointCloud2Ptr vlp_ptr = msg.instantiate<sensor_msgs::PointCloud2>();
        if (vlp_ptr != nullptr) {
            wrbag.write("/velodyne_points", vlp_ptr->header.stamp, vlp_ptr);

            auto line = absl::StrFormat("%-15.9lf\n", vlp_ptr->header.stamp.toSec());
            vlp_stampfile << line;
        }
    }
    LOG(INFO) << "Total " << index << " images";

    rdbag.close();
    wrbag.close();

    sleep(5);

    return 0;
}
