/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "fusion_ros.h"
#include "drawer_rviz.h"

#include "common/angle.h"
#include "common/gpstime.h"
#include "common/logging.h"
#include "misc/misc.h"
#include "tracking/frame.h"

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <sensor_msgs/image_encodings.h>

#include <atomic>
#include <csignal>
#include <memory>

std::atomic<bool> isfinished{false};

void sigintHandler(int sig);
void checkStateThread(std::shared_ptr<FusionROS> fusion);

/**
 *  安全关闭gvins
 */
void FusionROS::setFinished() {
    if (gvins_ && gvins_->isRunning()) {
        gvins_->setFinished();
    }
}

/**
 *  修改话题，设置必要的参数，
 */
void FusionROS::run() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // message topic
    string imu_topic, gnss_topic, image_topic, livox_topic;
    pnh.param<string>("imu_topic", imu_topic, "/imu0");
    pnh.param<string>("gnss_topic", gnss_topic, "/gnss0");
    pnh.param<string>("image_topic", image_topic, "/cam0");

    // GVINS parameter
    string configfile;
    pnh.param<string>("configfile", configfile, "gvins.yaml");

    // Load configurations
    YAML::Node config;
    std::vector<double> vecdata;
    try {
        config = YAML::LoadFile(configfile);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return;
    }

    auto outputpath        = config["outputpath"].as<string>();
    auto is_make_outputdir = config["is_make_outputdir"].as<bool>();

    // Create the output directory
    if (!boost::filesystem::is_directory(outputpath)) {
        boost::filesystem::create_directory(outputpath);
    }
    if (!boost::filesystem::is_directory(outputpath)) {
        std::cout << "Failed to open outputpath" << std::endl;
        return;
    }

    // creat result dir use absl for daily
    // 使用absl和boost创建日志保存路径
    if (is_make_outputdir) {
        absl::CivilSecond cs = absl::ToCivilSecond(absl::Now(), absl::LocalTimeZone());
        absl::StrAppendFormat(&outputpath, "/T%04d%02d%02d%02d%02d%02d", cs.year(), cs.month(), cs.day(), cs.hour(),
                              cs.minute(), cs.second());
        boost::filesystem::create_directory(outputpath);
    }

    // GNSS outage configurations
    // 根据这个来决定是否添加gnss信息到buffer中
    isusegnssoutage_ = config["isusegnssoutage"].as<bool>();
    gnssoutagetime_  = config["gnssoutagetime"].as<double>();
    gnssthreshold_   = config["gnssthreshold"].as<double>();

    // set Glog output path
    // 设置glog的日志输出路径
    FLAGS_log_dir = outputpath;

    // The GVINS object
    // 启动gvins，处理传感器数据，并使用ROS进行显示。
    // 多态
    Drawer::Ptr drawer = std::make_shared<DrawerRviz>(nh);
    gvins_             = std::make_shared<GVINS>(configfile, outputpath, drawer);

    // check is initialized
    if (!gvins_->isRunning()) {
        LOGE << "Fusion ROS terminate";
        return;
    }

    // subscribe message
    ros::Subscriber imu_sub   = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200, &FusionROS::imuCallback, this);
    ros::Subscriber gnss_sub  = nh.subscribe<sensor_msgs::NavSatFix>(gnss_topic, 1, &FusionROS::gnssCallback, this);
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>(image_topic, 20, &FusionROS::imageCallback, this);

    LOGI << "Waiting ROS message...";

    // enter message loopback
    ros::spin();
}

/**
 * 把IMU的Unix时间戳转换为GPS周内秒,最终保存到gvins::imu_buffer_内
 * @param imumsg ROS的IMU数据
 */
void FusionROS::imuCallback(const sensor_msgs::ImuConstPtr &imumsg) {
    imu_pre_ = imu_;

    // Time convertion
    double unixsecond = imumsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    imu_.time = weeksec;
    // delta time
    imu_.dt = imu_.time - imu_pre_.time;

    // IMU measurements, Front-Right-Down & x-y-z
    imu_.dtheta[0] = imumsg->angular_velocity.x * imu_.dt;
    imu_.dtheta[1] = imumsg->angular_velocity.y * imu_.dt;
    imu_.dtheta[2] = imumsg->angular_velocity.z * imu_.dt;
    imu_.dvel[0]   = imumsg->linear_acceleration.x * imu_.dt;
    imu_.dvel[1]   = imumsg->linear_acceleration.y * imu_.dt;
    imu_.dvel[2]   = imumsg->linear_acceleration.z * imu_.dt;

    // 第一帧IMU，直接跳过。
    // not ready
    if (imu_pre_.time == 0) {
        return;
    }

    // 保存IMU数据到buffer中
    imu_buffer_.push(imu_);

    //  通知gvins去处理IMU数据。
    while (!imu_buffer_.empty()) {
        auto imu = imu_buffer_.front();

        // Add new IMU to GVINS
        if (gvins_->addNewImu(imu)) {
            imu_buffer_.pop();
        } else {
            // Thread lock failed, try next time
            //
            break;
        }
    }
}

/**
 * 保存有效的gnss信息到gvins::gnss_
 * @param gnssmsg
 */
void FusionROS::gnssCallback(const sensor_msgs::NavSatFixConstPtr &gnssmsg) {

    // Time convertion
    double unixsecond = gnssmsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    // 经纬高rad NED协方差
    gnss_.time   = weeksec;
    gnss_.blh[0] = gnssmsg->latitude * D2R;
    gnss_.blh[1] = gnssmsg->longitude * D2R;
    gnss_.blh[2] = gnssmsg->altitude;
    gnss_.std[0] = sqrt(gnssmsg->position_covariance[4]); // N
    gnss_.std[1] = sqrt(gnssmsg->position_covariance[0]); // E
    gnss_.std[2] = sqrt(gnssmsg->position_covariance[8]); // D

    // 默认无有效航向
    gnss_.isyawvalid = false;

    // Exception
    // 无效gnss数据
    if ((gnss_.std[0] == 0) || (gnss_.std[1] == 0) || (gnss_.std[2] == 0)) {
        LOGE << "abandon gnss data---- cov = 0";
        return;
    }

    // Remove bad GNSS
    bool isoutage = false;
    if ((gnss_.std[0] < gnssthreshold_) && (gnss_.std[1] < gnssthreshold_) && (gnss_.std[2] < gnssthreshold_)) {

        // 仿真GNSS失锁
        if (isusegnssoutage_ && (weeksec >= gnssoutagetime_)) {
            isoutage = true;
        }

        // add new GNSS to GVINS
        if (!isoutage) {
            gvins_->addNewGnss(gnss_);
        }
    } else {
        LOGE << "abandon gnss data---- too big cov";
    }
}

/**
 * ROS图像格式数据转换
 * @param imagemsg
 */
void FusionROS::imageCallback(const sensor_msgs::ImageConstPtr &imagemsg) {
    Mat image;

    // Copy image data 读取原始图像数据
    if (imagemsg->encoding == sensor_msgs::image_encodings::MONO8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);
    } else if (imagemsg->encoding == sensor_msgs::image_encodings::BGR8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
    }

    // Time convertion 时间戳转换
    double unixsecond = imagemsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    // 创建图像帧
    frame_ = Frame::createFrame(weeksec, image);

    // Add new Image to GVINS
    frame_buffer_.push(frame_);
    while (!frame_buffer_.empty()) {
        auto frame = frame_buffer_.front();
        if (gvins_->addNewFrame(frame)) {
            frame_buffer_.pop();
        } else {
            break;
        }
    }

    // 输出对应
    LOG_EVERY_N(INFO, 20) << "Raw data time in image recv image,imu,gnss " << Logging::doubleData(frame_->stamp())
                          << ", " << Logging::doubleData(imu_.time) << ", " << Logging::doubleData(gnss_.time);
}

/**
 * 调用Ctrl+c 来关闭节点
 * @param sig
 */
void sigintHandler(int sig) {
    std::cout << "Terminate by Ctrl+C " << sig << std::endl;
    isfinished = true;
}

/**
 * 安全退出GVINS，保存相关信息
 * @param fusion
 */
void checkStateThread(std::shared_ptr<FusionROS> fusion) {
    std::cout << "Check thread is started..." << std::endl;

    auto fusion_ptr = std::move(fusion);

    // 休眠1s
    while (!isfinished) {
        sleep(1);
    }

    // Exit the GVINS thread
    fusion_ptr->setFinished();

    std::cout << "GVINS has been shutdown ..." << std::endl;

    // Shutdown ROS
    ros::shutdown();

    std::cout << "ROS node has been shutdown ..." << std::endl;
}

int main(int argc, char *argv[]) {

    // Glog initialization
    //    初始化glog
    Logging::initialization(argv, true, true);

    // ROS node
    ros::init(argc, argv, "gvins_node", ros::init_options::NoSigintHandler);

    // Register signal handler
    // self define signal handler repalce ctrl+c
    std::signal(SIGINT, sigintHandler);

    // 创建对象
    auto fusion = std::make_shared<FusionROS>();

    // Check thread for shutdown program
    std::thread check_thread(checkStateThread, fusion);

    // Enter message loop
    // 开始接收传感器数据。
    std::cout << "Fusion process is started..." << std::endl;
    fusion->run();

    return 0;
}
