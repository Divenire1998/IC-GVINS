#include "fusion_raw.h"
#include "drawer_rviz.h"

#include "common/angle.h"
#include "common/gpstime.h"
#include "common/logging.h"
#include "tracking/frame.h"

#include <boost/filesystem.hpp>
#include <sensor_msgs/image_encodings.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <csignal>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <memory>
std::atomic<bool> isfinished{false};

void sigintHandler(int sig);
void checkStateThread(std::shared_ptr<FusionRaw> fusion);

/**
 *  安全关闭gvins
 */
void FusionRaw::setFinished() {
    if (gvins_ && gvins_->isRunning()) {
        gvins_->setFinished();
    }
}

/**
 *  修改话题，设置必要的参数，
 */
void FusionRaw::run() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

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

    // 判断是否使用轮速
    isuseodo_ = config["odometer"]["isuseodo"].as<bool>();

    // GNSS outage configurations
    // 根据这个来决定是否添加gnss信息到buffer中
    isusegnssoutage_ = config["isusegnssoutage"].as<bool>();
    gnssoutagetime_  = config["gnssoutagetime"].as<double>();
    gnssthreshold_   = config["gnssthreshold"].as<double>();

    // 各个传感器的频率
    imu_freq_   = config["imudatarate"].as<double>();
    image_freq_ = config["imagedatarate"].as<double>();
    gnss_freq_  = config["gnssdatarate"].as<double>();
    wheel_freq_ = config["wheeldatarate"].as<double>();

    // 设置结果保存相关选项
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
    // set Glog output path
    // 设置glog的日志输出路径
    FLAGS_log_dir = outputpath;

    // The GVINS object
    // 启动gvins，处理传感器数据，并使用ROS进行显示。
    // 多态
    Drawer::Ptr drawer = std::make_shared<DrawerRviz>(nh);
    gvins_             = std::make_shared<GVINS>(configfile, outputpath, drawer);

    pubRawLeftImage = nh.advertise<sensor_msgs::Image>("image_raw", 1000);
    pubRawImu       = nh.advertise<sensor_msgs::Imu>("imu_raw", 1000);
    pubRawVrsGps    = nh.advertise<sensor_msgs::NavSatFix>("gps_raw", 100, true);

    // 准备传感器数据
    strPathToSequence            = config["sequencepath"].as<string>();
    string PathImage             = strPathToSequence + "/image/stereo_left";
    string PathImageStamp        = strPathToSequence + "/sensor_data/stereo_stamp.csv";
    string PathImu               = strPathToSequence + "/sensor_data/xsens_imu.csv";
    string PathGps               = strPathToSequence + "/sensor_data/gps.csv";
    string PathVrsGPs            = strPathToSequence + "/sensor_data/vrs_gps.csv";
    string PathWheelEncoder      = strPathToSequence + "/sensor_data/encoder.csv";
    string PathWHeelEncoderCalib = strPathToSequence + "/calibration/EncoderParameter.txt";

    imufile.open(PathImu);
    vrsGpsFile.open(PathVrsGPs);
    wheelEncoderFile.open(PathWheelEncoder);
    wheelEncoderCalibFile.open(PathWHeelEncoderCalib);

    // 读取图像时间戳
    FILE *fp = fopen(PathImageStamp.c_str(), "r");
    int64_t stamp;
    while (fscanf(fp, "%ld\n", &stamp) == 1) {
        vTimestamps.push_back(stamp);
    }
    fclose(fp);

    // 读取轮速的标定参数
    if (isuseodo_) {
        std::string str;
        // Step 1 读取标定参数文件
        if (encoder_resolution_ == 0) {
            while (std::getline(wheelEncoderCalibFile, str)) {
                std::stringstream ss(str);
                std::string token;
                std::vector<std::string> strs;
                while (std::getline(ss, token, ' ')) {
                    strs.push_back(token);
                }
                if (!strs[1].compare("resolution:")) {
                    encoder_resolution_ = std::stoi(strs[2]);
                }
                if (!strs[1].compare("left")) {
                    encoder_left_diameter_ = std::stod(strs[4]);
                }
                if (!strs[1].compare("right")) {
                    encoder_right_diameter_ = std::stod(strs[4]);
                }
                if (!strs[1].compare("wheel")) {
                    encoder_wheel_base_ = std::stod(strs[3]);
                }
            }
        }
    }

    double imagePeriod = 1 / image_freq_;
    // 对图像进行遍历
    for (size_t i = 0; i < vTimestamps.size(); ++i) {

        // check is initialized
        if (!gvins_->isRunning()) {
            LOGE << "Fusion ROS terminate";
            return;
        }

        // 输入图像数据
        inputImage(i);

        // 输入IMU数据
        while (imu_.time < imageTimeGps_ + imagePeriod) {
            inputIMU();
        }

        // 输入GNSS数据
        while (gnss_.time + 1 < imageTimeGps_) {
            inputGnss();
        }

        usleep(50000);
    }
}

/**
 * 从文件中加在一行IMU数据到imumsg
 * @param imumsg
 */
void FusionRaw::loadImuData(sensor_msgs::Imu &imumsg) {
    string lineStr_imu;

    std::getline(imufile, lineStr_imu);

    if (imu_pre_.time == 0)
        std::getline(imufile, lineStr_imu);

    if (imufile.eof()) {
        LOGE << "IMU DATA IS END OF IMU FILE!! " << std::endl;
        return;
    }

    std::stringstream ss(lineStr_imu);
    vector<string> lineArray;
    string str;
    // 按照逗号分隔
    while (getline(ss, str, ','))
        lineArray.push_back(str);

    // 修改符合右前上
    uint64_t imu_time            = stoll(lineArray[0]);
    imumsg.header.stamp          = ros::Time().fromNSec(imu_time);
    imumsg.angular_velocity.x    = stod(lineArray[8]);
    imumsg.angular_velocity.y    = -1 * stod(lineArray[9]);
    imumsg.angular_velocity.z    = -1 * stod(lineArray[10]);
    imumsg.linear_acceleration.x = stod(lineArray[11]);
    imumsg.linear_acceleration.y = -1 * stod(lineArray[12]);
    imumsg.linear_acceleration.z = -1 * stod(lineArray[13]);
}

Wheel FusionRaw::loadWheelData() {

    // Step 1 读取轮速数据
    string lineStr_wheel;
    if (wheelEncoderFile.eof()) {
        LOGE << "END OF WHEELS FILE " << std::endl;
        return {0, 0, 0};
    }

    std::getline(wheelEncoderFile, lineStr_wheel);
    std::stringstream ssLine(lineStr_wheel);
    vector<string> lineArray;
    string str;
    // 按照逗号分隔
    while (getline(ssLine, str, ','))
        lineArray.push_back(str);

    uint64_t wheel_time, left_count, right_count;
    wheel_time  = stoll(lineArray[0]);
    left_count  = stoll(lineArray[1]);
    right_count = stoll(lineArray[2]);

    // LOGE << wheel_time << left_count << right_count;
    // Time convertion
    double unixsecond = 1e-9 * static_cast<double>(wheel_time);

    double weektime;
    int week;
    GpsTime::unix2gps(unixsecond, week, weektime);
    //    LOGE << std::fixed << unixsecond << "   " << weektime << "  " << 9.98;
    return {weektime, left_count, right_count};
}

/**
 * 读取文件中的IMU数据，输入到ic_gvins中
 */
void FusionRaw::inputIMU() {
    imu_pre_ = imu_;

    sensor_msgs::Imu imumsg;
    // readIMU
    loadImuData(imumsg);
    pubRawImu.publish(imumsg);

    // Time convertion
    double unixsecond = imumsg.header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    imu_.time = weeksec;
    // delta time
    imu_.dt = imu_.time - imu_pre_.time;

    // IMU measurements, Front-Right-Down & x-y-z
    imu_.dtheta[0] = imumsg.angular_velocity.x * imu_.dt;
    imu_.dtheta[1] = imumsg.angular_velocity.y * imu_.dt;
    imu_.dtheta[2] = imumsg.angular_velocity.z * imu_.dt;
    imu_.dvel[0]   = imumsg.linear_acceleration.x * imu_.dt;
    imu_.dvel[1]   = imumsg.linear_acceleration.y * imu_.dt;
    imu_.dvel[2]   = imumsg.linear_acceleration.z * imu_.dt;

    static int count = 0;
    if (isuseodo_) {
        wheel_pre_ = wheel_;
        wheel_     = loadWheelData();
        if (abs(wheel_.time - imu_.time) > 0.02) {
            ++count;
            LOGE << "IMU 和 轮速 丢数据了" << count;
            std::string lineStr_imu;
            std::getline(imufile, lineStr_imu);
        }
    }

    // 第一帧IMU，直接跳过。
    // not ready
    if (imu_pre_.time == 0) {
        return;
    }

    if (isuseodo_) {
        // IMU和轮速进行插值
        double vel_left =
            (wheel_.left_count - wheel_pre_.left_count) * encoder_left_diameter_ * M_PI / encoder_resolution_;
        double vel_right =
            (wheel_.right_count - wheel_pre_.right_count) * encoder_right_diameter_ * M_PI / encoder_resolution_;
        imu_.odovel = (vel_left + vel_right) * 0.5;

        // 编码器循环处理
        if (imu_.odovel > 1)
            imu_.odovel = imu_pre_.odovel;
    }

    // 原始数据调试
    {
        // imu_odovel调试
        //        LOGE << std::fixed << wheel_.left_count << " " << wheel_pre_.left_count << " " <<
        //        encoder_left_diameter_ << " "
        //             << imu_.odovel;

        // 调试时间对其
        //    LOG_EVERY_N(INFO, 1) << "Raw data time in image recv image,imu,gnss,wheel " <<
        //    Logging::doubleData(frame_->stamp())
        //                         << ", " << Logging::doubleData(imu_.time) << ", " << Logging::doubleData(gnss_.time)
        //                         <<
        //                         ", "
        //                         << Logging::doubleData(wheel_.time);
    }

    // 保存IMU数据到buffer中 DEBUG
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

void FusionRaw::LoadGnssData(sensor_msgs::NavSatFix &gnssmsg) {
    string FileGetline;

    if (!vrsGpsFile.eof())
        std::getline(vrsGpsFile, FileGetline);
    else {
        std::cout << "END OF WHEELS FILE " << std::endl;
        return;
    }

    std::stringstream gps_ss(FileGetline);
    vector<string> line_data_vec;
    string value_str;
    while (getline(gps_ss, value_str, ',')) {
        line_data_vec.push_back(value_str);
    }

    // 时间戳
    uint64_t time        = std::stoll(line_data_vec[0]);
    gnssmsg.header.stamp = ros::Time().fromNSec(time);

    // 经纬高
    gnssmsg.latitude  = std::stod(line_data_vec[1]);
    gnssmsg.longitude = std::stod(line_data_vec[2]);
    gnssmsg.altitude  = std::stod(line_data_vec[5]);

    int fix_state = stod(line_data_vec[6]); // 状态：1正常，2DGPS，4固定这个精度最高，5浮动

    // 导航状态
    sensor_msgs::NavSatStatus navsatstate;
    navsatstate.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
    navsatstate.service = sensor_msgs::NavSatStatus::SERVICE_GPS + sensor_msgs::NavSatStatus::SERVICE_GLONASS;
    gnssmsg.status      = navsatstate;

    // 协方差
    gnssmsg.position_covariance_type = sensor_msgs::NavSatFix ::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    // ENU系
    double lat_std                 = std::stod(line_data_vec[9]);
    double lon_std                 = std::stod(line_data_vec[10]);
    double altitude_std            = std::stod(line_data_vec[11]);
    gnssmsg.position_covariance[0] = lon_std * lon_std;
    gnssmsg.position_covariance[4] = lat_std * lat_std;
    gnssmsg.position_covariance[8] = altitude_std * altitude_std;
}

/**
 * 保存有效的gnss信息到gvins::gnss_
 * @param gnssmsg
 */
void FusionRaw::inputGnss() {

    sensor_msgs::NavSatFix gnssmsg;
    LoadGnssData(gnssmsg);
    pubRawVrsGps.publish(gnssmsg);

    // Time convertion
    double unixsecond = gnssmsg.header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    // 经纬高rad NED协方差
    gnss_.time   = weeksec;
    gnss_.blh[0] = gnssmsg.latitude * D2R;
    gnss_.blh[1] = gnssmsg.longitude * D2R;
    gnss_.blh[2] = gnssmsg.altitude;
    gnss_.std[0] = sqrt(gnssmsg.position_covariance[4]); // N
    gnss_.std[1] = sqrt(gnssmsg.position_covariance[0]); // E
    gnss_.std[2] = sqrt(gnssmsg.position_covariance[8]); // D

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
        LOGE << "abandon gnss data---- too big cov:"
             << sqrt(gnss_.std[0] * gnss_.std[0] + gnss_.std[1] * gnss_.std[1] + gnss_.std[2] * gnss_.std[2]);
    }
}

/**
 * 从数据集中读取图像时间戳，保存在vstrImageFilenames0
 * @param strPathToSequence
 * @param vstrImageFilenames0
 * @param vstrImageFilenames1
 * @param vTimestamps
 */
void FusionRaw::loadImage(sensor_msgs::Image &imagemsg, size_t index) {
    // 时间戳和图像名称
    std::string st         = std::to_string(vTimestamps[index]);
    std::string frame_file = strPathToSequence + "/image/stereo_left/" + st + ".png";

    cv::Mat img = cv::imread(frame_file, cv::IMREAD_ANYDEPTH);
    cv_bridge::CvImage img_msg;
    sensor_msgs::Image imageMsg;
    img_msg.header.stamp.fromNSec(vTimestamps[index]);
    img_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
    img_msg.image    = img;
    img_msg.toImageMsg(imageMsg);

    Mat raw(static_cast<int>(imageMsg.height), static_cast<int>(imageMsg.width), CV_8UC1,
            (void *) imageMsg.data.data());
    Mat gray;
    cv::cvtColor(raw, gray, cv::COLOR_BayerBG2GRAY);

    // 彩色转为灰度
    imagemsg.header   = img_msg.header;
    imagemsg.height   = gray.rows;
    imagemsg.width    = gray.cols;
    imagemsg.encoding = sensor_msgs::image_encodings::MONO8;
    imagemsg.step     = imagemsg.width;
    size_t size       = imagemsg.height * imagemsg.width;
    imagemsg.data.resize(size);
    memcpy(imagemsg.data.data(), gray.data, size);
}

/**
 * ROS图像格式数据转换
 * @param imagemsg
 */
void FusionRaw::inputImage(size_t index) {

    Mat image;
    sensor_msgs::Image imagemsg;
    loadImage(imagemsg, index);

    pubRawLeftImage.publish(imagemsg);

    // Copy image data 读取原始图像数据
    if (imagemsg.encoding == sensor_msgs::image_encodings::MONO8) {
        image = Mat(static_cast<int>(imagemsg.height), static_cast<int>(imagemsg.width), CV_8UC1);
        memcpy(image.data, imagemsg.data.data(), imagemsg.height * imagemsg.width);
    } else if (imagemsg.encoding == sensor_msgs::image_encodings::BGR8) {
        image = Mat(static_cast<int>(imagemsg.height), static_cast<int>(imagemsg.width), CV_8UC3);
        memcpy(image.data, imagemsg.data.data(), imagemsg.height * imagemsg.width * 3);
    }

    // Time convertion 时间戳转换
    double unixsecond = imagemsg.header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    imageTimeGps_ = weeksec;

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
    //    LOG_EVERY_N(INFO, 20) << "Raw data time in image recv image,imu,gnss,wheel " <<
    //    Logging::doubleData(frame_->stamp())
    //                          << ", " << Logging::doubleData(imu_.time) << ", " << Logging::doubleData(gnss_.time) <<
    //                          ", "
    //                          << Logging::doubleData(wheel_.time);
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
void checkStateThread(std::shared_ptr<FusionRaw> fusion) {
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
    Logging::initialization(argv, true, true);

    // ROS节点初始化
    // Register signal handler self define signal handler repalce ctrl+c
    ros::init(argc, argv, "gvins_node", ros::init_options::NoSigintHandler);
    std::signal(SIGINT, sigintHandler);

    // 创建Fusion Raw对象
    auto fusion = std::make_shared<FusionRaw>();

    // Check thread for shutdown program
    // 安全关闭程序
    std::thread check_thread(checkStateThread, fusion);

    // Enter message loop
    // 开始接收传感器数据。
    std::cout << "Fusion process is started..." << std::endl;
    fusion->run();

    return 0;
}
