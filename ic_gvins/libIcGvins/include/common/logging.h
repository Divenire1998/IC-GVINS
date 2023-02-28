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

#ifndef LOGGING_H
#define LOGGING_H

#include "thirdparty/abseil-cpp/absl/strings/str_format.h"
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <iostream>

using std::string;

#define LOGI (LOG(INFO))
#define LOGW (LOG(WARNING))
#define LOGE (LOG(ERROR))
#define LOGF (LOG(FATAL))

// todo 调试模式支持？
#if !DCHECK_IS_ON()
#define DLOGI (static_cast<void>(0), true ? (void) 0 : google::LogMessageVoidify() & LOG(INFO))
#define DLOGW (static_cast<void>(0), true ? (void) 0 : google::LogMessageVoidify() & LOG(WARNING))
#define DLOGE (static_cast<void>(0), true ? (void) 0 : google::LogMessageVoidify() & LOG(ERROR))
#define DLOGF (static_cast<void>(0), true ? (void) 0 : google::LogMessageVoidify() & LOG(FATAL))
#else
#define DLOGI LOGI
#define DLOGW LOGW
#define DLOGE LOGE
#define DLOGF LOGF
#endif

class Logging {

public:
    /**
     *
     * @param argv main函数的参数
     * @param logtostderr 是否输出到命令行
     * @param logtofile 是否输出日志文件
     */
    static void initialization(char **argv, bool logtostderr = true, bool logtofile = true) {
        // glog初始化
        // argv[0]
        //        google::InitGoogleLogging(argv[0]);
        google::InitGoogleLogging("GVINS");

        FLAGS_minloglevel = google::GLOG_INFO; // 设置最小处理日志的级别

        if (logtostderr & logtofile) {
            FLAGS_alsologtostderr = true;
        } else if (logtostderr) {
            FLAGS_logtostderr = true;
        }

        // 设置输出到命令行信息的颜色
        if (logtostderr) {
            // 输出颜色
            FLAGS_colorlogtostderr = true;
        }

        // 这儿不能用glog因为输出路径在后面才设置
        //        std::cout << argv[0] << std::endl;
    }

    template <typename T, int Rows, int Cols>
    static void printMatrix(const Eigen::Matrix<T, Rows, Cols> &matrix, const string &prefix = "Matrix: ") {
        std::cout << prefix << matrix.rows() << "x" << matrix.cols() << std::endl;
        if (matrix.cols() == 1) {
            std::cout << matrix.transpose() << std::endl;
        } else {
            std::cout << matrix << std::endl;
        }
    }

    static string doubleData(double data) {
        return absl::StrFormat("%0.6lf", data);
    }

    static void shutdownLogging() {
        google::ShutdownGoogleLogging();
    }
};

#endif // LOGGING_H
