/*
 * @Author: Divenire qiuyue_olw@163.com
 * @Date: 2023-02-01 21:13:55
 * @LastEditors: Divenire qiuyue_olw@163.com
 * @LastEditTime: 2023-02-03 16:04:25
 * @FilePath: /icgvins_ws/src/IC-GVINS/ic_gvins/ic_gvins/fileio/filesaver.cc
 * @Description:
 *
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved.
 */
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

#include "fileio/filesaver.h"

#include "thirdparty/abseil-cpp/absl/strings/str_format.h"

FileSaver::FileSaver(const string &filename, int columns, int filetype) {
    open(filename, columns, filetype);
}

bool FileSaver::open(const string &filename, int columns, int filetype) {
    auto type = filetype == TEXT ? std::ios_base::out : (std::ios_base::out | std::ios_base::binary);
    filefp_.open(filename, type);

    columns_  = columns;
    filetype_ = filetype;

    return isOpen();
}

void FileSaver::dump(const vector<double> &data) {
    dump_(data);
}

void FileSaver::dumpn(const vector<vector<double>> &data) {
    for (const auto &k : data) {
        dump_(k);
    }
}

void FileSaver::dump_(const vector<double> &data) {
    if (filetype_ == TEXT) {
        string line;

        constexpr absl::string_view format = "%-.9lf";

        line = absl::StrFormat(format, data[0]);
        for (size_t k = 1; k < data.size(); k++) {
            line += " ";
            absl::StrAppendFormat(&line, format, data[k]);
        }

        filefp_ << line << "\n";
    } else {
        filefp_.write(reinterpret_cast<const char *>(data.data()), sizeof(double) * data.size());
    }
}

FileSaver::~FileSaver() {
    if (isOpen()) {
        flush();
        close();
    }
}
