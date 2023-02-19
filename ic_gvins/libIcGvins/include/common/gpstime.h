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

#ifndef GPS_TIME_H
#define GPS_TIME_H

#include <cmath>

// GPS is now ahead of UTC by 18 seconds
#define GPS_LEAP_SECOND 18

class GpsTime {

public:
    /**
     *  gps时间戳转换为unix时间戳
     *  315964800 为Unix和UTC时间的差值
     * @param week gps周
     * @param sow 周内秒
     * @param unixs unix时间戳
     */
    static void gps2unix(int week, double sow, double &unixs) {
        unixs = sow + week * 604800 + 315964800 - GPS_LEAP_SECOND;
    };


    /**
     * unix时间戳转换为gps时间戳
     * 315964800 为Unix和UTC时间的差值
     * @param unixs unix时间
     * @param week gps周
     * @param sow 周内秒
     */
    static void unix2gps(double unixs, int &week, double &sow) {
        double seconds = unixs + GPS_LEAP_SECOND - 315964800;

        week = floor(seconds / 604800);
        sow  = seconds - week * 604800;
    };
};

#endif // GPS_TIME_H
