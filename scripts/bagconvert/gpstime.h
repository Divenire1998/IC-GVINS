/*
 * gpstime.h
 *
 *  Created on: 22/7/8
 *      Author: hailiang
 */

#ifndef GPS_TIME_H
#define GPS_TIME_H

#include <math.h>

/* GPS is now ahead of UTC by 18 seconds */
#define GPS_LEAP_SECOND 18

class GpsTime {

public:
    static void gpsSecond2UnixSecond(double &unixsecond, double const gpsweek, double const gpsweeksec) {
        unixsecond = gpsweeksec + gpsweek * 604800 + 315964800 - GPS_LEAP_SECOND;
    };

    static void unixSecond2GpsSecond(double &gpsweek, double &gpsweeksec, double const unixsecond) {
        double seconds = unixsecond + GPS_LEAP_SECOND - 315964800;

        gpsweek    = floor(seconds / 604800);
        gpsweeksec = seconds - gpsweek * 604800;
    };
};

#endif /* GPS_TIME_H */
