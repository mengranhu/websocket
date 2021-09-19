/**
 *  Copyright (C) 2019 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef POINTCLOUD_SERVER_COMMON_WGS_84_HPP_
#define POINTCLOUD_SERVER_COMMON_WGS_84_HPP_

#include <assert.h>

class WGS84 {
 public:
  static double get_meter_per_lat_degree(double lat) {
    // see https://www.stevemorse.org/nearest/distance.php
    static constexpr const double m_[] = {
      11058,  // 0
      11058,  // 5
      11061,  // 10
      11064,  // 15
      11071,  // 20
      11077,  // 25
      11085,  // 30
      11095,  // 35
      11103,  // 40
      11112,  // 45
      11124,  // 50
      11132,  // 55
      11141,  // 60
      11150,  // 65
      11156,  // 70
      11162,  // 75
      11166,  // 80
      11169,  // 85
      11169   // 90
    };
    assert(lat <= 90);
    assert(lat >= -90);
    if (lat < 0) {
      lat = -lat;
    }
    int idx = lat / 5;
    double ret = m_[idx] +
                 (m_[idx + 1] - m_[idx]) / 5.0 * (lat - idx * 5.0);
    return ret * 10;
  }

 private:
};

#endif  // POINTCLOUD_SERVER_COMMON_WGS_84_HPP_
