
/**
 *  Copyright (C) 2019 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef POINTCLOUD_SERVER_COMMON_POINTCLOUD_MATRIX_CALIBRATION_H_
#define POINTCLOUD_SERVER_COMMON_POINTCLOUD_MATRIX_CALIBRATION_H_

#include "pointcloud_kvp.h"       // NOLINT
#include <cmath>
#include "src/inno_lidar_api.h"
#include "src/inno_lidar_api_experimental.h"

#ifndef LL_DEGREE_TO_INT
#define LL_DEGREE_TO_INT(n) ((n) * 10000000.0)
#endif

class MatrixCalibration {
 public:
  int set_wgs(double gps[3]) {
    wgs_ = true;
    origin_point_gps_[0] = gps[0];  // longitude
    origin_point_gps_[1] = gps[1];  // latitude
    origin_point_gps_[2] = gps[2];  // elevator

    try {
      double B = origin_point_gps_[1] * M_PI / 180.0;
      double f = 1/298.257223563;   // WGS84 ellipsode of radius if curvature
      double r = 6378137.0;         // semi-major axis
      // the first eccentricity if the ellipse
      double e = std::sqrt(2.0 * f - f * f);
      // radius of eliipse curvature
      GPS_CurvatureRadius_ = r / std::sqrt(1.0 - e * e * sin(B) * sin(B));
      gpstowgs(origin_point_gps_[0], origin_point_gps_[1], origin_point_gps_[2],
               &GPS_anchor_xyz_[0], &GPS_anchor_xyz_[1], &GPS_anchor_xyz_[2]);
      fprintf(stderr, "GPS_CurvatureRadius_ = %.13f\n", GPS_CurvatureRadius_);
      fprintf(stderr, "GPS_anchor_xyz_: %.13f %.13f %.13f\n",
               GPS_anchor_xyz_[0], GPS_anchor_xyz_[1], GPS_anchor_xyz_[2]);
    } catch(...) {
      fprintf(stderr, "set_wgs exception: %f %f %f",
                      origin_point_gps_[0],
                      origin_point_gps_[1],
                      origin_point_gps_[2]);
      exit(-1);
    }
    return 0;
  }

  // pcs_xyz2gps_matrix_line0/1/2/3 : data1,  data2,   data3,   data4
  // pcs_gps2xyz_matrix_line0/1/2/3 : data1,  data2,   data3,   data4
  // pcs_xyz2xyz_matrix_line0/1/2/3 : data1,  data2,   data3,   data4
  int set_matrix(const KvpItem &item) noexcept {
    double (*m)[4];
    bool *rdy;
    if (item.token.find("pcs_xyz2gps_matrix_line") == 0) {
      m = xyz2gps_;
      rdy = xyz2gps_ready_;
    } else if (item.token.find("pcs_gps2xyz_matrix_line") == 0) {
      m = gps2xyz_;
      rdy = gps2xyz_ready_;
    } else if (item.token.find("pcs_xyz2xyz_matrix_line") == 0) {
      m = xyz2xyz_;
      rdy = xyz2xyz_ready_;
    } else {
      fprintf(stderr, "impossible MatrixCalibration set_matrix error: %s\n",
              item.token.c_str());
      exit(-1);
    }

    char t = item.token[item.token.size() - 1];
    int index = t - '0';
    if (index < 0 || index > 3) {
      fprintf(stderr, "MatrixCalibration %s: index %d [0,3]\n",
              item.token.c_str(), index);
      exit(-1);
    }

    if (item.value.size() != 4) {
      fprintf(stderr, "MatrixCalibration %s: no 4 values\n",
              item.token.c_str());
      exit(-1);
    }

    for (int i = 0; i < 4; ++i) {
      try {
        m[index][i] = std::stod(item.value[i]);
        fprintf(stderr, "matrix calibraion %s: add [%d][%d] = %lf\n",
                item.token.c_str(),
                index, i, m[index][i]);
      } catch (...) {
        fprintf(stderr, "MatrixCalibration %s: value %s is not double\n",
                item.token.c_str(), item.value[i].c_str());
        exit(-1);
      }
    }
    rdy[index] = true;
    return 0;
  }

  void get_origin_gps(double *lo, double *la, double *el) noexcept {
    xyz2gps(0, 0, 0, lo, la, el);
  }

  static double get_angle(double delta_x, double delta_y, double ratio = 1) {
    if (fabs(delta_y) < 1e-12 && fabs(delta_x) < 1e-12) {
      return 0;
    } else if (fabs(delta_y) < 1e-12) {
      return delta_x > 0 ? 90.0 : 270.0;    // horizon
    } else if (fabs(delta_x) < 1e-12) {
      return delta_y > 0 ? 0.0 : 180.0;     // vertical
    } else {
      double angle = std::atan(fabs(ratio * delta_x / delta_y)) * \
                     180.0 / M_PI;
      if (delta_x > 0 && delta_y < 0) {
        angle = 180.0 - angle;    // 2nd qurdrant
      } else if (delta_x < 0 && delta_y < 0) {
        angle += 180.0;           // 3nd qurdrant
      } else if (delta_x < 0 && delta_y > 0) {
        angle = 360.0 - angle;    // 4nd qurdrant
      }
      return angle;
    }
  }

  // from origin point (0,0,0) to point (0,0,1)
  // this direction is speed_z direction
  double get_speed_z_angle(double ratio) {
    double lo0, la0, el0;
    xyz2gps(0.0, 0.0, 0.0, &lo0, &la0, &el0);
    double lo1, la1, el1;
    xyz2gps(0.0, 0.0, 1.0, &lo1, &la1, &el1);
    double delta_lo = lo1 - lo0;
    double delta_la = la1 - la0;
    return get_angle(delta_lo, delta_la, ratio);
  }

  void xyz2gps(double x, double y, double z,
               double *lo, double *la, double *el) noexcept {
    CheckMatrixReady();
    double m, n, q;
    multiply(xyz2gps_, x, y, z, &m, &n, &q);
    if (wgs_) {
      localtogps(m, n, q, lo, la, el);
    } else {
      *lo = m;
      *la = n;
      *el = q;
    }
  }

  void gps2xyz(double lo, double la, double el,
               double *x, double *y, double *z) noexcept {
    CheckMatrixReady();
    double m, n, q;
    if (wgs_) {
      gpstolocal(lo, la, el, &m, &n, &q);
    } else {
      m = lo;
      n = la;
      q = el;
    }
    multiply(gps2xyz_, m, n, q, x, y, z);
  }

  void xyz2xyz(double x, double y, double z,
               double *xo, double *yo, double *zo) noexcept {
    CheckMatrixReady_xyz();
    multiply(xyz2xyz_, x, y, z, xo, yo, zo);
  }

  void bbox_xyz2gps(inno_cframe_header *h) noexcept {
    if (h->type != INNO_CFRAME_BBOX) {
      return;
    }

    for (unsigned int i = 0; i < h->item_number; ++i) {
      inno_bbox &b = h->bboxes[i];
      double longtitude, latitude, elevation;
      xyz2gps(b.x, b.y, b.z,
              &longtitude, &latitude, &elevation);
      b.longtitude = LL_DEGREE_TO_INT(longtitude);
      b.latitude = LL_DEGREE_TO_INT(latitude);
      b.elevation = elevation * 100.0;
    }
  }

 private:
  void CheckMatrixReady() {
    if (!ready_) {
      for (int i = 0; i < 4; ++i) {
        if (!xyz2gps_ready_[i] || !gps2xyz_ready_[i]) {
          fprintf(stderr, "MatrixCalibration Matrix not ready\n");
          exit(-1);
        }
      }
      ready_ = true;
    }
  }

  void CheckMatrixReady_xyz() {
    if (!ready_xyz_) {
      for (int i = 0; i < 4; ++i) {
        if (!xyz2xyz_ready_[i]) {
          fprintf(stderr, "MatrixCalibration Matrix XYZ not ready\n");
          exit(-1);
        }
      }
      ready_xyz_ = true;
    }
  }

  void multiply(const double (*m)[4],
                double x, double y, double z,
                double *nx, double *ny, double *nz) const noexcept {
    double in[4] = {x, y, z, 1};
    double out[4];
    for (int i = 0; i < 4; i++) {
      double sum = 0;
      for (int k = 0; k < 4; k++) {
        double p = m[i][k] * in[k];
        sum += p;
      }
      out[i] = sum;
    }
    *nx = out[0];
    *ny = out[1];
    *nz = out[2];
  }

  // gps -> geodetic coordinates
  void gpstowgs(double longitude, double latitude, double altitude,
                double *LO, double *BO, double *HO) {
    try {
      double B = latitude * M_PI / 180.0;
      double L = longitude * M_PI / 180.0;
      double H = altitude;
      double f = 1/298.257223563;  // WGS84 ellipsode of radius if curvature
      double r = 6378137.0;        // semi-major axis
      // the first eccentricity if the ellipse
      double e = std::sqrt(2.0 * f - f * f);
      // radius of eliipse curvature
      double N = r / std::sqrt(1.0 - e * e * sin(B) * sin(B));
      *LO = (N + H) * cos(B) * cos(L);
      *BO = (N + H) * cos(B) * sin(L);
      *HO = (N * (1.0 - e * e) + H) * sin(B);
    } catch(...) {
      fprintf(stderr, "gpstowgs exception: %f %f %f",
                       longitude, latitude, altitude);
      exit(-1);
    }
  }

  // geodetic coordinates -> gps
  void wgstogps(double m, double n, double q,
                double *lo, double *la, double *el) {
    try {
      double N = GPS_CurvatureRadius_;
      double f = 1/298.257223563;
      double e = std::sqrt(2 * f - f * f);
      double L = atan(n / m) + M_PI;
      double B = atan(cos(L) * q / m / (1.0 - N * e * e / (N + 40.0038)));
      double H = (m / cos(L) / cos(B) - N + n / sin(L) / cos(B) - N) / 2.0;
      *lo = L / M_PI * 180.0;
      *la = B / M_PI * 180.0;
      *el = H;
    } catch(...) {
      fprintf(stderr, "wgstogps exception: %f %f %f",
                       m, n, q);
      exit(-1);
    }
  }

  // gps -> geodetic coordinates releative to anchor point
  void gpstolocal(double longitude, double latitude, double altitude,
                  double *L, double *B, double *h) {
    gpstowgs(longitude, latitude, altitude, L, B, h);
    *L -= GPS_anchor_xyz_[0];
    *B -= GPS_anchor_xyz_[1];
    *h -= GPS_anchor_xyz_[2];
  }

  // geodetic -> gps coordinates releative to anchor point
  void localtogps(double m, double n, double q,
                  double *lo, double *la, double *el) {
    m += GPS_anchor_xyz_[0];
    n += GPS_anchor_xyz_[1];
    q += GPS_anchor_xyz_[2];
    wgstogps(m, n, q, lo, la, el);
  }

 private:
  // matrix calibration mode:
  // True : frank mode, directly use matrix divide and inverse
  // False: xi peng mode(wgs mode), use geodetic coordinates and svd
  bool wgs_ = false;
  // wgs mode: the first lidar zero point gps
  double origin_point_gps_[3] = {0, 0, 0};
  double GPS_CurvatureRadius_ = 0.0;
  double GPS_anchor_xyz_[3] = {0, 0, 0};

  double xyz2gps_[4][4];
  double gps2xyz_[4][4];
  double xyz2xyz_[4][4];    // other channels to reference channel

  bool ready_ = false;
  bool xyz2gps_ready_[4] {false, false, false, false};
  bool gps2xyz_ready_[4] {false, false, false, false};

  bool ready_xyz_ = false;
  bool xyz2xyz_ready_[4] {false, false, false, false};
};

#endif    // POINTCLOUD_SERVER_COMMON_POINTCLOUD_MATRIX_CALIBRATION_H_
