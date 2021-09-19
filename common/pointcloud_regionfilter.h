
/**
 *  Copyright (C) 2019 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef POINTCLOUD_SERVER_COMMON_POINTCLOUD_REGIONFILTER_H_
#define POINTCLOUD_SERVER_COMMON_POINTCLOUD_REGIONFILTER_H_

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "src/inno_lidar_api.h"
#include "src/inno_lidar_api_experimental.h"
#include "pointcloud_kvp.h"       // NOLINT

// in kvp file:
//   token = pcs_regionfilter_coordinate
//   value = xyz or gps   (default gps)
//
//   token = pcs_regionfilter_height
//   value = meter   (default gps)
//
//   token = pcs_regionfilter
//   value = subtoken:subvalue , subtoken:subvalue , subtoken:subvalue ...
//   subtoken  subvalue
//     id      integer
//     type    '+' positive, display area
//             '-' negative, not display area
//     name    string (optional) do not use comma because it is delimiter
//     otherwise  x:y
//
//   points should be clockwise or anticlockwise
//   latter region has priority
//   if first region is negtive, then boundary is postive, vice versa
//

namespace RegionFilterSp {

//---------------------------------------------------------------

class Point {
 public:
  Point() : x(0), y(0) {}
  Point(double xi, double yi) : x(xi), y(yi) {}
  double x;
  double y;
};

//---------------------------------------------------------------

enum class RegionType {
  Negative = 0,   // no display area
  Positive = 1,   // display area
};

//---------------------------------------------------------------

class Region {
 public:
  Region() : id_(0), type_(RegionType::Positive), name_("") {}

  int get_id() const noexcept { return id_; }
  void set_id(int id) noexcept { id_ = id; }

  RegionType get_type() const noexcept { return type_; }
  void set_type(RegionType t) noexcept { type_ = t; }

  const std::string &get_name() const noexcept { return name_; }
  void set_name(std::string s) noexcept { name_ = s; }

  void PrintInfo() {
    std::cout.setf(std::ios::fixed);
    std::cout << "region id=" << id_ << " " <<
           "type=" <<
           (type_ == RegionType::Positive ? "postive " : "negtive ") <<
           "name=" << name_ << " points=";
    for (const auto &p : points_) {
      std::cout << p.x << ":" << p.y << " ";
    }
    std::cout << std::endl;
  }

  int SetValue(const std::string &token, const std::string &value) {
    if (token == "id") {
      set_id(stoi(value));
    } else if (token == "type") {
      if (value == "+") {
        set_type(RegionType::Positive);
      } else if (value == "-") {
        set_type(RegionType::Negative);
      } else {
        fprintf(stderr, "region type(+/-) error (%s)\n", value.c_str());
        return -1;
      }
    } else if (token == "name") {
      set_name(value);
    } else {
      AddPoint(stod(token), stod(value));
    }
    return 0;
  }

  void AddPoint(double x, double y) {
    points_.push_back(Point(x, y));
  }

  bool IsInside(double x, double y) const noexcept {
    size_t j = points_.size()-1;
    bool oddNodes = false;

    for (size_t i = 0; i < points_.size(); i++) {
      const Point &pi = points_[i];
      const Point &pj = points_[j];
      if ((pi.y < y && pj.y >= y) || (pj.y < y && pi.y >= y)) {
        if (x < pi.x + (y - pi.y) / (pj.y - pi.y) * (pj.x - pi.x)) {
          oddNodes = !oddNodes;
        }
      }
      j = i;
    }
    return oddNodes;
  }

 private:
  int id_;
  RegionType type_;
  std::string name_;
  std::vector<Point> points_;
};

//---------------------------------------------------------------

const double kInvalidHeight = 9999.0;

class RegionFilter {
 public:
  RegionFilter() {
    Init();
  }

  void Init() {
    has_filter_parameters_ = false;
    height_ = kInvalidHeight;
    use_gps_ = true;              // true: use gps    false: use xyz
    regions_.clear();
  }

  // is already set filter paramters?
  bool has_filter_param() const noexcept {
      return has_filter_parameters_;
  }

  int set_height(const KvpItem &item) noexcept {
    if (item.value.size() == 0) {
      fprintf(stderr, "region filter set height error : no value\n");
      return -1;
    } else {
      height_ = stod(item.value[0]);
      has_filter_parameters_ = true;
      fprintf(stderr, "region filter set height : %f\n", height_);
      return 0;
    }
  }

  double get_height() const noexcept { return height_; }

  int SetCoordinate(const KvpItem &item) noexcept {
    if (item.value.size() == 0) {
      fprintf(stderr, "region filter SetCoordinate error : no value\n");
      return -1;
    }

    std::string s = item.value[0];
    if (s == "xyz") {
      use_gps_ = false;
      fprintf(stderr, "region filter use xyz\n");
    } else if (s == "gps") {
      use_gps_ = true;
      fprintf(stderr, "region filter use gps\n");
    } else {
      fprintf(stderr, "RegionFilter coodinate error (%s)\n", s.c_str());
      return -1;
    }
    return 0;
  }

  bool IsUseGps() const noexcept { return use_gps_; }

  int AddRegion(const KvpItem &item) {
    if (item.value.size() == 0) {
      fprintf(stderr, "region filter SetCoordinate error : no value\n");
      return -1;
    }

    Region r;
    for (const auto &t : item.sub_value) {
      if (t.size() == 2) {
        r.SetValue(t[0], t[1]);
      } else {
        fprintf(stderr, "region filter AddRegion error : not 2 value\n");
        return -1;
      }
    }
    regions_.push_back(r);
    r.PrintInfo();
    has_filter_parameters_ = true;
    return 0;
  }

  RegionType CheckHeight(double h,
                         std::string &prompt) const noexcept {   // NOLINT
    if (h > height_) {
      prompt = "filter: bbox height " + std::to_string(h) +
               " is higher than " + std::to_string(height_);
      return RegionType::Negative;
    } else {
      return RegionType::Positive;
    }
  }

  // return 0: no objects filtered
  //        1: have objects filtered
  //           already free old frame, generate new frame
  int CheckRegions(inno_cframe_header **frame,
          std::vector<std::string> &prompts) const noexcept {   // NOLINT
    if (!has_filter_param()) {
      return 0;
    }

    inno_cframe_header *oldf = *frame;   // old frame
    if (oldf->type != INNO_CFRAME_BBOX) {
      return 0;
    }

    std::vector<unsigned int> regular_obj_index;
    for (unsigned int i = 0; i < oldf->item_number; ++i) {
      std::string prompt;
      if (CheckRegions(oldf->bboxes[i], prompt) != RegionType::Negative) {
        regular_obj_index.push_back(i);
      } else {
        prompts.push_back(prompt);
      }
    }

    unsigned int num = regular_obj_index.size();
    if (oldf->item_number == num) {
      return 0;
    }

    inno_cframe_header *h = reinterpret_cast<inno_cframe_header *>
          (calloc(1, sizeof(inno_cframe_header) + sizeof(inno_bbox) * num));
    if (!h) {
      fprintf(stderr, "out of memory, calloc bbox, number=%d\n", num);
      return 0;
    }
    memcpy(h, oldf, sizeof(inno_cframe_header));
    h->item_number = num;
    for (unsigned int i = 0; i < num; ++i) {
      unsigned int j = regular_obj_index[i];
      memcpy(&h->bboxes[i], &oldf->bboxes[j], sizeof(inno_bbox));
    }
    free(oldf);
    *frame = h;
    return 1;
  }

  RegionType CheckRegions(const inno_bbox & b,
                          std::string &prompt) const noexcept {   // NOLINT
    if (!has_filter_param()) {
      return RegionType::Positive;
    }

    if (IsUseGps()) {
      return CheckRegions(b.longtitude,
                          b.latitude,
                          b.elevation/100.0 + b.height / 2,
                          prompt);
    } else {
      return CheckRegions(b.y, b.z, b.x + b.height / 2, prompt);
    }
  }

  RegionType CheckRegions(double x, double y, double h,
                          std::string &prompt) const noexcept {   // NOLINT
    if (!has_filter_param()) {
      return RegionType::Positive;
    }

    if (CheckHeight(h, prompt) == RegionType::Negative) {
      return RegionType::Negative;
    }

    if (regions_.empty()) {
      prompt = "";
      return RegionType::Positive;
    }
    // latter region has priority
    for (auto it = regions_.rbegin(); it != regions_.rend(); ++it) {
      if (it->IsInside(x, y)) {
        prompt = "filter: bbox in region[" +
                 std::to_string(it->get_id()) + "] " +
                 it->get_name() +
                 (it->get_type() == RegionType::Positive ?
                  " positive" : " negative") +
                 (IsUseGps() ? " long=" : " x=") + std::to_string(x) +
                 (IsUseGps() ? " lat= " : " y=") + std::to_string(y) +
                 (IsUseGps() ? " elev=" : " h=") + std::to_string(h);
        return it->get_type();
      }
    }
    // not found
    RegionType t;
    if (regions_[0].get_type() == RegionType::Positive) {
      t = RegionType::Negative;
    } else {
      t = RegionType::Positive;
    }
    prompt = std::string("point in boundary ") +
             (t == RegionType::Positive ? "positive" : "negative");
    return t;
  }

 private:
  // init very big height, then height filter will not work
  bool has_filter_parameters_;
  double height_;
  bool use_gps_;              // true: use gps    false: use xyz
  std::vector<Region> regions_;
};

}  // namespace RegionFilterSp

#endif  // POINTCLOUD_SERVER_COMMON_POINTCLOUD_REGIONFILTER_H_
