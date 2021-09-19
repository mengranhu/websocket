/*
 * @Author: Peng Xi
 * @Date: 2020-10-28 10:37:58
 * @LastEditors: Peng Xi
 * @LastEditTime: 2020-10-29 14:57:47
 * @Description: file content
 */
#ifndef PLUGIN_BASE_HPP
#define PLUGIN_BASE_HPP

#include <iostream>

#include "json.hpp"
#include "src/inno_lidar_api.h"
#include "src/inno_lidar_api_experimental.h"

using json = nlohmann::json;
using namespace std;

/*
You need to inherit this class and implement makfile,
The important thing is:
1. You need to ensure that manager can delete the class normally
2. You need to ensure that there is no re-bind problem
3. Try to reduce execution time as much as possible if it is synchronous func
*/
struct message {
  int type;  // 0:json 1:raw
  json j;
  int len;
  char* data;
};

class plugin_base {
 public:
  plugin_base(){};
  virtual ~plugin_base(){};
  virtual vector<message> sync_process(inno_cframe_header** frame,
                                        void* ext = NULL) = 0;
  // get global/own config,return own cfg
  virtual json sync_cfg(json j, void* ext = NULL) = 0;
  virtual string get_name() = 0;  // specify the unique name to auto sync config
  virtual string get_verison() = 0;      // just for you to identify
  virtual string get_description() = 0;  // just for you to identify
  virtual string get_priority() = 0;     // specify the MAJOR.MINOR priority
};

extern "C" {
typedef plugin_base* create_t();
typedef void destroy_t(plugin_base*);
}

#endif