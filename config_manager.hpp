#ifndef __CONFIG_MANAGER_HPP
#define __CONFIG_MANAGER_HPP

#define LOG_CLRSTR_NONE "\033[m"
#define LOG_CLRSTR_RED "\033[0;32;31m"
#define LOG_CLRSTR_GREEN "\033[0;32;32m"
#define LOG_CLRSTR_BLUE "\033[0;32;34m"
#define LOG_CLRSTR_DARY_GRAY "\033[1;30m"
#define LOG_CLRSTR_CYAN "\033[0;36m"
#define LOG_CLRSTR_PURPLE "\033[0;35m"
#define LOG_CLRSTR_BROWN "\033[0;33m"
#define LOG_CLRSTR_YELLOW "\033[1;33m"
#define LOG_CLRSTR_WHITE "\033[1;37m"

#include <dlfcn.h>
#include <sys/auxv.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "./3rdparty/json.hpp"
#include "./common/pointcloud_logger.h"
#include "./plugin_base.h"
using namespace std;
#ifndef __SINGLETON__
#define __SINGLETON__
template <typename T>
class Singleton {
 public:
  static T& get_instance() noexcept(std::is_nothrow_constructible<T>::value) {
    static T instance{token()};
    return instance;
  }
  virtual ~Singleton() = default;
  Singleton(const Singleton&) = delete;
  Singleton& operator=(const Singleton&) = delete;

 protected:
  struct token {};  // helper class
  Singleton() noexcept = default;
};
#endif

class config_manager : public Singleton<config_manager> {
 public:
  config_manager(token) : sub_thread() {
    running = 1;
    sub_thread = std::thread(&config_manager::sync_persistence_cfg, this);
  };
  ~config_manager() {
    running = 0;
    if (sub_thread.joinable()) sub_thread.join();
  };
  config_manager(const config_manager&) = delete;
  config_manager& operator=(const config_manager&) = delete;
  json get_newest_cfg() {
    std::lock_guard<std::recursive_mutex> lock(cfg_lock);
    return persistence_newest_cfg;
  };
  json get_old_cfg() {
    std::lock_guard<std::recursive_mutex> lock(cfg_lock);
    return persistence_old_cfg;
  };
  void update_newest_cfg(json j) {
    if (j.empty()) return;
    std::lock_guard<std::recursive_mutex> lock(cfg_lock);
    try {
      persistence_newest_cfg.update(j);
    } catch (const json::exception& e) {
      std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                << __FUNCTION__ << std ::endl;
    }
  };
  void update_old_cfg(json j) {
    if (j.empty()) return;
    std::lock_guard<std::recursive_mutex> lock(cfg_lock);
    try {
      persistence_old_cfg.update(j);
    } catch (const json::exception& e) {
      std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                << __FUNCTION__ << std ::endl;
    }
  };
  void set_persistence_path(const std::string path = "") {
    std::lock_guard<std::recursive_mutex> lock(cfg_lock);
    persistence_path = path;
    memset(&cfg_stat, 0, sizeof(cfg_stat));
  };
  json load_cfg_from_file(const std::string path) {
    json j;
    std::ifstream loading(path);
    if (loading) {
      try {
        loading >> j;
      } catch (const nlohmann::detail::parse_error& e) {
        // inno_log_error("ConfigManager: Parse cfg[err]");
      } catch (const json::exception& e) {
        std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                  << __FUNCTION__ << std ::endl;
      } catch (const std::exception& e) {
        std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                  << __FUNCTION__ << std ::endl;
      } catch (...) {
        std ::cerr << "unknown exception"
                   << " " << __FILE__ << " " << __LINE__ << " " << __FUNCTION__
                   << std ::endl;
      }
    }
    return j;
  };
  int dump_cfg_to_file(const std::string path, json j) {
    if (j.empty() || path.empty()) return -1;
    std::ofstream dumping(path);
    if (dumping) {
      try {
        dumping << j.dump(4) << std::endl;
      } catch (const json::exception& e) {
        std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                  << __FUNCTION__ << std ::endl;
      } catch (const std::exception& e) {
        std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                  << __FUNCTION__ << std ::endl;
      } catch (...) {
        std ::cerr << "unknown exception"
                   << " " << __FILE__ << " " << __LINE__ << " " << __FUNCTION__
                   << std ::endl;
      }
      return 0;
    } else {
      inno_log_error("%sConfigManager: DUMP cfg[err path] %s%s", LOG_CLRSTR_RED,
                     path.c_str(), LOG_CLRSTR_NONE);
    }
    return -1;
  };

 private:
  void sync_persistence_cfg() {
    while (running) {
      sleep(1);
      json file_cfg;
      struct stat attr;
      try {
        if (stat(persistence_path.c_str(), &attr) == 0) {
          // file exist
          file_cfg = load_cfg_from_file(persistence_path);
          std::lock_guard<std::recursive_mutex> lock(cfg_lock);
          if (file_cfg.empty()) {
            // old cfg or empty cfg
            update_newest_cfg(persistence_old_cfg);
            if (dump_cfg_to_file(persistence_path, persistence_newest_cfg) ==
                0) {
              stat(persistence_path.c_str(), &cfg_stat);
              inno_log_info("%sConfigManager: DUMP cfg[reinit]%s",
                            LOG_CLRSTR_YELLOW, LOG_CLRSTR_NONE);
            }
          } else {
            if (attr.st_mtime != cfg_stat.st_mtime &&
                file_cfg != persistence_newest_cfg) {
              // been edit
              update_newest_cfg(file_cfg);
              stat(persistence_path.c_str(), &cfg_stat);
              inno_log_info("%sConfigManager: LOAD cfg[file edit]%s",
                            LOG_CLRSTR_YELLOW, LOG_CLRSTR_NONE);
            } else if (attr.st_mtime != cfg_stat.st_mtime &&
                       file_cfg == persistence_newest_cfg) {
              // noyet sync stat
              stat(persistence_path.c_str(), &cfg_stat);
              inno_log_info("%sConfigManager: SYNC stat[init]%s",
                            LOG_CLRSTR_GREEN, LOG_CLRSTR_NONE);
            } else if (attr.st_mtime == cfg_stat.st_mtime &&
                       file_cfg != persistence_newest_cfg) {
              if (file_cfg.contains("accept_sync_from_plugin")) {
                // check if accept sync cfg from plugin
                if (file_cfg["accept_sync_from_plugin"] == "true") {
                  dump_cfg_to_file(persistence_path, persistence_newest_cfg);
                  stat(persistence_path.c_str(), &cfg_stat);
                  inno_log_info("%sConfigManager: SYNC cfg[plugin push]%s",
                                LOG_CLRSTR_GREEN, LOG_CLRSTR_NONE);
                }
              } else {
                dump_cfg_to_file(persistence_path, persistence_newest_cfg);
                stat(persistence_path.c_str(), &cfg_stat);
                inno_log_info("%sConfigManager: SYNC cfg[manager push]%s",
                              LOG_CLRSTR_GREEN, LOG_CLRSTR_NONE);
              }
            } else {
              // no need to sync,check global cfg
              if (!file_cfg.contains("accept_sync_from_plugin")) {
                persistence_newest_cfg["accept_sync_from_plugin"] = "true";
              }
            }
          }
        } else {
          // file delete
          update_newest_cfg(persistence_old_cfg);
          if (dump_cfg_to_file(persistence_path, persistence_newest_cfg) == 0) {
            stat(persistence_path.c_str(), &cfg_stat);
            inno_log_info("%sConfigManager: DUMP cfg[reinit from delete]%s",
                          LOG_CLRSTR_YELLOW, LOG_CLRSTR_NONE);
          }
        }
      } catch (const json::exception& e) {
        std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                  << __FUNCTION__ << std ::endl;
      } catch (const std::exception& e) {
        std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                  << __FUNCTION__ << std ::endl;
      } catch (...) {
        std ::cerr << "unknown exception"
                   << " " << __FILE__ << " " << __LINE__ << " " << __FUNCTION__
                   << std ::endl;
      }
      sleep(1);
    }
  };

 private:
  std::string persistence_path;
  json persistence_newest_cfg;
  json persistence_old_cfg;
  struct stat cfg_stat;
  std::thread sub_thread;
  int running;
  std::recursive_mutex cfg_lock;
};
#endif