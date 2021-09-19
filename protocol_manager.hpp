#ifndef __PROTOCOL_MANEGER_HPP
#define __PROTOCOL_MANEGER_HPP
#include <dlfcn.h>
#include <sys/auxv.h>

#include <algorithm>
#include <future>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "./3rdparty/concurrentqueue/concurrentqueue.h"
#include "./3rdparty/json.hpp"
#include "./common/pointcloud_logger.h"
#include "./config_manager.hpp"
#include "./plugin_base.h"
#include "./protocol_websocket.hpp"

using namespace std;
using namespace moodycamel;

class protocol {
 public:
  protocol(){};
  virtual ~protocol(){};
  virtual void send_message(message msg, string ip = "", int port = 0){};
  virtual vector<json> read_message() {
    vector<json> jsonlist;
    return jsonlist;
  };
};

class protocol_websocket : public protocol, public broadcast_server {
 public:
  protocol_websocket(int port) : protocol(), broadcast_server(), sub_thread() {
    run(port);
  };
  ~protocol_websocket() {
    stop_server();
    if (sub_thread.joinable()) sub_thread.join();
  };
  void run(int port) {
    if (port > 0) sub_thread = std::thread(&broadcast_server::run, this, port);
  };
  void send_message(message msg, string ip = "", int port = 0) {
    con_list::iterator it;
    try {
      if (!msg.j.empty()) {
        lock_guard<mutex> guard(m_connection_lock);
        auto send_stream = msg.j.dump();
        for (it = m_connections.begin(); it != m_connections.end(); ++it) {
          ws_server.send(*it, send_stream, websocketpp::frame::opcode::text);
        };
      }
      if (msg.len && msg.data) {
        lock_guard<mutex> guard(m_connection_lock);
        for (it = m_connections.begin(); it != m_connections.end(); ++it) {
          ws_server.send(*it, msg.data, msg.len,
                         websocketpp::frame::opcode::binary);
        };
      }
    } catch (const json::exception& e) {
      std::cout << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                << __FUNCTION__ << std ::endl;
    } catch (const websocketpp::lib::error_code e) {
      std::cerr << e.message() << std::endl;
    } catch (const std::exception& e) {
      std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                << __FUNCTION__ << std ::endl;
    } catch (...) {
      std ::cerr << "unknown exception"
                 << " " << __FILE__ << " " << __LINE__ << " " << __FUNCTION__
                 << std ::endl;
    }
  }

  vector<json> read_message() {
    vector<json> jsonlist;
    unique_lock<mutex> lock(m_action_lock);
    if (m_actions.empty()) return jsonlist;
    action a = m_actions.front();
    m_actions.pop();
    lock.unlock();
    if (a.type == SUBSCRIBE) {
      lock_guard<mutex> guard(m_connection_lock);
      m_connections.insert(a.hdl);
    } else if (a.type == UNSUBSCRIBE) {
      lock_guard<mutex> guard(m_connection_lock);
      m_connections.erase(a.hdl);
    } else if (a.type == MESSAGE) {
      lock_guard<mutex> guard(m_connection_lock);
      con_list::iterator it;
      try {
        json j = json::parse(a.msg->get_payload());
        jsonlist.push_back(j);
      } catch (const json::exception& e) {
        std::cout << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                  << __FUNCTION__ << std ::endl;
      } catch (const websocketpp::lib::error_code e) {
        std::cerr << e.message() << std::endl;
      } catch (const std::exception& e) {
        std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                  << __FUNCTION__ << std ::endl;
      } catch (...) {
        std ::cerr << "unknown exception" << __FILE__ << __LINE__
                   << __FUNCTION__ << std ::endl;
      }
      for (it = m_connections.begin(); it != m_connections.end(); ++it) {
        try {
          ws_server.send(*it, a.msg);
        } catch (websocketpp::lib::error_code e) {
          std::cerr << e.message() << std::endl;
        } catch (const std::exception& e) {
          std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                    << __FUNCTION__ << std ::endl;
        } catch (...) {
          std ::cerr << "unknown exception" << __FILE__ << __LINE__
                     << __FUNCTION__ << std ::endl;
        }
      }
    } else {
      // undefined msg type
    }
    return jsonlist;
  };

 private:
  std::thread sub_thread;
  vector<json> con_list;
};

class protocol_udp : public protocol {
 public:
  protocol_udp(int Bind_Port = 0)
      : io_service_udp(), socket_udp(io_service_udp) {
    socket_udp.open(boost::asio::ip::udp::v4());
    socket_udp.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_udp.non_blocking(true);
    local_endpoint = nullptr;
    if (Bind_Port > 0) {
      bind_port = Bind_Port;
      local_endpoint = new boost::asio::ip::udp::endpoint(
          boost::asio::ip::address::from_string("0.0.0.0"), Bind_Port);
      try {
        socket_udp.bind(*local_endpoint);
      } catch (const std::exception& e) {
        std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                  << __FUNCTION__ << std ::endl;
      } catch (...) {
        std ::cerr << "unknown exception" << __FILE__ << __LINE__
                   << __FUNCTION__ << std ::endl;
      }
    }
  };
  ~protocol_udp() {
    if (local_endpoint != nullptr) delete local_endpoint;
    socket_udp.close();
    io_service_udp.stop();
  }

  vector<json> read_message() {
    char buff[4096];
    int len = 0;
    vector<json> jsonlist;
    if (bind_port == 0) return jsonlist;
    boost::system::error_code error;
    while (error == boost::asio::error::would_block) {
      try {
        len = socket_udp.receive_from(boost::asio::buffer(buff, sizeof(buff)),
                                      *local_endpoint, 0, error);
        if (len > 0) {
          string s = buff;
          json j = json::parse(s);
          jsonlist.push_back(j);
        }
      } catch (const json::exception& e) {
        std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                  << __FUNCTION__ << std ::endl;
      } catch (const std::exception& e) {
        std::cerr << e.what() << " " << __FILE__ << " " << __LINE__ << " "
                  << __FUNCTION__ << std ::endl;
      } catch (...) {
        std ::cerr << "unknown exception" << __FILE__ << __LINE__
                   << __FUNCTION__ << std ::endl;
      }
    }
    return jsonlist;
  };

  void send_message(message msg, string ip = "", int port = 0) {
    if (ip == "" || port == 0) return;
    if (ip != dest_ip || port != dest_port) {
      dest_ip = ip;
      dest_port = port;
    }
    boost::asio::ip::udp::endpoint remote_endpoint(
        boost::asio::ip::address::from_string(dest_ip), dest_port);

    if (!msg.j.empty()) {
      auto send_stream = msg.j.dump();
      socket_udp.send_to(boost::asio::buffer(send_stream, send_stream.size()),
                         remote_endpoint);
    }
    if (msg.len && msg.data) {
      socket_udp.send_to(boost::asio::buffer(msg.data, msg.len),
                         remote_endpoint);
    }
  };

 private:
  boost::asio::io_service io_service_udp;
  boost::asio::ip::udp::socket socket_udp;
  boost::asio::ip::udp::endpoint* local_endpoint;
  string dest_ip = "";
  int dest_port = 0;
  int bind_port = 0;
};

class protocol_manager : public Singleton<protocol_manager> {
  // enum protocol_type { UDP, WS, TCP, OTH };
  struct protocol_attr {
    protocol* obj;
    int port;                                       // bind port
    std::map<string, std::map<string, int>> dests;  // ip port
  };

 public:
  protocol_manager(token) : sub_thread() {
    running = 1;
    throughput[0] = 0;  // out
    throughput[1] = 0;  // int
    sub_thread = std::thread(&protocol_manager::monitor_protocol_msgs, this);
  };
  ~protocol_manager() {
    running = 0;
    if (sub_thread.joinable()) sub_thread.join();
    for (auto plugin = messages.begin(); plugin != messages.end();) {
      while ((*plugin->second).size_approx() >= 1) {
        message msg;
        if ((*plugin->second).try_dequeue(msg) == false) break;
        if (msg.data) free(msg.data);
      }
      delete plugin->second;
      messages.erase(plugin++);
    }
  };
  protocol_manager(const protocol_manager&) = delete;
  protocol_manager& operator=(const protocol_manager&) = delete;
  void register_message(string name, vector<message> msgs) {
    if (!name.empty()) {
      if (messages.count(name) == 0)
        messages[name] = new moodycamel::ConcurrentQueue<message>;
      for (auto& msg : msgs) {
        (*messages[name]).enqueue(msg);
      }
    } else {
      for (auto& msg : msgs) {
        if (msg.data) free(msg.data);
      }
      msgs.clear();
    }
  };
  void monitor_protocol_msgs() {
    time_t now = time(NULL);
    time_t ts_monitor = time(NULL);
    json newest_cfg;
    while (running) {
      {
        // sync protocol cfg persecond
        if (now + 1 < time(NULL)) {
          now = time(NULL);
          newest_cfg = config_manager::get_instance().get_newest_cfg();
          for (auto& plugin : messages) {
            json plugin_protocol_cfg;
            if (!newest_cfg[plugin.first]["protocol"].is_null()) {
              // update own plugin_cfg
              plugin_protocol_cfg.update(newest_cfg[plugin.first]["protocol"]);
              // protocol cfg changed ,need init or reinit
              if (protocol_cfgs[plugin.first] != plugin_protocol_cfg) {
                protocol_cfgs[plugin.first] = plugin_protocol_cfg;
                try {
                  for (auto& type : plugin_protocol_cfg.items()) {
                    // checkout dest ip:port, maybe changed
                    if (type.value().contains("dest")) {
                      plugin_protocols[plugin.first][type.key()].dests.clear();
                      for (auto& dest : type.value()["dest"].items()) {
                        string src = dest.value().get<std::string>();
                        std::smatch match;
                        std::regex reg(
                            "(\\d+\\.\\d+\\.\\d+\\.\\d+):(\\d{2,5})");
                        if (std::regex_search(src, match, reg) &&
                            match.size() >= 2) {
                          string dest_ip = match[1];
                          int dest_port = std::stoi(match[2]);
                          plugin_protocols[plugin.first][type.key()]
                              .dests[src][dest_ip] = dest_port;
                          inno_log_info(
                              "%sProtocolManager: UPDATE %s %s DEST %s%s",
                              LOG_CLRSTR_GREEN, plugin.first.c_str(),
                              type.key().c_str(), src.c_str(), LOG_CLRSTR_NONE);
                        } else
                          inno_log_info(
                              "%sProtocolManager: INVALID %s %s DEST %s%s",
                              LOG_CLRSTR_YELLOW, plugin.first.c_str(),
                              type.key().c_str(), src.c_str(), LOG_CLRSTR_NONE);
                      }
                    }
                    // check bind_port, maybe changed
                    if (type.value().contains("port")) {
                      int port = type.value()["port"].get<int>();
                      // port changed, delete
                      if (plugin_protocols[plugin.first][type.key()].obj &&
                          port !=
                              plugin_protocols[plugin.first][type.key()].port) {
                        inno_log_info(
                            "%sProtocolManager: RESET %s %s BIND %d->%d%s",
                            LOG_CLRSTR_YELLOW, plugin.first.c_str(),
                            type.key().c_str(),
                            plugin_protocols[plugin.first][type.key()].port,
                            port, LOG_CLRSTR_NONE);
                        delete plugin_protocols[plugin.first][type.key()].obj;
                        plugin_protocols[plugin.first].erase(type.key());
                      }
                      // checkou if port been bind
                      int already_bind = 0;
                      if (port > 0) {
                        for (auto exist : plugin_protocols) {
                          if (exist.second[type.key()].port == port) {
                            already_bind++;
                            // already_bind by oth plugin,pass and log
                            if (exist.first != plugin.first)
                              inno_log_info(
                                  "%sProtocolManager: ALREADY BIND %s[%d],PASS "
                                  "%s[%d]%s",
                                  LOG_CLRSTR_YELLOW, exist.first.c_str(), port,
                                  plugin.first.c_str(), port, LOG_CLRSTR_NONE);
                          }
                        }
                        plugin_protocols[plugin.first][type.key()].port = port;
                      }
                      // init
                      if (already_bind == 0 && port >= 0) {
                        if (type.key() == "udp") {
                          plugin_protocols[plugin.first][type.key()].obj =
                              new protocol_udp(port);
                        } else if (type.key() == "ws" && port > 0) {
                          plugin_protocols[plugin.first][type.key()].obj =
                              new protocol_websocket(port);
                        } else if (type.key() == "tcp")
                          ;
                        inno_log_info(
                            "%sProtocolManager: %s %s %s BIND 0.0.0.0:%d%s",
                            LOG_CLRSTR_YELLOW, port > 0 ? "VALID" : "INVALID",
                            plugin.first.c_str(), type.key().c_str(), port,
                            LOG_CLRSTR_NONE);
                      }
                    }
                  }
                } catch (const json::exception& e) {
                  std::cerr << e.what() << " " << __FILE__ << " " << __LINE__
                            << " " << __FUNCTION__ << std ::endl;
                } catch (const std::exception& e) {
                  std::cerr << e.what() << " " << __FILE__ << " " << __LINE__
                            << " " << __FUNCTION__ << std ::endl;
                } catch (...) {
                  std ::cerr << "unknown exception" << __FILE__ << __LINE__
                             << __FUNCTION__ << std ::endl;
                }
              }
            } else if (newest_cfg.contains(plugin.first)) {
              // no protocol fields ,set some example
              inno_log_info("%sProtocolManager: EXAMPLE %s in kvp%s",
                            LOG_CLRSTR_YELLOW, plugin.first.c_str(),
                            LOG_CLRSTR_NONE);
              json plugin_protocol_cfg;
              json udp;
              json ws;
              udp["port"] = 0;
              udp["dest"].push_back("127.0.0.1:0");
              ws["port"] = 0;
              plugin_protocol_cfg["udp"].update(udp);
              plugin_protocol_cfg["ws"].update(ws);
              newest_cfg[plugin.first]["protocol"].update(plugin_protocol_cfg);
              config_manager::get_instance().update_newest_cfg(newest_cfg);
            }
          }
        }
        // send msg
        for (auto& plugin : messages) {
          while ((*plugin.second).size_approx() >= 1) {
            message msg;
            if ((*plugin.second).try_dequeue(msg) == false) break;
            // monitor out/int byte
            int playload_len = 0;
            if (!msg.j.empty()) {
              auto send_stream = msg.j.dump();
              playload_len += send_stream.size();
            }
            if (msg.len && msg.data) {
              playload_len += msg.len;
            }
            for (auto protocol : plugin_protocols[plugin.first]) {
              if (protocol.first == "udp" && protocol.second.dests.size()) {
                for (auto iter : protocol.second.dests) {
                  for (auto dest : iter.second) {
                    if (protocol.second.obj)
                      protocol.second.obj->send_message(msg, dest.first,
                                                        dest.second);
                    throughput[0] += playload_len;
                  }
                }
              }
              if (protocol.first == "ws") {
                if (protocol.second.obj) {
                  protocol.second.obj->send_message(msg, "", 0);
                  throughput[0] += playload_len;
                }
              }
            }
            if (msg.data) free(msg.data);
          }
        }
        // read msg
        json tmp = newest_cfg;
        for (auto plugin : plugin_protocols) {
          vector<json> jsonlist;
          for (auto protocol : plugin.second) {
            if (protocol.second.obj)
              jsonlist = protocol.second.obj->read_message();
          }
          for (auto& j : jsonlist) {
            auto recv_stream = j.dump();
            throughput[1] += recv_stream.size();
            tmp[plugin.first].update(j);
          }
          if (tmp != newest_cfg) {
            config_manager::get_instance().update_newest_cfg(tmp);
          }
        }
        // Monitor info
        if (ts_monitor + 4 < time(NULL)) {
          ts_monitor = time(NULL);
          inno_log_info(
              "%sProtocolManager: Network Traffic, up %.3f, down %.3f (Mb/s)%s",
              LOG_CLRSTR_GREEN, throughput[0] / 1024.0 / 1024.0 / 5,
              throughput[1] / 1024.0 / 1024.0 / 5, LOG_CLRSTR_NONE);
          throughput[0] = 0;  // out
          throughput[1] = 0;  // in
        }
      }
      usleep(1000);
    }
  };

 private:
  std::thread sub_thread;
  int running;
  std::recursive_mutex msg_lock;
  uint64_t throughput[2];
  std::map<string, moodycamel::ConcurrentQueue<message>*>
      messages;                          // plugin-name msg
  std::map<string, json> protocol_cfgs;  // name cfg
  std::map<string, std::map<string, struct protocol_attr>>
      plugin_protocols;  // plugin-name  udp/ws/...  protocol_attr
};

#endif