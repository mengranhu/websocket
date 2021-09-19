#include <iostream>
#include <set>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

/*#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>*/
#include <boost/thread/mutex.hpp>
#include <websocketpp/common/thread.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::connection_hdl;
using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;

using websocketpp::lib::condition_variable;
using websocketpp::lib::lock_guard;
using websocketpp::lib::mutex;
using websocketpp::lib::thread;
using websocketpp::lib::unique_lock;

/* on_open insert connection_hdl into channel
 * on_close remove connection_hdl from channel
 * on_message queue send to all channels
 */

enum action_type { SUBSCRIBE, UNSUBSCRIBE, MESSAGE };

struct action {
  action(action_type t, connection_hdl h) : type(t), hdl(h) {}
  action(action_type t, connection_hdl h, server::message_ptr m)
      : type(t), hdl(h), msg(m) {}

  action_type type;
  websocketpp::connection_hdl hdl;
  server::message_ptr msg;
};

class broadcast_server {
 public:
  broadcast_server() {
    // Initialize Asio Transport
    ws_server.init_asio();
    ws_server.set_reuse_addr(true);

    // Register handler callbacks
    ws_server.set_open_handler(bind(&broadcast_server::on_open, this, ::_1));
    ws_server.set_close_handler(bind(&broadcast_server::on_close, this, ::_1));
    ws_server.set_message_handler(
        bind(&broadcast_server::on_message, this, ::_1, ::_2));
    ws_server.set_access_channels(websocketpp::log::alevel::none);
    ws_server.clear_access_channels(websocketpp::log::alevel::frame_header |
                                    websocketpp::log::alevel::frame_payload);
    ws_server.clear_access_channels(websocketpp::log::alevel::all);
  }
  ~broadcast_server() {
    // ws_server.stop();
  }
  void run(uint16_t port) {
    try {
      ws_server.listen(port);
      ws_server.start_accept();
      ws_server.run();
    } catch (websocketpp::exception const& e) {
      std::cout << e.what() << std::endl;
    }
  }

  void on_open(connection_hdl hdl) {
    {
      lock_guard<mutex> guard(m_action_lock);
      m_actions.push(action(SUBSCRIBE, hdl));
    }
    m_action_cond.notify_one();
  }

  void on_close(connection_hdl hdl) {
    {
      lock_guard<mutex> guard(m_action_lock);
      m_actions.push(action(UNSUBSCRIBE, hdl));
    }
    m_action_cond.notify_one();
  }

  void on_message(connection_hdl hdl, server::message_ptr msg) {
    // queue message up for sending by processing thread
    {
      lock_guard<mutex> guard(m_action_lock);
      m_actions.push(action(MESSAGE, hdl, msg));
    }
    m_action_cond.notify_one();
  }
  void stop_server() {
    // ws_server.stop_accept();
    ws_server.stop_listening();
    close_all_client();
  }
  void close_all_client() {
    unique_lock<mutex> lock(m_action_lock);
    while (!m_actions.empty()) {
      action a = m_actions.front();
      m_actions.pop();
      if (a.type == SUBSCRIBE) {
        lock_guard<mutex> guard(m_connection_lock);
        m_connections.insert(a.hdl);
      } else if (a.type == UNSUBSCRIBE) {
        lock_guard<mutex> guard(m_connection_lock);
        m_connections.erase(a.hdl);
      }
    }
    lock.unlock();
    {
      lock_guard<mutex> guard(m_connection_lock);
      con_list::iterator it;
      for (it = m_connections.begin(); it != m_connections.end();) {
        ws_server.close(*it, (websocketpp::close::status::value)0, "");
        it = m_connections.erase(it);
      }
      m_connections.clear();
    }
  }
  void process_messages() {
    while (1) {
      unique_lock<mutex> lock(m_action_lock);

      while (m_actions.empty()) {
        m_action_cond.wait(lock);
      }

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
        for (it = m_connections.begin(); it != m_connections.end(); ++it) {
          ws_server.send(*it, a.msg);
        }
      } else {
        // undefined.
      }
    }
  }

  typedef std::set<connection_hdl, std::owner_less<connection_hdl>> con_list;
  con_list m_connections;
  std::queue<action> m_actions;
  mutex m_action_lock;
  mutex m_connection_lock;
  condition_variable m_action_cond;
  server ws_server;
};
