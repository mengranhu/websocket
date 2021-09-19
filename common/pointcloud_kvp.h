
/**
 *  Copyright (C) 2019 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef POINTCLOUD_SERVER_COMMON_POINTCLOUD_KVP_H_
#define POINTCLOUD_SERVER_COMMON_POINTCLOUD_KVP_H_

#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

//---------------------------------------------------------------
class StringTrim {
 public:
  static std::string &Trim(std::string &s) {  // NOLINT
    if (s.empty()) {
      return s;
    }
    s.erase(0, s.find_first_not_of(" \t"));
    s.erase(s.find_last_not_of(" \t\n\r") + 1);
    return s;
  }
};

//---------------------------------------------------------------

// ex: token    value
//       T   =  A , B : C ,  D : E : F
class KvpItem {
 public:
  // ex: "T"
  std::string token;
  // ex: ["A", "B : C", "D : E : F"]
  std::vector<std::string> value;
  // ex: [["A"], ["B", "C"], ["D", "E", "F"]]
  std::vector<std::vector<std::string>> sub_value;

  KvpItem(const std::string &tok, const std::string &val) {
    token = tok;
    StringTrim::Trim(token);

    Process(val, ',', &value);

    size_t n = value.size();
    sub_value.resize(n);
    for (size_t i = 0; i < n; ++i) {
      Process(value[i], ':', &sub_value[i]);
    }
  }

 private:
  void Process(const std::string &val, char delimiter,
               std::vector<std::string> *r) {
    size_t start = 0;
    size_t pos = 0;
    std::string tok;
    while ((pos = val.find(delimiter, start)) != std::string::npos) {
      tok = val.substr(start, pos - start);
      StringTrim::Trim(tok);
      r->push_back(tok);
      start = pos + 1;
    }
    tok = val.substr(start);
    StringTrim::Trim(tok);
    r->push_back(tok);
  }
};

//---------------------------------------------------------------

typedef std::function<int(const KvpItem &)> KvpHook;

class KvpFile {
 public:
  // add a function for a token
  void AddHook(std::string token, KvpHook hook) {
    hooks_.insert(std::make_pair(token, hook));
  }

  // add a function of an instance for a token
  template <class T>
  void AddHook(std::string token, T *instance,
               int (T::*hook)(const KvpItem &)) {
    auto hookn = std::bind(hook, instance, std::placeholders::_1);
    hooks_.insert(std::make_pair(token, hookn));
  }

  void LoadKvpFile(const std::string fname) const;

 private:
  struct KeyEqual {
    bool operator()(const std::string &l, const std::string &r) const {
      // return l == r;
      return (l.find(r) == 0) || (r.find(l) == 0);
    }
  };
  std::map<std::string, KvpHook> hooks_;
};

#endif  // POINTCLOUD_SERVER_COMMON_POINTCLOUD_KVP_H_
