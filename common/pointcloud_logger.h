/**
 *  Copyright (C) 2019 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef POINTCLOUD_SERVER_COMMON_POINTCLOUD_LOGGER_H_
#define POINTCLOUD_SERVER_COMMON_POINTCLOUD_LOGGER_H_

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <sys/time.h>
#include <stdarg.h>
#include <iostream>
#include <sstream>
#include <iomanip>

// please add a global variable in your project, like following:
//   InnoLog g_inno_log;
class  InnoLogger;
extern InnoLogger g_inno_log;

#define INNO_LOG_NUM  7

enum class PointCloudLogLevel {
  INNO_LOG_FATAL   = 0,
  INNO_LOG_ERROR   = 1,
  INNO_LOG_TEMP    = 2,
  INNO_LOG_WARNING = 3,
  INNO_LOG_INFO    = 4,
  INNO_LOG_TRACE   = 5,
  INNO_LOG_VERBOSE = 6,
};
#define inno_log_panic_if(_condition, ...)                                \
  do {                                                                    \
    if (_condition) {                                                     \
      inno_log_print(PointCloudLogLevel::INNO_LOG_FATAL,                  \
                     "TRUE condition check failed: (" #_condition ")");   \
      *reinterpret_cast<char *>(NULL) = 0;                                \
    }                                                                     \
  } while (0)

#define inno_log_panic_if_not(_condition, ...)                              \
  do {                                                                      \
    if (!(_condition)) {                                                    \
      inno_log_print(PointCloudLogLevel::INNO_LOG_FATAL,                    \
                     "FALSE condition check failed: (" #_condition ")");    \
      *reinterpret_cast<char *>(NULL) = 0;                                  \
    }                                                                       \
  } while (0)

#define inno_log_fatal(...)                                \
  do {                                                     \
    inno_log_print(PointCloudLogLevel::INNO_LOG_FATAL,     \
                   __VA_ARGS__);                           \
  } while (0)

#define inno_log_error(...)                                \
  do {                                                     \
    inno_log_print(PointCloudLogLevel::INNO_LOG_ERROR,     \
                   __VA_ARGS__);                           \
  } while (0)

#define inno_log_error_errno(_fmt, ...)                    \
  do {                                                     \
    inno_log_print(PointCloudLogLevel::INNO_LOG_ERROR,     \
                   _fmt,                                   \
                   __VA_ARGS__);                           \
  } while (0)

#define inno_log_warning_errno(_fmt, ...)                  \
  do {                                                     \
      inno_log_print(PointCloudLogLevel::INNO_LOG_WARNING, \
                     "strerror: '%s' " _fmt,               \
                     strerror(errno), __VA_ARGS__);        \
  } while (0)

#define inno_log_warning(...)                              \
  do {                                                     \
      inno_log_print(PointCloudLogLevel::INNO_LOG_WARNING, \
                     __VA_ARGS__);                         \
  } while (0)

#define inno_log_info(...)                                 \
  do {                                                     \
      inno_log_print(PointCloudLogLevel::INNO_LOG_INFO,    \
                     __VA_ARGS__);                         \
  } while (0)

#define inno_log_trace(...)                                \
  do {                                                     \
      inno_log_print(PointCloudLogLevel::INNO_LOG_TRACE,   \
                     __VA_ARGS__);                         \
  } while (0)

#define inno_log_verbose(...)                              \
  do {                                                     \
      inno_log_print(PointCloudLogLevel::INNO_LOG_VERBOSE, \
                     __VA_ARGS__);                         \
  } while (0)

#define inno_log_print_no_line_info(_level, ...)           \
  do {                                                     \
    g_inno_log.inno_log_internal(_level, false, __FILE__,  \
                                 __LINE__, __VA_ARGS__);   \
  } while (0)

#define inno_log_print(_level, ...)                        \
  do {                                                     \
    g_inno_log.inno_log_internal(_level, true, __FILE__,   \
                                 __LINE__, __VA_ARGS__);   \
  } while (0)

class InnoLogger {
 public:
  explicit InnoLogger(PointCloudLogLevel level
                      = PointCloudLogLevel::INNO_LOG_INFO)
                      : level_(level) {
  }

  ~InnoLogger() {
  }

  PointCloudLogLevel get_level() const noexcept { return level_; }

  void set_level(PointCloudLogLevel lvl) noexcept {
    level_ = lvl;
    int t = static_cast<int>(level_);
    fprintf(stderr, "set level %s\n", inno_log_header[t]);
  }

  void set_level(int lvl) noexcept {
    if (lvl < 0 || lvl >= INNO_LOG_NUM) {
      fprintf(stderr, "set level error %d\n", lvl);
      return;
    }
    level_ = static_cast<PointCloudLogLevel>(lvl);
    fprintf(stderr, "set level %s\n", inno_log_header[lvl]);
  }

  void set_level(const char *lvl) noexcept {
    int t = 0;
    if (sscanf(lvl, "%d", &t) == 1) {
      set_level(t);
    } else {
      char *p = strdup(lvl);
      for (char *str = p; *str != '\0'; str++) {
          *str = toupper(*str);
      }
      for (int i = 0; i < INNO_LOG_NUM; ++i) {
        if (strstr(inno_log_header[i], p)) {
          level_ = static_cast<PointCloudLogLevel>(i);
          fprintf(stderr, "set level %s[%d]\n", p, i);
          free(p);
          return;
        }
      }
      fprintf(stderr, "set level error: %s\n", p);
      free(p);
    }
  }

  void inno_log_internal(PointCloudLogLevel level,
                         bool with_line_info,
                         const char *file, int line,
                         const char *format, ...)
                         const noexcept
                         __attribute__((format(printf, 6, 7))) {
                         // first is pointer 'this', then start from 5
    if (level_ < level) {
      return;
    }

    char tbuffer[32];
    char tbuffer2[64];
    struct tm* tm_info;
    struct tm result_time;

    timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;

    tm_info = localtime_r(&curTime.tv_sec, &result_time);
    strftime(tbuffer, sizeof(tbuffer) - 1, "%Y-%m-%d %H:%M:%S", tm_info);
    tbuffer[sizeof(tbuffer) - 1] = 0;
    snprintf(tbuffer2, sizeof(tbuffer2), "%s.%03d", tbuffer, milli);

    std::stringstream ss;
    ss << tbuffer2;

    ss << " " << inno_log_header[static_cast<int>(level)];
    if (with_line_info && level <= PointCloudLogLevel::INNO_LOG_WARNING) {
      ss << " " << file << ":" << line;
    }

    static const ssize_t kMaxSize = 1000;
    char buffer[kMaxSize] = {0};
    va_list valist;
    va_start(valist, format);
    vsnprintf(buffer, kMaxSize, format, valist);
    va_end(valist);
    buffer[kMaxSize - 1] = 0;

    int bl = strlen(buffer);
    char *c = buffer;
    // change non-printable to '_'
    while (*c) {
      if (*c < 0) {
        *c = '_';
      }
      c++;
    }
    if (bl > 0 && buffer[bl - 1] == '\n') {
      // remove unnecessary \n
      buffer[bl - 1] = 0;
    }

    ss << " " << buffer << std::endl;
    std::cout << ss.str();
  }

 private:
  PointCloudLogLevel level_;
  const char *inno_log_header[INNO_LOG_NUM] = {
      "[FATAL]",
      "[ERROR]",
      "[ TEMP]",
      "[ WARN]",
      "[ INFO]",
      "[TRACE]",
      "[VERBO]",
    };
};  // class InnoLog

#endif  // POINTCLOUD_SERVER_COMMON_POINTCLOUD_LOGGER_H_

