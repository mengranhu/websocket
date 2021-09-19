/*
 *  Copyright (C) 2018 Innovusion Inc.
 *
 *  License: BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Innovusion LIDAR SDK Header File
 *
 *  The file provides the data structures definition and exported functions of
 *  Innovusion LIDAR SDK.
 */

/**
 *  Simple 7-step introduction describing how to use the SDK to write
 *  an Innovusion Lidar driver.
 *
 *  step 1a: (OPTIONAL)
 *    Use inno_lidar_setup_sig_handler() to setup handling of SIGSEGV, SIGBUS,
 *        and SIGFPE signals.
 * 
 *  step 1b: (OPTIONAL)
 *    Use inno_lidar_set_logs() to specify the log file fds (file descriptors)
 *        and callback.
 *    Use inno_lidar_set_log_level() to specify the log level.
 *
 *  step 2: (REQUIRED)
 *    Use inno_lidar_open_live() to connect to a sensor.
 *    Or use inno_lidar_open_file() to open a raw data file.
 *    Both methods return a lidar handle.
 *
 *  step 3: (REQUIRED)
 *    Use inno_lidar_set_parameters() to specify the yaml (configuration) file
 *    corresponding to the sensor or raw data file used in step 2 above.
 *
 *  step 4A-1: (REQUIRED)
 *    Use inno_lidar_set_callbacks() to specify the lidar_alarm and lidar_frame
 *        callbacks.
 *    Use the lidar_alarm callback to handle warning, error, and critical-error
 *        notifications that might occur.
 *    The lidar_frame callback provides access to the lidar pointcloud data.
 *
 *  step 4A-2: (OPTIONAL)
 *    Use inno_lidar_set_callbacks_2() to specify the lidar_cframe callback.
 *    The lidar_cframe callback provides access to lidar pointcloud data as
 *        well as bounding boxes and other data.  This is also the callback
 *        that should be used if spherical coordinates are desired instead
 *        of the default Cartesian coordinates.
 *
 *  step 4A-3: (REQUIRED)
 *    Use inno_lidar_start() to start reading from live lidar or file.
 *    Corresponding callbacks will be triggered.  The callbacks are called in
 *    the context of a thread dedicated to each lidar handle, i.e. one callback
 *    queue per lidar handle.  Please note that multiple callbacks may be
 *    called AT THE SAME TIME for different lidar handles.
 *
 *  step 4B: (ALTERNATIVE to steps 4As)
 *    Use inno_lidar_sync_read() to read the cframe data. The read is blocking
 *    until received a full cframe or timeout.
 *
 *  step 5: (OPTIONAL)
 *    Use inno_lidar_stop() to stop reading.  No further callbacks will be
 *    called once this call returns.
 *
 *  step 6: (REQUIRED)
 *    Use inno_lidar_close() to close a lidar handle and release any
 *    associated resources.
 *
 * 
 *  Other notes....
 * 
 *  Memory management:
 *    In the lidar_frame callback, you can specify whether the caller will free
 *    the point cloud data after callback by return 0.  Or, use return 1 to
 *    specify that the callee is responsible for freeing the point cloud data
 *    (by calling free(lidar_frame)).
 *
 *  Threading:
 *    In inno_lidar_start(), the SDK library will use pthread library to spawn
 *    threads to read and process data and make callbacks.
 */


#ifndef SRC_INNO_LIDAR_API_H_
#define SRC_INNO_LIDAR_API_H_

#include <math.h>
#include <pthread.h>
#include <stdint.h>

#ifndef _MSC_VER
#define DEFINE_COMPACT_STRUCT(x) struct __attribute__((packed)) x
#define DEFINE_COMPACT_STRUCT_END
#else
#define DEFINE_COMPACT_STRUCT(x) __pragma(pack(push, 1)) struct x
#define DEFINE_COMPACT_STRUCT_END __pragma(pack(pop))
#endif

/*****************
 * SDK VERSION
 *****************/
#define INNO_SDK_V_MAJOR "1"
#define INNO_SDK_V_MINOR "6"
#define INNO_SDK_V_DOT "0"
#define INNO_SDK_VERSION_IN_HEADER \
  INNO_SDK_V_MAJOR "." INNO_SDK_V_MINOR "." INNO_SDK_V_DOT "."

/*****************
 * data structure
 *****************/
typedef double inno_timestamp_us_t;

enum inno_timestamp_sync {
  INNO_TIMESTAMP_SYNC_NONE = 0,
  INNO_TIMESTAMP_SYNC_RECORDED = 1,
  INNO_TIMESTAMP_SYNC_HOST = 2,
  INNO_TIMESTAMP_SYNC_GPS_INIT = 3,
  INNO_TIMESTAMP_SYNC_GPS_LOCKED = 4,
  INNO_TIMESTAMP_SYNC_GPS_UNLOCKED = 5,
  INNO_TIMESTAMP_SYNC_PTP_INIT = 6,
  INNO_TIMESTAMP_SYNC_PTP_LOCKED = 7,
  INNO_TIMESTAMP_SYNC_PTP_UNLOCKED = 8,
  INNO_TIMESTAMP_SYNC_FILE_INIT = 9,
  INNO_TIMESTAMP_SYNC_MAX = 10,
};

enum inno_lidar_status {
  INNO_LIDAR_STATUS_ERROR = -2,
  INNO_LIDAR_STATUS_INVALID_HANDLE = -1,
  INNO_LIDAR_STATUS_STARTING = 0,
  INNO_LIDAR_STATUS_READY = 1,
  INNO_LIDAR_STATUS_STREAMING = 2,
  INNO_LIDAR_STATUS_UNKNOWN = 3,
  INNO_LIDAR_STATUS_INVALID = 4,
  // < -2 means LIDAR error code
};

enum inno_cframe_type {
  INNO_CFRAME_NONE = 0,
  INNO_CFRAME_POINT = 1,
  INNO_CFRAME_CPOINT = 2,
  INNO_CFRAME_BBOX = 3,
  INNO_CFRAME_TEXT = 4,
  INNO_CFRAME_ALARM = 5,
  INNO_CFRAME_CPOINT_EXT = 6,
  INNO_CFRAME_MAX = 7,
};

enum inno_alarm {
  INNO_ALARM_NO = 0,
  INNO_ALARM_WARNING = 1,
  INNO_ALARM_ERROR = 2,
  INNO_ALARM_CRITICAL = 3,
  INNO_ALARM_FATAL = 4,
};

enum inno_alarm_code {
  INNO_ALARM_CODE_NO = 0,
  INNO_ALARM_CODE_CANNOT_READ = 1,
  INNO_ALARM_CODE_INVALID_DATA_TYPE = 2,
  INNO_ALARM_CODE_TRIGGER_BUFFER_FULL = 3,
  INNO_ALARM_CODE_PULSE_BUFFER_FULL = 4,
  INNO_ALARM_CODE_LOW_FRAME_RATE = 6,
  INNO_ALARM_CODE_HIGH_FRAME_RATE = 7,
  INNO_ALARM_CODE_SLOW_NETWORK = 8,
  INNO_ALARM_CODE_LOW_FRAME_INTEGRITY = 9,
  INNO_ALARM_CODE_POINT_DATA_DROP = 10,
  INNO_ALARM_CODE_FRAME_DATA_DROP = 11,
  INNO_ALARM_CODE_READ_TIMEOUT = 12,
  INNO_ALARM_CODE_DROP_DATA_1 = 13,
  INNO_ALARM_CODE_DROP_DATA_2 = 14,
  INNO_ALARM_CODE_DROP_DATA_3 = 15,
  INNO_ALARM_CODE_DROP_DATA_4 = 16,
  INNO_ALARM_CODE_SENSOR_ERROR = 17,
  INNO_ALARM_CODE_BAD_CONFIG_YAML = 18,
  INNO_ALARM_CODE_NO_POINT_MEMORY = 19,
  INNO_ALARM_CODE_FRAME_CALL_TOO_LONG = 20,
  INNO_ALARM_CODE_TEMP_TOO_LOW = 21,
  INNO_ALARM_CODE_TEMP_TOO_HIGH = 22,
  INNO_ALARM_CODE_LIB_VERSION_MISMATCH = 23,
  INNO_ALARM_CODE_CLOCK_DRIFT = 24,
  INNO_ALARM_CODE_CORRUPT_DATA = 25,
  INNO_ALARM_CODE_OUT_OF_MEMORY = 26,
  INNO_ALARM_CODE_DROP_DATA_FILTER = 27,
  INNO_ALARM_CODE_FILTER_TOO_LONG = 28,
};

/* 22 bytes per point */
DEFINE_COMPACT_STRUCT(inno_point) {
  float x;                 /* in meter, pointing up                     */
  float y;                 /* in meter, pointing right                  */
  float z;                 /* in meter, pointing forward                */
  float radius;            /* in meter                                  */
  uint16_t ts_100us;       /* relative timestamp (to ts_us_start) in 100us */
  uint16_t ref;            /* reflectance, 1-254, 255 means a reflector */
                           /* or intensity, also 1-254 & 255=reflector  */
  unsigned char flags;     /* which channel (top or bottom)             */
  unsigned int scan_id: 10;
  unsigned int scan_idx: 10;
  unsigned int reserved: 4;
#if 0
  int ha_index;
  int va_index;
  float bg_dist;
#endif
};
DEFINE_COMPACT_STRUCT_END

static const int cframe_version_c = 6;
static const double cpoint_angel_unit_c = M_PI / 8192;
static const double cframe_geo_co_unit_c = 0.00000000174532927777;
static const double cframe_geo_co_unit_degree_c = 1.0/10000000;

/* compact format, 10 bytes per point */
DEFINE_COMPACT_STRUCT(inno_cpoint) {
  /* distance in cm, range [0, 655.35m] */
  unsigned int radius: 16;
  /* horizontal angle, 0 is straight forward, right is positive,
     unit is  cpoint_angel_unit_c rad, range (-PI/2 to -PI/2) */
  int h_angle: 13;
  /* vertical angle, 0 is the horizon, up is positive,
     unit is  cpoint_angel_unit_c rad, range (-PI/4 to PI/4) */
  int v_angle: 12;
  unsigned int ts_100us: 14; /* relative timestamp (to ts_us_start) in 100us */
  unsigned int scan_id: 10;
  unsigned int flags: 4;     /* which channel (top or bottom) /bowl       */
  unsigned int scan_idx: 10;
  unsigned int reserved: 1;
  unsigned int ref: 16;      /* reflectance, 1-254, 255 means a reflector */
                             /* or intensity, also 1-254 & 255=reflector  */
#if 0
  int ha_index;
  int va_index;
  float bg_dist;

  unsigned int scan_seq: 16;
  unsigned int cluster: 16;
#endif
};
DEFINE_COMPACT_STRUCT_END

DEFINE_COMPACT_STRUCT(inno_cpoint_ext) {
  struct inno_cpoint p;
  float x;
  float y;
  float z;
};
DEFINE_COMPACT_STRUCT_END

DEFINE_COMPACT_STRUCT(inno_bbox) {
  float x;                 /* center in meter    */
  float y;                 /* in meter           */
  float z;                 /* in meter           */
  float width;             /* in meter           */
  float height;            /* in meter           */
  float depth;             /* in meter           */
  float pose_w;
  float pose_x;
  float pose_y;
  float pose_z;
  float speed_x;
  float speed_y;
  float speed_z;
  int label;               /* object type */
  int value;
  int type;
  unsigned int id;
  int16_t confidence;
  int16_t priority;
  uint16_t speed;          /* in cm/s */
  uint16_t speed_heading;  /* in 0.01 degree */
  uint16_t radius;         /* in cm */
  uint16_t radius_heading; /* in 0.01 degree */
  int16_t region_idx;
  uint16_t pc_count;
  uint16_t cluster_id;
  int16_t reserved[1];
  float alpha;
  unsigned int color;      /* rgb, 0 means use viewer assigned color */
  unsigned int style;      /* 0: solid, 1: edge only                 */
  unsigned int ts_100us;   /* relative timestamp in 100us            */
  /* in 1/10000000 degree or cframe_geo_co_unit_c rad */
  int32_t longtitude;
  /* in 1/10000000 degree or cframe_geo_co_unit_c rad */
  int32_t latitude;
  /* in cm, range (-327.68 m, 327.68 m] */
  int16_t elevation;
  int16_t region;
  unsigned int source_id;
#define MAX_MERGED_IDS 3
  /* ids from previous frame merged to this bbox in current frame */
  unsigned int merged_ids[MAX_MERGED_IDS];
  int reserved2[7-MAX_MERGED_IDS];
  float text_x;                 /* center in meter    */
  float text_y;                 /* in meter           */
  float text_z;                 /* in meter           */
  float text_alpha;
  unsigned int text_color;      /* rgb, 0 means use viewer assigned color */
  unsigned int text_style;
  char text[64];                /* zero end, str max len is 63            */
};
DEFINE_COMPACT_STRUCT_END

DEFINE_COMPACT_STRUCT(inno_text) {
  float x;                 /* center of text in meter                */
  float y;                 /* in meter                               */
  float z;                 /* in meter                               */
  float alpha;
  int label;               /* object type */
  int value;
  int type;
  unsigned int id;
  int confidence;
  int reserved[4];
  unsigned int color;      /* rgb, 0 means use viewer assigned color */
  unsigned int style;
  char text[64];           /* zero end, str max len is 63            */
  unsigned int ts_100us;   /* relative timestamp in 100us            */
  unsigned int source_id;
  int reserved2[7];
};
DEFINE_COMPACT_STRUCT_END

DEFINE_COMPACT_STRUCT(inno_alarm_message) {
  enum inno_alarm level;
  enum inno_alarm_code code;
  char message[256];
  unsigned int source_id;
  int reserved2[7];
};
DEFINE_COMPACT_STRUCT_END

DEFINE_COMPACT_STRUCT(inno_frame) {
  uint64_t idx;            /* frame index, start from 0                     */
  uint16_t sub_idx;        /* sub-frame index, start from 0 for every frame */
  uint16_t sub_seq;        /* sequence  within a sub-frame                  */
  inno_timestamp_us_t ts_us_start;  /* start of frame, UNIX epoch in us     */
  inno_timestamp_us_t ts_us_end;    /* end of frame, UNIX epoch in us       */
  unsigned int points_number;  /* number of points                          */
  unsigned char conf_level;    /* indicate confidence level for each frame, */
                               /* low = 0 indicates the frame may have issue*/
                               /* high = 255 indicates a good frame         */
  unsigned char reserved[3];
  enum inno_timestamp_sync timestamp_sync_type;
  struct inno_point points[0];
};
DEFINE_COMPACT_STRUCT_END

DEFINE_COMPACT_STRUCT(inno_cframe_header) {
  uint8_t version;
  // lowest bit 0: not the last sequence of a sub-frame
  // lowest bit 1: not the last sub-frame of a frame
  uint8_t flags;
  uint16_t header_size;
  uint32_t checksum;

  /* in 1/10000000 degree or cframe_geo_co_unit_c rad */
  int32_t longtitude;
  /* in 1/10000000 degree or cframe_geo_co_unit_c rad */
  int32_t latitude;
  /* in cm, range (-327.68 m, 327.68 m] */
  int16_t elevation;

  /* yaw angle, 0 is to the north, PI/2 is east,
     unit is cpoint_angel_unit_c rad, range [-PI to PI] */
  int16_t pose_yaw_angle;
  /* pitch angle, 0 is the horizon, up is positive,
     unit is cpoint_angel_unit_c rad, range [-PI/2 to PI/2] */
  int16_t pose_pitch_angle;
  /* roll angle, 0 is level, positive is clock-wise,
     unit is cpoint_angel_unit_c rad, range [-PI to PI] */
  int16_t pose_roll_angle;

  uint64_t idx;            /* frame index, start from 0                     */
  uint16_t sub_idx;        /* sub-frame index, start from 0 for every frame */
  uint16_t sub_seq;        /* sequence within a sub-frame                   */

  inno_timestamp_us_t ts_us_start;   /* start of frame, in microsecond      */
  inno_timestamp_us_t ts_us_end;     /* end of frame, in microsecond        */
  enum inno_cframe_type type;
  /* one lidar instance can have multiple topics,
     each topic has a serial of frames,
     within one topic, when the new frame arrives,
     the viewer should STOP showing the
     previous frame of the same topic
   */
  unsigned int topic;
  unsigned int item_number;
  /* indicate confidence level for each frame,
     low = 0 indicates the frame may have issue,
     high = 255 indicates a good frame
   */
  unsigned char conf_level;
  unsigned int source_id;
  unsigned char reserved[1];
  uint16_t item_size;
  enum inno_timestamp_sync timestamp_sync_type;
  union {
    char c[0];
    inno_cpoint cpoints[0];
    inno_cpoint_ext cpoints_ext[0];
    inno_point points[0];
    inno_bbox bboxes[0];
    inno_text texts[0];
    inno_alarm_message alarm[0];
  };
};
DEFINE_COMPACT_STRUCT_END

enum inno_log_level {
  INNO_LOG_FATAL_LEVEL = 0,
  INNO_LOG_ERROR_LEVEL = 1,
  INNO_LOG_WARNING_LEVEL = 3,
  INNO_LOG_INFO_LEVEL = 4,
  INNO_LOG_TRACE_LEVEL = 5,
  INNO_LOG_EVERYTHING_LEVEL = 6,
};

/* default is REFLECTANCE_MODE_REFLECTIVITY since 1.5.0 */
enum reflectance_mode {
  REFLECTANCE_MODE_INTENSITY = 0,
  REFLECTANCE_MODE_REFLECTIVITY,
};

static inline size_t inno_get_cframe_item_size(enum inno_cframe_type type) {
  switch (type) {
    case INNO_CFRAME_NONE:
      return sizeof(inno_cframe_header);
    case INNO_CFRAME_POINT:
      return sizeof(inno_point);
    case INNO_CFRAME_CPOINT:
      return sizeof(inno_cpoint);
    case INNO_CFRAME_BBOX:
      return sizeof(inno_bbox);
    case INNO_CFRAME_TEXT:
      return sizeof(inno_text);
    case INNO_CFRAME_ALARM:
      return sizeof(inno_alarm_message);
    case INNO_CFRAME_CPOINT_EXT:
      return sizeof(inno_cpoint_ext);
    default:
      return 0;
  }
}

static inline size_t inno_get_cframe_size_from_header(
    const inno_cframe_header *cframe) {
  return cframe->header_size +
      cframe->item_number * cframe->item_size;
}

static inline void inno_set_cframe_size_header(
    inno_cframe_header *cframe) {
  cframe->header_size = sizeof(inno_cframe_header);
  cframe->item_size = inno_get_cframe_item_size(cframe->type);
}

static inline int inno_verify_cframe_size_header(
    inno_cframe_header *cframe) {
  return (cframe->header_size == sizeof(inno_cframe_header) &&
          cframe->item_size == inno_get_cframe_item_size(cframe->type)) ?
      1 : 0;
}

static inline size_t inno_get_cframe_size_from_calculation(
    const inno_cframe_header *cframe) {
  return sizeof(inno_cframe_header) +
      cframe->item_number * inno_get_cframe_item_size(cframe->type);
}

extern "C" {
  /*******************
   * callbacks typedef
   *******************/

  /*
   * Input:
   *   int lidar_handle: The handle of the lidar that triggered the callback.
   *   void *context: callback context passed in inno_lidar_set_callbacks()
   *   enum inno_alarm error_level: severity of alarm
   *   int error_code: error code
   *   const char *error_message: error message
   * Return: None
   */
  typedef void (*inno_lidar_alarm_callback_t)(int lidar_handle,
                                              void *context,
                                              enum inno_alarm error_level,
                                              enum inno_alarm_code,
                                              const char *error_message);


  /*
   * Input:
   *   int lidar_handle: The handle of the lidar that triggered the callback.
   *   void *context: callback context passed in inno_lidar_set_callbacks()
   *   inno_frame *frame: Pointer to inno_frame struct
   * Return:
   *   int: Indicates whether caller can free the frame after callback returns.
   *        0 means caller (i.e. the SDK library) can free the frame after
   *            the callback returns,
   *        1 means callee (i.e. the user code) is responsible to call free().
   */
  typedef int (*inno_lidar_frame_callback_t)(int lidar_handle,
                                             void *context,
                                             struct inno_frame *frame);


  /*
   * Input:
   *   int lidar_handle: The handle of the lidar that triggered the callback.
   *   void *context: callback context passed in inno_lidar_set_callbacks()
   *   inno_cframe_header *frame: Pointer to inno_cframe_header struct
   * Return:
   *   int: Indicates whether caller can free the frame after callback returns.
   *        0 means caller (i.e. the SDK library) can free the frame after
   *            the callback returns,
   *        1 means callee (i.e. the user code) is responsible to call free()
   *            or inno_lidar_return_to_external_cframe_memory_pool(), if
   *            external memory pool is used.
   */
  typedef int (*inno_lidar_cframe_callback_t)(int lidar_handle,
                                              void *context,
                                              struct inno_cframe_header*cframe);


  /*
   * Input:
   *   void *context: callback context passed in inno_lidar_set_callbacks()
   * Return:
   *   double: return the unix time in second defined in
   *           https://en.wikipedia.org/wiki/Unix_time
   *           e.g. ros::Time::now().toSec();
   */
  typedef double (*inno_lidar_host_time_in_second_t)(void *context);


  /*
   * Input:
   *   enum inno_log_level level: log level
   *   const char *header1: log message header, e.g.
   *       '2018-12-06 23:54:09'
   *   const char *header2: log message header, e.g.
   *       '[ERROR] 7252 rawdata.cpp:3259'
   *   const char *msg: log message body
   * Return:
   *   None
   */
  typedef void (*inno_lidar_log_t)(enum inno_log_level level,
                                   const char *header1,
                                   const char *header2,
                                   const char *msg);

  /********************
   * exported functions
   ********************/

  /*
   * Name: inno_api_version
   * Description: Get Innovusion lidar API version
   * Input:
   *   None
   * Return:
   *   const char *: Version string
   */
  const char *inno_api_version(void);

  /*
   * Name: inno_api_build_tag
   * Description: Get Innovusion lidar API build tag
   * Input:
   *   None
   * Return:
   *   const char *: build tag string
   */
  const char *inno_api_build_tag(void);


  /*
   * Name: inno_api_build_time
   * Description: Get Innovusion lidar API build time
   * Input:
   *   None
   * Return:
   *   const char *: Build time string
   */
  const char *inno_api_build_time(void);


  /*
   * Name: inno_lidar_setup_sig_handler
   * Description: Setup sigaction for SIGSEGV SIGBUS SIGFPE (optional)
   *              The signal handler will print out the stack backtrace
   *              and then call old signal handler
   * Input:
   *   None
   * Return:
   *   None
   */
  void inno_lidar_setup_sig_handler();


  /*
   * Name: inno_lidar_set_logs
   * Description: Set log files fds (file descriptors).
   *              This function can be only called once.
   * Input:
   *   int out_fd :
   *   int error_fd :
   *   inno_lidar_log_t log_callback:
   * Return:
   *   None
   */
  void inno_lidar_set_logs(int out_fd,
                           int error_fd,
                           inno_lidar_log_t log_callback);


  /*
   * Name: inno_lidar_set_log_level
   * Description: Set debug level for all lidar handles.
   *              This function can be called any time.
   *              It may be called multiple times, e.g. to decrease the log
   *              level before executing some code and then increasing the
   *              log level again after the section of code completes.
   * Input:
   *   enum inno_log_level log_level: debug level
   * Return:
   *   None
   */
  void inno_lidar_set_log_level(enum inno_log_level log_level);


  /*
   * Name: inno_lidar_open_live
   * Description: To open a lidar handle for a live Innovusion lidar unit
   * Input:
   *   const char *name: Contains the name of lidar, e.g. "forward-1"
   *                     The name should be less than 32 characters.
   *   const char *lidar_ip: Contains the lidar IP string, e.g. "172.168.1.10"
   *   int port: Lidar PORT, e.g. 8001
   *   int use_tcp: Uses TCP or other protocol, 1 means TCP, other values are 
   *                not supported for now
   * Return:
   *   int: return a lidar handle
   */
  int inno_lidar_open_live(const char *name,
                           const char *lidar_ip,
                           int port, int use_tcp);


  /*
   * Name: inno_lidar_open_file
   * Description: To open a lidar handle from an Innovusion lidar proprietary
   *              data file
   * Input:
   *   const char *name: Contains the name of lidar
   *                     The name should be less than 32 characters.
   *   const char *filename: Contains the filename of an Innovusion lidar
   *                         proprietary data file
   *   int play_rate: The playback rate, normal speed is 20 (MB/s)
   *   int rewind: How many times rewind before stop, 0 means no rewind, 
   *               < 0 means rewind infinity times
   * Return:
   *   int: return a lidar handle
   */
  int inno_lidar_open_file(const char *name,
                           const char *filename,
                           int play_rate,
                           int rewind,
                           uint64_t skip);


  /*
   * Name: inno_lidar_set_callbacks
   * Description: Set callbacks for a lidar handle. Developers need to
   *              set callbacks before calling inno_lidar_start().
   *              For every lidar handle, inno_lidar_set_callbacks can
   *              only be called once.
   * Input:
   *   int handle: Lidar handle (from either inno_lidar_open_live or
   *               inno_lidar_open_file)
   *   callback alarm_callback: This callback is called when some warning/error
   *                            happens.
   *                            For a given lidar handle, no alarm_callback or
   *                            frame_callback will be called until the
   *                            previous callback (for the same lidar handle)
   *                            has returned.
   *                            Multiple callbacks may happen at the same time
   *                            for different lidar handles.
   *                            Pass NULL pointer means no callback.
   *   callback frame_callback: This callback is called when one frame or
   *                            sub-frame is available.  The callback happens
   *                            in a thread dedicated to that lidar_handle.
   *                            For a given lidar handle, no alarm_callback or
   *                            frame_callback will be called until the
   *                            previous callback (for the same lidar handle)
   *                            has returned.
   *                            Multiple callbacks may happen at the same time
   *                            for different lidar handles.
   *                            Pass NULL pointer means no callback.
   *   callback get_host_time: This callback will get the current host time.
   *                           Pass NULL pointer means clock_gettime(CLOCK_REALTIME)
   *                           will be used.
   *   void *callback_context: context passed in when callback is invoked
   * Return:
   *   int: 0 means success, otherwise failure (e.g. invalid lidar handle)
   */
  int inno_lidar_set_callbacks(int handle,
                               inno_lidar_alarm_callback_t alarm_callback,
                               inno_lidar_frame_callback_t frame_callback,
                               inno_lidar_host_time_in_second_t get_host_time,
                               void *callback_context);


  /*
   * Name: inno_lidar_set_callbacks_2
   * Description: Set callbacks for a lidar handle. Developers need to
   *              set callbacks before calling inno_lidar_start().
   *              For every lidar handle, inno_lidar_set_callbacks2 can
   *              only be called once.
   * Input:
   *   int handle: Lidar handle
   *   callback cframe_callback: This callback is called when one frame or
   *                             sub-frame is available.  The callback happens
   *                             in a thread dedicated to that lidar_handle.
   *                             For a given lidar handle, no alarm_callback or
   *                             frame_callback will be called until the
   *                             previous callback (for the same lidar handle)
   *                             has returned.
   *                             Multiple callbacks may happen at the same time
   *                             for different lidar handles.
   *   enum inno_cframe_type cframe_type: in the cframe_callback, the cframe type
   *                         for pointcloud. Valid values are: INNO_CFRAME_POINT,
   *                         INNO_CFRAME_CPOINT, INNO_CFRAME_CPOINT_EXT
   *   void *callback_context: context passed in when callback is invoked
   * Return:
   *   int: 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_set_callbacks_2(int handle,
                                 inno_lidar_cframe_callback_t cframe_callback,
                                 enum inno_cframe_type cframe_type,
                                 void *callback_context);


  /*
   * Name: inno_lidar_set_parameters
   * Description: Use lidar configuration parameter file for a lidar handle.
   *              This function should be called before inno_lidar_start is called.
   * Input:
   *   int handle: Lidar handle
   *   const char *lidar_model: lidar model name, e.g. "E" or "REV_E"
   *   const char *lidar_home_dir: full path of the directory where the SDK can
   *                               store small temporary files.
   *   const char *yaml_filename: full file path of the lidar configuration
   *                              parameter file. The file is in yaml format.
   *                              Using the default NULL will read parameters from
   *                              the configuration file on the sensor, if handle
   *                              is for a live sensor.  If handle is for a raw
   *                              data file, this parameter MUST be a valid file.
   * Return:
   *   int: 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_set_parameters(int handle, const char *lidar_model,
                                const char *lidar_home,
                                const char *yaml_filename = NULL);

  /*
   * Name: inno_lidar_set_config_name_value
   * Description: Set config name-value pair for a lidar handle.
   *              This function should be called before inno_lidar_start is called,
   *              and can be called multiple times.
   * Input:
   *   int handle: Lidar handle
   *   const char *cfg_name: name of the config item
   *   const char *cfg_value: value of the config item
   * Return:
   *   int:  0 means success
   *         1 invalid handle
   *         2 invalid name
   *         3 invalid value
   */
  int inno_lidar_set_config_name_value(int handle,
                                       const char *cfg_name,
                                       const char *cfg_value);

  /*
   * Name: inno_lidar_set_reflectance_mode
   * Description: Change the meaning of the ref data in inno_point and
   *              inno_cpoint structures to indicate either reflectance or
   *              intensity.
   * Input:
   *   int handle: Lidar handle
   *   reflectance_mode mode: either REFLECTANCE_MODE_INTENSITY
   *                          or REFLECTANCE_MODE_REFLECTIVITY
   *                          default is REFLECTANCE_MODE_REFLECTIVITY
   *                          since 1.5.0
   * Return:
   *   int: 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_set_reflectance_mode(int handle, enum reflectance_mode mode);

#if defined(_QNX_) || defined(__QNX__)
struct cpu_set_t;
#endif
  /*
   * Name: inno_lidar_thread_setaffinity_np
   * Description: Specify the CPU affinity for all threads created by the
   *              library for this lidar instance. This function can only
   *              be called before inno_lidar_start() is called. Please see
   *              pthread_setaffinity_np() for reference.
   * Input:
   *   int handle: Lidar handle
   *   size_t cpusetsize: size of *cpuset
   *   const cpu_set_t *cpuset: cupset that will be passed to
   *                            pthread_setaffinity_np()
   *   int   exclude_callback_thread: 1 means do not set affinity for callback
   *                                  thread, 0 otherwise
   * Return:
   *   int:  0 means successful stored the affinity settings that will be
   *           passed in when calling pthread_setaffinity_np(). It doesn't
   *           not guarantee that pthread_setaffinity_np() will be successful.
   *         1 invalid handle
   */
  int inno_lidar_thread_setaffinity_np(int handle,
                                       size_t cpusetsize,
                                       const cpu_set_t *cpuset,
                                       int exclude_callback_thread);

  /*
   * Name: inno_lidar_get_status
   * Description: Query lidar unit's state
   * Input:
   *   int handle: Lidar handle
   *   unsigned int *error_code: address to store error code. NULL means
   *                             do not store error code
   *
   * Return:
   *   enum inno_lidar_status: Lidar status. If returns INNO_LIDAR_STATUS_ERROR,
   *                           the corresponding error code will be stored in
   *                           error_code
   */
  enum inno_lidar_status inno_lidar_get_status(int handle,
                                               unsigned int *error_code);


  /*
   * Name: inno_lidar_get_fw_version
   * Description: Query lidar unit's firmware version
   * Input:
   *     int handle: Lidar handle
   *   char *buffer: buffer to store the firmware version
   * int buffer_len: length of buffer, the recommended buffer_len is
   *                 512, the size of buffer needs to be >= buffer_len
   * Return:
   *   int: 0 cannot get firmware version
   *       -1 invlid lidar handle
   *       -2 buffer_len is too small
   *       -3 source is file
   *       otherwise return the size of firmware version string
   *        stored in buffer, not include the trailing zero
   *    Sample content in the buffer:

App Version: app-2.3.0-rc8.134
  build-time: 2019-08-14-18-19-25
FPGA Datecode: 0x190814e2
  fpga-ver: 0x13
  fpga-rev: 0x0e
  board-rev: 0x2
Firmware Version: 2.3.1-rc3-418.2019-08-15-17-41-40
  build-tag: 2.3.1-rc3-418
  build-time: 2019-08-15-17-41-40
  build-git-tag: 1.0.19

   */
  int inno_lidar_get_fw_version(int handle, char *buffer, int buffer_len);

  /*
   * Name: inno_lidar_get_sn
   * Description: Query lidar unit's S/N
   * Input:
   *     int handle: Lidar handle
   *   char *buffer: buffer to store the S/N
   * int buffer_len: length of buffer, the recommended buffer_len is
   *                 128, the size of buffer needs to be >= buffer_len
   * Return:
   *   int: 0 cannot get S/N
   *       -1 invlid lidar handle
   *       -2 buffer_len is too small
   *       -3 source is file
   *       otherwise return the size of S/N string
   *        stored in buffer, not include the trailing zero
   */
  int inno_lidar_get_sn(int handle, char *buffer, int buffer_len);

  /*
   * Name: inno_lidar_get_model
   * Description: Query lidar unit's model
   * Input:
   *     int handle: Lidar handle
   *   char *buffer: buffer to store the model
   * int buffer_len: length of buffer, the recommended buffer_len is
   *                 32, the size of buffer needs to be >= buffer_len
   * Return:
   *   int: 0 cannot get model
   *       -1 invlid lidar handle
   *       -2 buffer_len is too small
   *       -3 source is file
   *       otherwise return the size of model string
   *        stored in buffer, not include the trailing zero
   */
  int inno_lidar_get_model(int handle, char *buffer, int buffer_len);

  /*
   * Name: inno_lidar_start
   * Description: Start to read data from live lidar unit or from a data file.
   *              The alarm_callback, frame_callback and cframe_callback will
   *              be called only after this function is called.  They may
   *              be called before this function returns since they are in
   *              different threads.
   * Input:
   *   int handle: Lidar handle
   * Return:
   *   int: 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_start(int handle);


  /*
   * Name: inno_lidar_stop
   * Description: Stop reading data from live lidar unit or from a data file.
   *              The alarm_callback, frame_callback and cframe_callback will
   *              not be called once this function is returned.
   * Input:
   *   int handle: Lidar handle
   * Return:
   *   int: 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_stop(int handle);

  /*
   * Name: inno_sync_read
   * Description: Read cframe data from live lidar unit or from a data file.
   *              Note that the first read call can take longer than the specified
   *              timeout value by 3 seconds.
   *              The cframe will be filed with a pointer that points to a cframe
   *              upon a successful call. Caller is responsible for freeing the
   *              returned cframe pointer.
   * Input:
   *   int handle: Lidar handle
   *   double timeout_sec: timeout value in second
   *   enum inno_cframe_type cframe_type: in the cframe_callback, the cframe type
   *                         for pointcloud. Valid values are: INNO_CFRAME_POINT,
   *                         INNO_CFRAME_CPOINT, INNO_CFRAME_CPOINT_EXT
   *   struct inno_cframe_header **: point to place that stores the return
   *                                 cframe pointer 
   * Return:
   *   int: 0 means success,
   *        1 means bad handle
   *       -1 means has error
   *       -2 means timeout
   */
  int inno_lidar_sync_read(int handle, double timeout_sec,
                           enum inno_cframe_type cframe_type,
                           struct inno_cframe_header **cframe);

  /*
   * Name: inno_lidar_close
   * Description: Close the lidar handle and release all resources.
   *              This function can be called on a valid lidar handle only
   *              after inno_lidar_stop() call has returned,
   *              or inno_lidar_start() has never been called before.
   * Input:
   *   int handle: Lidar handle
   * Return:
   *   int: 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_close(int handle);


  /*
   * Name: inno_lidar_get_min_buffer_size_per_cframe
   * Description: Get the minimal buffer size that can hold a struct cframe
   *              which has max number of points.  This function can be called
   *              on a valid lidar handle only after inno_lidar_set_callbacks_2
   *              has returned.
   * Input:
   *   int handle: Lidar handle
   * Return:
   *   int: minimal size of the buffer, -1 if handle is invalid
   */
  int inno_lidar_get_min_buffer_size_per_cframe(int handle);


  /*
   * Name: inno_set_external_cframe_mem_pool
   * Description: Pass in an external buffer with size = unit_size * unit_count
   *              bytes.  The SDK library will divide this buffer to a memory
   *              pool that has unit_count units, each unit is unit_size bytes.
   *              When the sdk make the inno_lidar_cframe_callback_t, it will
   *              allocate a unit from the pool and store the cframe data and
   *              points data in the unit.  The unit will be returned to the
   *              pool if the callback returns 0, or needs to be freed by
   *              explicitly calling
   *              inno_lidar_return_to_external_cframe_memory_pool()
   *              if callback returns 1.
   *              The mem_pool can ONLY be used by cframe_callback (NOT
   *              frame_callback), which is done in inno_lidar_set_callbacks_2
   *              only. Here is the correct calling sequence:
   *                  1. inno_lidar_set_callbacks_2()
   *                  2. inno_lidar_get_min_buffer_size_per_cframe()
   *                  3. alloc the memory based on the size return from step 2
   *                  4. inno_lidar_set_external_cframe_mem_pool()
   *                  5. inno_lidar_start()
   *                  6. do the real work
   *                  7. inno_lidar_stop()
   *                  8. inno_lidar_unset_external_cframe_mem_pool()
   *                     (this is optional but good for integrity check)
   *                  9. inno_lidar_close()
   *             In the lidar log, the SDK library will write something like
   *                  "mem_pool_manager %p created pool=%p, unit_size=%u,
   *                  unit_count=%u"
   *                  "delete mem_pool_manager %p pool=%p, called=%lu,
   *                  return-null=%lu request_too_big=%lu"
   *             User can use these to verify the external memory pool is
   *             working as expected.
   * 
   *             In the inno_lidar_cframe_callback(), return 0 means the buffer
   *             is not needed anymore, so the SDK library can internally make
   *             the memory block free; return 1 means user will call
   *             inno_lidar_return_to_external_cframe_memory_pool() later
   *             explicitly to return the memory block to the pool.
   *             Internally a FIFO queue is used to keep the free list, so the
   *             freed block will be appended to the tail of the free list and
   *             we return the head of the free list to an alloc() request.
   * Input:
   *   int handle: Lidar handle
   *   void *buffer: starting address of the external memory pool buffer
   *   unsigned int unit_size: size of each pool unit
   *   unsigned int unit_count: number of pool units
   * Return:
   *   int: 0 means success, -1 if handle is invalid
   */
  int inno_lidar_set_external_cframe_mem_pool(int handle, void *buffer,
                                              unsigned int unit_size,
                                              unsigned int unit_count);


  /*
   * Name: inno_unset_external_cframe_mem_pool
   * Description: Inform SDK library to stop using the buffer that was
   *              passed in as a memory pool.
   * Input:
   *   int handle: Lidar handle
   *   void *buffer: starting address of the external memory pool buffer that
   *                 was passed in inno_lidar_set_external_cframe_mem_pool()
   * Return:
   *   int: 0 means success, -1 if handle is invalid
   */
  int inno_lidar_unset_external_cframe_mem_pool(int handle, void *buffer);


  /*
   * Name: inno_lidar_return_to_external_cframe_memory_pool
   * Description: Return the buffer unit to the memory pool
   * Input:
   *   int handle: Lidar handle
   *   void *buffer: the buffer received from the inno_lidar_cframe_callback_t
   * Return:
   *   int: 0 means success, -1 if handle is invalid
   */
  int inno_lidar_return_to_external_cframe_memory_pool(int handle, void *buf);

  /*
   * Name: inno_lidar_load_filter
   * Description: Load and use filter plugin
   * Input:
   *   int handle: Lidar handle
   *   const char *path: the path to the filter plugin .so file
   * Return:
   *   int: 0 means success, -1 if handle is invalid
  */
  int inno_lidar_load_filter(int handle, const char *path);
};

#endif  // SRC_INNO_LIDAR_API_H_
