/*
 *  Copyright (C) 2018 Innovusion Inc.
 *
 *  License: BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef SRC_INNO_LIDAR_API_EXPERIMENTAL_H_
#define SRC_INNO_LIDAR_API_EXPERIMENTAL_H_

#include "./inno_lidar_api.h"
#include <stdint.h>

/*****************
 * data structure
 *****************/

enum inno_cframe_topic {
  INNO_FULL_POINTCLOUD = 0,
  INNO_ALARM_POINTCLOUD = 1,
  INNO_STATIC_POINTCLOUD = 2,
  INNO_STATIC_POINTCLOUD_MAX = 100,
  INNO_DYNAMIC_POINTCLOUD = 101,
  INNO_DYNAMIC_BBOX = 110,
  INNO_DYNAMIC_TEXT = 120,
  INNO_MARKER_BBOX = 130,
  INNO_MARKER_TEXT = 140,
  INNO_REGION_BBOX = 150,
  INNO_REGION_TEXT = 160,
};

enum object_type {
  TYPE_UNKNOWN = 0,
  TYPE_PEDESTRIAN = 1,
  TYPE_CAR = 2,
  TYPE_BIKE = 3,
  TYPE_REGION = 4,
  TYPE_MARKER = 5,
  TYPE_TOTAL = 6,
};

enum inno_exp_callback_type {
  INNO_EXP_CALLBACK_NONE = 0,
  INNO_EXP_CALLBACK_RAW_WRITE,
  INNO_EXP_CALLBACK_BACKGROUND_WRITE,
  INNO_EXP_CALLBACK_BACKGROUND_WRITE2,
  INNO_EXP_CALLBACK_MAX,
};

extern "C" {
  typedef void (*inno_lidar_exp_callback_t)(int lidar_handle, void *context,
                                            enum inno_exp_callback_type type,
                                            const char *buffer, int len);

  int inno_lidar_set_interactive(int handle, bool interactive,
                                 int interactive_bytes);

  int inno_lidar_set_skip_file_size(int handle, int size);

  int inno_lidar_set_new_intensity_format(int handle,
                                          bool new_intensity_format);

  int inno_lidar_get_received_file_size(int handle, int *size);

  int inno_lidar_set_delay_correction_file(int handle,
                                         const char *delay_correction_filename);

  int inno_lidar_set_exp_callbacks(int handle, enum inno_exp_callback_type type,
                                   inno_lidar_exp_callback_t callback);

  /*
   * Name: inno_lidar_set_ila_connect_string
   * Description: set ILA connect string
   * Input:
   *   int handle: Lidar handle
   *   char * connect_string  :
   * Return:
   *   int: 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_set_ila_connect_string(int handle,
                                        const char *connect_string);

/*
   * Name: inno_lidar_enable_prerecord
   * Description: enable (default) or disable memory allocation for a
   *              'prerecord' buffer.
   * Input:
   *   int handle: Lidar handle
   *   bool enabled: set aside memory for 'prerecord' buffer if true, don't
   *                 allocate this memory buffer if false
   * Return:
   *   int: 0 means success, otherwise failure (e.g. invalid handle,
   *        recording already in progress, etc.)
   */
  int inno_lidar_enable_prerecord(int handle, bool enabled);

  /*
   * Name: inno_lidar_save_w_prerecord
   * Description: save raw data with some amount of 'prerecorded' data (in a
   *              separate thread)
   * Input:
   *   int handle: Lidar handle
   *   uint16_t prerecord_count: approximate # prerecord frames to include
   *   const char *filename: where to save the raw data
   * Return:
   *   int: 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_save_w_prerecord(int handle, uint16_t prerecord_count,
                                  const char *filename);

  /*
   * Name: inno_lidar_stop_saving_prerecord
   * Description: stop saving raw data (started by inno_lidar_save_w_prerecord)
   *              BLOCKING call -- only return after file is closed and saving
   *              thread terminates (joins).
   * Input:
   *   int handle: Lidar handle
   * Return:
   *   int: 0 means success, otherwise failure (e.g. invalid handle)
   */
  int inno_lidar_stop_saving_prerecord(int handle);

  /*
   * Name: inno_lidar_set_motion_compensation
   * Description: Set the velocity and angular velocity for motion correction
   * Input:
   *   int handle: Lidar handle
   *   double velocity[3]: velocities in x, y, z axis
   *   double angular_velocity[3]: angular velocities in x, y, z axis
   * Return:
   *   int: 0 means success, -1 if handle is invalid
   */
  int inno_lidar_set_motion_compensation(int handle, double velocity[3],
                                         double angular_velocity[3]);

  /*
   * Name: inno_lidar_set_play_rate
   * Description: Adjust the file play rate
   * Input:
   *   int handle: Lidar handle
   *   int rate: (0,100] -- playback speed (MB/s) -- normal speed is 20
   *             > 100 -- rate / 10000 = speed multiplier
   *                      e.g. rate = 10000 ==> multiplier = 1
   *                           rate = 20000 ==> multiplier = 2 (2x normal)
   *                           rate = 5000 ==> multiplier = 0.5 (1/2x normal)
   * Return:
   *   int: 0 means success, -1 if handle is invalid
   */
  int inno_lidar_set_play_rate(int handle, int rate);
  int inno_lidar_get_play_rate(int handle);
  double inno_lidar_get_fps_multiplier(int handle);
};

#endif  // SRC_INNO_LIDAR_API_EXPERIMENTAL_H_
