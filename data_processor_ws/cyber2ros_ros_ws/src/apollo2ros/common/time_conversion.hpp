#pragma once

#include <stdint.h>

namespace common {

// calc with the formula.
static const int32_t LEAP_SECONDS[][2] = {
    {1483228800, 18},  // 2017/01/01
    {1435708800, 17},  // 2015/07/01
    {1341100800, 16},  // 2012/07/01
    {1230768000, 15},  // 2009/01/01
    {1136073600, 14},  // 2006/01/01
    {915148800, 13},   // 1999/01/01
    {867711600, 12},   // 1997/07/01
    {820480320, 11},   // 1996/01/01 ;)
                       //....
                       // etc.
};

const int32_t GPS_AND_SYSTEM_DIFF_SECONDS = 315964800;

// array_size(a) returns the number of elements in a.
template <class T, size_t N>
constexpr size_t array_size(T (&)[N]){
  return N;
}

template <typename T>
T unix2gps(const T unix_seconds) {
  for (size_t i = 0; i < array_size(LEAP_SECONDS); ++i) {
    if (unix_seconds >= LEAP_SECONDS[i][0]) {
      return unix_seconds - (GPS_AND_SYSTEM_DIFF_SECONDS - LEAP_SECONDS[i][1]);
    }
  }
  return static_cast<T>(0);
}

template <typename T>
T gps2unix(const T gps_seconds) {
  for (size_t i = 0; i < array_size(LEAP_SECONDS); ++i) {
    T result = gps_seconds + (GPS_AND_SYSTEM_DIFF_SECONDS - LEAP_SECONDS[i][1]);
    if (result >= LEAP_SECONDS[i][0]) {
      return result;
    }
  }
  return static_cast<T>(0);
}

}  // namespace common
