#ifndef ROSBAG2_STORAGE_SIMPLEFILE__VISIBILITY_CONTROL_H_
#define ROSBAG2_STORAGE_SIMPLEFILE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSBAG2_STORAGE_SIMPLEFILE_EXPORT __attribute__ ((dllexport))
    #define ROSBAG2_STORAGE_SIMPLEFILE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSBAG2_STORAGE_SIMPLEFILE_EXPORT __declspec(dllexport)
    #define ROSBAG2_STORAGE_SIMPLEFILE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSBAG2_STORAGE_SIMPLEFILE_BUILDING_LIBRARY
    #define ROSBAG2_STORAGE_SIMPLEFILE_PUBLIC ROSBAG2_STORAGE_SIMPLEFILE_EXPORT
  #else
    #define ROSBAG2_STORAGE_SIMPLEFILE_PUBLIC ROSBAG2_STORAGE_SIMPLEFILE_IMPORT
  #endif
  #define ROSBAG2_STORAGE_SIMPLEFILE_PUBLIC_TYPE ROSBAG2_STORAGE_SIMPLEFILE_PUBLIC
  #define ROSBAG2_STORAGE_SIMPLEFILE_LOCAL
#else
  #define ROSBAG2_STORAGE_SIMPLEFILE_EXPORT __attribute__ ((visibility("default")))
  #define ROSBAG2_STORAGE_SIMPLEFILE_IMPORT
  #if __GNUC__ >= 4
    #define ROSBAG2_STORAGE_SIMPLEFILE_PUBLIC __attribute__ ((visibility("default")))
    #define ROSBAG2_STORAGE_SIMPLEFILE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSBAG2_STORAGE_SIMPLEFILE_PUBLIC
    #define ROSBAG2_STORAGE_SIMPLEFILE_LOCAL
  #endif
  #define ROSBAG2_STORAGE_SIMPLEFILE_PUBLIC_TYPE
#endif

#endif  // ROSBAG2_STORAGE_SIMPLEFILE__VISIBILITY_CONTROL_H_
