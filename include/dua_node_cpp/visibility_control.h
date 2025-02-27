#ifndef DUA_NODE__VISIBILITY_CONTROL_H_
#define DUA_NODE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DUA_NODE_EXPORT __attribute__ ((dllexport))
    #define DUA_NODE_IMPORT __attribute__ ((dllimport))
  #else
    #define DUA_NODE_EXPORT __declspec(dllexport)
    #define DUA_NODE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DUA_NODE_BUILDING_LIBRARY
    #define DUA_NODE_PUBLIC DUA_NODE_EXPORT
  #else
    #define DUA_NODE_PUBLIC DUA_NODE_IMPORT
  #endif
  #define DUA_NODE_PUBLIC_TYPE DUA_NODE_PUBLIC
  #define DUA_NODE_LOCAL
#else
  #define DUA_NODE_EXPORT __attribute__ ((visibility("default")))
  #define DUA_NODE_IMPORT
  #if __GNUC__ >= 4
    #define DUA_NODE_PUBLIC __attribute__ ((visibility("default")))
    #define DUA_NODE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DUA_NODE_PUBLIC
    #define DUA_NODE_LOCAL
  #endif
  #define DUA_NODE_PUBLIC_TYPE
#endif

#endif  // DUA_NODE__VISIBILITY_CONTROL_H_
