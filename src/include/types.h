#pragma once

#if !defined(__UINT8_TYPE__)
typedef unsigned char uint8_t;
#else
typedef __UINT8_TYPE__ uint8_t;
#endif

#if !defined(__UINT16_TYPE__)
typedef unsigned int uint16_t;
#else
typedef __UINT16_TYPE__ uint16_t;
#endif

#if !defined(__UINT32_TYPE__)
typedef unsigned long uint32_t;
#else
typedef __UINT32_TYPE__ uint32_t;
#endif

#if !defined(__UINT64_TYPE__)
typedef unsigned long long uint64_t;
#else
typedef __UINT64_TYPE__ uint64_t;
#endif

#if !defined(__INT8_TYPE__)
typedef signed char int8_t;
#else
typedef __INT8_TYPE__ int8_t;
#endif

#if !defined(__INT16_TYPE__)
typedef signed int int16_t;
#else
typedef __INT16_TYPE__ int16_t;
#endif

#if !defined(__INT32_TYPE__)
typedef signed long int32_t;
#else
typedef __INT32_TYPE__ int32_t;
#endif

#if !defined(__INT64_TYPE__)
typedef signed long long int64_t;
#else
typedef __INT64_TYPE__ int64_t;
#endif
