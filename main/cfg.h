#ifndef _CFG_H_
#define _CFG_H_

#include <esp_types.h>

#define UNUSED __attribute__((unused))
#define PACKED __attribute__((packed))
#define NORET  __attribute__((noreturn))

typedef uint8_t u8;
typedef int8_t  i8;

typedef uint16_t u16;
typedef int16_t  i16;

typedef uint32_t u32;
typedef int32_t  i32;

typedef uint64_t u64;
typedef int64_t  i64;

typedef float  f32;
typedef double f64;

typedef unsigned char byte;

#endif
