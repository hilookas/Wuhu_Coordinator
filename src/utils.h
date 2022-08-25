#ifndef _UTILS_H_
#define _UTILS_H_

// #pragma once

#define CAM_W 140
#define CAM_H 70

// 数据类型声明
typedef unsigned char u8;       //  8 bits
typedef unsigned short int u16; // 16 bits
typedef unsigned long int u32;  // 32 bits
typedef unsigned long long u64; // 64 bits

typedef char i8;       //  8 bits
typedef short int i16; // 16 bits
typedef long int i32;  // 32 bits
typedef long long i64; // 64 bits

typedef volatile i8 vi8;   //  8 bits
typedef volatile i16 vi16; // 16 bits
typedef volatile i32 vi32; // 32 bits
typedef volatile i64 vi64; // 64 bits

typedef volatile u8 vu8;   //  8 bits
typedef volatile u16 vu16; // 16 bits
typedef volatile u32 vu32; // 32 bits
typedef volatile u64 vu64; // 64 bits

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_WHITE   "\x1b[37m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#endif