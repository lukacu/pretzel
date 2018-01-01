#ifndef PRETZEL_TIME_H
#define PRETZEL_TIME_H

#define TIME_ILLEGAL 0
#define TIME_FRAMES 1
#define TIME_MILLISECONDS 2
#define TIME_PERCENT 3

#include <opencv2/highgui.hpp>

typedef struct time_spec {
    short format;
    float time;
} time_spec;

time_spec create_time(short type, float time);

time_spec parse_time(const char* s);

long time_to_frames(const time_spec time, long length, float fps);

#endif

