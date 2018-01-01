
#include "time.h"

using namespace cv;

time_spec create_time(short type, float time) {
    time_spec result;

    result.format = type;
    result.time = time;

    return result;

}

time_spec parse_time(const char* s) {

    int i, j, k;
    char temp[2];
    float percent;
    time_spec result;

    result.format = TIME_ILLEGAL;
    if (sscanf(s, "%d:%d:%d", &i, &j, &k) == 3) {
        result.format = TIME_MILLISECONDS;
        result.time = i * 60 * 1000 + j * 1000 + k;
    } else if (sscanf(s, "%d:%d", &i, &j) == 2) {
        result.format = TIME_MILLISECONDS;
        result.time = i * 1000 + j;
    } else if (sscanf(s, "%f%[%%]", &percent, temp) == 2) {
        result.format = TIME_PERCENT;
        result.time = percent / 100;
    } else if (sscanf(s, "%d", &i) == 1) {
        result.format = TIME_FRAMES;
        result.time = i;
    }

    return result;
}

long time_to_frames(const time_spec time, long length, float fps) {

    long frames = 0;

    assert(time.format != TIME_ILLEGAL);

    switch(time.format) {
        case TIME_FRAMES: {
            frames = (long) time.time;
            break;
        }
        case TIME_PERCENT: {
            frames = (long) (time.time * length);
            break;
        }
        case TIME_MILLISECONDS: {
            frames = (long) ((float) time.time * fps / 1000.0);
            break;
        }

    }

    return std::min(length, frames);
}
