#ifndef VIDEO_READER_P_H
#define VIDEO_READER_P_H

#include "video_reader.h"

#include <iostream>
#include <vector>
#include <map>

using namespace std;

// ffmpeg/libav includes
#ifdef __cplusplus
#define __STDC_CONSTANT_MACROS
#ifdef _STDINT_H
#undef _STDINT_H
#endif
#include <stdint.h>
#endif

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}

namespace cove {

class VideoReaderPrivate {
    friend VideoReader;
public:
    VideoReaderPrivate ();
    virtual ~VideoReaderPrivate ();

public:
    bool openVideo(const string&);
    bool closeVideo();
    bool isOpened();

    bool buildSeekTable ();
    bool loadSeekTable (const string &);
    bool saveSeekTable (const string &);

    bool stepForward ();

    void seekToNearestKeyFrame (uint32_t) throw (string);
    void seekToFrame (uint32_t) throw (string);

    void retrieveImage (const uint8_t *&, int &, int &, int &) const;

    class SeekTableEntry
    {
    public:
        friend std::istream& operator >> (std::istream&, SeekTableEntry &);
        friend std::ostream& operator << (std::ostream&, const SeekTableEntry &);
    public:
        uint32_t index;
        double time;
        int64_t firstPacketDts;
        int64_t lastPacketDts;
    };

protected:
    string getSeekTableFile();

    void clearState ();

    void resetFrameCache ();

    void seekToNearestKeyFrame (uint32_t, uint32_t) throw (string);

    SeekTableEntry getNearestSeekTableEntry (uint32_t, uint32_t) throw (string);
    void appendSeekTableEntry (const SeekTableEntry &);

protected:
    const int fastSeekDistance;

    string videoFileName;

    // ffmpeg/libav stuff
    AVFormatContext *formatContext;
    AVCodecContext *codecContext;
    SwsContext *scaleContext;
    int videoStreamIndex;
    AVCodec *videoCodec;
    AVFrame *frame;

    AVFrame *frameRGB;
    uint8_t *bufferRGB;

    // Video reader's state variables
    bool middleOfKeyFrameSeek;

    bool frameAvailable;
    bool seekTableComplete;

    int64_t firstDts;
    int64_t firstFramePts;
    int64_t previousDts;
    int64_t currentDts;
    int64_t keyFramePacketDts;

    uint32_t currentKeyFrameEntry;
    uint32_t currentFrameIndex;
    double currentFrameTime;

    uint32_t numberOfFrames;
    double videoDuration;

    // *** Seek table ***
    vector<SeekTableEntry> seekTable;
    map<uint32_t, uint32_t> reverseKeyFrameMap;

};

}

#endif
