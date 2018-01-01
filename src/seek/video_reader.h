#ifndef VIDEO_READER_H
#define VIDEO_READER_H

#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

namespace cove {

class VideoReaderPrivate;

class VideoReader {
public:
    VideoReader ();
    VideoReader (const string& filename);
    virtual ~VideoReader ();

    double get(int propid) const;
    bool set(int propid, double value);

    bool isOpened() const;

    bool open(const string &filename);
    void release();

    bool grab();
    bool retrieve (cv::OutputArray image, int flag = 0 );   

    void read(cv::OutputArray image);

protected:

    VideoReaderPrivate *impl;

};

}

#endif
