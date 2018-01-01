
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>

class NCCTracker
{
public:

    void init(cv::Mat & img, cv::Rect rect);

    cv::Rect track(cv::Mat img);
    
    void recenter(cv::Point2f position);

private:
    cv::Point2f p_position;

    cv::Size p_size;

    float p_window;

    cv::Mat p_template;
};