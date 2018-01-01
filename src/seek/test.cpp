#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <ctime>

#include "video_reader.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {

    if (argc < 2) {

        Mat image(Size(640, 480), CV_8UC3);

        VideoWriter writer;

        writer.open("output.avi", 
            VideoWriter::fourcc('X','2','6','4'), 
            29, image.size(), true);

        for (int i = 0; i < 100; i++) {

            image.setTo(0);

            putText(image, format("%03d", i), Point(100, 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(255, 255, 255), 3);

            writer.write(image);

        }

        writer.release();        

        return 0;

    }

    bool run = true;

    cove::VideoReader seek_reader(argv[1]);
    cv::VideoCapture ocv_reader(argv[1]);

    while (run) {

        Mat ocv_image, seek_image;

        seek_reader.read(seek_image);
        ocv_reader.read(ocv_image);

        if (ocv_image.empty()) {
            cout << "OpenCV stream done" << endl;
        } else {
            imshow("OpenCV image", ocv_image);
        }

        if (seek_image.empty()) {
            cout << "Seek stream done" << endl;
        } else {
            imshow("Seek image", seek_image);
        }      

        if (ocv_image.empty() && seek_image.empty()) run = false;

        int i = waitKey(-1);

        switch ((char) i) {

            case 'q': { // quit
                run = false;
                break;
            }

            case 'b': { // quit
                ocv_reader.set(CAP_PROP_POS_FRAMES, 0);
                seek_reader.set(CAP_PROP_POS_FRAMES, 0);
                break;
            }

            case '1': { // quit
                ocv_reader.set(CAP_PROP_POS_FRAMES, 10);
                seek_reader.set(CAP_PROP_POS_FRAMES, 10);
                break;
            }

        }

    }

    ocv_reader.release();
    seek_reader.release();

    return 0;
}
