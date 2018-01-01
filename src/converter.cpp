#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#if defined(__OS2__) || defined(__WINDOWS__) || defined(WIN32) || defined(WIN64) || defined(_MSC_VER)
#include <ctype.h>
#include <windows.h>
__inline void sleep(long time) {
    Sleep(time * 1000);
}
#else
#ifdef _MAC_
#else
    #include <unistd.h>
#endif
#include <signal.h>
#endif

#include "lib/projections.h"
#include "lib/time.h"
#include "lib/getopt.h"

using namespace cv;
using namespace std;

void configure_signals() {

    #if defined(__OS2__) || defined(__WINDOWS__) || defined(WIN32) || defined(WIN64) || defined(_MSC_VER)

    // TODO: anything to do here?

    #else

    struct sigaction sa;
    sa.sa_handler = SIG_DFL;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags =  SA_RESTART | SA_NOCLDSTOP; //SA_NOCLDWAIT;
    if (sigaction(SIGCHLD, &sa, 0) == -1) {
      perror(0);
      exit(1);
    }

    #endif

}

#define CMD_OPTIONS "hf:t:i"
#define WINDOW_NAME "PreTZel converter"

void print_help() {

    cout << "PreTZel converter" << "\n\n";
    cout << "Built on " << __DATE__ << " at " << __TIME__ << "\n\n";

    cout << "Usage: converter [-h] [-f from] [-t to] <file>";

    cout << "\n\nProgram arguments: \n";
    cout << "\t-f\tStart at frame (integer) or seconds (m:s:ms).\n";
    cout << "\t-t\tStop at frame (integer) or seconds (m:s:ms).\n";
    cout << "\t-i\tExport interactivelly (toggle export for multiple intervals).\n";
    cout << "\t-h\tDisplay this help.\n";
    cout << "\n";

    cout << "\n";
}

int main(int argc, char** argv) {

	int c;
	opterr = 0;
    bool interactive = false;

    time_spec from_time = create_time(TIME_PERCENT, 0);
    time_spec to_time = create_time(TIME_PERCENT, 1);

	int from = 0;
	int to = -1;

	configure_signals();

	string fullname;

    while ((c = getopt(argc, argv, CMD_OPTIONS)) != -1)
        switch (c) {
        case 'h':
            print_help();
            exit(0);
        case 'f':
            from_time = parse_time(optarg);
            break;
        case 't':
            to_time = parse_time(optarg);
            break;
        case 'i':
            interactive = true;
            break;
        default:
            print_help();
            return -1;
        }

    if (optind < argc) {

		fullname = string(argv[optind]);

    } else {
        print_help();
        exit(-1);
    }

	VideoCapture reader(fullname);

	Size equirectangularSize = Size((int)reader.get(CAP_PROP_FRAME_WIDTH), (int)reader.get(CAP_PROP_FRAME_HEIGHT));

	Mat mapx, mapy;

	cubemap_precompute(mapx, mapy, equirectangularSize);

	size_t lastindex = fullname.find_last_of(".");
	string rawname = fullname.substr(0, lastindex);

	VideoWriter writer;

    to = time_to_frames(to_time, (long) reader.get(CAP_PROP_FRAME_COUNT), reader.get(CAP_PROP_FPS));
    from = time_to_frames(from_time, (long) reader.get(CAP_PROP_FRAME_COUNT), reader.get(CAP_PROP_FPS));

	Mat equirectangular, cubemap;

	int length = (int) reader.get(CAP_PROP_FRAME_COUNT);

    if (to < from) {
        to = length;
    }

	reader.set(CAP_PROP_POS_FRAMES, from);

	cout << format("Cubemap resolution %dx%d, full length %d frames", mapx.cols, mapx.rows, length) << endl;

    if (!interactive) {
        string filename = rawname + ((to - from == length) ? String("_cubemap.avi") : format("_%d_%d_cubemap.avi", from, to));
        writer.open(filename,
            VideoWriter::fourcc('X','2','6','4'),
            reader.get(CAP_PROP_FPS), mapx.size(), true);
    } else {

        namedWindow(WINDOW_NAME, CV_GUI_NORMAL);

    }

    int timeout = -1;
    int clips = 1;
    int realtime = (int) 1000.0 / reader.get(CAP_PROP_FPS);

    Mat preview;

	while (true) {

		reader.read(equirectangular);
		if (equirectangular.empty())
			break;

        int position = (int) reader.get(CAP_PROP_POS_FRAMES);

        if (interactive) {

            resize(equirectangular, preview, Size(640, (int) ((equirectangular.rows * 640.0) / (equirectangular.cols))));

            putText(preview,
                format("Frame %d/%d (%.1f%%)", position - from, to - from,
                (float)(position - from) / (float)(to - from)), Point(2, 12),
                FONT_HERSHEY_COMPLEX_SMALL, 0.5f, Scalar(255, 0, 255));

            putText(preview, writer.isOpened() ? "Capturing" : "Not capturing",
                Point(2, 24), FONT_HERSHEY_COMPLEX_SMALL, 0.5f, Scalar(255, 0, 255));

            imshow(WINDOW_NAME, preview);

            int pressed_key = waitKey(timeout);

            switch ((char) pressed_key) {
                case ' ': {
                    break;
                }
                case 'p': {
                    timeout = (timeout < 1) ? realtime : -1;
                    break;
                }
                case 'c': {

                    if (writer.isOpened()) {
                        writer.release();
                    } else {
                        writer.open(rawname + format("_cubemap_%02d.avi", clips),
                            VideoWriter::fourcc('X','2','6','4'),
                            reader.get(CAP_PROP_FPS), mapx.size(), true);
                        clips ++;
                    }

                    break;
                }
                case 'q': {
                    position = to;
                    break;
                }
            }

        } else {

            cout << format("Frame %d/%d (%.1f%%)", position - from, to - from,
                (100.0f * (float)(position - from) / (float)(to - from))) << endl;

        }

        if (writer.isOpened()) {

            cubemap_map(equirectangular, cubemap, mapx, mapy);

            writer.write(cubemap);
        }

        if (position >= to)
            break;

	}

	reader.release();
	writer.release();
	return 0;
}


