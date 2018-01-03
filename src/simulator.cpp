
#include <iostream>
#include <algorithm>
#include <fstream>
#include <trax/client.hpp>
#include <trax/opencv.hpp>
#include <opencv2/core.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if defined(__OS2__) || defined(__WINDOWS__) || defined(WIN32) || defined(WIN64) || defined(_MSC_VER)
#define NOMINMAX
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

#include <time.h>

#include "lib/projections.h"
#include "lib/annotations.h"
#include "lib/camera.h"
#include "lib/getopt.h"

using namespace cv;
using namespace std;
using namespace trax;

#define WINDOW_NAME "PreTZel"

Image convert_image(Mat& sensor, int formats) {

    Image image;

    if TRAX_SUPPORTS(formats, TRAX_IMAGE_BUFFER) {
        vector<uchar> buffer;
        imencode(".jpg", sensor, buffer);
        image = Image::create_buffer(buffer.size(), (char*) &buffer[0]);
    } else if TRAX_SUPPORTS(formats, TRAX_IMAGE_MEMORY) {
        image = trax::mat_to_image(sensor);
    } else
        throw std::runtime_error("No supported image format allowed");

    return image;

}

void save_timings(const string& file, vector<long>& timings) {

    std::ofstream output;

    output.open(file.c_str(), std::ofstream::out);

    for (vector<long>::iterator it = timings.begin(); it != timings.end(); it++) {

        output << *it << endl;

    }

    output.close();

}

bool read_frame(VideoCapture& reader, int frame, Mat& image) {

    int current = reader.get(CAP_PROP_POS_FRAMES);

    if (frame < current) return false;

    while (current != frame + 1) {
        reader.read(image);
        current++;
    }

    return !image.empty();

}


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

#define GROUNDTRUTH_COLOR Scalar(100, 255, 100)
#define TRACKER_COLOR Scalar(100, 100, 255)

#define CMD_OPTIONS "hsdV:G:f:O:r:t:T:p:e:c:xXgm:"

void print_help() {

    cout << "PreTZel simulator client" << "\n\n";
    cout << "Built on " << __DATE__ << " at " << __TIME__ << "\n";
    cout << "TraX library version: " << trax_version() << "\n";
    cout << "Protocol version: " << TRAX_VERSION << "\n\n";

    cout << "Usage: pretzel_simulator [-h] [-d] [-V video_file] [-O output_file] \n";
    cout << "\t [-f threshold] [-r frames] [-G groundtruth_file] [-e name=value] \n";
    cout << "\t [-p name=value] [-t timeout] [-s] [-T timing_file] [-x] [-X]\n";
    cout << "\t [-c controller] [-C key=value] [-m frame] [-S WxH] [-g] -- <command_part1> <command_part2> ...";

    cout << "\n\nProgram arguments: \n";
    cout << "\t-S\tSensor size.\n";
    cout << "\t-c\tCamera views file.\n";
    cout << "\t-d\tEnable debug\n";
    cout << "\t-e\tEnvironmental variable (multiple occurences allowed)\n";
    cout << "\t-f\tFailure threshold\n";
    cout << "\t-G\tGroundtruth annotations file\n";
    cout << "\t-g\tVisualize result in a window\n";
    cout << "\t-h\tPrint this help and exit\n";
    cout << "\t-V\tVideo file for the input cubemap sequence.\n";
    cout << "\t-O\tOutput region file\n";
    cout << "\t-p\tTracker parameter (multiple occurences allowed)\n";
    cout << "\t-r\tReinitialization offset\n";
    cout << "\t-T\tOutput timings file\n";
    cout << "\t-t\tSet timeout period\n";
    cout << "\t-m\tMove to specified frame before starting\n";
    cout << "\t-x\tUse explicit streams, not standard ones.\n";
    cout << "\t-X\tUse TCP/IP sockets instead of file streams.\n";
    cout << "\n";

    cout << "\n";
}

#define DEBUGMSG(...) if (verbosity == VERBOSITY_DEBUG) { fprintf(stdout, "CLIENT: "); fprintf(stdout, __VA_ARGS__); }

int main(int argc, char** argv) {

    int c;
    opterr = 0;
    int result = 0;
    ConnectionMode connection = CONNECTION_DEFAULT;
    VerbosityMode verbosity = VERBOSITY_DEFAULT;
    float threshold = -1;
    int move_to = 0;
    int timeout = 30;
    int reinitialize = 0;
    bool gui = false;
    Size sensor_size(640, 480);

    string timing_file;
    string tracker_command;
    string video_file("input.avi");
    string camera_file("camera.yaml");
    string groundtruth_file("groundtruth.txt");
    string output_file("output.txt");
    vector<CameraView> camera_views;

    Properties properties;
    map<string, string> environment;

    configure_signals();

    try {

        while ((c = getopt(argc, argv, CMD_OPTIONS)) != -1)
            switch (c) {
            case 'h':
                print_help();
                exit(0);
            case 'd':
                verbosity = VERBOSITY_DEBUG;
                break;
            case 's':
                verbosity = VERBOSITY_SILENT;
                break;
            case 'x':
                connection = CONNECTION_EXPLICIT;
                break;
            case 'X':
                connection = CONNECTION_SOCKETS;
                break;
            case 'g':
                gui = true;
                break;
            case 'V':
                video_file = string(optarg);
                break;
            case 'G':
                groundtruth_file = string(optarg);
                break;
            case 'O':
                output_file = string(optarg);
                break;
            case 'f':
                threshold = (float) MIN(1, MAX(0, (float)atof(optarg)));
                break;
            case 'r':
                reinitialize = MAX(1, atoi(optarg));
                break;
            case 't':
                timeout = MAX(0, atoi(optarg));
                break;
            case 'm':
                move_to = MAX(0, atoi(optarg));
                break;
            case 'T':
                timing_file = string(optarg);
                break;
            case 'e': {
                char* var = optarg;
                char* ptr = strchr(var, '=');
                if (!ptr) break;
                environment[string(var, ptr - var)] = string(ptr + 1);
                break;
            }
            case 'p': {
                char* var = optarg;
                char* ptr = strchr(var, '=');
                if (!ptr) break;
                string key(var, ptr - var);
                string value(ptr + 1);
                properties.set(key, value);
                break;
            }
            case 'c': {
                load_camera_views(optarg, camera_views);
                break;
            }
            case 'S': {
                char* var = optarg;
                char* ptr = strchr(var, 'x');
                if (!ptr) break;
                string ws(var, ptr - var);
                string hs(ptr + 1);
                sensor_size.width = atoi(ws.c_str());
                sensor_size.height = atoi(hs.c_str());
                break;
            }
            default:
                print_help();
                throw std::runtime_error(string("Unknown switch -") + string(1, (char) optopt));
            }

        if (optind < argc) {

            stringstream buffer;

            for (int i = optind; i < argc; i++) {
                buffer << " \"" << string(argv[i]) << "\" ";
            }

            tracker_command = buffer.str();

        } else {
            print_help();
            exit(-1);
        }

        Ptr<Camera> camera(new Camera(sensor_size));

        VideoCapture reader(video_file);
        Mat cubemap;

        int video_length = reader.get(CAP_PROP_FRAME_COUNT);

        vector<SpherePolygon> groundtruth;
        vector<SpherePolygon> output(video_length);
        vector<long> timings(video_length, 0);

        if (!groundtruth_file.empty()) {
            load_trajectory(groundtruth_file, groundtruth);
            DEBUGMSG("Groundtruth loaded from file %s.\n", groundtruth_file.c_str());
        }

        DEBUGMSG("Video will be loaded from file %s.\n", video_file.c_str());

        if (gui)
            namedWindow(WINDOW_NAME, CV_GUI_NORMAL);

        if (video_length < (int)groundtruth.size()) {
            DEBUGMSG("Warning: Image sequence shorter that groundtruth. Truncating.\n");
            groundtruth = vector<SpherePolygon>(groundtruth.begin(), groundtruth.begin() + video_length);
        }

        if (video_length > (int)groundtruth.size()) {
            DEBUGMSG("Warning: Image sequence longer that groundtruth. Extending with empty values.\n");
            for (int i = (int)groundtruth.size(); i < video_length; i++)
                groundtruth.push_back(SpherePolygon());
        }

        DEBUGMSG("Sequence length: %d frames.\n", video_length);

        TrackerProcess tracker(tracker_command, environment, timeout, connection, verbosity);

        int frame = 0;

        if (move_to > 0) {
            for (; frame < move_to && frame < video_length; frame++) {
                output[frame] = SpherePolygon::code(0);
            }
        }

		tracker.query();

        if (!tracker.ready()) {
            throw std::runtime_error("Tracker process not alive anymore.");
        }

		Metadata metadata = tracker.metadata();

        while (frame < video_length) {

            SpherePolygon initialization_region;

            if (frame < (int)camera_views.size())
                camera->set_view(camera_views[frame]);

            for (; frame < video_length; frame++) {
                if (groundtruth[frame].valid()) {
                    initialization_region = groundtruth[frame];
                    break;
                }
                if (gui) {
                    if (!read_frame(reader, frame, cubemap)) {
                        DEBUGMSG("No more data available. Stopping. \n");
                        break;
                    }
                    initialization_region = get_region_interactive(WINDOW_NAME, cubemap, camera);
                    if (initialization_region.valid()) {
                        DEBUGMSG("Using interactive region.\n");
                        break;
                    }
                }

                DEBUGMSG("Skipping frame %d, no initialization data. \n", frame);
                output[frame] = SpherePolygon::code(0);
            }

            if (!read_frame(reader, frame, cubemap)) {
                DEBUGMSG("No more data available. Stopping. \n");
                break;
            }

            if (!tracker.ready()) {
                throw std::runtime_error("Tracker process not alive anymore.");
            }

            DEBUGMSG("Initialization on frame %d. \n", frame);

            Region initialize = initialization_region.project(camera).region();

            Mat sensor;
            project_image(cubemap, camera, sensor);
            Image image = convert_image(sensor, metadata.image_formats());

            // Start timing a frame
            clock_t timing_toc;
            clock_t timing_tic = clock();

            if (!tracker.initialize(image, initialize, properties)) {
                throw std::runtime_error("Unable to initialize tracker.");
            }

            if (gui) {
                draw_region(sensor, initialize, GROUNDTRUTH_COLOR, 3);
                imshow(WINDOW_NAME, sensor);
                waitKey(25);
            }

            while (frame < video_length) {
                // Repeat while tracking the target.
                Region status;
                Properties additional;

                bool result = tracker.wait(status, additional);

                // Stop timing a frame
                timing_toc = clock();

                if (frame < (int)camera_views.size())
                    camera->set_view(camera_views[frame]);

                Region reference = groundtruth[frame].project(camera).region();

                if (result) {
                    // Default option, the tracker returns a valid status.
                    if (threshold >= 0) {
                        trax::Bounds bounds(0, 0, sensor_size.width, sensor_size.height);
                        float overlap = reference.overlap(status, bounds);

                        DEBUGMSG("Region overlap: %.2f\n", overlap);

                        // Check the failure criterion.
                        if (overlap <= threshold) {
                            // Break the tracking loop if the tracker failed.
                            break;
                        }
                    }

                    output[frame] = PlanarPolygon(status).unproject(camera);
                    timings[frame] = ((timing_toc - timing_tic) * 1000) / CLOCKS_PER_SEC;

                } else {
                    if (tracker.ready()) {
                        // The tracker has requested termination of connection.
                        DEBUGMSG("Termination requested by tracker.\n");
                        break;
                    } else {
                        // In case of an error ...
                        throw std::runtime_error("Unable to contact tracker.");
                    }
                }

                if (gui) {
                    draw_region(sensor, reference, GROUNDTRUTH_COLOR, 1);
                    draw_region(sensor, status, TRACKER_COLOR, 1);
                    imshow(WINDOW_NAME, sensor);
                    waitKey(25);
                }

                frame++;

                if (!read_frame(reader, frame, cubemap)) {
                    DEBUGMSG("No more data available. Stopping. \n");
                    break;
                }

                if (frame < (int)camera_views.size())
                    camera->set_view(camera_views[frame]);

                project_image(cubemap, camera, sensor);
                image = convert_image(sensor, metadata.image_formats());

                // Start timing a frame
                timing_tic = clock();

                Properties no_properties;
                if (!tracker.frame(image, no_properties))
                    throw std::runtime_error("Unable to send new frame.");

            }

            if (frame < video_length) {
                // If the tracker was not successful and we have to consider the remaining frames.

                if (reinitialize > 0) {
                    // If reinitialization is specified after 1 or more frames ...
                    output[frame] = SpherePolygon::code(2);
                    int j = frame + 1;
                    for (; j < frame + reinitialize && j < video_length; j++) {
                        output[j] = SpherePolygon::code(0);
                    }
                    frame = j;

                    if (frame < video_length) {
                        reader.set(CAP_PROP_POS_FRAMES, frame);
                        tracker.reset();
                    }

                } else {
                    // ... otherwise just fill the remaining part of sequence with empty frames.
                    output[frame] = SpherePolygon::code(2);
                    int j = frame + 1;
                    for (; j < video_length; j++) {
                        output[j] = SpherePolygon::code(0);
                    }
                    break;
                }
            }
        }

        reader.release();

        if (output.size() > 0)
            save_trajectory(output_file, output);

        if (timings.size() > 0 && timing_file.size() > 0)
            save_timings(timing_file, timings);

    } catch (std::runtime_error e) {

        fprintf(stderr, "Error: %s\n", e.what());
        result = -1;

    }

    exit(result);

}
