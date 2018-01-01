#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>

#if defined(__OS2__) || defined(__WINDOWS__) || defined(WIN32) || defined(WIN64) || defined(_MSC_VER)
#include <ctype.h>
#include <windows.h>
#include <direct.h>
__inline void sleep(long time) {
    Sleep(time * 1000);
}
#else
#ifdef _MAC_
#else
    #include <unistd.h>
    #include <sys/stat.h>
#endif
#include <signal.h>
#endif

#include "lib/projections.h"
#include "lib/annotations.h"
#include "lib/camera.h"
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

void save_image_list(const string& file, vector<string>& image_list) {

    std::ofstream output;

    output.open(file.c_str(), std::ofstream::out);

    for (vector<string>::iterator it = image_list.begin(); it != image_list.end(); it++) {

        output << *it << endl;

    }

    output.close();

}

void save_camera_views(const string& file, vector<CameraView>& camera_views) {

    std::ofstream output;

    output.open(file.c_str(), std::ofstream::out);

    for (vector<CameraView>::iterator it = camera_views.begin(); it != camera_views.end(); it++) {

        CameraRotation r = (*it).orientation;

        output << r.x << "," << r.y << "," << r.z << "," << (*it).focal_length << endl;

    }

    output.close();

}


#define CMD_OPTIONS "hpPavlf:t:o:c:C:S:"
#define WINDOW_NAME "PreTZel exporter"

void print_help() {

    cout << "PreTZel exporter" << "\n\n";
    cout << "Built on " << __DATE__ << " at " << __TIME__ << "\n\n";

    cout << "Usage: exporter [-h] [-f from] [-t to] [-o path] <file>";

    cout << "\n\nProgram arguments: \n";
    cout << "\t-f\tStart at frame (integer) or seconds (m:s:ms).\n";
    cout << "\t-t\tStop at frame (integer) or seconds (m:s:ms).\n";
    cout << "\t-c\tCamera views file.\n";
    cout << "\t-o\tOutput to a given directory.\n";
    cout << "\t-p\tOutput only the first (preview) image.\n";
    cout << "\t-a\tOutput annotations.\n";
    cout << "\t-v\tOutput camera view parameters.\n";
    cout << "\t-l\tOutput image list file.\n";
    cout << "\t-P\tPrint camera status.\n";
    cout << "\t-h\tDisplay this help.\n";
    cout << "\n";

    cout << "\n";
}

int main(int argc, char** argv) {

	int c;
    bool preview = false;
    bool annotations = false;
    bool parameters = false;
    bool list = false;
    bool print_camera = false;
	opterr = 0;
    vector<SpherePolygon> trajectory;
    vector<string> image_list;
    string output;
    vector<CameraView> camera_views;
    Size sensor_size(640, 480);

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
        case 'o':
            output = string(optarg);
            break;
        case 'p':
            preview = true;
            break;
        case 'a':
            annotations = true;
            break;
        case 'v':
            parameters = true;
            break;
        case 'l':
            list = true;
            break;
        case 'P':
            print_camera = true;
            break;
        case 'S': {
            char* var = optarg;
            char* ptr = strchr(var, 'x');
            if (!ptr) break;
            string ws(var, ptr-var);
            string hs(ptr+1);
            sensor_size.width = atoi(ws.c_str());
            sensor_size.height = atoi(hs.c_str());
            break;
        }
        case 'c': {
            load_camera_views(optarg, camera_views);
            break;
        } default:
            print_help();
            return -1;
        }

    if (optind < argc) {

		fullname = string(argv[optind]);

    } else {
        print_help();
        exit(-1);
    }

    Mat cubemap, sensor;
    Ptr<Camera> camera(new Camera(sensor_size));

	size_t lastindex = fullname.find_last_of(".");
	string rawname = fullname.substr(0, lastindex);
    string annotations_file = rawname + string(".txt");

    if (0 == load_trajectory(annotations_file, trajectory)) {
        cout << "Cannot read annotations from " << annotations_file << endl;
        return -1;
    }

    VideoCapture reader(fullname);
    if (!reader.isOpened()) {
        cout << "Unable to open file." << endl;
        return -1;
    }

    to = time_to_frames(to_time, (long) reader.get(CAP_PROP_FRAME_COUNT), reader.get(CAP_PROP_FPS));
    from = time_to_frames(from_time, (long) reader.get(CAP_PROP_FRAME_COUNT), reader.get(CAP_PROP_FPS));

	int length = (int) reader.get(CAP_PROP_FRAME_COUNT);

    if (to < from) {
        to = length;
    }

    size_t video_length = reader.get(CAP_PROP_FRAME_COUNT);

    if (video_length < trajectory.size()) {
        trajectory = vector<SpherePolygon>(trajectory.begin(), trajectory.begin() + video_length);
    }

    if (video_length > trajectory.size()) {
        for (size_t i = trajectory.size(); i < video_length; i++)
            trajectory.push_back(SpherePolygon());
    }

	reader.set(CAP_PROP_POS_FRAMES, from);

    camera->set_orientation(Point3f());
    camera->set_focal_length(0.2);

    if (output.empty()) {
        output = rawname;
    }

    cout << "Exporting to folder " << output << endl;

#ifdef _WIN32
    _mkdir(output.c_str());
#else
    mkdir(output.c_str(), ACCESSPERMS);
#endif

    vector<PlanarPolygon> projected_trajectory;

    int position = from;

	while (true) {

        cout << format("Frame %d/%d (%.1f%%)", position - from, to - from,
            (100.0f * (float)(position - from) / (float)(to - from))) << endl;

        if (position < (int)camera_views.size())
            camera->set_view(camera_views[position]);

        if (print_camera) {
            CameraRotation r = camera->get_orientation();
            cout << format("Camera status: %.2f,%.2f,%.2f f=%.2f", r.x, r.y, r.z, camera->get_focal_length()) << endl;
        }

        if (position >= (int) trajectory.size()) break;

        if (!preview || position == 0) {

            reader.read(cubemap);
            if (cubemap.empty())
                break;

            project_image(cubemap, camera, sensor);

            imwrite(format("%s/%08d.jpg", output.c_str(), position - from + 1), sensor);
        }

        image_list.push_back(format("%08d.jpg", position - from + 1));

        projected_trajectory.push_back(trajectory[position].project(camera));

        camera_views.push_back(camera->get_view());

        position++;

        if (position >= to)
            break;
	}

    if (annotations)
        save_trajectory(format("%s/groundtruth.txt", output.c_str()), projected_trajectory);

    if (list)
        save_image_list(format("%s/images.txt", output.c_str()), image_list);

    if (parameters)
        save_camera_views(format("%s/camera.txt", output.c_str()), camera_views);

	reader.release();
	return 0;
}


