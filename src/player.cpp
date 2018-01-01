#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include "lib/projections.h"
#include "lib/annotations.h"
#include "lib/camera.h"
#include "lib/time.h"
#include "lib/getopt.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace cv;
using namespace std;

void on_mouse(int event, int x, int y, int, void* data) {

	Camera* camera = (Camera*) data;

	if ( event == EVENT_LBUTTONDOWN ) {
		Point2d goal_point = point_sensor_to_sphere(Point(x, y),
		                     camera->get_sensor_size(), camera->get_orientation(),
		                     camera->get_focal_length(), camera->get_dpi());
		camera->set_orientation(sphere_to_rotation(goal_point, Point3f(0, 1, 0)));
	}

}

#define CMD_OPTIONS "hpc:S:o:f:t:0nd"
#define WINDOW_NAME "PreTZel Player"

void print_help() {

	cout << "PreTZel player" << "\n\n";
	cout << "Built on " << __DATE__ << " at " << __TIME__ << "\n\n";

	cout << "Usage: player [-h] [-f] [-c camera] <file> [trajectory1 ... trajectoryN]";

	cout << "\n\nProgram arguments: \n";
	cout << "\t-c\tCamera view parameters.\n";
	cout << "\t-p\tPause on startup.\n";
	cout << "\t-h\tDisplay this help.\n";
	cout << "\t-o\tOutput rendered video.\n";
	cout << "\t-0\tDisplay failure count.\n";
	cout << "\t-n\tDisplay no metadata.\n";
	cout << "\t-d\tDisplay all metadata.\n";
	cout << "\t-f\tDisplay from frame.\n";
	cout << "\t-t\tDisplay to frame.\n";
	cout << "\n";

	cout << "\n";
}

// player AVI -c camera.txt gt1, gt2
// player AVI gt1, gt2

class TrajectoryData {
public:
	string id;
	Scalar color;
	int failures;
	vector<vector<SpherePolygon> > annotations;

	TrajectoryData() {}

	TrajectoryData(string id) : id(id), failures(0) {

		color = Scalar(255, 0, 0);

		//color = Scalar(255 * (color_counter % 2), 255 * ((color_counter / 2) % 2), 255 * ((color_counter / 4) % 2));
		//color_counter++;

	}

	void append(string source) {

		vector<SpherePolygon> t;
		load_trajectory(source, t);
		annotations.push_back(t);

	}

};

Vec3f hsv2bgr(Vec3f hsv) {

	float h = hsv[0], s = hsv[1], v = hsv[2];
	float b, g, r;

	if ( s == 0 ) {
		b = g = r = v;
	} else {
		static const int sector_data[][3] =
		{{1, 3, 0}, {1, 0, 2}, {3, 0, 1}, {0, 2, 1}, {0, 1, 3}, {2, 1, 0}};
		float tab[4];
		int sector;
		h *= 6;
		cout << h << endl;
		if ( h < 0 )
				do h += 6; while ( h < 0 );
		else if ( h >= 6 )
				do h -= 6; while ( h >= 6 );
		sector = cvFloor(h);
		h -= sector;
		if ( (unsigned)sector >= 6u )
		{
			sector = 0;
			h = 0.f;
		}

		tab[0] = v;
		tab[1] = v * (1.f - s);
		tab[2] = v * (1.f - s * h);
		tab[3] = v * (1.f - s * (1.f - h));

		b = tab[sector_data[sector][0]];
		g = tab[sector_data[sector][1]];
		r = tab[sector_data[sector][2]];
	}

	Vec3f bgr;
	bgr[0] = b * 255; bgr[1] = g * 255; bgr[2] = r * 255;
	cout << hsv << bgr << endl;
	return bgr;
}



vector<Scalar> generate_colormap(int colors) {

	vector<Scalar> colormap(colors);

	for (int i = 0; i < colors; i++) {
		float saturation = (colors < 9) ? 1 : (i % (int) ceil(colors / 8.0f)) / (ceil(colors / 8.0f) - 1);
		float value = (saturation == 0) ? (i / (float) std::max(1, colors - 1)) : 1;

		colormap[i] = hsv2bgr(Vec3f((i / (float) std::max(1, colors)), saturation, value));
	}

	return colormap;
}

typedef map<string, TrajectoryData> Trajectories;

#define DISPLAY_POSITION 0x1
#define DISPLAY_LEGEND 0x2
#define DISPLAY_FAILURES 0x4

int main(int argc, char** argv) {

	int c;
	string videoname("");
	string outname("");
	Size sensor_size(640, 480);
	vector<CameraView> camera_views;
	Trajectories trajectories;
	vector<string> trajectory_order;
	bool free = true;
	bool run = true;
	bool play = true;
	bool step = false;
	bool repeat = false;
	int display = DISPLAY_POSITION;

    time_spec from_time = create_time(TIME_PERCENT, 0);
    time_spec to_time = create_time(TIME_PERCENT, 1);


	while ((c = getopt(argc, argv, CMD_OPTIONS)) != -1)
		switch (c) {
		case 'h':
			print_help();
			exit(0);
		case 'c':
			load_camera_views(optarg, camera_views);
			free = false;
			break;
		case 'p':
			play = false;
			break;
		case 'd':
			display |= DISPLAY_POSITION | DISPLAY_LEGEND;
			break;
        case 'f':
            from_time = parse_time(optarg);
            break;
        case 't':
            to_time = parse_time(optarg);
            break;
		case '0':
			display |= DISPLAY_FAILURES;
			break;
		case 'n':
			display = 0;
			break;
		case 'o':
			outname = string(optarg);
			break;
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
			return -1;
		}

	if (optind < argc) {

		videoname = string(argv[optind++]);

		while (optind < argc) {
			char* var = argv[optind];
			char* ptr = strchr(var, '=');
			string id, filename;
			if (!ptr) {
				id = var;
				filename = var;
			} else {
				id = string(var, ptr - var);
				filename = string(ptr + 1);
			}

			Trajectories::iterator it = trajectories.find(id);
			if (it == trajectories.end()) {
				TrajectoryData data(id);
				trajectories[id] = data;
				trajectory_order.push_back(id);
			}

  			trajectories[id].append(filename);

			optind++;
		}

		if (trajectory_order.size() > 0) {
			int colors = trajectories.size();
			vector<Scalar> colormap = generate_colormap(colors);

			for (size_t i = 0; i < trajectory_order.size(); i++) {
				trajectories[trajectory_order[i]].color  = colormap[i];
			}
		}

	} else {
		print_help();
		exit(-1);
	}

	Mat cubemap, sensor;

	VideoCapture reader(videoname);

	size_t lastindex = videoname.find_last_of(".");
	string rawname = videoname.substr(0, lastindex);

	if (!reader.isOpened()) {
		cout << "Unable to open video file." << endl;
		return -1;
	}

    size_t to = time_to_frames(to_time, (long) reader.get(CAP_PROP_FRAME_COUNT), reader.get(CAP_PROP_FPS));
    size_t from = time_to_frames(from_time, (long) reader.get(CAP_PROP_FRAME_COUNT), reader.get(CAP_PROP_FPS));


	int video_length = (int) reader.get(CAP_PROP_FRAME_COUNT);

    if (to < from) {
        to = video_length;
    }

	Ptr<Camera> camera(new Camera(sensor_size));

	namedWindow(WINDOW_NAME);

	if (free)
		setMouseCallback(WINDOW_NAME, on_mouse, camera.get());

	if (!play) reader.read(cubemap);

	int realtime_delta = (int) 1000.0 / reader.get(CAP_PROP_FPS);

	VideoWriter writer;

	if (!outname.empty()) {
    	writer.open(outname, VideoWriter::fourcc('X','2','6','4'), reader.get(CAP_PROP_FPS),
        	sensor_size, true);
    	repeat = false;
	}

	size_t written_image = 0;

    reader.set(CAP_PROP_POS_FRAMES, from);

	while (run) {

		size_t frame = (size_t) reader.get(CAP_PROP_POS_FRAMES);

		if (play || step) {
			reader.read(cubemap);
			if (cubemap.empty() || frame > to) {
				if (repeat) {
					reader.set(CAP_PROP_POS_FRAMES, from);
					reader.read(cubemap);
					// Reset failure counter
					for (Trajectories::iterator it = trajectories.begin(); it != trajectories.end(); it++) {
						it->second.failures = 0;
					}

				} else {
					run = false;
					break;
				}
			}
			step = false;
		}

		if (!free) {
			if (frame < camera_views.size())
				camera->set_view(camera_views[frame]);
		}

		project_image(cubemap, camera, sensor);

		for (size_t i = 0; i < trajectory_order.size(); i++) {
			for (size_t j = 0; j < trajectories[trajectory_order[i]].annotations.size(); j++) {

				if (frame >= trajectories[trajectory_order[i]].annotations[j].size()) continue;

				SpherePolygon r = trajectories[trajectory_order[i]].annotations[j][frame];

				if (!r.valid()) {
					if (r.size() == 1 && r.get(0).x == 2)
						trajectories[trajectory_order[i]].failures++;
				} else {
				    r.project(camera).draw(sensor, trajectories[trajectory_order[i]].color, 2);
                }
			}
		}

		if ((display & DISPLAY_LEGEND) != 0) {
			int k = 0;
			for (size_t i = 0; i < trajectory_order.size(); i++) {
				k++;
				string label = trajectories[trajectory_order[i]].id;
				if ((display & DISPLAY_FAILURES) != 0) {
					label = format("%s (%.1f)", trajectory_order[i].c_str(), trajectories[trajectory_order[i]].failures / (float)trajectories[trajectory_order[i]].annotations.size());
				}
				cv::putText(sensor, label, Point(10, 40 + k * 20), FONT_HERSHEY_SIMPLEX, 0.5, trajectories[trajectory_order[i]].color, 1);
			}
		}

		if ((display & DISPLAY_POSITION) != 0) {
			cv::putText(sensor, cv::format("%d/%d", frame + 1, video_length), Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
		}

		imshow(WINDOW_NAME, sensor);

		if (writer.isOpened() && written_image != frame) {
			writer.write(sensor);
			written_image = frame;
		}

		int i = waitKey((play || !free) ? realtime_delta : -1);

		switch ((char)i) {
		case 'r': {
			if (free) {
				camera->set_orientation(Point3f());
				camera->set_focal_length(0.2f);
			}
			break;
		}
		case 'q': {
			run = false;
			break;
		}
		case 'p': {
			play = !play;
			break;
		}
		case 'n': {
			display = 0;
			break;
		}
		case 's': {
			string imagefilename = rawname + format("_%04d.jpg", frame);
			imwrite(imagefilename, sensor);
			cout << "Written image to " << imagefilename << endl;
			break;
		}
		case ' ': {
			step = true;
			break;
		}
		}

	}

	if (writer.isOpened()) writer.release();
	reader.release();

	return 0;
}


