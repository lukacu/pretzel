#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stack>
#include <ctime>
#include <csignal>

#include "lib/projections.h"
#include "lib/annotations.h"
#include "lib/camera.h"
#include "lib/tracker.h"

#ifdef FAST_SEEK
#include "seek/video_reader.h"
typedef cove::VideoReader VideoReader;
#else
typedef cv::VideoCapture VideoReader;
#endif

#ifdef _WIN32
#include <direct.h>
#elif defined __linux__
#include <sys/stat.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// constants
#define WINDOW_NAME "PreTZel annotator"

// used namespaces
using namespace cv;
using namespace std;

// global vars
vector<SpherePolygon> trajectory;
int position = 0;
int goto_frame = 0;
int marked_frame = -1;
size_t video_length;
bool modified = false;
bool auto_recenter = false;

Mat timeline;

string file_name, file_dir;

#ifdef _WIN32
#define KEY_ARROW_LEFT (37)
#define KEY_ARROW_RIGHT (39)
#define KEY_ARROW_UP (38)
#define KEY_ARROW_DOWN (40)

#define KEY_TOGGLE_PLAYBACK (int)('p')
#define KEY_FRAME_BACK (int)('a')
#define KEY_FRAME_FORTH (int)('d')
#define KEY_JUMP_BACK (int)('s')
#define KEY_JUMP_FORTH (int)('w')
#define KEY_JUMP_BACK_MORE ((int)('S'))
#define KEY_JUMP_FORTH_MORE ((int)('W'))
#define KEY_JUMP_EMPTY (int)('n')
#define KEY_COPY_FORWARD (int)('y')
#define KEY_RECENTER (int)('r')
#define KEY_RECENTER_TOGGLE ((int)(18))  // ctrl + r
#define KEY_RESET_CAMERA ((int)('R'))
#define KEY_RECENTER (int)('r')
#define KEY_CLEAR (int)('C')
#define KEY_JUMP_START (int)('x')
#define KEY_SAVE ((int)(23))  // ctrl + w
#define KEY_SAVE_SCREEN ((int)(19))  // ctrl + s
#define KEY_TRACK_FILL ((int)('i'))
#define KEY_TRACK_SMOOTH TODO // ((int)('k'))
#define KEY_TRACK_FORWARD ((int)('t'))
#define KEY_JUMP_MARKED ((int)('m'))
#define KEY_SET_MARKED ((int)(13))  // ctrl + m
#define KEY_CLEAR_MARKED ((int)('M'))
#define KEY_QUIT ((int)('q'))
#define KEY_QUIT_FORCE ((int)('Q'))  // shift + q
#define KEY_CANCEL ((int)27)

string tmp_path = string("");

#else

#define MODIFIER_CTRL (1 << 18)
#define MODIFIER_SHIFT (1 << 16)

#define KEY_ARROW_LEFT (65361)
#define KEY_ARROW_RIGHT (65363)
#define KEY_ARROW_UP (65362)
#define KEY_ARROW_DOWN (65364)

#define KEY_TOGGLE_PLAYBACK (int)('p')
#define KEY_FRAME_BACK (int)('a')
#define KEY_FRAME_FORTH (int)('d')
#define KEY_JUMP_BACK (int)('s')
#define KEY_JUMP_FORTH (int)('w')
#define KEY_JUMP_BACK_MORE ((int)('s') | MODIFIER_SHIFT)
#define KEY_JUMP_FORTH_MORE ((int)('w') | MODIFIER_SHIFT)
#define KEY_JUMP_EMPTY (int)('n')
#define KEY_COPY_FORWARD (int)('y')
#define KEY_RECENTER (int)('r')
#define KEY_RECENTER_TOGGLE ((int)('r') | MODIFIER_CTRL)
#define KEY_RESET_CAMERA ((int)('r') | MODIFIER_SHIFT)
#define KEY_RECENTER (int)('r')
#define KEY_CLEAR ((int)('c') | MODIFIER_SHIFT)
#define KEY_JUMP_START (int)('x')
#define KEY_SAVE ((int)('w') | MODIFIER_CTRL)
#define KEY_SAVE_SCREEN ((int)('s') | MODIFIER_CTRL)
#define KEY_TRACK_FILL ((int)('i'))
#define KEY_TRACK_SMOOTH ((int)('k') | MODIFIER_CTRL)
#define KEY_TRACK_FORWARD ((int)('t'))
#define KEY_JUMP_MARKED ((int)('m'))
#define KEY_SET_MARKED ((int)('m') | MODIFIER_CTRL)
#define KEY_CLEAR_MARKED ((int)('m') | MODIFIER_SHIFT)
#define KEY_QUIT ((int)('q'))
#define KEY_QUIT_FORCE ((int)('q') | MODIFIER_SHIFT)
#define KEY_CANCEL ((int)27)

string tmp_path = string("/tmp/");

#endif

#define TOOL_NONE 0
#define TOOL_CREATE 1
#define TOOL_CREATE_CENTER 2
#define TOOL_MOVE 3
#define TOOL_RESIZE 4
#define TOOL_ROTATE 5
#define TOOL_ZOOM 6

#define STATUS_BACKGROUND Scalar(0, 0, 0)
#define STATUS_CURSOR Scalar(0, 0, 255)
#define STATUS_MARKER Scalar(255, 0, 0)
#define STATUS_FOREGROUND Scalar(255, 255, 255)
#define COLOR_DRAG_REGION Scalar(255, 0, 0)

#define DISTANCE(P) sqrt( (float)((P).x * (P).x) + (float)((P).y * (P).y))

void preprocess_trajectory(vector<SpherePolygon>& trajectory) {

    if (!getenv("PRETZEL_PREPROCESS")) return;

    string preprocess = string(getenv("PRETZEL_PREPROCESS"));
    CameraReference camera(new Camera());

    cout << "Preprocessing with operation " << preprocess << endl;

    if (preprocess == "flip") {
        // Temporary conversion to polygons
        for (size_t j = 0; j < trajectory.size(); j++) {
            SpherePolygon r = trajectory[j];
            for (int i = 0; i < r.size(); i++) {
                Point2f p = r.get(i);
                p.x = truncate_angle(p.x);
                //x = (x > 0 && x < M_PI) ? (M_PI - x) : (M_PI );
                p.x = truncate_angle(M_PI - p.x);
                r[i] = p;
            }
            trajectory[j] = r;
        }
        return;
    }

    if (preprocess == "shift45") {
        // Temporary conversion to polygons

        Matx33f rotation = euler_to_rotation(Point3f( M_PI / 4, 0, 0));
        for (size_t j = 0; j < trajectory.size(); j++) {
            SpherePolygon r = trajectory[j];
            for (int i = 0; i < r.size(); i++) {

                Point3f s = point_sphere_to_space(r.get(j));

                Matx33f norm = euler_to_rotation(Point3f(0, atan2(s.x, s.z), 0));
                Matx33f inv;

                transpose(norm, inv);

                r[i] = point_space_to_sphere( (inv * rotation * norm) * s );
            }
            trajectory[j] = r;
        }
        return;
    }

}


void read_frame(VideoReader& reader, Mat& img) {
    reader.read(img);
    if (img.empty()) {
        goto_frame = 0;
        position = 0;
        reader.set(CAP_PROP_POS_FRAMES, 0);
        reader.read(img);
    }
}

Mat generate_timeline(vector<SpherePolygon> &trajectory) {

    Mat timeline(Size(trajectory.size(), 4), CV_8UC3);
    timeline.setTo(0);

    for (size_t i = 0; i < trajectory.size(); i++) {
        if (!trajectory[i].valid())
            continue;

        timeline.at<Vec3b>(0, i) = Vec3b(255, 255, 255);

    }

    return timeline;

}

void copy_resize(Mat &src, Mat &dst, Point position, Size size) {

    vector<Point2f> src_points(3);
    vector<Point2f> dst_points(3);
    src_points[0] = Point2f(0, 0);
    src_points[1] = Point2f(0, src.rows);
    src_points[2] = Point2f(src.cols, src.rows);
    //src_points.push_back(Point(src.cols, 0));

    dst_points[0] = Point2f(position.x, position.y);
    dst_points[1] = Point2f(position.x, position.y + size.height);
    dst_points[2] = Point2f(position.x + size.width, position.y + size.height);
    //dst_points.push_back(position + Point(size.width, 0));

    Mat transform = getAffineTransform(src_points, dst_points);

    warpAffine(src, dst, transform, dst.size(), INTER_LINEAR, BORDER_TRANSPARENT);

}

void touch_trajectory() {

    timeline = generate_timeline(trajectory);
    modified = true;

}

void set_frame(size_t frame, const SpherePolygon region) {
    if (frame < 0 || frame >= trajectory.size())
        return;
    else
        trajectory[frame] = region;
    touch_trajectory();
}

#define MAX_FILL_INTERVAL 20

bool track_between(std::vector<SpherePolygon> &trajectory, VideoReader& reader, CameraReference camera, size_t start, size_t end) {

    if (end - start > MAX_FILL_INTERVAL || end - start < 2)
        return false;

    if (start < 0 || start >= trajectory.size() || end >= trajectory.size())
        return false;

    if (!trajectory[start].valid() || !trajectory[end].valid())
        return false;

    int backup_camera_position = reader.get(CAP_PROP_POS_FRAMES);
    bool success = true;

    reader.set(CAP_PROP_POS_FRAMES, start);

    Mat cubemap, sensor;

    reader.read(cubemap);
    if (cubemap.empty()) {
        reader.set(CAP_PROP_POS_FRAMES, backup_camera_position);
        return false;
    }

    CameraReference working(new Camera(*camera));
    center_camera_to_region(working, trajectory[start]);

    sensor.create(working->get_sensor_size(), cubemap.type());
    project_image(cubemap, working, sensor);

    NCCTracker tracker;

    PlanarPolygon initialize = trajectory[start].project(working);
    tracker.init(sensor, initialize.bounds());

    vector<SpherePolygon> tracked;

    for (size_t i = start + 1; i <= end; i++) {

        reader.read(cubemap);
        if (cubemap.empty()) {
            success = false;
            break;
        }

        project_image(cubemap, working, sensor);

        SpherePolygon predicted = PlanarPolygon(tracker.track(sensor)).unproject(working);

        tracked.push_back(predicted);

        center_camera_to_region(working, predicted);

        tracker.recenter(Point2f(sensor.cols / 2, sensor.rows / 2));

    }

    PlanarPolygon verify = trajectory[end].project(working);
    PlanarPolygon final = tracked[tracked.size()-1].project(working);

    float intersection = (verify.bounds() & final.bounds()).area();

    float overlap = (intersection) / (verify.bounds().area() + final.bounds().area() - intersection);

    if (overlap < 0.5) {
        success = false;
    }

    reader.set(CAP_PROP_POS_FRAMES, backup_camera_position);

    if (success) {

        for (size_t i = 0; i < tracked.size() - 1; i++) {
            trajectory[start + i + 1] = tracked[i];

        }

    }

    if (success) {
        interpolate_size_between(trajectory, camera, start, end);
    }

    return success;
}

SpherePolygon track_forward(CameraReference camera, const Mat& from_cubemap, SpherePolygon origin, const Mat& to_cubemap) {

    if (!origin.valid())
        return SpherePolygon();

    CameraReference working(new Camera(*camera));
    center_camera_to_region(working, origin);
    PlanarPolygon initialize = origin.project(working);
    NCCTracker tracker;

    Mat sensor;

    sensor.create(working->get_sensor_size(), from_cubemap.type());
    project_image(from_cubemap, working, sensor);

    tracker.init(sensor, initialize.bounds());

    project_image(to_cubemap, working, sensor);

    PlanarPolygon tracked(tracker.track(sensor));

    Point2f offset = tracked.center() - initialize.center();
    return initialize.move(offset.x, offset.y).unproject(working);

}

class AnnotatorEvents : WindowEvents {
public:

    AnnotatorEvents(const string& name, CameraReference camera) : WindowEvents(name), camera(camera) {

    }

    virtual ~AnnotatorEvents() {

    }

    virtual void handle_event(int event, int flags, Point point) {

        if (event == EVENT_MOUSEMOVE) {
            current = point;
            switch (tool_type) {
            case TOOL_MOVE: {

                break;
            }
            case TOOL_CREATE_CENTER:
            case TOOL_CREATE: {

                break;
            }
            case TOOL_ROTATE: {

                Size s = snapshot->get_sensor_size();
                Point2f vector = -(current - origin) + Point(s.width / 2, s.height / 2);

                Point2f p = point_sensor_to_sphere(vector, snapshot->get_sensor_size(), snapshot->get_orientation(),
                                                   snapshot->get_focal_length(), snapshot->get_dpi());
                camera->set_orientation(sphere_to_rotation(p, Point3f(0, 1, 0)));

                break;
            }
            case TOOL_ZOOM: {

                Point center = snapshot->get_sensor_size() / 2;
                float diff_current = DISTANCE(current - center);
                float diff_origin = DISTANCE(origin - center);

                float diff = (diff_current - diff_origin) / diff_origin;

                camera->set_focal_length(MAX(0.1, MIN(3, snapshot->get_focal_length() + diff)));

                break;
            }
            case TOOL_NONE: {



            }
            }
        }

        if (event == EVENT_LBUTTONDOWN) {

            origin = current = point;

            if (flags & EVENT_FLAG_CTRLKEY) {

                if (trajectory[position].valid()) {
                    PlanarPolygon old_region = trajectory[position].project(camera);
                    if (old_region.contains(origin.x, origin.y)) {
                        tool_type = TOOL_RESIZE;
                        return;
                    }
                }

                tool_type = TOOL_CREATE_CENTER;

            } else {

                if (trajectory[position].valid()) {
                    PlanarPolygon old_region = trajectory[position].project(camera);
                    if (old_region.contains(origin.x, origin.y)) {
                        tool_type = TOOL_MOVE;
                        return;
                    }
                }

                tool_type = TOOL_CREATE;

            }

        }

        if (event == EVENT_RBUTTONDOWN) {

            origin = current = point;

            if (flags & EVENT_FLAG_SHIFTKEY) {

                Point2f p = point_sensor_to_sphere(point, camera->get_sensor_size(), camera->get_orientation(),
                                                   camera->get_focal_length(), camera->get_dpi());
                camera->set_orientation(sphere_to_rotation(p, Point3f(0, 1, 0)));

            } else if (flags & EVENT_FLAG_CTRLKEY) {

                snapshot = CameraReference(new Camera(*camera));

                tool_type = TOOL_ZOOM;

            } else {

                snapshot = CameraReference(new Camera(*camera));

                tool_type = TOOL_ROTATE;

            }

        }

        if (event == EVENT_LBUTTONUP) {

            switch (tool_type) {
            case TOOL_MOVE: {
                if (trajectory[position].valid()) {
                    Point offset = current - origin;
                    PlanarPolygon projected = trajectory[position].project(camera);
                    projected = projected.move(offset.x, offset.y);
                    set_frame(position, projected.unproject(camera));
                }
                break;
            }
            case TOOL_CREATE: {

                float x = min(origin.x, current.x);
                float y = min(origin.y, current.y);
                float w = max(origin.x, current.x) - x;
                float h = max(origin.y, current.y) - y;

                PlanarPolygon region(x, y, w, h);
                set_frame(position, region.unproject(camera));

                break;
            }
            case TOOL_RESIZE: {
                PlanarPolygon projected = trajectory[position].project(camera);
                Rect2f bounds = projected.bounds();
                Point center = Point((bounds.x + bounds.width / 2), (bounds.y + bounds.height / 2));
                Point offset = current - center;
                Point original = origin - center;

                projected = projected.resize((float) offset.x / (float)original.x, (float) offset.y / (float)original.y);
                set_frame(position, projected.unproject(camera));
                break;
            }
            case TOOL_CREATE_CENTER: {

                Point offset = current - origin;

                float x = min(origin.x - offset.x, origin.x + offset.x);
                float y = min(origin.y - offset.y, origin.y + offset.y);
                float w = max(origin.x - offset.x, origin.x + offset.x) - x;
                float h = max(origin.y - offset.y, origin.y + offset.y) - y;

                PlanarPolygon region(x, y, w, h);
                set_frame(position, region.unproject(camera));

                break;
            }
            }

            tool_type = TOOL_NONE;

        }

        if (event == EVENT_RBUTTONUP) {

            tool_type = TOOL_NONE;

        }

    }

    void draw(Mat& canvas) {

        switch (tool_type) {
        case TOOL_MOVE: {
            Point offset = current - origin;
            PlanarPolygon projected = trajectory[position].project(camera);
            projected = projected.move(offset.x, offset.y);

            projected.draw(canvas, COLOR_DRAG_REGION, 3);

            break;
        }
        case TOOL_RESIZE: {
            PlanarPolygon projected = trajectory[position].project(camera);
            Rect2f bounds = projected.bounds();
            Point center = Point((bounds.x + bounds.width / 2), (bounds.y + bounds.height / 2));
            Point offset = current - center;
            Point original = origin - center;

            projected = projected.resize((float) offset.x / (float)original.x, (float) offset.y / (float)original.y);

            projected.draw(canvas, COLOR_DRAG_REGION, 3);

            break;
        }
        case TOOL_CREATE: {

            rectangle(canvas, origin, current, COLOR_DRAG_REGION, 3);

            break;
        }
        case TOOL_CREATE_CENTER: {

            Point offset = current - origin;

            rectangle(canvas, origin - offset, origin + offset, COLOR_DRAG_REGION, 3);

            break;
        }
        case TOOL_ROTATE: {

            show_sphere_helper(camera, canvas);

            break;
        }
        case TOOL_ZOOM: {

            show_sphere_helper(camera, canvas);

            break;
        }
        case TOOL_NONE: {
            show_status(camera, canvas);
            break;
        }
        }

    }

    void reset() {

        tool_type = TOOL_NONE;

    }

    void show_status(CameraReference &camera, Mat &img) {

        rectangle(img, Point(0, 0), Point(img.cols, 20), STATUS_BACKGROUND, -1);

        string status = format("%04d/%04d %s f=%.4f", position, video_length-1, (auto_recenter ? "center" : ""), camera->get_focal_length());

        putText(img, status, Point(2, 18), FONT_HERSHEY_COMPLEX_SMALL, 0.8f, STATUS_FOREGROUND);

        copy_resize(timeline, img, Point(0, img.rows - 15), Size(img.cols, 15));

        if (marked_frame > -1) {
            float p = ((float) marked_frame / (float) video_length) * img.cols;

            line(img, Point((int)p, img.rows - 15), Point((int)p, img.rows), STATUS_MARKER, 1);
        }

        float p = ((float) position / (float) video_length) * img.cols;

        line(img, Point((int)p, img.rows - 15), Point((int)p, img.rows), STATUS_CURSOR, 1);

    }

    void show_sphere_helper(CameraReference &camera, Mat &img) {

        draw_sphere(img, camera->get_orientation(), camera->get_focal_length(), camera->get_dpi());

    }

    CameraReference camera;
    CameraReference snapshot;
    vector<Point> points;
    Point current;
    int tool_type = 0;
    Point origin;

};

#define WAIT_TIME 25

void signal_handler(int signum) {
    string tmp_filename = tmp_path + file_name + string("_") + to_string(time(0)) + string(".tmp");
    cout << "Terminating. Saving backup state in: " << tmp_filename << endl;
    save_trajectory(tmp_filename, trajectory);

    exit(-1);
}

int main(int argc, char **argv) {

    if (argc < 2) {
        cout << "Name of video file is required." << endl;
        return -1;
    }

    string video_file(argv[1]);
    string annotations_file;
    size_t lastindex = video_file.find_last_of(".");
    size_t path_index = video_file.find_last_of("/") + 1; // +1 to include '/' in the file path
    string rawname = video_file.substr(0, lastindex);
    file_dir = video_file.substr(0, path_index); // tmp dir should be in the same dir as
    file_name = rawname.substr(path_index);

    if (argc >= 3) {
        annotations_file = string(argv[2]);
    } else {
        annotations_file = rawname + string(".txt");
    }

    if (0 == load_trajectory(annotations_file, trajectory)) {
        cout << "Cannot read annotations from " << annotations_file << endl;
    }

#ifndef DEBUG
    signal(SIGSEGV, signal_handler);
    signal(SIGABRT, signal_handler);
#endif

    Mat cubemap, sensor;

    CameraReference camera(new Camera());

    VideoReader reader(argv[1]);
    if (!reader.isOpened()) {
        cout << "Unable to open file." << endl;
        return -1;
    }

    video_length = reader.get(CAP_PROP_FRAME_COUNT);

    if (video_length < trajectory.size()) {

        trajectory = vector<SpherePolygon>(trajectory.begin(), trajectory.begin() + video_length);
    }

    if (video_length > trajectory.size()) {
        for (size_t i = trajectory.size(); i < video_length; i++)
            trajectory.push_back(SpherePolygon());
    }

    preprocess_trajectory(trajectory);

    touch_trajectory();
    modified = false;

    bool run = true;
    bool play = false;
    goto_frame = 0;
    position = -1;

    marked_frame = -1;

    namedWindow(WINDOW_NAME);
    AnnotatorEvents capture(WINDOW_NAME, camera);

    camera->set_orientation(Point3f());
    camera->set_focal_length(0.5);

    while (run) {

        if (play || position != goto_frame) {
            if (goto_frame < 0) {
                goto_frame = 0;
                if (position != 0) {
                    position = 0;
                    reader.set(CAP_PROP_POS_FRAMES, 0);
                    read_frame(reader, cubemap);
                }
            } else {
                if (position != goto_frame) {
                    if (position != goto_frame - 1)
                        reader.set(CAP_PROP_POS_FRAMES, goto_frame);
                    position = goto_frame;
                } else {
                    goto_frame++;
                    position++;
                }
                read_frame(reader, cubemap);
            }

            if (auto_recenter)
                center_camera_to_region(camera, trajectory[position]);
        }

        sensor.create(camera->get_sensor_size(), cubemap.type());

        project_image(cubemap, camera, sensor);

        if (trajectory[position].valid()) {
            trajectory[position].project(camera).draw(sensor, Scalar(0, 255, 255), 2);
        }

        capture.draw(sensor);

        imshow(WINDOW_NAME, sensor);

        int pressed_key = waitKey(WAIT_TIME);

#ifdef _WIN32


#else
        if (pressed_key != -1) {
            // Remove uppercase letters
            pressed_key = (pressed_key & 0x000FFF00) | ((0x00FF) & tolower(pressed_key & 0x00FF));
        }
#endif

        switch (pressed_key) {
        case KEY_RESET_CAMERA: {
            camera->set_orientation(Point3f());
            camera->set_focal_length(0.5);
        }
        case KEY_RECENTER: {
            center_camera_to_region(camera, trajectory[position]);
            break;
        }
        case KEY_RECENTER_TOGGLE: {
            auto_recenter = !auto_recenter;
            break;
        }
        case KEY_CANCEL: {
            capture.reset();
            marked_frame = -1;
            break;
        }
        case KEY_QUIT: {
            if (modified) {
                cout << "Press Shift + Q to quit without saving." << endl;
                break;
            }
        }
        case KEY_QUIT_FORCE: {
            cout << "Quitting." << endl;
            run = false;
            break;
        }
        case KEY_TOGGLE_PLAYBACK: {
            play = !play;
            break;
        }
        case KEY_JUMP_START: {
            goto_frame = 0;
            break;
        }
        case KEY_CLEAR: {
            set_frame(position, SpherePolygon());
            break;
        }
        case KEY_FRAME_FORTH:
        case KEY_ARROW_RIGHT: {
            goto_frame = min((int)video_length - 1, position + 1);
            break;
        }
        case KEY_FRAME_BACK:
        case KEY_ARROW_LEFT: {
            goto_frame = max(0, position - 1);
            break;
        }
        case KEY_JUMP_BACK:
        case KEY_ARROW_DOWN: {
            goto_frame = max(0, position - 10);
            break;
        }
        case KEY_JUMP_BACK_MORE: {
            goto_frame = max(0, position - 100);
            break;
        }
        case KEY_JUMP_FORTH:
        case KEY_ARROW_UP: {
            goto_frame = min((int)video_length - 1, position + 10);
            break;
        }
        case KEY_JUMP_FORTH_MORE: {
            goto_frame = min((int)video_length - 1, position + 100);
            break;
        }
        case KEY_SAVE: {
            if (save_trajectory(annotations_file, trajectory)) {
                cout << "Writing data to " << annotations_file << endl;
                modified = false;
            }
            break;
        }
        case KEY_TRACK_SMOOTH: {

            size_t start = marked_frame < 0 ? 0 : std::min(position, marked_frame);
            size_t end = marked_frame < 0 ? trajectory.size()-1 : std::max(position, marked_frame);

            bool result = smooth_size_between(trajectory, camera, start, end, 7);

            if (result) {
                touch_trajectory();
                cout << format("Smooth between %ld and %ld succesful.", start, end) << endl;
            }
            else
                cout << format("Smooth between %ld and %ld failed.", start, end) << endl;

            break;
        }
        case KEY_TRACK_FILL: {

            size_t start = find_previous_frame(trajectory, position, FIND_FULL);
            size_t end = find_next_frame(trajectory, position, FIND_FULL);

            bool result = track_between(trajectory, reader, camera, start, end);

            if (result) {
                touch_trajectory();
                cout << format("Autofill between %ld and %ld succesful.", start, end) << endl;
            }
            else
                cout << format("Autofill between %ld and %ld failed.", start, end) << endl;

            break;
        }
        case KEY_TRACK_FORWARD: {

            Mat cubemap_current;

            if (!trajectory[position].valid()) break;

            if ((position + 1) >= (int) trajectory.size()) break;

            SpherePolygon current = trajectory[position];

            cubemap.copyTo(cubemap_current);

            goto_frame++;
            position++;
            read_frame(reader, cubemap);

            SpherePolygon tracked = track_forward(camera, cubemap_current, current, cubemap);

            if (tracked.valid()) {
                set_frame(position, tracked);
            }

            if (auto_recenter)
                center_camera_to_region(camera, trajectory[position]);

            break;
        }
        case KEY_COPY_FORWARD: {

            Mat cubemap_current;

            if (!trajectory[position].valid()) break;

            if ((position + 1) >= (int) trajectory.size()) break;

            SpherePolygon current = trajectory[position];

            goto_frame++;
            position++;
            read_frame(reader, cubemap);
            set_frame(position, current);

            break;
        }

        case KEY_JUMP_EMPTY: {

            size_t f = find_next_frame(trajectory, 0, FIND_EMPTY);

            if (f < trajectory.size())
                goto_frame = f;

            break;
        }
        case KEY_JUMP_MARKED: {
            goto_frame = marked_frame;
            break;
        }
        case KEY_SET_MARKED: {

            marked_frame = position;

            break;
        }
        case KEY_CLEAR_MARKED: {

            marked_frame = -1;


            break;
        }
        default: {
#ifdef DEBUG
            if (pressed_key != -1)
                cout << format("Unknown key code %d", pressed_key) << endl;
#endif
            break;
        }
        }
    }

    reader.release();

    return 0;
}
