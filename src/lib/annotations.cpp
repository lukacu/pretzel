// includes
#include <limits>
#include <trax/opencv.hpp>
#include "annotations.h"

// definition of global constants
#define MIN_FOCAL_LENGTH 0.8
#define MAX_FRAMES_TO_INTERPOLATE 10

Polygon::Polygon() {

}


Polygon::Polygon(vector<Point2f> points) : _points(points) {


}

Polygon::Polygon(int i) {

    _points.resize(i);

}

Polygon::~Polygon() {

}

bool Polygon::valid() const {
    return _points.size() > 2;
}

int Polygon::size() const {
    return _points.size();
}

Point2f& Polygon::operator[](int i) {
     return _points[i];
}

Point2f Polygon::get(int i) const {
    return _points[i];
}

SpherePolygon::SpherePolygon(vector<Point2f> points) : Polygon(points) {

}

SpherePolygon::SpherePolygon(int i) : Polygon(i) {

}

SpherePolygon::SpherePolygon() {

}

PlanarPolygon::PlanarPolygon() {

}

PlanarPolygon::PlanarPolygon(int x, int y, int w, int h) {

    _points.push_back(Point2f(x, y));
    _points.push_back(Point2f(x + w, y));
    _points.push_back(Point2f(x + w, y + h));
    _points.push_back(Point2f(x, y + h));

}

PlanarPolygon::PlanarPolygon(Rect2f rect) {

    _points.push_back(Point2f(rect.x, rect.y));
    _points.push_back(Point2f(rect.x + rect.width, rect.y));
    _points.push_back(Point2f(rect.x + rect.width, rect.y + rect.height));
    _points.push_back(Point2f(rect.x, rect.y + rect.height));

}

PlanarPolygon::PlanarPolygon(int i) : Polygon(i) {
}

PlanarPolygon::PlanarPolygon(vector<Point2f> points) : Polygon(points) {

}


PlanarPolygon::~PlanarPolygon() {

}
SpherePolygon::~SpherePolygon() {

}


PlanarPolygon SpherePolygon::project(CameraReference camera) const {

    if (!valid())
        return PlanarPolygon();

    CameraRotation orientation = camera->get_orientation();
    float focal_length = camera->get_focal_length();
    int dpi = camera->get_dpi();
    Size sensor_size = camera->get_sensor_size();

    PlanarPolygon result(size());

    for (int i = 0; i < size(); i++) {
        result[i] = point_sphere_to_sensor(get(i), sensor_size, orientation, focal_length, dpi);
    }

    return result;

}

/*
void draw_spherical_region(cv::Mat& canvas, const SpherePolygon& region, CameraReference camera, cv::Scalar color, int width) {

    if (!region.valid())
        return;

    CameraRotation orientation = camera->get_orientation();
    float focal_length = camera->get_focal_length();
    int dpi = camera->get_dpi();
    Size sensor_size = camera->get_sensor_size();

    std::vector<cv::Point> points(region.size());

    for (int i = 0; i < region.size(); i++) {
        Point2d t = point_sphere_to_sensor(region.get(i), sensor_size, orientation, focal_length, dpi);
        points[i] = Point((int)t.x,  (int)t.y);
    }

    const cv::Point* ppoints = &points[0];
    int npoints = (int) points.size();

    if (width < 1)
        cv::fillPoly(canvas, &ppoints, &npoints, 1, color);
    else
        cv::polylines(canvas, &ppoints, &npoints, 1, true, color, width);

}
*/
Point2f Polygon::center() const {

    if (!valid())
        return Point2f(NAN, NAN);

    Point2f center;
    int valid = 0;
    for (int i = 0; i < size(); i++) {
        Point2f p = get(i);
        if (isnan(p.x) || isnan(p.y)) continue;
        center.x += p.x;
        center.y += p.y;
        valid++;
    }

    center.x /= valid;
    center.y /= valid;

    return center;


}

SpherePolygon PlanarPolygon::unproject(CameraReference camera) const {

    if (!valid())
        return SpherePolygon();

    CameraRotation orientation = camera->get_orientation();
    float focal_length = camera->get_focal_length();
    int dpi = camera->get_dpi();
    Size sensor_size = camera->get_sensor_size();

    SpherePolygon result(size());

    for (int i = 0; i < size(); i++) {
        result[i] = point_sensor_to_sphere(get(i), sensor_size, orientation, focal_length, dpi);
    }

    return result;
}


#ifdef PRETZEL_TRAX

trax::Region PlanarPolygon::region() {

    trax::Region region = trax::Region::create_polygon(_points.size());

    for (int i = 0; i < size(); i++) {
        region.set_polygon_point(i, _points[i].x, _points[i].y);
    }

    return region;
}

PlanarPolygon::PlanarPolygon(const trax::Region& region) {

    trax::Region r = region;

    if (region.type() == TRAX_REGION_RECTANGLE) {
        r = region.convert(TRAX_REGION_POLYGON);
    }


    if (r.type() == TRAX_REGION_POLYGON) {

        for (int i = 0; i < r.get_polygon_count(); i++) {
            float x, y;
            r.get_polygon_point(i, &x, &y);
            _points.push_back(Point2f(x, y));
        }

    }

}

SpherePolygon SpherePolygon::code(int code) {

    SpherePolygon p(1);
    p[0].x = code;
    return p;

}

#endif

bool PlanarPolygon::contains(float x, float y) const {
	size_t i, j;
    bool c = false;
	for (i = 0, j = _points.size() - 1; i < _points.size(); j = i++) {
		if ( ((_points[i].y > y) != (_points[j].y > y)) &&
		        (x < (_points[j].x - _points[i].x) * (y - _points[i].y) / (_points[j].y - _points[i].y) + _points[i].x) )
			c = !c;
	}
	return c;
}

Rect2f PlanarPolygon::bounds() const {

    float x1 = std::numeric_limits<float>::max(), x2 = std::numeric_limits<float>::min();
    float y1 = std::numeric_limits<float>::max(), y2 = std::numeric_limits<float>::min();

    for (size_t i = 0; i < _points.size(); i++) {
        x1 = std::min(x1, _points[i].x);
        x2 = std::max(x2, _points[i].x);
        y1 = std::min(y1, _points[i].y);
        y2 = std::max(y2, _points[i].y);

    }

    return Rect2f(x1, y1, x2 - x1, y2 - y1);
}


PlanarPolygon PlanarPolygon::move(float x, float y) const {

    PlanarPolygon result(size());

    for (int i = 0; i < size(); i++) {
        Point2f p = get(i);
        result[i] = Point2f(x + p.x, y + p.y);
    }
    return result;

}

PlanarPolygon PlanarPolygon::resize(float sx, float sy) const {

    PlanarPolygon result(size());

    Rect2f bounds = result.bounds();

    float cx = bounds.x + (bounds.width) / 2;
    float cy = bounds.y + (bounds.height) / 2;

    for (int i = 0; i < size(); i++) {
        Point2f p = get(i);
        result[i] = Point2f(((p.x - cx) * sx) + cx, ((p.y - cy) * sy) + cy);
    }

    return result;
}

float PlanarPolygon::area() const {

    if (!valid()) return 0;

    double area = 0;
    int i, j = size() - 1;

    for (i = 0; i < size(); i++) {
        Point2f a = get(i);
        Point2f b = get(j);
        area += (b.x + a.x) * (b.y - a.y); j = i;
    }

    return std::abs(area) * 0.5;
}

void PlanarPolygon::draw(cv::Mat& canvas, cv::Scalar color, int width) const {

    if (!valid()) return;

    std::vector<cv::Point> points(size());

    for (int i = 0; i < size(); i++) {
        points[i] = get(i);
    }

    const cv::Point* ppoints = &points[0];
    int npoints = (int) points.size();

    if (width < 1)
        cv::fillPoly(canvas, &ppoints, &npoints, 1, color);
    else
        cv::polylines(canvas, &ppoints, &npoints, 1, true, color, width);

}

vector<Point2f> PlanarPolygon::points() const {
    return vector<Point2f>(_points);
}


WindowEvents::WindowEvents(const string& name) : name(name) {

    setMouseCallback(name, WindowEvents::on_mouse, this);

}

WindowEvents::~WindowEvents() {

    setMouseCallback(name, NULL, NULL);

}

void WindowEvents::on_mouse(int event, int x, int y, int flags, void* data) {
    ((WindowEvents *) data)->handle_event(event, flags, Point(x, y));
}

class RegionCapture : WindowEvents {
public:
    RegionCapture(const string& name, CameraReference camera) : WindowEvents(name), camera(camera) {
        dragging = false;
        rotating = true;
    }

    virtual ~RegionCapture() {

    }

    virtual void handle_event(int event, int flags, Point point) {

        if ( event == EVENT_LBUTTONDOWN ) {
            dragging = true;
            start = point;
        } else if ( event == EVENT_LBUTTONUP ) {
            dragging = false;
            if (!rotating) {
                region = PlanarPolygon(start.x, start.y, point.x - start.x, point.y - start.y);
            }
        }

        if (dragging) {
            current = point;
            if (rotating) {
                Point2f point = point_sensor_to_sphere(point, camera->get_sensor_size(), camera->get_orientation(),
                                                       camera->get_focal_length(), camera->get_dpi());
                camera->set_orientation(sphere_to_rotation(point, Point3f(0, 1, 0)));
            }
        }

    }

    CameraReference camera;
    bool dragging;
    Point start;
    Point current;
    PlanarPolygon region;
    bool rotating;

};

SpherePolygon get_region_interactive(const string& window, Mat& cubemap, CameraReference camera) {

    Mat sensor;
    RegionCapture capture(window, camera);
    bool run = true;

    while (run) {

        project_image(cubemap, camera, sensor);

        string message;

        if (capture.dragging) {
            if (capture.rotating) {
                line(sensor, camera->get_sensor_size() / 2, capture.current, Scalar(0, 255, 0));
                circle(sensor, capture.current, 3, Scalar(0, 255, 0), 2);
            } else {
                rectangle(sensor, capture.start, capture.current, Scalar(0, 255, 0), 3);
            }
        } else {
            if (capture.rotating) {
                message = "Drag mouse to rotate, press R to enter region.";
            } else {
                message = "Drag mouse to enter region, press R to rotate.";
            }
        }

        if (capture.region.valid()) {
            capture.region.draw(sensor, Scalar(0, 255, 0), 1);
        }

        putText(sensor, message, Point(10, 10), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 0));

        imshow(window, sensor);

        int i = waitKey(25);

        switch ((char)i) {
        case 'r': {
            capture.rotating = !capture.rotating;
            capture.dragging = false;
            break;
        }
        case 'c': {
            capture.region = PlanarPolygon();
            break;
        }
        case ' ': {
            run = false;
            break;
        }
        }

    }

    return capture.region.unproject(camera);
}

bool focus_camera_to_region(CameraReference camera, const SpherePolygon polygon) {
    if (!polygon.valid()) return false;

    float f_best = camera->get_focal_length();

    Matx33f matrix = euler_to_rotation(camera->get_orientation()), inverse;
    transpose(matrix, inverse);

    for (int i = 0; i < polygon.size(); i++) {
        Point2f p = polygon.get(i);
        Point3f t = inverse * point_sphere_to_space(p);

        if (t.x != 0)
            f_best = min(f_best, abs((t.z / t.x) * (camera->get_sensor_size().width / 2) / camera->get_dpi()));
        if (t.y != 0)
            f_best = min(f_best, abs((t.z / t.y) * (camera->get_sensor_size().height / 2) / camera->get_dpi()));
    }


    camera->set_focal_length(f_best);

    return true;
}

bool center_camera_to_region(CameraReference camera, const SpherePolygon region, Point3f up) {
    if (!region.valid()) return false;

    CameraReference temp(new Camera(*camera));

    Point3f mean(0, 0, 0);
    int n = 0;
    for (int i = 0; i < region.size(); i++) {
        mean += point_sphere_to_space(region.get(i));
        n++;
    }

    if (mean.x == 0 && mean.y == 0 && mean.z == 0) return false;

    mean /= n;

    temp->set_orientation(look_at_rotation(mean, up));

    focus_camera_to_region(temp, region);

    PlanarPolygon rect = region.project(temp);

    if (!rect.valid()) return false;

    Point2f center = rect.center();
    Point2f p = point_sensor_to_sphere(center,
                                       temp->get_sensor_size(), temp->get_orientation(),
                                       temp->get_focal_length(), temp->get_dpi());
    camera->set_orientation(sphere_to_rotation(p, up));


    return true;

}

bool interpolate_size_between(std::vector<SpherePolygon> &trajectory, CameraReference camera, size_t start, size_t end) {

    if (end - start < 1)
        return false;

    if (start < 0 || start >= trajectory.size() || end >= trajectory.size())
        return false;

    if (!trajectory[start].valid() || !trajectory[end].valid())
        return false;

    CameraReference working(new Camera(*camera));
    center_camera_to_region(working, trajectory[start]);
    Size2f size_start = trajectory[start].project(working).bounds().size();

    center_camera_to_region(working, trajectory[end]);
    Size2f size_end = trajectory[end].project(working).bounds().size();


    for (size_t i = start + 1; i < end; i++) {

        center_camera_to_region(working, trajectory[i]);

        float width_new, height_new;

        PlanarPolygon original = trajectory[i].project(working);
        Rect2f bounds = original.bounds();

        float a = (i - start) / (float) (end - start);

        width_new = size_start.width + a * (size_end.width - size_start.width);
        height_new = size_start.height + a * (size_end.height - size_start.height);

        original = PlanarPolygon(bounds.x + bounds.width / 2 - width_new / 2, bounds.y + bounds.height / 2 - height_new / 2,
                     width_new, height_new);

        trajectory[i] = original.unproject(working);

    }

    return true;

}

bool smooth_size_between(std::vector<SpherePolygon> &trajectory, CameraReference camera, size_t start, size_t end, int window) {

    if (end - start < 1)
        return false;

    if (start < 0 || start >= trajectory.size() || end >= trajectory.size())
        return false;

    if (!trajectory[start].valid() || !trajectory[end].valid())
        return false;

    vector<CameraReference> cameras;
    vector<vector<Point2f> > frames;

    // Reconstruct trajectories for individual points of regions
    for (size_t i = start; i <= end; i++) {
        CameraReference working(new Camera(*camera));
        center_camera_to_region(working, trajectory[i]);
        cameras.push_back(working);
        PlanarPolygon projected = trajectory[i].project(working);
        vector<Point2f> points = projected.points();
        if (i == start) {
            frames.push_back(points);
        } else {
            if (frames.back().size() != points.size()) {
                frames.push_back(vector<Point2f>());
                continue;
            }

            vector<Point2f> matched;

            for (vector<Point2f>::iterator itp = frames.back().begin(); itp != frames.back().end(); itp++) {

                vector<Point2f>::iterator match = points.begin();
                double score = std::numeric_limits<double>::max();

                for (vector<Point2f>::iterator it = points.begin(); it != points.end(); it++) {
                    double res = cv::norm((*itp) - (*it));
                    if (res < score) {
                        match = it; score = res;
                    }

                }

                matched.push_back(*match);
                points.erase(match);

            }

            frames.push_back(matched);
        }
    }

    for (size_t i = 0; i < cameras.size(); i++) {

        if (frames[i].size() == 0) continue;

        vector<Point2f> points;
        float normalization = 0;

        for (size_t p = 0; p < frames[i].size(); p++) { points.push_back(Point2f()); }

        for (int o = -window / 2; o <= window / 2; o++) {

            if (i + o < 0 || i + o >= cameras.size()) continue;

            vector<Point2f> frame = frames[i + o];
            if (frame.size() == 0) continue;
            float weight = 1;

            for (size_t p = 0; p < frame.size(); p++) {
                points[p].x += weight * frame[p].x;
                points[p].y += weight * frame[p].y;
            }
            normalization += weight;
        }

        for (size_t p = 0; p < points.size(); p++) { points[p].x /= normalization; points[p].y /= normalization; }

        PlanarPolygon planar = PlanarPolygon(points);

        trajectory[i + start] = planar.unproject(cameras[i]);

    }

    return true;

}


float vector_magnitude(Point3f vec) {
    return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

float angle_between_vertices(Point3f a, Point3f b) {
    float angle = a.dot(b) / (vector_magnitude(a) * vector_magnitude(b));
    if (angle >= 1.0) {
        return 0.0;
    } else if (angle <= -1.0) {
        return (float) CV_PI;
    } else {
        return acos(angle);
    }
}

vector<Point> extract_coordinates(PlanarPolygon &region) {
    vector<Point> coords;
    Rect2f b = region.bounds();

    coords.push_back(Point(b.x, b.y));
    coords.push_back(Point(b.x + b.width, b.y + b.height));

    return coords;
}

Point interpolation_step(Point start, Point end, long step, long step_count) {
    Point2f interpolated_point;
    interpolated_point.x = start.x + step * ((end.x - start.x) / (float) step_count);
    // Y = ( ( X - X1 )( Y2 - Y1) / ( X2 - X1) ) + Y1
    interpolated_point.y = start.y + (end.y - start.y) * ( (interpolated_point.x - start.x) / (float) (end.x - start.x));
    return interpolated_point;
}



/**
 * Region parameters should be given in camera space
 */
vector<PlanarPolygon> interpolate(PlanarPolygon start_rectangle, PlanarPolygon end_rectangle, long step_count) {
    vector<PlanarPolygon> interpolated_regions;

    vector<Point> start_coords = extract_coordinates(start_rectangle);
    vector<Point> end_coords = extract_coordinates(end_rectangle);

    for (int i = 1; i <= step_count; i++) {
        Point2f step_upper_left = interpolation_step(start_coords[0], end_coords[0], i, step_count);
        Point2f step_lower_right = interpolation_step(start_coords[1], end_coords[1], i, step_count);

        int width = (int) round(abs(step_upper_left.x - step_lower_right.x));
        int height = (int) round(abs(step_upper_left.y - step_lower_right.y));

        PlanarPolygon new_region(step_upper_left.x, step_upper_left.y, width, height);

        interpolated_regions.push_back(new_region);
    }

    return interpolated_regions;
}


/**
 * fills the trajectory with rectangles between certain indices
 */
/*
void fill_between(std::vector<SpherePolygon> &trajectory, long id_start_rectangle, long id_end_rectangle, CameraReference original_camera) {
    // restraints:
    if (id_start_rectangle < 0 || id_end_rectangle < 0) {
        std::cerr << "ERROR: Invalid indices!";
        return;
    }

    if (id_start_rectangle == id_end_rectangle) {
        cout << "ERROR: Cannot interpolate on only one frame." << endl;
        return;
    }

    if (abs(id_start_rectangle - id_end_rectangle) > 10) {
        cout << "ERROR: Cannot interpolate more than " << MAX_FRAMES_TO_INTERPOLATE << " frames at once." << endl;
        return;
    }

    if (id_end_rectangle < id_start_rectangle) {
        // switch the frame numbers if the user entered them in the wrong order
        long tmp = id_end_rectangle;
        id_end_rectangle = id_start_rectangle;
        id_start_rectangle = tmp;
    }

    // logic:
    Rect2f tmp;
    // TODO: TEST to improve
    tmp = trajectory[id_start_rectangle].center
    trajectory[id_start_rectangle].get(&x, &y, &w, &h);
    Point3f first(x, y, 0);
    trajectory[id_end_rectangle].get(&x, &y, &w, &h);
    Point3f second(x, y, 0);

    float angle = angle_between_vertices(first, second);

    cout << "rotation first = " << first << endl;
    cout << "rotation second = " << second << endl;
    cout << "angle = " << angle * 180 / CV_PI << endl;

    // TODO: check if this is correct
    trajectory[id_start_rectangle].get(&x, &y, &w, &h);
    Point2f middle_point(x, y);
    trajectory[id_end_rectangle].get(&x, &y, &w, &h);
    middle_point.x = (middle_point.x + x) / 2.0f;
    middle_point.y = (middle_point.y + y) / 2.0f;

    CameraRotation rotation = sphere_to_rotation(middle_point, Point3f(0, 1, 0));
    CameraReference camera(new Camera());
    camera->set_orientation(rotation);
    PlanarPolygon start_camera_space = trajectory[id_start_rectangle].project(camera);
    PlanarPolygon end_camera_space = trajectory[id_end_rectangle].project(camera);
    vector<PlanarPolygon> newly_generated = interpolate(start_camera_space, end_camera_space, (id_end_rectangle - id_start_rectangle));
    for (long i = id_start_rectangle + 1, j = 0; i < id_end_rectangle; i++, j++) {
        trajectory.at(i) = newly_generated[j].unproject_region(camera); // TODO: check if the camera is working properly
    }
}
*/

/**
 * Interpolation of missing rectangle annotations
 */
void fill_missing_rectangles(std::vector<SpherePolygon> &trajectory) {
    SpherePolygon begin_rect, end_rect;
    bool stored = false;
    int count = 0, begin_frame = 0, end_frame;

    CameraReference camera(new Camera());
    camera->set_focal_length(0.2);
    for (uint i = 0; i < trajectory.size(); i++) {
        if (!trajectory[i].valid()) {
            count++;
            continue;
        }
        if (!stored) {
            begin_rect = trajectory[i];
            begin_frame = i;
            stored = true;
        } else { // TODO: set a limit for the interpolation
            end_rect = trajectory[i];
            end_frame = i;

            if (begin_frame >= (int) trajectory.size()) {
                return;
            }

//            begin_rect.get(&x, &y, &w, &h);
            Point2f middle = begin_rect.center();

            CameraRotation rotation = sphere_to_rotation(middle, Point3f(0, 1, 0));
            camera->set_orientation(rotation);
            PlanarPolygon begin_camera_space = begin_rect.project(camera);
            PlanarPolygon end_camera_space = end_rect.project(camera);
            vector<PlanarPolygon> interpolated_rects = interpolate(begin_camera_space, end_camera_space, count);

            /* code to view the interpolation results

            Mat visualize(500,500, CV_8UC3, Scalar(0,0,0));
            begin_camera_space.get(&x, &y, &w, &h);
            cout << "begin = " << x << ", " << y << ", " << w << ", " << h << endl;
            rectangle(visualize, Point(x, y), Point(x + w, y + h), Scalar(0, 255, 0));
            for(uint p = 0; p < interpolated_rects.size(); p++) {
                interpolated_rects[p].get(&x, &y, &w, &h);
                cout << "rect no " << p << " = " << x << ", " << y << ", " << w << ", " << h << endl;
                rectangle(visualize, Point(x, y), Point(x + w, y + h), Scalar(255, 0, 0));
            }
            end_camera_space.get(&x, &y, &w, &h);
            cout << "end   = " << x << ", " << y << ", " << w << ", " << h << endl;
            rectangle(visualize, Point(x, y), Point(x + w, y + h), Scalar(0, 255, 0));
            imshow("test_interpolation", visualize);
            waitKey();
             */

            for (int j = begin_frame + 1, k = 0; j < end_frame; j++, k++) {
                trajectory.at(j) = interpolated_rects[k].unproject(camera);
            }

            stored = false;
            count = 0;
            begin_rect = SpherePolygon();
            // return; // uncomment this to get the result only for the first 2 regions
        }
    }
}

size_t find_previous_frame(std::vector<SpherePolygon> &trajectory, size_t start_index, int type) {
    if (trajectory.empty()) {
        return 1;
    }

    if (start_index >= trajectory.size()) start_index = trajectory.size() - 1;

    if (start_index < 1) return trajectory.size();

    switch (type) {
    case FIND_EMPTY: {

        for (size_t i = start_index - 1; i >= 0; i--) {
            if (trajectory[i].valid()) {
                continue;
            }
            return i;
        }

        break;
    }
    case FIND_FULL: {

        for (size_t i = start_index - 1; i >= 0; i--) {
            if (trajectory[i].valid()) {
                continue;
            }
            return i;
        }

        break;
    }
    }

    return trajectory.size();
}

size_t find_next_frame(std::vector<SpherePolygon> &trajectory, size_t start_index, int type) {

    if (trajectory.empty()) {
        return 1;
    }

    if (start_index < 0) start_index = 0;

    if (start_index >= trajectory.size()) return trajectory.size();

    switch (type) {
    case FIND_EMPTY: {

        for (size_t i = start_index + 1; i < trajectory.size(); i++) {

            if (trajectory[i].valid()) {

                continue;
            }
            return i;
        }

        break;
    }
    case FIND_FULL: {

        for (size_t i = start_index + 1; i < trajectory.size(); i++) {
            if (!trajectory[i].valid()) {
                continue;
            }
            return i;
        }

        break;
    }


    }

    return trajectory.size();
}
