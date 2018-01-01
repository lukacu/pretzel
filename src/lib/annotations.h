//
// Created by pr3mar on 8/4/16.
//

#ifndef PRETZEL_ANNOTATE_UTILS_H
#define PRETZEL_ANNOTATE_UTILS_H

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <fstream>

#include "camera.h"
#include "projections.h"

using namespace std;
using namespace cv;

#ifdef PRETZEL_TRAX
#include <trax.h>
#endif

class WindowEvents {
public:
    WindowEvents(const string& name);

    virtual ~WindowEvents();

    virtual void handle_event(int event, int flags, Point point) = 0;

protected:

    string name;

    static void on_mouse(int event, int x, int y, int flags, void* data);

};

class PlanarPolygon;
class SpherePolygon;

class Polygon {
public:
    Polygon();
    Polygon(vector<Point2f> points);
    Polygon(int i);
    ~Polygon();

    bool valid() const;
    int size() const;
    Point2f get(int i) const;
    Point2f& operator[](int i);

    Point2f center() const;

protected:

    vector<Point2f> _points;

};

class SpherePolygon : public Polygon {
public:
    SpherePolygon();
    SpherePolygon(int i);
    SpherePolygon(vector<Point2f> points);
    ~SpherePolygon();

    static SpherePolygon code(int code);

    PlanarPolygon project(CameraReference camera) const;

};


class PlanarPolygon : public Polygon {
public:
    PlanarPolygon();
    PlanarPolygon(int x, int y, int w, int h);
    PlanarPolygon(Rect2f rect);
    PlanarPolygon(int i);
    PlanarPolygon(vector<Point2f> points);
    ~PlanarPolygon();

    float area() const;

    bool contains(float x, float y) const;

    Rect2f bounds() const;

    vector<Point2f> points() const;

    PlanarPolygon move(float x, float y) const;
    PlanarPolygon resize(float x, float y) const;

    SpherePolygon unproject(CameraReference camera) const;

    void draw(cv::Mat& canvas, cv::Scalar color, int width) const;

#ifdef PRETZEL_TRAX

    trax::Region region();
    PlanarPolygon(const trax::Region&);

#endif

};

#define FIND_EMPTY 0
#define FIND_FULL 1

template <typename T> int load_trajectory(const std::string& file, std::vector<T>& trajectory) {

    std::ifstream input;

    input.open(file.c_str(), std::ifstream::in);

    if (!input.is_open()) {
        std::cerr << "ERROR: Could not load trajectory " << file << endl;
        return 0;
    }
    int elements = 0;
    trajectory.clear();

    while (input) {
        string s;
        if (!getline( input, s )) break;

        istringstream ss( s );
        vector <string> record;

        vector<float> numbers;

        while (ss)
        {
          string s;
          if (!getline( ss, s, ',' )) break;
            numbers.push_back(std::stof( s ));
        }

        if (!input.good()) break;

        if ((int)numbers.size() == 1) {
            SpherePolygon p(1);
            p[0].x = numbers[0];
            trajectory.push_back(p);
        } else if ((int)numbers.size() < 6) {
            trajectory.push_back(SpherePolygon());
        } else {

            T region(numbers.size() / 2);

            for (size_t i = 0; i < numbers.size() / 2; i++)
                region[i] = Point2f(numbers[i*2], numbers[i*2+1]);

            trajectory.push_back(region);

        }

        elements++;
    }

    input.close();

    return elements;

}

template <typename T> bool save_trajectory(const std::string& file, std::vector<T>& trajectory) {

    std::ofstream output;

    output.open(file.c_str(), std::ofstream::out);

    if (output.is_open()) {
        for (typename std::vector<T>::iterator it = trajectory.begin(); it != trajectory.end(); it++) {
            if (it->valid()) {
                for (int i = 0; i < it->size(); i++) {
                    if (i != 0) output << ",";
                    output << (*it)[i].x << "," << (*it)[i].y;
                }
            } else {
                if (it->size() == 1)
                    output << it->get(0).x;
            }
            output << endl;
        }
        output.close();
        return true;
    } else {
        std::cerr << "ERROR: Could not open output file: "  << file << endl;
        return false;
    }

}

SpherePolygon get_region_interactive(const string& window, Mat& cubemap, CameraReference camera);

bool center_camera_to_region(CameraReference camera, const SpherePolygon region, Point3f up = Point3f(0, 1, 0));

//void draw_spherical_region(cv::Mat& canvas, const SpherePolygon& region, CameraReference camera, cv::Scalar color, int width);

bool interpolate_size_between(std::vector<SpherePolygon> &trajectory, CameraReference camera, size_t start, size_t end);

bool smooth_size_between(std::vector<SpherePolygon> &trajectory, CameraReference camera, size_t start, size_t end, int window);

size_t find_previous_frame(std::vector<SpherePolygon> &trajectory, size_t start_index, int type = FIND_FULL);

size_t find_next_frame(std::vector<SpherePolygon> &trajectory, size_t start_index, int type = FIND_FULL);

#endif //PRETZEL_ANNOTATE_UTILS_H
