
#ifndef __PROJECTIONS_H
#define __PROJECTIONS_H

#include <opencv2/imgproc.hpp>

using namespace cv;

typedef Point3f CameraRotation;

Matx33f euler_to_rotation(Point3f angles);

Point3f rotation_to_euler(Matx33f rotation);

void cubemap_precompute(Mat& mapx, Mat& mapy,const Size size, int cubesize = -1);

void cubemap_map(const Mat& equirectangular, Mat& cubemap, Mat mapx = Mat(), Mat mapy = Mat());

void cubemap_dummy(Mat& cubemap, int cubesize);

void cubemap_face_project(const Mat& cubemap, Mat& sensor, int i, Matx33f rotation, float focal_length, int dpi, bool wireframe = false, Mat mask = Mat());

void cubemap_project(const Mat& cubemap, Mat& sensor, const CameraRotation rotation, float focal_length, int dpi, bool accurate = false);

void draw_sphere(Mat& sensor, const CameraRotation rotation, float focal_length, int dpi);

Point3f point_sphere_to_space(Point2f src);

Point2f point_space_to_sphere(Point3f src);

Point2d point_sphere_to_sensor(Point2f src, Size sensor, CameraRotation rotation, float focal_length, int dpi);

Point2f point_sensor_to_sphere(Point2d src, Size sensor_size, CameraRotation rotation, float focal_length, int dpi);

float truncate_angle(float angle);

Point3f look_at_rotation(Point3f at, Point3f up = Point3f(0, 1, 0));

Point3f sphere_to_rotation(Point2f src, Point3f up = Point3f(0, 1, 0));

#endif
