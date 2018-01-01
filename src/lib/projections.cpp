
#include <iostream>
#include <cmath>
#include "projections.h"

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Based on answer from here: https://stackoverflow.com/questions/29678510/convert-21-equirectangular-panorama-to-cube-map

// Define our six cube faces.
// 0 - 3 are side faces, clockwise order
// 4 and 5 are top and bottom, respectively
float face_angles[6][2] =
{
    {0, 0},
    {M_PI / 2, 0},
    {M_PI, 0},
    { -M_PI / 2, 0},
    {0, -M_PI / 2},
    {0, M_PI / 2}
};

int face_bounds[6][4] =
{
    {0, 1, 0, 1},
    {0, 1, 1, 2},
    {0, 1, 2, 3},
    {1, 2, 0, 1},
    {1, 2, 1, 2},
    {1, 2, 2, 3}
};

float cube_coordinates[8][3] =
{
    // X, Y, Z
    { -1, -1, -1}, // 0
    {1, -1, -1}, // 1
    { -1, 1, -1}, // 2
    {1, 1, -1}, // 3
    { -1, -1, 1}, // 4
    {1, -1, 1}, // 5
    { -1, 1, 1}, // 6
    {1, 1, 1} // 7
};

int face_coordinates[6][4] =
{
    {7, 6, 4, 5},
    {6, 2, 0, 4},
    {2, 3, 1, 0},
    {3, 7, 5, 1},
    {2, 6, 7, 3},
    {1, 5, 4, 0}
};

#define VISIBLE_THRESHOLD(F) (std::min(((F) / 3), 0.5f))

#define POINT_VISIBLE(P, F) ((P).z > VISIBLE_THRESHOLD(F))

#define NORMAL(V) (sqrt((V).x * (V).x + (V).y * (V).y + (V).z * (V).z))

float truncate_angle(float angle) {
    if (angle < 0) {
        float revolutions = ceil(-angle / (2 * M_PI));
        return angle + revolutions * (2 * M_PI);
    } else {
        float revolutions = floor(angle / (2 * M_PI));
        return angle - revolutions * (2 * M_PI);
    }

}

Matx33f euler_to_rotation(Point3f angles) {

    Matx33f RX (1,          0,           0,
                0, cos(angles.x), -sin(angles.x),
                0, sin(angles.x),  cos(angles.x));

    Matx33f RY(cos(angles.y), 0, -sin(angles.y),
               0,         1,          0,
               sin(angles.y), 0,  cos(angles.y));

    Matx33f RZ(cos(angles.z), -sin(angles.z), 0,
               sin(angles.z),  cos(angles.z), 0,
               0,          0,           1);

    return RZ * RY * RX;

}

Point3f rotation_to_euler(Matx33f rotation) {

    float sy = std::sqrt(rotation(0, 0) * rotation(0, 0) + rotation(1, 0) * rotation(1, 0));
    Point3f angles;

    if (sy < 10 * std::numeric_limits<float>::epsilon()) { // Singular matrices need special treatment

        angles.x = std::atan2(-rotation(1, 2), rotation(1, 1));
        angles.y = std::atan2(-rotation(2, 0), sy);
        angles.z = 0;

    } else {

        angles.x = std::atan2(rotation(2, 1), rotation(2, 2));
        angles.y = std::atan2(rotation(2, 0), sy);
        angles.z = std::atan2(rotation(1, 0), rotation(0, 0));

    }

    return angles;

}


// Map a part of the equirectangular panorama (in) to a cube face
// (face). The ID of the face is given by faceId.
void cubemap_precompute_face(Size size, Mat &mapx, Mat& mapy, int face) {

    float inWidth = size.width;
    float inHeight = size.height;

    int width = mapx.cols;
    int height = mapx.rows;

    // Calculate adjacent (ak) and opposite (an) of the
    // triangle that is spanned from the sphere center
    //to our cube face.
    const float an = sin(M_PI / 4);
    const float ak = cos(M_PI / 4);

    const float ftu = face_angles[face][0];
    const float ftv = face_angles[face][1];

    // For each point in the target image,
    // calculate the corresponding source coordinates.
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {

            // Map face pixel coordinates to [-1, 1] on plane
            float nx = (float)y / (float)height - 0.5f;
            float ny = (float)x / (float)width - 0.5f;

            nx *= 2;
            ny *= 2;

            // Map [-1, 1] plane coords to [-an, an]
            // thats the coordinates in respect to a unit sphere
            // that contains our box.
            nx *= an;
            ny *= an;

            float u, v;

            // Project from plane to sphere surface.
            if (ftv == 0) {
                // Center faces
                u = atan2(nx, ak);
                v = atan2(ny * cos(u), ak);
                u += ftu;
            } else if (ftv > 0) {
                // Bottom face
                float d = sqrt(nx * nx + ny * ny);
                v = M_PI / 2 - atan2(d, ak);
                u = atan2(ny, nx);
            } else {
                // Top face
                float d = sqrt(nx * nx + ny * ny);
                v = -M_PI / 2 + atan2(d, ak);
                u = atan2(-ny, nx);
            }

            // Map from angular coordinates to [-1, 1], respectively.
            u = u / (M_PI);
            v = v / (M_PI / 2);

            // Warp around, if our coordinates are out of bounds.
            while (v < -1) {
                v += 2;
                u += 1;
            }
            while (v > 1) {
                v -= 2;
                u += 1;
            }

            while (u < -1) {
                u += 2;
            }
            while (u > 1) {
                u -= 2;
            }

            // Map from [-1, 1] to in texture space
            u = u / 2.0f + 0.5f;
            v = v / 2.0f + 0.5f;

            u = u * (inWidth - 1);
            v = v * (inHeight - 1);

            // Save the result for this pixel in map
            mapx.at<float>(x, y) = u;
            mapy.at<float>(x, y) = v;
        }
    }

}

void cubemap_precompute(Mat& mapx, Mat& mapy, const Size size, int cubesize) {

    if (cubesize < 1)
        cubesize = size.width / 4;

    mapx.create(cubesize * 2, cubesize * 3, CV_32F);
    mapy.create(cubesize * 2, cubesize * 3, CV_32F);

    Mat facex, facey;

    for (int i = 0; i < 6; i++) {
        facex = mapx(Range(face_bounds[i][0] * cubesize, face_bounds[i][1] * cubesize), Range(face_bounds[i][2] * cubesize, face_bounds[i][3] * cubesize));
        facey = mapy(Range(face_bounds[i][0] * cubesize, face_bounds[i][1] * cubesize), Range(face_bounds[i][2] * cubesize, face_bounds[i][3] * cubesize));
        cubemap_precompute_face(size, facex, facey, i);
    }

}

void cubemap_map(const Mat& equirectangular, Mat& cubemap, Mat mapx, Mat mapy) {

    int cubesize = (equirectangular.cols / 4);

    if (!cubemap.empty()) {
        cubesize = cubemap.cols / 3;
    } else {
        cubemap.create(Size(cubesize * 3, cubesize * 2), equirectangular.type());
    }

    if (mapx.empty() || mapy.empty()) {

        cubemap_precompute(mapx, mapy, equirectangular.size(), cubesize);
    }

    Mat facex, facey, face;

    for (int i = 0; i < 6; i++) {
        face = cubemap(Range(face_bounds[i][0] * cubesize, face_bounds[i][1] * cubesize), Range(face_bounds[i][2] * cubesize, face_bounds[i][3] * cubesize));
        facex = mapx(Range(face_bounds[i][0] * cubesize, face_bounds[i][1] * cubesize), Range(face_bounds[i][2] * cubesize, face_bounds[i][3] * cubesize));
        facey = mapy(Range(face_bounds[i][0] * cubesize, face_bounds[i][1] * cubesize), Range(face_bounds[i][2] * cubesize, face_bounds[i][3] * cubesize));
        remap(equirectangular, face, facex, facey, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
    }

}

void cubemap_dummy(Mat& cubemap, int cubesize) {

    cubemap.create(cubesize * 2, cubesize * 3, CV_8UC3);

    Mat face;

    for (int i = 0; i < 6; i++) {

        face = cubemap(Range(face_bounds[i][0] * cubesize, face_bounds[i][1] * cubesize), Range(face_bounds[i][2] * cubesize, face_bounds[i][3] * cubesize));

        Scalar color(255 * (((i + 1) >> 2) % 2), 255 * (((i + 1) >> 1) % 2), 255 * ((i + 1) % 2));

        face.setTo(color);
        putText(face, format("%d", i), Point(cubesize / 2, cubesize / 2), FONT_HERSHEY_SIMPLEX, 3, Scalar(0, 0, 0), 3);

        circle(face, Point(cubesize / 2, cubesize / 2), 10, Scalar(0, 0, 0), 3);

    }

}

#include <opencv2/highgui.hpp>

void cubemap_face_project(const Mat& cubemap, Mat& sensor, int i, Matx33f rotation, float focal_length, int dpi, bool wireframe, Mat outmask) {

    int cx = sensor.cols / 2;
    int cy = sensor.rows / 2;
    int cubesize = cubemap.cols / 3;

    CV_Assert((cubemap.cols == cubesize * 3) && (cubemap.rows == cubesize * 2) && focal_length > 0 && dpi > 0);

    Mat face = cubemap(Range(face_bounds[i][0] * cubesize, face_bounds[i][1] * cubesize), Range(face_bounds[i][2] * cubesize, face_bounds[i][3] * cubesize));

    float behind_threshold = VISIBLE_THRESHOLD(focal_length);

    int behind = 0;
    vector<Point3f> facepoints(4);
    vector<Point2f> src(4);
    vector<Point2f> dst(4);
    src[0] = Point2f(0, 0);
    src[1] = Point2f(cubesize - 1, 0);
    src[2] = Point2f(cubesize - 1, cubesize - 1);
    src[3] = Point2f(0, cubesize - 1);

    for (size_t j = 0; j < 4; j++) {
        facepoints[j] = rotation * Point3f(cube_coordinates[face_coordinates[i][j]][0], cube_coordinates[face_coordinates[i][j]][1], cube_coordinates[face_coordinates[i][j]][2]);
        if (facepoints[j].z <= behind_threshold) behind++;
    }

    // 4 points behind: do not draw face
    if (behind == 4) return;

    for (size_t j = 0; j < 4; j++) {
        dst[j] = Point2f((-focal_length * (facepoints[j].x / facepoints[j].z) * dpi + cx), (-focal_length * (facepoints[j].y / facepoints[j].z) * dpi + cy));
    }

    Mat transform = getPerspectiveTransform(src, dst);

    Mat mask = Mat::zeros(sensor.size(), CV_8U);
    Mat temp = Mat::zeros(sensor.size(), sensor.type());
    mask.setTo(0);
    vector<Point> mask_points;

    for (size_t j = 0; j < 4; j++) {

        if (facepoints[j].z <= behind_threshold) {
            bool n1 = facepoints[(j - 1) % 4].z > behind_threshold;
            bool n2 = facepoints[(j + 1) % 4].z > behind_threshold;

            if (n1 && n2) {
                Point3f o1 = facepoints[(j - 1) % 4];
                Point3f o2 = facepoints[(j + 1) % 4];
                Point3f v1 = facepoints[j] - o1;
                Point3f v2 = facepoints[j] - o2;
                v1 = v1 * ((o1.z - behind_threshold) / -v1.z) + facepoints[(j - 1) % 4];
                v2 = v2 * ((o2.z - behind_threshold) / -v2.z) + facepoints[(j + 1) % 4];

                mask_points.push_back(Point2f((-focal_length * (v1.x / v1.z) * dpi + cx), (-focal_length * (v1.y / v1.z) * dpi + cy)));
                mask_points.push_back(Point2f((-focal_length * (v2.x / v2.z) * dpi + cx), (-focal_length * (v2.y / v2.z) * dpi + cy)));
            } else if (n1 || n2) {
                Point3f o = n2 ? facepoints[(j + 1) % 4] : facepoints[(j - 1) % 4];
                Point3f v = facepoints[j] - o;
                v = v * ((o.z - behind_threshold) / -v.z) + o;
                mask_points.push_back(Point2f((-focal_length * (v.x / v.z) * dpi + cx), (-focal_length * (v.y / v.z) * dpi + cy)));

            }

        } else mask_points.push_back(dst[j]);
    }

    int n = (int) mask_points.size();
    const Point* p = &mask_points[0];
    fillPoly(mask, &(p), &n, 1, Scalar(1));
    warpPerspective(face, temp, transform, temp.size(), INTER_LINEAR, BORDER_REPLICATE);

    uchar* mptr = mask.ptr();
    uchar* sptr = temp.ptr();
    uchar* dptr = sensor.ptr();

    if (!outmask.empty()) {
        uchar* optr = outmask.ptr();

        for (int i = 0; i < mask.cols * mask.rows; i++) {
            if (mptr[i] && (sptr[0] + sptr[1] + sptr[2]) > (dptr[0] + dptr[1] + dptr[2])) {
                dptr[0] = sptr[0]; dptr[1] = sptr[1]; dptr[2] = sptr[2];
                optr[0] = 255;
            }
            sptr += 3; dptr += 3; optr++;
        }
    } else {

        for (int i = 0; i < mask.cols * mask.rows; i++) {
            if (mptr[i] && (sptr[0] + sptr[1] + sptr[2]) > (dptr[0] + dptr[1] + dptr[2])) {
                dptr[0] = sptr[0]; dptr[1] = sptr[1]; dptr[2] = sptr[2];
            }
            sptr += 3; dptr += 3;
        }
    }


    //temp.copyTo(sensor, mask);
    if (wireframe) {
        for (int j = 0; j < (int) mask_points.size(); j++) {
            line(sensor, mask_points[j], mask_points[(j + 1) % (int) mask_points.size()], Scalar(100, 100, 100), 1);
        }
    }

}


void cubemap_project(const Mat& cubemap, Mat& sensor, const CameraRotation rotation, float focal_length, int dpi, bool accurate) {

    int cubesize = cubemap.cols / 3;

    CV_Assert((cubemap.cols == cubesize * 3) && (cubemap.rows == cubesize * 2));

    sensor.setTo(0);
    Matx33f matrix = euler_to_rotation(rotation);

    Mat mask = accurate ? Mat::zeros(sensor.size(), CV_8U) : Mat();

    for (size_t i = 0; i < 6; i++) {
        cubemap_face_project(cubemap, sensor, i, matrix, focal_length, dpi, false, mask);
    }

    if (accurate) {

    }

    //if (accurate) imshow("Mask", mask);

}

#define CIRCLE_SEGMENTS 24
#define PHI_SEGMENTS 10
#define THETA_SEGMENTS 20

#define MAJOR_LINE_COLOR Scalar(0, 255, 0)
#define MINOR_LINE_COLOR Scalar(100, 100, 100)

void draw_sphere(Mat& sensor, const CameraRotation rotation, float focal_length, int dpi) {

    CV_Assert(focal_length > 0 && dpi > 0);

    Size sensor_size(sensor.cols, sensor.rows);

    Matx33f matrix = euler_to_rotation(rotation);

    for (int i = 0; i < THETA_SEGMENTS; i++) {

        float phi;
        float theta = ((float)i * M_PI) / THETA_SEGMENTS;

        for (int j = 0; j < CIRCLE_SEGMENTS; j++) {

            phi = ((float)j * 2 * M_PI) / CIRCLE_SEGMENTS;
            Point3f p1 = matrix * Point3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
            phi = ((float)(j + 1) * 2 * M_PI) / CIRCLE_SEGMENTS;
            Point3f p2 = matrix * Point3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));

            if (!POINT_VISIBLE(p1, focal_length) || !POINT_VISIBLE(p2, focal_length))
                continue;

            Point2f t1 = Point2f(  (-focal_length * (p1.x / p1.z) * (float)dpi + sensor_size.width / 2),
                                   (-focal_length * (p1.y / p1.z) * (float)dpi + sensor_size.height / 2));

            Point2f t2 = Point2f(  (-focal_length * (p2.x / p2.z) * (float)dpi + sensor_size.width / 2),
                                   (-focal_length * (p2.y / p2.z) * (float)dpi + sensor_size.height / 2));

            line(sensor, t1, t2, (i == CIRCLE_SEGMENTS / 2) ? MAJOR_LINE_COLOR : MINOR_LINE_COLOR, 1);

        }

    }

    for (int i = 0; i < PHI_SEGMENTS; i++) {

        float phi = ((float)i * M_PI) / PHI_SEGMENTS;
        float theta;

        for (int j = 0; j < CIRCLE_SEGMENTS; j++) {

            theta = ((float)j * 2 * M_PI) / CIRCLE_SEGMENTS;
            Point3f p1 = matrix * Point3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
            theta = ((float)(j + 1) * 2 * M_PI) / CIRCLE_SEGMENTS;
            Point3f p2 = matrix * Point3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));

            if (!POINT_VISIBLE(p1, focal_length) || !POINT_VISIBLE(p2, focal_length))
                continue;

            Point2f t1 = Point2f(  (-focal_length * (p1.x / p1.z) * (float)dpi + sensor_size.width / 2),
                                   (-focal_length * (p1.y / p1.z) * (float)dpi + sensor_size.height / 2));

            Point2f t2 = Point2f(  (-focal_length * (p2.x / p2.z) * (float)dpi + sensor_size.width / 2),
                                   (-focal_length * (p2.y / p2.z) * (float)dpi + sensor_size.height / 2));

            line(sensor, t1, t2, (i == 0 || i == CIRCLE_SEGMENTS / 2) ? MAJOR_LINE_COLOR : MINOR_LINE_COLOR, 1);

        }


    }

}

Point3f point_sphere_to_space(Point2f src) {

    float theta = truncate_angle(src.y);
    float phi = truncate_angle(src.x);

    if (theta > M_PI) {
        theta = 2 * M_PI - theta;
        phi = truncate_angle(phi + M_PI);
    }

    return Point3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));

}

Point2f point_space_to_sphere(Point3f src) {

    float phi = atan2(src.y, src.x);
    float theta = acos(src.z / sqrt(src.x * src.x + src.y * src.y + src.z * src.z));

    return Point2f(phi, theta);

}


Point2d point_sphere_to_sensor(Point2f src, Size sensor_size,
                               CameraRotation rotation, float focal_length, int dpi) {

    CV_Assert(focal_length > 0 && dpi > 0);

    Point3f space = point_sphere_to_space(src);

    Matx33f matrix = euler_to_rotation(rotation);

    Point3f t = matrix * space;

    Point2f result;

    if (!POINT_VISIBLE(t, focal_length)) {

        result = Point2f(NAN, NAN);

    } else {

        result = Point2f(  (-focal_length * (t.x / t.z) * (float)dpi + sensor_size.width / 2),
                           (-focal_length * (t.y / t.z) * (float)dpi + sensor_size.height / 2));
    }

    return result;

}

Point2f point_sensor_to_sphere(Point2d src, Size sensor_size,
                               CameraRotation rotation, float focal_length, int dpi) {

    CV_Assert(focal_length > 0 && dpi > 0);

    float dx = -(float) (src.x - sensor_size.width / 2) / ((float)dpi * focal_length);
    float dy = -(float) (src.y - sensor_size.height / 2) / ((float)dpi * focal_length);

    // Compute relative spherical coordinates
    float phi = atan2(dy, dx);
    float theta = acos(1.0 / sqrt(1 + dx * dx + dy * dy));

    // Conversion to cartesian and rotation
    Matx33f matrix = euler_to_rotation(rotation), inverse;
    transpose(matrix, inverse);
    Point3f t = inverse * Point3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));

    // Recompute absolute spherical coordinates
    return point_space_to_sphere(t);

}

Point3f look_at_rotation(Point3f at, Point3f up) {

    Point3f z(at), x, y;
    z /= NORMAL(z);
    x = up.cross(z);
    x /= NORMAL(x);
    y = z.cross(x);

    Matx33f rotation;
    rotation(0, 0) = x.x; rotation(0, 1) = y.x; rotation(0, 2) = z.x;
    rotation(1, 0) = x.y; rotation(1, 1) = y.y; rotation(1, 2) = z.y;
    rotation(2, 0) = x.z; rotation(2, 1) = y.z; rotation(2, 2) = z.z;

    return rotation_to_euler(rotation.t());

}

Point3f sphere_to_rotation(Point2f src, Point3f up) {

    float theta = truncate_angle(src.y);
    float phi = truncate_angle(src.x);

    if (theta > M_PI) {
        theta = 2 * M_PI - theta;
        phi = truncate_angle(phi + M_PI);
    }

    Point3f at = Point3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));

    return look_at_rotation(at, up);

}
