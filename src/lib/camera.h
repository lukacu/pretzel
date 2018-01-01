
#ifndef PRETZEL_CAMERA_H
#define PRETZEL_CAMERA_H


#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

#include "projections.h"

using namespace cv;
using namespace std;

typedef struct CameraView {
	CameraRotation orientation;
	float focal_length;
} CameraView;


class Camera {
public:
	Camera();

	Camera(const Camera& camera);

	Camera(const FileNode& config);

	Camera(Size sensor, int dpi = 1000);

	virtual ~Camera();

	CameraRotation get_orientation() const;

    void set_orientation(CameraRotation o);

	float get_focal_length() const;

    void set_focal_length(float f);

	int get_dpi() const;

	int get_exposure() const;

	Size get_sensor_size() const;

	void print() const;

	CameraView get_view() const;

	void set_view(CameraView view);

private:

    CameraRotation orientation;

	float focal_length;

	Size sensor_size;

	int dpi;

	int exposure;

};

typedef Ptr<Camera> CameraReference;

void project_image(const Mat& cubemap, Ptr<Camera> camera, Mat& sensor, bool accurate = false);

int load_camera_views(const std::string& file, std::vector<CameraView>& views);

#endif
