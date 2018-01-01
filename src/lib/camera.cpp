
#include <iostream>
#include <fstream>
#include "camera.h"

Camera::Camera() : Camera(Size(640, 480), 1000) {

}


Camera::Camera(const Camera& camera) {

    exposure = camera.exposure;
    dpi = camera.dpi;
    sensor_size = camera.sensor_size;

    orientation = camera.orientation;
    focal_length = camera.focal_length;

}


Camera::Camera(Size sensor, int dpi) : sensor_size(sensor), dpi(dpi) {

    exposure = 1000;

    orientation.x = 0;
    orientation.y = 0;
    orientation.z = 0;
    focal_length = 0.1; // 1.0

}

Camera::Camera(const FileNode& config) {

    exposure = (int) config["intrinsic"]["exposure"];
    dpi = (int) config["intrinsic"]["dpi"];
    sensor_size = Size( (int) config["intrinsic"]["width"], (int) config["intrinsic"]["height"]);

    orientation.x = (float) config["extrinsic"]["x"];
    orientation.y = (float) config["extrinsic"]["y"];
    orientation.z = (float) config["extrinsic"]["z"];
    focal_length = (float) config["extrinsic"]["focal"];

}

Camera::~Camera() {

}

CameraRotation Camera::get_orientation() const {
    return orientation;
}

void Camera::set_orientation(CameraRotation o) {
    orientation = o;
}

float Camera::get_focal_length() const {
    return focal_length;
}

void Camera::set_focal_length(float f) {
    focal_length = f;
}

int Camera::get_dpi() const {
    return dpi;
}

int Camera::get_exposure() const {
    return exposure;
}

Size Camera::get_sensor_size() const {
    return sensor_size;
}

void Camera::print() const {
    std::cout << "Camera: " << orientation << " f=" << focal_length << std::endl;
}

CameraView Camera::get_view() const {

    CameraView view;
    view.orientation = get_orientation();
    view.focal_length = get_focal_length();

    return view;

}

void Camera::set_view(CameraView view) {

    set_orientation(view.orientation);
    set_focal_length(view.focal_length);

}


void project_image(const Mat& cubemap, Ptr<Camera> camera, Mat& sensor, bool accurate) {

    sensor.create(camera->get_sensor_size(), cubemap.type());

    cubemap_project(cubemap, sensor, camera->get_orientation(), camera->get_focal_length(), camera->get_dpi(), accurate);

}

int load_camera_views(const std::string& file, std::vector<CameraView>& views) {

	std::ifstream input;

	input.open(file.c_str(), std::ifstream::in);

	if (!input.is_open())
		return 0;

	int elements = 0;

	CameraView view;

	std::string         line;

	while (std::getline(input, line)) {

		std::stringstream   lineStream(line);
		std::string         cell;

		vector<float> numbers;
		while (std::getline(lineStream, cell, ','))
			numbers.push_back(atof(cell.c_str()));

		if (numbers.size() < 4) continue;

		view.orientation.x = numbers[0];
		view.orientation.y = numbers[1];
		view.orientation.z = numbers[2];
		view.focal_length = numbers[3];

		views.push_back(view);

		elements++;

	}

	input.close();

	return elements;

}
