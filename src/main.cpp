#include "io/camera.hpp"
#include "tasks/light_detect/Detect.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/yaml.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846，
#endif

int main() {
    std::string config_path = "configs/test.yaml";
    
    io::Camera camera(config_path);
    LightDetect light_detect(config_path);
    
    io::Gimbal gimbal(config_path);
    
    cv::Mat img;
    bool is_startup = true; 

    auto yaml = tools::load(config_path);
    auto camera_matrix = yaml["camera_matrix"].as<std::vector<double>>();
    const double fx = camera_matrix[0];
    const double cx = camera_matrix[2];

    // green threshold in HSV (configurable)
    std::vector<int> green_lower, green_upper;
    if (yaml["green_lower"]) green_lower = yaml["green_lower"].as<std::vector<int>>();
    else green_lower = {35,100,100};
    if (yaml["green_upper"]) green_upper = yaml["green_upper"].as<std::vector<int>>();
    else green_upper = {85,255,255};
    cv::Scalar hsv_lower(green_lower[0], green_lower[1], green_lower[2]);
    cv::Scalar hsv_upper(green_upper[0], green_upper[1], green_upper[2]);

    cv::namedWindow("Camera Image", cv::WINDOW_AUTOSIZE);

    while (true) {
        std::chrono::steady_clock::time_point timestamp;
        camera.read(img, timestamp);
        
        if (img.empty()) {
            std::cout << "Empty frame!" << std::endl;
            continue;
        }

        cv::Mat display_img;
        cv::resize(img, display_img, cv::Size2d(640, 480));

        auto lights = light_detect.detect(img, cv::Size2d(640, 480), 0, is_startup);
        is_startup = false; 

        if (!lights.empty()) {
            auto best_it = std::max_element(lights.begin(), lights.end(),
                                           [](const auto &a, const auto &b) { return a.score < b.score; });
            const auto &best = *best_it;

            // draw only the best detection
            cv::rectangle(display_img, best.box, cv::Scalar(0, 0, 255), 2);
            cv::circle(display_img, best.center_point, 5, cv::Scalar(0, 0, 255), -1);

            // traditional vision post‑processing on the best bbox
            cv::Rect2d roi_d = best.box & cv::Rect2d(0, 0, img.cols, img.rows);
            cv::Rect roi(static_cast<int>(roi_d.x), static_cast<int>(roi_d.y),
                         static_cast<int>(roi_d.width), static_cast<int>(roi_d.height));
            cv::Point2f selected_center = best.center_point;
            bool refined_found = false;
            if (roi.area() > 0) {
                cv::Mat patch = img(roi).clone();
                cv::Mat hsv, mask;
                cv::cvtColor(patch, hsv, cv::COLOR_BGR2HSV);
                cv::inRange(hsv, hsv_lower, hsv_upper, mask);
                cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
                cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);

                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                if (!contours.empty()) {
                    auto max_it = std::max_element(contours.begin(), contours.end(),
                        [](const auto &a, const auto &b){ return cv::contourArea(a) < cv::contourArea(b); });
                    cv::Moments m = cv::moments(*max_it);
                    if (m.m00 > 1e-5) {
                        cv::Point2f c(m.m10/m.m00, m.m01/m.m00);
                        selected_center = c + cv::Point2f(roi.x, roi.y);
                        refined_found = true;
                        cv::circle(display_img, selected_center, 5, cv::Scalar(255,255,0), -1);
                    }
                }
            }

            float pixel_offset = selected_center.x - (640.0f / 2.0f)+75.0;
            if (std::fabs(pixel_offset) < 0.001f) {
                pixel_offset = 0.0f;
            }
            io::VisionToGimbal send_data;
            send_data.yaw_offset = pixel_offset;
            if (refined_found) {
                printf("Best pixel offset (score %.3f, refined detection used): %.2f\n", best.score, pixel_offset);
            } else {
                printf("Best pixel offset (score %.3f, fallback to openvino center): %.2f\n", best.score, pixel_offset);
            }
            gimbal.send(send_data);
        }

        cv::imshow("Camera Image", display_img);
        if (cv::waitKey(1) == 'q') break;
    }

    cv::destroyAllWindows();
    return 0;
}