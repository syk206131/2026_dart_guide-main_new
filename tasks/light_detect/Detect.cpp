#include "Detect.hpp"

LightDetect::LightDetect(const std::string &config_path) {
    this->openvino_infer_ = std::make_unique<OpenvinoInfer>(config_path);
}

std::vector<OpenvinoInfer::Light> LightDetect::detect(cv::Mat &src, const cv::Size2d &dst_size, const int &my_color, const bool &startup) {
    return this->openvino_infer_->infer(src, dst_size, my_color, startup);
}


