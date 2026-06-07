// src/fanblade.cpp
#include "../include/fanblade.hpp"
#include <cmath>

namespace auto_buff {

Fanblade::Fanblade(const cv::RotatedRect& rect) 
    : cv::RotatedRect(rect), rotate_angle_(0.0) {
    // 计算扇叶中心
    center_.x = (rect.center.x); 
    center_.y = (rect.center.y);
}

void Fanblade::calculateRotateAngle(const cv::Point2f& buff_center) {
    double dx = std::abs(buff_center.x - center_.x);
    double dy = std::abs(buff_center.y - center_.y);
    rotate_angle_ = cv::fastAtan2(dy, dx);
}

Fanblade Fanblade::predict(double dt) const {
    // 预留预测逻辑
    return *this;
}

} // namespace auto_buff