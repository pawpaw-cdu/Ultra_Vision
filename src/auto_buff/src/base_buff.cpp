#include "../include/base_buff.hpp"
#include "../include/utils.hpp"

namespace auto_buff {

// 计算扇叶几何中心
cv::Point2f BaseBuff::calcFanbladesCenter() const {
    if (fanblades_.empty()) {
        return cv::Point2f(100.0f, 100.0f);
    }
    float sum_x = 0.0f, sum_y = 0.0f;
    int count = static_cast<int>(fanblades_.size());
    for (const auto& f : fanblades_) {
        sum_x += f.getCenter().x;
        sum_y += f.getCenter().y;
    }
    return cv::Point2f(sum_x / count, sum_y / count);
}

// 计算半径r：中心到扇叶的距离
void BaseBuff::calcHitRadius() {
    if (fanblades_.empty() || center_.x <= 0) return;
    // 取第一个扇叶计算距离
    const auto& fan = fanblades_[0];
    float dx = fan.getCenter().x - center_.x;
    float dy = fan.getCenter().y - center_.y;
    hit_radius_ = std::sqrt(dx*dx + dy*dy);
    //std::cout << "[半径] 中心→扇叶距离 r = " << hit_radius_ << " px" << std::endl;
}

// 在圆外找击打区域
void BaseBuff::findHitAreaOutsideCircle() {
    if (hit_radius_ <= 0 || center_.x <= 0) {
        hit_area_ = cv::Point2f(center_.x + 100, center_.y); // 默认偏移
        return;
    }

    // 规则：沿中心→扇叶的延长线，向外偏移
    const auto& fan = fanblades_[0];

    float dx = fan.getCenter().x - center_.x;
    float dy = fan.getCenter().y - center_.y;
    float len = std::sqrt(dx*dx + dy*dy);
    dx /= len; dy /= len;

    // 击打区域 = 中心 + 方向*(r + 50)
    hit_area_.x = center_.x + dx * (hit_radius_ + 40);
    hit_area_.y = center_.y + dy * (hit_radius_ + 40);

    //std::cout << "[击打区] 圆外坐标: (" << hit_area_.x << ", " << hit_area_.y << ")" << std::endl;
}

void BaseBuff::init(const std::vector<Fanblade>& fanblades, const cv::RotatedRect& center_rect) {

    fanblades_ = fanblades;

    // 初始化中心
    if (center_rect.center.x > 0 && center_rect.center.y > 0 ) {
        center_ = center_rect.center;
    } else {
        center_ = calcFanbladesCenter();
    }

    // 计算半径 + 找击打区域
    calcHitRadius();
    findHitAreaOutsideCircle();

    // 计算角度
    for (auto& fanblade : fanblades_) {
        fanblade.calculateRotateAngle(center_);
    }
}

void BaseBuff::debugDraw(cv::Mat& img) const {
    cv::circle(img, center_, 2, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::circle(img, center_, 4, cv::Scalar(0, 255, 255), 2);

    if (img.empty() || fanblades_.empty()) return;

    for (const auto& fanblade : fanblades_) {
        cv::line(img, center_, fanblade.getCenter(), cv::Scalar(255, 0, 0), 2);
        drawFanblade(img, fanblade);
    }
    // 绘制能量机关中心
    
}

// 绘制：禁止圆 + 击打区域
void BaseBuff::drawHitArea(cv::Mat& img) const {
    if (img.empty() || hit_radius_ <= 0) return;

    cv::circle(img, center_, static_cast<int>(hit_radius_), cv::Scalar(0, 0, 255), 2);

    hit_area_.x + 5;
    hit_area_.y + 5;

    cv::circle(img, hit_area_, 4, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::putText(img, "HIT AREA", hit_area_ + cv::Point2f(10, -10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

    cv::line(img, center_, hit_area_, cv::Scalar(0, 255, 0), 1);
}

void SmallBuff::update(const std::vector<Fanblade>& fanblades) {
    fanblades_ = fanblades;
    for (auto& fanblade : fanblades_) {
        fanblade.calculateRotateAngle(center_);
    }
}

void LargeBuff::update(const std::vector<Fanblade>& fanblades) {
    fanblades_ = fanblades;
    for (auto& fanblade : fanblades_) {
        fanblade.calculateRotateAngle(center_);
    }
}

} // namespace auto_buff