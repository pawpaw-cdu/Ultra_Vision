#ifndef AUTO_BUFF_HANDLE_HPP
#define AUTO_BUFF_HANDLE_HPP

#include <opencv2/opencv.hpp>

namespace auto_buff {

// 符柄（流水灯）
class Handle {
private:
    cv::Point2f center_;      // 符柄中心
    cv::Point2f direction_;   // 从能量机关中心指向符柄中心的单位向量
    double angle_;            // 角度（度）
    double length_;           // 符柄长度
public:
    Handle() : center_(0,0), direction_(0,0), angle_(0.0), length_(0.0) {}
    Handle(const cv::Point2f& center, const cv::Point2f& center_of_buff)
        : center_(center) {
        cv::Point2f vec = center - center_of_buff;
        length_ = cv::norm(vec);
        if (length_ > 1e-6) {
            direction_ = vec / length_;
        }
        angle_ = std::atan2(direction_.y, direction_.x) * 180.0 / CV_PI;
    }

    const cv::Point2f& getCenter() const { return center_; }
    const cv::Point2f& getDirection() const { return direction_; }
    double getAngle() const { return angle_; }
    double getLength() const { return length_; }
};

} // namespace auto_buff

#endif