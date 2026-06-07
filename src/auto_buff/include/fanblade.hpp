// include/auto_buff/fanblade.hpp
#ifndef AUTO_BUFF_FANBLADE_HPP
#define AUTO_BUFF_FANBLADE_HPP

#include <opencv2/opencv.hpp>

namespace auto_buff {

class Fanblade : public cv::RotatedRect {
private:
    double rotate_angle_; // 相对中心的旋转角度
    cv::Point2f center_;  // 扇叶中心
public:
    Fanblade(const cv::RotatedRect& rect);

    // 计算相对Buff中心的旋转角度
    void calculateRotateAngle(const cv::Point2f& buff_center);
    // 获取扇叶中心
    const cv::Point2f& getCenter() const { return center_; }
    // 获取旋转角度
    double getRotateAngle() const { return rotate_angle_; }

    // 预测下一帧扇叶位置（预留接口）
    Fanblade predict(double dt) const;
};

} // namespace auto_buff

#endif 