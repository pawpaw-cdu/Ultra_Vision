// include/auto_buff/base_buff.hpp
#ifndef AUTO_BUFF_BASE_BUFF_HPP
#define AUTO_BUFF_BASE_BUFF_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include "fanblade.hpp"
#include "utils.hpp"

namespace auto_buff {

class BaseBuff {
protected:
    std::vector<Fanblade> fanblades_;
    cv::Point2f center_;           // 能量机关中心
    float rotate_velocity_;
    float hit_radius_ = 0.0f;      // 半径r：中心到扇叶的距离
    cv::Point2f hit_area_;         // 圆外击打区域坐标

    // 计算扇叶几何中心
    cv::Point2f calcFanbladesCenter() const;
    // 计算中心到扇叶的半径r
    void calcHitRadius();
    // 在圆外找击打区域
    void findHitAreaOutsideCircle();

public:
    BaseBuff() : rotate_velocity_(0.0), center_(0,0), hit_area_(0,0) {}
    virtual ~BaseBuff() = default;

    virtual void init(const std::vector<Fanblade>& fanblades, const cv::RotatedRect& center_rect);
    virtual void debugDraw(cv::Mat& img) const;
    // 绘制击打区域（圆+击打标记）
    void drawHitArea(cv::Mat& img) const;
    virtual void update(const std::vector<Fanblade>& fanblades) = 0;

    // 获取接口
    const cv::Point2f& getCenter() const { return center_; }
    const cv::Point2f& getHitArea() const { return hit_area_; }
    float getHitRadius() const { return hit_radius_; }
    const std::vector<Fanblade>& getFanblades() const { return fanblades_; }
};

class SmallBuff : public BaseBuff {
public:
    SmallBuff() { rotate_velocity_ = 60.0; }
    void update(const std::vector<Fanblade>& fanblades) override;
};

class LargeBuff : public BaseBuff {
public:
    LargeBuff() { rotate_velocity_ = 30.0; }
    void update(const std::vector<Fanblade>& fanblades) override;
};

} // namespace auto_buff

#endif // AUTO_BUFF_BASE_BUFF_HPP