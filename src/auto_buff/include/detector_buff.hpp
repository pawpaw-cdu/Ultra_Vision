#ifndef AUTO_BUFF_DETECTOR_BUFF_HPP
#define AUTO_BUFF_DETECTOR_BUFF_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include "config.hpp"
#include "fanblade.hpp"

namespace auto_buff {

class DetectorBuff {
private:
    cv::Mat src_img_;       
    cv::Mat resize_img_;    
    cv::Mat mask_img_;      
    cv::Mat roi_rgb_;       
    BuffParams config_;
    const double SCALE_ = 0.5; // 缩放比例

    cv::Mat generateMask();
    cv::Mat morphProcess(const cv::Mat& mask);
    void findContours(std::vector<Fanblade>& fanblades, cv::RotatedRect& center_rect);

public:
    DetectorBuff(const cv::Mat& img, const BuffParams& config);
    ~DetectorBuff() = default;

    void process(std::vector<Fanblade>& fanblades, cv::RotatedRect& center_rect);
    void debugShow() const;

    const cv::Mat& getResizeImg() const { return resize_img_; }
    const cv::Mat& getRoiRgb() const { return roi_rgb_; }
};

} // namespace auto_buff

#endif // AUTO_BUFF_DETECTOR_BUFF_HPP