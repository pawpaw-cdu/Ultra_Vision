#ifndef AUTO_BUFF_UTILS_HPP
#define AUTO_BUFF_UTILS_HPP

#include <opencv2/opencv.hpp>

namespace auto_buff {
    class Fanblade;
}

namespace auto_buff {

cv::Mat safeResize(const cv::Mat& img, double scale);
cv::Mat safeResize(const cv::Mat& img, const cv::Size& size);
bool isRoiValid(const cv::Mat& img, const cv::Rect& roi);
void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness = 2);
void drawFanblade(cv::Mat& img, const auto_buff::Fanblade& fanblade);

} // namespace auto_buff

#endif // AUTO_BUFF_UTILS_HPP