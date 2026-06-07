#include "../include/utils.hpp"
#include "../include/fanblade.hpp"  
#include <opencv2/opencv.hpp>

namespace auto_buff {

cv::Mat safeResize(const cv::Mat& img, double scale) {
    if (img.empty()) return cv::Mat();
    cv::Size size(img.cols * scale, img.rows * scale);
    return safeResize(img, size);
}

cv::Mat safeResize(const cv::Mat& img, const cv::Size& size) {
    if (img.empty()) return cv::Mat();
    cv::Mat resized;
    cv::resize(img, resized, size);
    return resized;
}

bool isRoiValid(const cv::Mat& img, const cv::Rect& roi) {
    if (img.empty()) return false;
    return (roi.x >= 0 && roi.y >= 0 &&
            roi.x + roi.width <= img.cols &&
            roi.y + roi.height <= img.rows);
}

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness) {
    cv::Point2f pts[4];
    rect.points(pts);
    for (int i = 0; i < 4; ++i) {
        cv::line(img, pts[i], pts[(i+1)%4], color, thickness);
    }
}

void drawFanblade(cv::Mat& img, const auto_buff::Fanblade& fanblade) {
    drawRotatedRect(img, static_cast<const cv::RotatedRect&>(fanblade), cv::Scalar(255, 0, 0), 2);
    cv::circle(img, fanblade.getCenter(), 2, cv::Scalar(0, 255, 0), cv::FILLED);
    
}

} // namespace auto_buff