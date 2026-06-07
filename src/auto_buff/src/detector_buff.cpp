#include "../include/detector_buff.hpp"
#include "../include/utils.hpp"
#include <iostream>

namespace auto_buff {

DetectorBuff::DetectorBuff(const cv::Mat& img, const BuffParams& config) 
    : src_img_(img), config_(config) {
    resize_img_ = safeResize(src_img_, SCALE_);
}

cv::Mat DetectorBuff::generateMask() {
    if (config_.color_type == 0) {
        cv::Mat gray;
        cv::cvtColor(resize_img_, gray, cv::COLOR_BGR2GRAY);
        cv::Mat mask;
        cv::threshold(gray, mask, config_.threshold, 255, cv::THRESH_BINARY);
        return mask;
    } else {
        cv::Mat hsv;
        cv::cvtColor(resize_img_, hsv, cv::COLOR_BGR2HSV);
        if (config_.detector_color == 1) {
            cv::Mat mask1, mask2;
            cv::inRange(hsv, cv::Scalar(config_.red_range1.h_min, config_.red_range1.s_min, config_.red_range1.v_min),
                        cv::Scalar(config_.red_range1.h_max, config_.red_range1.s_max, config_.red_range1.v_max), mask1);
            cv::inRange(hsv, cv::Scalar(config_.red_range2.h_min, config_.red_range2.s_min, config_.red_range2.v_min),
                        cv::Scalar(config_.red_range2.h_max, config_.red_range2.s_max, config_.red_range2.v_max), mask2);
            return mask1 | mask2;
        } else {
            cv::Mat mask;
            cv::inRange(hsv, cv::Scalar(config_.blue_range.h_min, config_.blue_range.s_min, config_.blue_range.v_min),
                        cv::Scalar(config_.blue_range.h_max, config_.blue_range.s_max, config_.blue_range.v_max), mask);
            return mask;
        }
    }
}

cv::Mat DetectorBuff::morphProcess(const cv::Mat& mask) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, 
                        cv::Size(config_.morph_size, config_.morph_size));
    cv::Mat processed;
    cv::morphologyEx(mask, processed, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, kernel);
    return processed;
}

void DetectorBuff::findContours(std::vector<Fanblade>& fanblades, cv::RotatedRect& center_rect) {
    fanblades.clear();
    center_rect = cv::RotatedRect();
    //std::cout << "\n==================== 调试打印 ====================" << std::endl;

    cv::Mat resized_mask = mask_img_;
    //std::cout << "[调试] 缩放后图像尺寸: " << resize_img_.size() << std::endl;

    int roi_x = (resized_mask.cols - config_.roi_width) / 2;
    int roi_y = (resized_mask.rows - config_.roi_height) / 2 + config_.roi_y_offset;
    cv::Rect roi(roi_x, roi_y, config_.roi_width, config_.roi_height);

    //std::cout << "[调试] ROI区域: x=" << roi.x << ", y=" << roi.y << std::endl;
    if (!isRoiValid(resized_mask, roi)) {
        std::cerr << "[错误] ROI越界！" << std::endl;
        return;
    }

    cv::Mat roi_mask = resized_mask(roi);
    roi_rgb_ = resize_img_(roi).clone();

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roi_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    std::cout << "[调试] 总轮廓数: " << contours.size() << std::endl;
    int i = 1;
    for(auto contour : contours){
        cv::RotatedRect rect = cv::minAreaRect(contour);
        cv::circle(roi_rgb_,rect.center,2,cv::Scalar(0,255,0));
        cv::putText(roi_rgb_,std::to_string(i),rect.center + cv::Point2f(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        std::cout <<i<<"] size: "<<contour.size()<<std::endl;
        std::cout <<i<<"] position: "<< rect.center<<std::endl;
        std::cout <<i<<"] height-width: "<<rect.size.height <<" "<<rect.size.width << std::endl;
        i++;
                }

    for (const auto& contour : contours) {
        if (contour.size() < 5) continue;
        cv::RotatedRect rect = cv::minAreaRect(contour);
        double area = cv::contourArea(contour);
        if (area < 5) continue;

        double hw_ratio = std::max(rect.size.height, rect.size.width) / std::min(rect.size.height, rect.size.width);
        double area_rect_ratio = area / (rect.size.height * rect.size.width);
        
        if ((rect.size.height < config_.center_max_h && rect.size.width < config_.center_max_w) && (rect.size.height > 5 && rect.size.width > 5)) {
            // int line_dx;
            // line_dx = std::abs(rect.center.x - roi.x);
            // if(line_dx > 5){
            //     continue;
            // }
            center_rect = rect;
            center_rect.center.x += roi.x;
            center_rect.center.y += roi.y;
            std::cout << "[调试] ✅ 找到Buff中心! 坐标: " << center_rect.center << std::endl;
            continue;
        }

        // 扇叶筛选
        if(contour.size() > 250 || contour.size() < 100)continue;
        if (hw_ratio < config_.hw_ratio_min || hw_ratio > config_.hw_ratio_max ||area_rect_ratio < config_.area_rect_ratio_min ) continue;
        if ((rect.size.height < config_.fanblade_min_h || rect.size.width < config_.fanblade_min_w) ||
            (rect.size.height > config_.fanblade_max_h || rect.size.width > config_.fanblade_max_w) )continue;
        rect.center.x += roi.x;
        rect.center.y += roi.y;
        fanblades.emplace_back(rect);
        std::cout << "[调试] ✅ 筛选到扇叶! 坐标: " << rect.center << std::endl;
    }

    //std::cout << "[调试] 最终有效扇叶数: " << fanblades.size() << std::endl;
    //std::cout << "[调试] 检测中心有效: " << (center_rect.center.x > 0 ? "是" : "否") << std::endl;
}

void DetectorBuff::process(std::vector<Fanblade>& fanblades, cv::RotatedRect& center_rect) {
    mask_img_ = generateMask();
    mask_img_ = morphProcess(mask_img_);
    findContours(fanblades, center_rect);
}

void DetectorBuff::debugShow() const {
    if (!config_.debug) return;
    //cv::imshow("Resize Image", resize_img_);
    cv::imshow("Mask Image", mask_img_);
    if (!roi_rgb_.empty()) cv::imshow("ROI RGB", roi_rgb_);
}

} // namespace auto_buff