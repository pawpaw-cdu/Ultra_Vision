#include <iostream>
#include <opencv2/opencv.hpp>
#include "../include/config.hpp"
#include "../include/detector_buff.hpp"
#include "../include/base_buff.hpp"
#include "../include/utils.hpp"

int main() {
    auto_buff::BuffConfig config;
    std::string yaml_path = "/home/pawpaw-ubuntu/rm_ultra/configs/buff.yaml";
    if (!config.load(yaml_path)) {
        std::cerr << "配置加载失败！" << std::endl;
        return -1;
    }
    auto& params = config.getParams();

    cv::VideoCapture cap("/home/pawpaw-ubuntu/rm_ultra/test_vid_pic/buff_test.mp4");
    if (!cap.isOpened()) {
        std::cerr << "视频打开失败！" << std::endl;
        return -1;
    }

    std::unique_ptr<auto_buff::BaseBuff> buff;
    if (params.buff_type == "small") {
        buff = std::make_unique<auto_buff::SmallBuff>();
    } else if (params.buff_type == "large") {
        buff = std::make_unique<auto_buff::LargeBuff>();
    } else {
        std::cerr << "不支持的BUFF类型" << std::endl;
        return -1;
    }

    int success = 0;
    cv::Mat frame;
    while (cap.read(frame)) {
        if (frame.empty()) break;

        std::vector<auto_buff::Fanblade> fanblades;
        cv::RotatedRect center_rect;
        static cv::RotatedRect R_center;
        auto_buff::DetectorBuff detector(frame, params);
        detector.process(fanblades, center_rect);
        if(success == 0 || !fanblades.empty()){
            success = 1;
        }
        if(success){
            R_center = center_rect;
        }else{
            R_center = R_center;
        }
        // 初始化BUFF
        buff->init(fanblades, R_center);

        
        cv::Mat display_img = detector.getResizeImg();
        if (!display_img.empty()) {
            buff->debugDraw(display_img);   
            buff->drawHitArea(display_img);  
        }

        // 显示
        cv::imshow("Buff Detection Result", display_img);
        detector.debugShow();
        int key = (params.debug == 1) ? 0 : 20;
        if (cv::waitKey(key) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}