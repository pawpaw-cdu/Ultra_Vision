// include/auto_buff/config.hpp
#ifndef AUTO_BUFF_CONFIG_HPP
#define AUTO_BUFF_CONFIG_HPP

#include <string>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace auto_buff {

struct HsvRange {
    int h_min, h_max;
    int s_min, s_max;
    int v_min, v_max;
};

struct BuffParams {
    // 通用参数
    std::string buff_type;
    int detector_color;   // 1:red 0:blue
    int color_type;       // 1:hsv 0:bgr
    int debug;            // 1:阻塞调试 0:实时播放
    int threshold;        // 二值化阈值
    
    // 形态学参数
    int morph_size;       // 形态学核大小
    
    // ROI参数
    int roi_width;
    int roi_height;
    int roi_y_offset;     // ROI Y轴偏移
    
    // 扇叶筛选参数
    int fanblade_min_h;
    int fanblade_min_w;
    int fanblade_max_h;
    int fanblade_max_w;
    double hw_ratio_min;  // 高宽比最小值
    double hw_ratio_max;  // 高宽比最大值
    double area_rect_ratio_min; // 轮廓面积/矩形面积最小值
    
    // 中心筛选参数
    int center_max_h;
    int center_max_w;
    double center_hw_ratio_min;
    double center_hw_ratio_max;

    // HSV颜色范围
    HsvRange red_range1;
    HsvRange red_range2;
    HsvRange blue_range;
};

class BuffConfig {
private:
    BuffParams params_;
    void loadFromYaml(const YAML::Node& node);
public:
    BuffConfig() = default;
    // 加载配置文件
    bool load(const std::string& yaml_path);
    // 获取配置参数
    const BuffParams& getParams() const { return params_; }
};

} // namespace auto_buff

#endif // AUTO_BUFF_CONFIG_HPP