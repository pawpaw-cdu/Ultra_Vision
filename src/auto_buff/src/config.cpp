#include "../include/config.hpp"
#include <stdexcept>
#include <iostream>

namespace auto_buff {

void BuffConfig::loadFromYaml(const YAML::Node& node) {

    params_.buff_type = node["buff_type"].as<std::string>();
    params_.detector_color = node["detector_color"].as<int>();
    params_.color_type = node["color_type"].as<int>();
    params_.debug = node["debug"].as<int>();
    

    if (params_.buff_type == "small") {
        params_.threshold = node["small_buff"]["detector"]["threshold"].as<int>();
    } else if (params_.buff_type == "large") {
        params_.threshold = node["large_buff"]["detector"]["threshold"].as<int>();
    }

    params_.morph_size = node["detector"]["morph_size"].as<int>();

    params_.roi_width = node["detector"]["roi"]["width"].as<int>();
    params_.roi_height = node["detector"]["roi"]["height"].as<int>();
    params_.roi_y_offset = node["detector"]["roi"]["y_offset"].as<int>();

    params_.fanblade_min_h = node["detector"]["fanblade"]["min_h"].as<int>();
    params_.fanblade_min_w = node["detector"]["fanblade"]["min_w"].as<int>();
    params_.fanblade_max_h = node["detector"]["fanblade"]["max_h"].as<int>();
    params_.fanblade_max_w = node["detector"]["fanblade"]["max_w"].as<int>();
    params_.hw_ratio_min = node["detector"]["fanblade"]["hw_ratio_min"].as<double>();
    params_.hw_ratio_max = node["detector"]["fanblade"]["hw_ratio_max"].as<double>();
    params_.area_rect_ratio_min = node["detector"]["fanblade"]["area_rect_ratio_min"].as<double>();

    params_.center_max_h = node["detector"]["center"]["max_h"].as<int>();
    params_.center_max_w = node["detector"]["center"]["max_w"].as<int>();
    params_.center_hw_ratio_min = node["detector"]["center"]["hw_ratio_min"].as<double>();
    params_.center_hw_ratio_max = node["detector"]["center"]["hw_ratio_max"].as<double>();

    auto& red1 = node["hsv"]["red_range1"];
    params_.red_range1 = {red1["h_min"].as<int>(), red1["h_max"].as<int>(),
                          red1["s_min"].as<int>(), red1["s_max"].as<int>(),
                          red1["v_min"].as<int>(), red1["v_max"].as<int>()};
    
    auto& red2 = node["hsv"]["red_range2"];
    params_.red_range2 = {red2["h_min"].as<int>(), red2["h_max"].as<int>(),
                          red2["s_min"].as<int>(), red2["s_max"].as<int>(),
                          red2["v_min"].as<int>(), red2["v_max"].as<int>()};
    
    auto& blue = node["hsv"]["blue_range"];
    params_.blue_range = {blue["h_min"].as<int>(), blue["h_max"].as<int>(),
                          blue["s_min"].as<int>(), blue["s_max"].as<int>(),
                          blue["v_min"].as<int>(), blue["v_max"].as<int>()};
}

bool BuffConfig::load(const std::string& yaml_path) {
    try {
        YAML::Node node = YAML::LoadFile(yaml_path);
        loadFromYaml(node);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "配置加载失败: " << e.what() << std::endl;
        return false;
    }
}

} // namespace auto_buff