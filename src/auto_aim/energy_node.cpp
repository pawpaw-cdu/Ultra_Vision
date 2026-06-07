#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <chrono>
#include <iomanip>

using namespace std;
using namespace cv;

// ========================== 能量机关检测器定义 ==========================

struct EnergyBlade {
    cv::Point2f center;
    int id;
    float angle_rad;
};

struct EnergyDetectorConfig {
    float angular_velocity_deg = 60.0f;   // 恒定角速度 (度/秒)
    int num_blades = 5;                   // 扇叶数量
    float jump_threshold_px = 15.0f;      // 圆心跳变阈值 (像素)
    float min_radius_px = 50.0f;          // 最小半径 (像素)
    float max_radius_px = 300.0f;         // 最大半径 (像素)
    int max_track_lost_frames = 5;        // 最大丢失帧数
    float max_match_distance_ratio = 0.4f; // 匹配同一扇叶的最大距离系数 (半径的倍数)
};

class EnergyDetector {
public:
    EnergyDetector(const EnergyDetectorConfig& cfg = EnergyDetectorConfig()) : cfg_(cfg) {
        reset();
    }

    void reset() {
        prev_center_ = Point2f(0, 0);
        current_center_ = Point2f(0, 0);
        circle_center_ = Point2f(0, 0);
        radius_px_ = 0;
        blade0_angle_rad_ = 0;
        last_timestamp_ = 0;
        lost_frame_count_ = 0;
        model_initialized_ = false;
    }

    bool process(const vector<Point2f>& blade_centers, vector<EnergyBlade>& predicted_blades) {
        double now = getTimestamp();
        predicted_blades.clear();

        // 未初始化模型
        if (!model_initialized_) {
            if (last_timestamp_ == 0 && !blade_centers.empty()) {
                // 第一帧：暂存第一个扇叶中心
                prev_center_ = blade_centers[0];
                last_timestamp_ = now;
                return false;
            }
            else if (last_timestamp_ != 0 && !blade_centers.empty()) {
                Point2f matched;
                if (findSameBlade(blade_centers, matched)) {
                    current_center_ = matched;
                    double dt = now - last_timestamp_;
                    if (dt < 1e-6) return false;
                    float delta_angle = cfg_.angular_velocity_deg * M_PI / 180.0f * dt;
                    if (computeCircleFromTwoPoints(prev_center_, current_center_, delta_angle,
                                                   circle_center_, radius_px_)) {
                        blade0_angle_rad_ = atan2(prev_center_.y - circle_center_.y,
                                                  prev_center_.x - circle_center_.x);
                        model_initialized_ = true;
                        lost_frame_count_ = 0;
                        predictAllBlades(predicted_blades);
                        return true;
                    }
                }
                // 初始化失败，重置第一帧
                last_timestamp_ = now;
                if (!blade_centers.empty()) prev_center_ = blade_centers[0];
                return false;
            }
            return false;
        }

        // 模型已初始化：跟踪同一扇叶
        Point2f matched;
        bool found = findSameBlade(blade_centers, matched);
        if (found) {
            current_center_ = matched;
            double dt = now - last_timestamp_;
            if (dt > 0) {
                float delta_angle = cfg_.angular_velocity_deg * M_PI / 180.0f * dt;
                Point2f new_center;
                float new_radius;
                if (computeCircleFromTwoPoints(prev_center_, current_center_, delta_angle,
                                               new_center, new_radius)) {
                    if (!isCenterJump(circle_center_, new_center)) {
                        circle_center_ = new_center;
                        radius_px_ = new_radius;
                        blade0_angle_rad_ = atan2(current_center_.y - circle_center_.y,
                                                  current_center_.x - circle_center_.x);
                        lost_frame_count_ = 0;
                    } else {
                        reset();
                        return false;
                    }
                }
            }
            prev_center_ = current_center_;
            last_timestamp_ = now;
            predictAllBlades(predicted_blades);
            return true;
        } else {
            lost_frame_count_++;
            if (lost_frame_count_ > cfg_.max_track_lost_frames) {
                reset();
                return false;
            }
            predictAllBlades(predicted_blades);
            return true;
        }
    }

    Point2f getCircleCenter() const { return circle_center_; }
    float getRadius() const { return radius_px_; }
    bool isModelInitialized() const { return model_initialized_; }

private:
    EnergyDetectorConfig cfg_;
    Point2f prev_center_, current_center_, circle_center_;
    float radius_px_;
    float blade0_angle_rad_;
    double last_timestamp_;
    int lost_frame_count_;
    bool model_initialized_;

    double getTimestamp() {
        auto now = chrono::steady_clock::now();
        return chrono::duration<double>(now.time_since_epoch()).count();
    }

    bool findSameBlade(const vector<Point2f>& centers, Point2f& matched) {
        if (centers.empty() || !model_initialized_) return false;
        float min_dist = 1e6;
        int best_idx = -1;
        for (size_t i = 0; i < centers.size(); ++i) {
            float dist = norm(centers[i] - prev_center_);
            if (dist < min_dist) {
                min_dist = dist;
                best_idx = i;
            }
        }
        float max_match_dist = (radius_px_ > 0) ? radius_px_ * cfg_.max_match_distance_ratio : 50.0f;
        if (best_idx != -1 && min_dist < max_match_dist) {
            matched = centers[best_idx];
            return true;
        }
        return false;
    }

    bool computeCircleFromTwoPoints(const Point2f& p1, const Point2f& p2, float delta_angle_rad,
                                    Point2f& center, float& radius) {
        float cos_a = cos(delta_angle_rad);
        float sin_a = sin(delta_angle_rad);
        float a = 1 - cos_a;
        float b = sin_a;
        float c = p2.x - p1.x * cos_a + p1.y * sin_a;
        float d = p2.y - p1.x * sin_a - p1.y * cos_a;
        float denom = a * a + b * b;
        if (fabs(denom) < 1e-6) return false;
        center.x = (a * c - b * d) / denom;
        center.y = (a * d + b * c) / denom;
        radius = norm(p1 - center);
        return (radius > cfg_.min_radius_px && radius < cfg_.max_radius_px);
    }

    bool isCenterJump(const Point2f& old_center, const Point2f& new_center) {
        return norm(old_center - new_center) > cfg_.jump_threshold_px;
    }

    void predictAllBlades(vector<EnergyBlade>& blades) {
        blades.clear();
        if (!model_initialized_ || radius_px_ <= 0) return;
        float angle_step = 2 * M_PI / cfg_.num_blades;
        for (int i = 0; i < cfg_.num_blades; ++i) {
            float angle = blade0_angle_rad_ + i * angle_step;
            EnergyBlade blade;
            blade.center = Point2f(circle_center_.x + radius_px_ * cos(angle),
                                   circle_center_.y + radius_px_ * sin(angle));
            blade.id = i;
            blade.angle_rad = angle;
            blades.push_back(blade);
        }
    }
};

// ==================== 基于形态学特征的扇叶中心检测（无模型） ====================

vector<Point2f> detectBladeCentersMorph(const Mat& bgr) {
    vector<Point2f> centers;
    if (bgr.empty()) return centers;

    Mat hsv;
    cvtColor(bgr, hsv, COLOR_BGR2HSV);
    Mat mask;
    // 红色范围（可微调）
    Mat mask1, mask2;
    inRange(hsv, Scalar(0, 50, 50), Scalar(10, 255, 255), mask1);
    inRange(hsv, Scalar(160, 50, 50), Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    // 形态学闭运算连接断裂区域，开运算去除噪点
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);
    morphologyEx(mask, mask, MORPH_OPEN, kernel);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto& cnt : contours) {
        // 面积筛选
        double area = contourArea(cnt);
        if (area < 150 || area > 2000) continue;

        // 矩形度 (轮廓面积 / 最小外接矩形面积)
        RotatedRect rect = minAreaRect(cnt);
        float rect_area = rect.size.width * rect.size.height;
        if (rect_area <= 0) continue;
        double rectangularity = area / rect_area;
        if (rectangularity < 0.5) continue;

        // 长宽比 (扇叶细长)
        float w = rect.size.width;
        float h = rect.size.height;
        float ratio = min(w, h) / max(w, h);
        if (ratio > 0.45) continue;

        // 圆形度 (扇叶不应是圆形)
        double perimeter = arcLength(cnt, true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        if (circularity > 0.4) continue;

        // 凸度 (area / hull_area)
        vector<Point> hull;
        convexHull(cnt, hull);
        double hull_area = contourArea(hull);
        double solidity = area / hull_area;
        if (solidity < 0.7) continue;

        // 计算质心
        Moments m = moments(cnt);
        if (m.m00 > 0) {
            centers.push_back(Point2f(m.m10 / m.m00, m.m01 / m.m00));
        }
    }
    return centers;
}

// ========================== 主程序 ==========================

int main() {
    string video_path = "/home/pawpaw-ubuntu/rm_ultra/test_vid_pic/buff.mp4";
    VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        cerr << "Error: Cannot open video file " << video_path << endl;
        return -1;
    }

    // 配置能量机关检测器
    EnergyDetectorConfig cfg;
    cfg.num_blades = 5;
    cfg.angular_velocity_deg = 60.0f;
    cfg.jump_threshold_px = 15.0f;
    cfg.min_radius_px = 50.0f;
    cfg.max_radius_px = 300.0f;
    cfg.max_track_lost_frames = 5;
    cfg.max_match_distance_ratio = 0.4f;

    EnergyDetector detector(cfg);

    Mat frame;
    int frame_idx = 0;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // 使用形态学特征检测扇叶中心
        vector<Point2f> blade_centers = detectBladeCentersMorph(frame);

        // 调用能量机关检测器
        vector<EnergyBlade> predicted_blades;
        bool ok = detector.process(blade_centers, predicted_blades);

        // 可视化
        Mat display = frame.clone();

        // 绘制检测到的扇叶中心（红色圆点）
        for (const auto& pt : blade_centers) {
            circle(display, pt, 5, Scalar(0, 0, 255), -1);
        }

        // 绘制模型信息
        if (detector.isModelInitialized()) {
            Point2f center = detector.getCircleCenter();
            float radius = detector.getRadius();
            circle(display, center, radius, Scalar(255, 0, 0), 2);
            circle(display, center, 5, Scalar(255, 0, 0), -1);
            for (const auto& blade : predicted_blades) {
                circle(display, blade.center, 5, Scalar(0, 255, 0), -1);
            }
        }

        // 文字提示
        putText(display, "Energy Detector (Morph)", Point(30, 30),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        string status = detector.isModelInitialized() ? "MODEL OK" : "MODEL WAITING";
        putText(display, status, Point(30, 60), FONT_HERSHEY_SIMPLEX, 0.6,
                detector.isModelInitialized() ? Scalar(0, 255, 0) : Scalar(0, 0, 255), 2);
        putText(display, "Blades: " + to_string(blade_centers.size()), Point(30, 90),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        if (detector.isModelInitialized()) {
            putText(display, "Radius: " + to_string((int)detector.getRadius()), Point(30, 120),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        }

        imshow("Energy Detection", display);
        if (waitKey(30) == 'q') break;
        frame_idx++;
    }

    return 0;
}