#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <opencv2/video/tracking.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include "/home/pawpaw-ubuntu/rm_ultra/common/types.hpp"

namespace auto_aim
{
    enum class TrackerState { DETECTING, TRACKING, TEMP_LOST, LOST };

    struct TrackerConfig {
        int min_detect_frames = 3;
        int max_lost_frames = 10;
        double max_match_distance = 0.5;
        int state_dim = 6;
        int measure_dim = 3;
        double process_noise_pos = 0.1;
        double process_noise_vel = 0.01;
        double measure_noise = 0.1;
        double init_error_cov = 1.0;
        double default_dt = 0.033;
    };

    class Tracker
    {
    public:
        Tracker(const TrackerConfig& cfg);
        ~Tracker() = default;

        void init(const Armor& armor, double timestamp);
        Armor predict(double timestamp);
        bool update(const Armor& armor, double timestamp);
        void lostUpdate();

        TrackerState getState() const { return state_; }
        const Armor& getLatestArmor() const { return latest_armor_; }
        double getConfidence() const { return confidence_; }
        int getLostCount() const { return lost_count_; }

    private:
        TrackerConfig cfg_;
        cv::KalmanFilter kf_;
        double last_timestamp_;
        TrackerState state_;
        int detect_count_;
        int lost_count_;
        double confidence_;
        Armor latest_armor_;

        void initKalman(double dt, const cv::Mat& init_state);
        double computeMahalanobisDistance(const Armor& armor) const;
    };
}

#endif