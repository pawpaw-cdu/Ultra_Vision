#include "tracker.hpp"
#include <cmath>

namespace auto_aim
{
    Tracker::Tracker(const TrackerConfig& cfg)
        : cfg_(cfg), last_timestamp_(0.0), state_(TrackerState::LOST),
          detect_count_(0), lost_count_(0), confidence_(0.0)
    {
        kf_ = cv::KalmanFilter(cfg_.state_dim, cfg_.measure_dim, 0);
    }

    void Tracker::initKalman(double dt, const cv::Mat& init_state)
    {
        // 状态转移矩阵 A
        cv::setIdentity(kf_.transitionMatrix);
        kf_.transitionMatrix.at<float>(0,1) = dt;
        kf_.transitionMatrix.at<float>(2,3) = dt;
        kf_.transitionMatrix.at<float>(4,5) = dt;

        // 观测矩阵 H
        kf_.measurementMatrix = cv::Mat::zeros(cfg_.measure_dim, cfg_.state_dim, CV_32F);
        kf_.measurementMatrix.at<float>(0,0) = 1;
        kf_.measurementMatrix.at<float>(1,2) = 1;
        kf_.measurementMatrix.at<float>(2,4) = 1;

        // 过程噪声 Q
        cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-2));
        for (int i = 0; i < cfg_.state_dim; i+=2)
            kf_.processNoiseCov.at<float>(i,i) = cfg_.process_noise_pos;
        for (int i = 1; i < cfg_.state_dim; i+=2)
            kf_.processNoiseCov.at<float>(i,i) = cfg_.process_noise_vel;

        // 测量噪声 R
        cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(cfg_.measure_noise));

        // 初始误差协方差 P
        cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(cfg_.init_error_cov));

        kf_.statePost = init_state.clone();
        kf_.statePre = init_state.clone();
    }

    void Tracker::init(const Armor& armor, double timestamp)
    {
        latest_armor_ = armor;
        last_timestamp_ = timestamp;

        cv::Mat init_state = cv::Mat::zeros(cfg_.state_dim, 1, CV_32F);
        init_state.at<float>(0) = armor.tvec.at<double>(0);
        init_state.at<float>(2) = armor.tvec.at<double>(1);
        init_state.at<float>(4) = armor.tvec.at<double>(2);

        initKalman(cfg_.default_dt, init_state);

        state_ = TrackerState::DETECTING;
        detect_count_ = 1;
        lost_count_ = 0;
        confidence_ = 0.3;
    }

    Armor Tracker::predict(double timestamp)
    {
        if (state_ == TrackerState::LOST)
            return latest_armor_;

        double dt = timestamp - last_timestamp_;
        if (dt < 1e-6) dt = cfg_.default_dt;

        kf_.transitionMatrix.at<float>(0,1) = dt;
        kf_.transitionMatrix.at<float>(2,3) = dt;
        kf_.transitionMatrix.at<float>(4,5) = dt;

        cv::Mat pred_state = kf_.predict();
        Armor pred_armor = latest_armor_;
        pred_armor.tvec = (cv::Mat_<double>(3,1) <<
            pred_state.at<float>(0),
            pred_state.at<float>(2),
            pred_state.at<float>(4));
        last_timestamp_ = timestamp;
        return pred_armor;
    }

    bool Tracker::update(const Armor& armor, double timestamp)
    {
        double dist = computeMahalanobisDistance(armor);
        if (dist > cfg_.max_match_distance && state_ == TrackerState::TRACKING) {
            lostUpdate();
            return false;
        }

        double dt = timestamp - last_timestamp_;
        if (dt < 1e-6) dt = cfg_.default_dt;
        kf_.transitionMatrix.at<float>(0,1) = dt;
        kf_.transitionMatrix.at<float>(2,3) = dt;
        kf_.transitionMatrix.at<float>(4,5) = dt;

        cv::Mat measurement(cfg_.measure_dim, 1, CV_32F);
        measurement.at<float>(0) = armor.tvec.at<double>(0);
        measurement.at<float>(1) = armor.tvec.at<double>(1);
        measurement.at<float>(2) = armor.tvec.at<double>(2);

        cv::Mat state = kf_.correct(measurement);
        latest_armor_.tvec = (cv::Mat_<double>(3,1) <<
            state.at<float>(0),
            state.at<float>(2),
            state.at<float>(4));
        latest_armor_.armor_type = armor.armor_type;
        latest_armor_.left = armor.left;
        latest_armor_.right = armor.right;

        last_timestamp_ = timestamp;
        lost_count_ = 0;

        if (state_ == TrackerState::DETECTING) {
            detect_count_++;
            if (detect_count_ >= cfg_.min_detect_frames) {
                state_ = TrackerState::TRACKING;
                confidence_ = 0.8;
            } else {
                confidence_ = 0.3 + 0.5 * detect_count_ / cfg_.min_detect_frames;
            }
        } else if (state_ == TrackerState::TRACKING) {
            confidence_ = std::min(1.0, confidence_ + 0.05);
        } else if (state_ == TrackerState::TEMP_LOST) {
            state_ = TrackerState::TRACKING;
            confidence_ = 0.7;
        }
        return true;
    }

    void Tracker::lostUpdate()
    {
        lost_count_++;
        if (state_ == TrackerState::TRACKING) {
            if (lost_count_ >= 1) {
                state_ = TrackerState::TEMP_LOST;
                confidence_ -= 0.1;
            }
        } else if (state_ == TrackerState::TEMP_LOST) {
            if (lost_count_ >= cfg_.max_lost_frames) {
                state_ = TrackerState::LOST;
                confidence_ = 0.0;
            } else {
                confidence_ -= 0.05;
            }
        } else if (state_ == TrackerState::DETECTING) {
            if (lost_count_ >= 2) {
                state_ = TrackerState::LOST;
                confidence_ = 0.0;
            }
        }
    }

    double Tracker::computeMahalanobisDistance(const Armor& armor) const
    {
        cv::Mat pos_pred = kf_.statePost(cv::Rect(0,0,1, cfg_.state_dim));
        double dx = pos_pred.at<float>(0) - armor.tvec.at<double>(0);
        double dy = pos_pred.at<float>(2) - armor.tvec.at<double>(1);
        double dz = pos_pred.at<float>(4) - armor.tvec.at<double>(2);
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
}