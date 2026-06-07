// src/kalman_filter.cpp
#include "../Kalman/kalman_filter.hpp"

namespace auto_buff {

KalmanFilter::KalmanFilter(double dt, double process_noise, double measure_noise)
    : dt_(dt), process_noise_(process_noise), measure_noise_(measure_noise) {
    kf_ = cv::KalmanFilter(4, 2, 0);
    measurement_ = cv::Mat::zeros(2, 1, CV_32F);

    kf_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1);

    kf_.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);

    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(process_noise_));
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(measure_noise_));
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1));
}

void KalmanFilter::init(const cv::Point2f& initial_pos) {
    kf_.statePost.at<float>(0) = initial_pos.x;
    kf_.statePost.at<float>(1) = initial_pos.y;
    kf_.statePost.at<float>(2) = 0;
    kf_.statePost.at<float>(3) = 0;
}

cv::Point2f KalmanFilter::predict() {
    cv::Mat pred = kf_.predict();
    return cv::Point2f(pred.at<float>(0), pred.at<float>(1));
}

void KalmanFilter::update(const cv::Point2f& measurement) {
    measurement_.at<float>(0) = measurement.x;
    measurement_.at<float>(1) = measurement.y;
    kf_.correct(measurement_);
}

cv::Point2f KalmanFilter::getPredictedPos() const {
    return cv::Point2f(kf_.statePre.at<float>(0), kf_.statePre.at<float>(1));
}

} // namespace auto_buff