#ifndef STEREOVISUALODOMETRY_H
#define STEREOVISUALODOMETRY_H
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <vector>
#include <chrono>

using namespace std;
using namespace cv;


class OpticalFlowTracker {
public:
    OpticalFlowTracker() :
        termCrit_(TermCriteria::COUNT|TermCriteria::EPS, 20, 0.03),
        winSize_(31, 31),
        maxLevel_(3) {}

    void calculateFlow(const Mat& prevImg, const Mat& nextImg,
                       vector<Point2f>& prevPts, vector<Point2f>& nextPts,
                       vector<uchar>& status);

    void drawFlow(Mat& img, const vector<Point2f>& prevPts,
                  const vector<Point2f>& nextPts, const vector<uchar>& status,
                  const Scalar& color);

    void updatePoints(vector<Point2f>& prevPts, vector<Point2f>& nextPts,
                      const vector<uchar>& status);

private:
    TermCriteria termCrit_;
    Size winSize_;
    int maxLevel_;
};

class StereoVisualOdometry {
public:
    StereoVisualOdometry(const cv::Mat& K_left, const cv::Mat& K_right,
                         const cv::Mat& dist_left, const cv::Mat& dist_right,
                         const cv::Mat& R, const cv::Mat& T,
                         float baseline, int num_features = 2000)
        : K_left_(K_left), K_right_(K_right),
        dist_left_(dist_left), dist_right_(dist_right),
        R_(R), T_(T), baseline_(baseline), num_features_(num_features),
        focal_length_(K_left.at<double>(0, 0)),
        principal_point_(K_left.at<double>(0, 2), K_left.at<double>(1, 2)) {

        // Инициализация детектора и матчера признаков
        detector_ = cv::ORB::create(num_features_);
        matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

        // Инициализация позиции (начало координат)
        current_pose_ = cv::Mat::eye(4, 4, CV_64F);

        // Инициализация стерео матчера
        stereo_matcher_ = cv::StereoSGBM::create(
            0, 96, 11, 100, 1000, 32, 0, 15, 100, 32, cv::StereoSGBM::MODE_SGBM_3WAY);
    }

    // Обработка пары кадров
    void processFrame(const cv::Mat& left_img, const cv::Mat& right_img);

    // Получить текущую позицию камеры
    cv::Point3f getCurrentPosition() const;

    // Получить общее пройденное расстояние
    double getTotalDistance();

    // Получить смещение от начальной точки
    double getDisplacement() const;

private:
    // Извлечение ключевых точек
    void extractKeypoints(const cv::Mat& image,
                          std::vector<cv::KeyPoint>& keypoints,
                          cv::Mat& descriptors);

    void initTrackingPoints(const Mat& img, vector<Point2f>& pts);

    OpticalFlowTracker flow_tracker_;
    vector<Point2f> left_flow_prev_pts_;
    vector<Point2f> right_flow_prev_pts_;

    // Фильтрация совпадений
    std::vector<cv::DMatch> filterMatches(const std::vector<cv::DMatch>& matches);

    // Вычисление глубины
    std::vector<float> computeDepths(const cv::Mat& left_img, const cv::Mat& right_img,
                                     const std::vector<cv::Point2f>& points);

    // Интеграция движения
    void integrateMotion(const cv::Mat& R, const cv::Mat& t);

    // Визуализация
    void visualizeOdometry(const cv::Mat& frame,
                           const std::vector<cv::KeyPoint>& keypoints,
                           const std::vector<cv::DMatch>& matches,
                           int inliers);

    // Параметры камеры
    cv::Mat K_left_, K_right_;
    cv::Mat dist_left_, dist_right_;
    cv::Mat R_, T_;
    float baseline_;
    int num_features_;

    // Параметры изображения
    float focal_length_;
    cv::Point2f principal_point_;

    // Текущая поза
    cv::Mat current_pose_;

    // Детекторы
    cv::Ptr<cv::Feature2D> detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    cv::Ptr<cv::StereoSGBM> stereo_matcher_;

    // Данные предыдущего кадра
    cv::Mat prev_left_, prev_right_;
    std::vector<cv::KeyPoint> prev_keypoints_;
    cv::Mat prev_descriptors_;

    // Трекинг расстояния
    cv::Point3f prev_position_ = cv::Point3f(0, 0, 0);
    double total_distance_ = 0.0;
};

#endif // STEREOVISUALODOMETRY_H
