#ifndef CALIBRATIONCAMERA_H
#define CALIBRATIONCAMERA_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <QImage>

class Calibration {

public:

    void findChessboardCorners(const cv::Mat &leftImage, const cv::Mat &rightImage);
    cv::Mat QImageToMat(const QImage &qImage);
    QImage MatToQImage(const cv::Mat &mat);

    cv::Mat K_left;
    cv::Mat K_right;
    cv::Mat distCoeffs_left;
    cv::Mat distCoeffs_right;
    cv::Mat R;
    cv::Mat T; // описал параметры, необходимо передать в метод реализующий ВО, здесь вынесены для удобства и чтобы не забыть.
private:

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePointsLeft, imagePointsRight;
};

#endif // CALIBRATIONCAMERA_H
