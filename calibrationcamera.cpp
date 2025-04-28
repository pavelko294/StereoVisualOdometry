#include "calibrationcamera.h"

using namespace cv;
using namespace std;

cv::Mat Calibration::QImageToMat(const QImage &qImage) {
    return cv::Mat(qImage.height(), qImage.width(), CV_8UC3,
                   const_cast<uchar*>(qImage.bits()), qImage.bytesPerLine()).clone();
}


void Calibration::findChessboardCorners(const cv::Mat &leftImage, const cv::Mat &rightImage) {

    cv::Size boardSize(9, 6); // Количество внутренних углов шахматной доски
    float squareSize = 1.0f; // Размер квадрата на шахматной доске

    // Генерация 3D точек для шахматной доски
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    cv::Mat grayLeft, grayRight;
    cv::cvtColor(leftImage, grayLeft, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rightImage, grayRight, cv::COLOR_BGR2GRAY);

    // Поиск углов на шахматной доске
    std::vector<cv::Point2f> cornersLeft, cornersRight;
    bool foundLeft = cv::findChessboardCorners(grayLeft, boardSize, cornersLeft);
    bool foundRight = cv::findChessboardCorners(grayRight, boardSize, cornersRight);

    if (foundLeft && foundRight) {
        // Уточнение найденных углов
        cv::cornerSubPix(grayLeft, cornersLeft, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        cv::cornerSubPix(grayRight, cornersRight, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

        // Сохранение точек
        imagePointsLeft.push_back(cornersLeft);
        imagePointsRight.push_back(cornersRight);
        objectPoints.push_back(obj);

        // Отображение углов на изображении
        cv::drawChessboardCorners(leftImage, boardSize, cornersLeft, foundLeft);
        cv::drawChessboardCorners(rightImage, boardSize, cornersRight, foundRight);
    }

    // Если собрано достаточно кадров, выполнить калибровку
    if (objectPoints.size() >= 10) {
        cv::Mat cameraMatrixLeft, distCoeffsLeft, cameraMatrixRight, distCoeffsRight;
        cv::Mat R, T, E, F;

        double rms = cv::stereoCalibrate(
            objectPoints, imagePointsLeft, imagePointsRight,
            cameraMatrixLeft, distCoeffsLeft,
            cameraMatrixRight, distCoeffsRight,
            leftImage.size(), R, T, E, F,
            cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 1e-6));

        // Сохранение параметров калибровки

        Mat K_left = cameraMatrixLeft;
        Mat R_right = cameraMatrixRight;
        Mat distCoeffs_left = distCoeffsLeft;
        Mat distCoeffs_right = distCoeffsRight;
        Mat rvec = R;
        Mat tvec = T; // описал параметры, необходимо передать в метод реализующий ВО, здесь вынесены для удобства и чтобы не забыть.
        //  В алгоритме используются параметры и кф дисторсии только левой камеры, от которой берем начало координат.
        cv::FileStorage fs("stereo_calibration.xml", cv::FileStorage::WRITE);
        fs << "cameraMatrixLeft" << cameraMatrixLeft;
        fs << "distCoeffsLeft" << distCoeffsLeft;
        fs << "cameraMatrixRight" << cameraMatrixRight;
        fs << "distCoeffsRight" << distCoeffsRight;
        fs << "R" << R;
        fs << "T" << T;
        fs << "E" << E;
        fs << "F" << F;
        fs.release();
    }
}

