#include "SCV4LFILE.h"
#include "calibrationcamera.h"
#include "stereovisualodometry.h"

namespace {
const std::string WIN_NAME = "Stereo Visual Odometry";
const string FLOW_WIN_NAME = "Optical Flow";
constexpr int ESC_KEY = 27;
}

// cv::Mat QImageToMat(const QImage &qImage) {
//     return cv::Mat(qImage.height(), qImage.width(), CV_8UC3,
//                    const_cast<uchar*>(qImage.bits()), qImage.bytesPerLine()).clone();
// }

int main(int argc, char *argv[])
{


    // Открытие окон и установка разрешения
    namedWindow(WIN_NAME, WINDOW_NORMAL);
    namedWindow(FLOW_WIN_NAME, WINDOW_NORMAL);
    resizeWindow(WIN_NAME, 1920, 1080);
    resizeWindow(FLOW_WIN_NAME, 1920, 1080);
    //namedWindow(RIGHT_FLOW_WIN_NAME, WINDOW_AUTOSIZE);

    // Открытие видеопотоков при обработке видеофайлов формата MP4:
    cv::VideoCapture cap_left("C:/Users/user/Desktop/grok_test/left_frames.mp4"), cap_right("C:/Users/user/Desktop/grok_test/right_frames.mp4");
    if (!cap_left.isOpened() || !cap_right.isOpened()) {
        cout << "Error: video is not opened\n";
        return -1;
    }

    // Чтение изображений в формате jpeg:
    cv::Mat image = cv::imread("input.jpg", cv::IMREAD_COLOR);

    // Параметры калибровки (должны быть получены заранее)
    cv::Mat K_left = (cv::Mat_<double>(3, 3) <<
                          1000, 0, 320,
                      0, 1000, 240,
                      0, 0, 1);

    cv::Mat K_right = K_left.clone();
    cv::Mat dist_left = cv::Mat::zeros(1, 5, CV_64F);
    cv::Mat dist_right = cv::Mat::zeros(1, 5, CV_64F);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat T = (cv::Mat_<double>(3, 1) << -0.8, 0, 0); // Базис 10 см

    // Создание объекта одометрии
    StereoVisualOdometry vo(K_left, K_right, dist_left, dist_right, R, T, 0.1);

    // Основной цикл
    cv::Mat frame_left, frame_right;
    while (true) {
        // MP4:
        cap_left >> frame_left;
        cap_right >> frame_right;
        // JPEG:
        //frame_left = cv::imread("left.jpg", cv::IMREAD_COLOR);
        //frame_right = cv::imread("right.jpg", cv::IMREAD_COLOR);
        // QImage:
        //frame_left = QImageToMat(imageLeft);
        //frame_right = QImageToMat(imageRight);

        if (frame_left.empty() || frame_right.empty()) {
            std::cerr << "End of video stream" << std::endl;
            break;
        }

        vo.processFrame(frame_left, frame_right);

        if (cv::waitKey(1) == ESC_KEY) break;
    }

    cap_left.release();
    cap_right.release();
    cv::destroyAllWindows();

    return 0;
}
