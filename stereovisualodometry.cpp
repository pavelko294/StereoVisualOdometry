#include "stereovisualodometry.h"

namespace {
const std::string WIN_NAME = "Stereo Visual Odometry";
const string FLOW_WIN_NAME = "Optical Flow";
constexpr int ESC_KEY = 27;
}

        // Обработка новой стереопары кадров
        void StereoVisualOdometry::processFrame(const cv::Mat& left_img, const cv::Mat& right_img) {
            auto start_time = std::chrono::high_resolution_clock::now();

            cv::Mat left_gray, right_gray;
            cv::cvtColor(left_img, left_gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right_img, right_gray, cv::COLOR_BGR2GRAY);

            // Коррекция дисторсии
            cv::Mat left_rect, right_rect;
            cv::undistort(left_gray, left_rect, K_left_, dist_left_);
            cv::undistort(right_gray, right_rect, K_right_, dist_right_);

            if (prev_left_.empty()) {
                // Первый кадр - инициализация
                prev_left_ = left_rect.clone();
                prev_right_ = right_rect.clone();
                extractKeypoints(prev_left_, prev_keypoints_, prev_descriptors_);

                // Инициализация точек для оптического потока
                initTrackingPoints(prev_left_, left_flow_prev_pts_);
                initTrackingPoints(prev_right_, right_flow_prev_pts_);

                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                std::cout << "Initialization time: " << duration.count() << " ms" << std::endl;

                return;
            }

            // 1. Расчет оптического потока для левой камеры
            vector<Point2f> left_flow_next_pts;
            vector<uchar> left_flow_status;
            flow_tracker_.calculateFlow(prev_left_, left_rect,
                                        left_flow_prev_pts_, left_flow_next_pts,
                                        left_flow_status);

            // 2. Расчет оптического потока для правой камеры
            vector<Point2f> right_flow_next_pts;
            vector<uchar> right_flow_status;
            flow_tracker_.calculateFlow(prev_right_, right_rect,
                                        right_flow_prev_pts_, right_flow_next_pts,
                                        right_flow_status);

            // Визуализация потоков
            Mat left_flow_vis = left_img.clone();
            flow_tracker_.drawFlow(left_flow_vis, left_flow_prev_pts_,
                                   left_flow_next_pts, left_flow_status,
                                   Scalar(0, 255, 0)); // Зеленый для левой камеры

            Mat right_flow_vis = right_img.clone();
            flow_tracker_.drawFlow(right_flow_vis, right_flow_prev_pts_,
                                   right_flow_next_pts, right_flow_status,
                                   Scalar(255, 0, 0)); // Синий для правой камеры

            Mat combined_window;
            hconcat(left_flow_vis, right_flow_vis, combined_window); // Горизонтальное объединение
            imshow(FLOW_WIN_NAME, combined_window);

            // Обновление точек для следующего кадра
            flow_tracker_.updatePoints(left_flow_prev_pts_, left_flow_next_pts, left_flow_status);
            flow_tracker_.updatePoints(right_flow_prev_pts_, right_flow_next_pts, right_flow_status);


            // Извлечение ключевых точек для текущего кадра
            std::vector<cv::KeyPoint> curr_keypoints;
            cv::Mat curr_descriptors;
            extractKeypoints(left_rect, curr_keypoints, curr_descriptors);

            // Сопоставление признаков между предыдущим и текущим кадром
            std::vector<cv::DMatch> matches;
            matcher_->match(prev_descriptors_, curr_descriptors, matches);

            // Фильтрация совпадений
            std::vector<cv::DMatch> good_matches = StereoVisualOdometry::filterMatches(matches);

            // Получение соответствующих точек
            std::vector<cv::Point2f> prev_points, curr_points;
            for (const auto& m : good_matches) {
                prev_points.push_back(prev_keypoints_[m.queryIdx].pt);
                curr_points.push_back(curr_keypoints[m.trainIdx].pt);
            }

            // Вычисление глубины для предыдущих точек с помощью стереозрения
            std::vector<float> depths = computeDepths(prev_left_, prev_right_, prev_points);

            // Фильтрация точек с валидной глубиной
            std::vector<cv::Point2f> filtered_prev_points, filtered_curr_points;
            std::vector<cv::Point3f> prev_3d_points;
            for (size_t i = 0; i < prev_points.size(); ++i) {
                if (depths[i] > 0) {
                    filtered_prev_points.push_back(prev_points[i]);
                    filtered_curr_points.push_back(curr_points[i]);

                    // Преобразование 2D точки в 3D
                    cv::Point3f p;
                    p.x = (prev_points[i].x - principal_point_.x) * depths[i] / focal_length_;
                    p.y = (prev_points[i].y - principal_point_.y) * depths[i] / focal_length_;
                    p.z = depths[i];
                    prev_3d_points.push_back(p);
                }
            }

            if (filtered_prev_points.size() < 8) {
                std::cerr << "Недостаточно точек для оценки движения (" << filtered_prev_points.size() << ")" << std::endl;

                // Обновление предыдущего кадра
                prev_left_ = left_rect.clone();
                prev_right_ = right_rect.clone();
                prev_keypoints_ = curr_keypoints;
                prev_descriptors_ = curr_descriptors.clone();
                return;
            }

            // Вычисление матрицы существенности
            cv::Mat E, mask;
            E = cv::findEssentialMat(filtered_curr_points, filtered_prev_points,
                                     focal_length_, principal_point_,
                                     cv::RANSAC, 0.999, 1.0, mask);

            // Восстановление относительного положения
            cv::Mat R, t;
            int inliers = cv::recoverPose(E, filtered_curr_points, filtered_prev_points,
                                          R, t, focal_length_, principal_point_, mask);

            // Интеграция движения
            integrateMotion(R, t);

            // Визуализация
            visualizeOdometry(left_img, curr_keypoints, good_matches, inliers);

            // Обновление предыдущего кадра
            prev_left_ = left_rect.clone();
            prev_right_ = right_rect.clone();
            prev_keypoints_ = curr_keypoints;
            prev_descriptors_ = curr_descriptors.clone();


            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            std::cout << "Processing time: " << duration.count() << " ms" << std::endl;
        }

        // Получить текущую позицию камеры
        cv::Point3f StereoVisualOdometry::getCurrentPosition() const {
            return cv::Point3f(
                current_pose_.at<double>(0, 3),
                current_pose_.at<double>(1, 3),
                current_pose_.at<double>(2, 3)
                );
        }

        // Получить общее пройденное расстояние
        double StereoVisualOdometry::getTotalDistance() {
            cv::Point3f current_pos = getCurrentPosition();

            if (prev_position_ != cv::Point3f(0, 0, 0)) {
                double delta_distance = cv::norm(current_pos - prev_position_);
                total_distance_ += delta_distance;
            }

            prev_position_ = current_pos;
            return total_distance_;
        }

        // Получить смещение от начальной точки
        double StereoVisualOdometry::getDisplacement() const {
            return cv::norm(getCurrentPosition());
        }

        // Извлечение ключевых точек
        void StereoVisualOdometry::extractKeypoints(const cv::Mat& image,
                              std::vector<cv::KeyPoint>& keypoints,
                              cv::Mat& descriptors) {
            detector_->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
        }

        void StereoVisualOdometry::initTrackingPoints(const Mat& img, vector<Point2f>& pts) {
            goodFeaturesToTrack(img, pts, 500, 0.01, 10);
            cornerSubPix(img, pts, Size(10,10), Size(-1,-1),
                         TermCriteria(TermCriteria::COUNT|TermCriteria::EPS, 20, 0.03));
        }

        OpticalFlowTracker flow_tracker_;
        vector<Point2f> left_flow_prev_pts_;
        vector<Point2f> right_flow_prev_pts_;

        // Фильтрация совпадений
        std::vector<cv::DMatch> StereoVisualOdometry::filterMatches(const std::vector<cv::DMatch>& matches) {
            if (matches.empty()) return {};

            double min_dist = std::min_element(matches.begin(), matches.end(),
                                               [](const cv::DMatch& a, const cv::DMatch& b) { return a.distance < b.distance; })->distance;

            std::vector<cv::DMatch> good_matches;
            for (const auto& m : matches) {
                if (m.distance < std::max(2.0 * min_dist, 30.0)) {
                    good_matches.push_back(m);
                }
            }

            return good_matches;
        }

        // Вычисление глубины
        std::vector<float> StereoVisualOdometry::computeDepths(const cv::Mat& left_img, const cv::Mat& right_img,
                                         const std::vector<cv::Point2f>& points) {
            std::vector<float> depths(points.size(), -1.0f);

            cv::Mat disparity;
            stereo_matcher_->compute(left_img, right_img, disparity);

            for (size_t i = 0; i < points.size(); ++i) {
                cv::Point2f pt = points[i];
                if (pt.x >= 0 && pt.x < disparity.cols && pt.y >= 0 && pt.y < disparity.rows) {
                    float disp = disparity.at<short>(pt.y, pt.x) / 16.0f;
                    if (disp > 0) {
                        depths[i] = focal_length_ * baseline_ / disp;
                    }
                }
            }

            return depths;
        }

        // Интеграция движения
        void StereoVisualOdometry::integrateMotion(const cv::Mat& R, const cv::Mat& t) {
            cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
            R.copyTo(T(cv::Rect(0, 0, 3, 3)));
            t.copyTo(T(cv::Rect(3, 0, 1, 3)));
            current_pose_ = current_pose_ * T.inv();
        }

        // Визуализация
        void StereoVisualOdometry::visualizeOdometry(const cv::Mat& frame,
                               const std::vector<cv::KeyPoint>& keypoints,
                               const std::vector<cv::DMatch>& matches,
                               int inliers) {
            cv::Mat img_matches;
            cv::drawMatches(prev_left_, prev_keypoints_, frame, keypoints, matches, img_matches);

            // Отображение информации о позе
            cv::Point3f pos = getCurrentPosition();
            double distance = getTotalDistance();
            double displacement = getDisplacement();

            std::stringstream ss;
            ss << "Position: [" << pos.x << ", " << pos.y << ", " << pos.z << "]";
            cv::putText(img_matches, ss.str(), cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            ss.str("");
            ss << "Inliers: " << inliers;
            cv::putText(img_matches, ss.str(), cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            ss.str("");
            ss << "Total distance: " << distance << " m";
            cv::putText(img_matches, ss.str(), cv::Point(10, 90),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            ss.str("");
            ss << "Displacement: " << displacement << " m";
            cv::putText(img_matches, ss.str(), cv::Point(10, 120),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            cv::imshow(WIN_NAME, img_matches);
        }

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

        void OpticalFlowTracker::calculateFlow(const Mat& prevImg, const Mat& nextImg,
                           vector<Point2f>& prevPts, vector<Point2f>& nextPts,
                           vector<uchar>& status) {
            if (prevPts.empty()) return;

            vector<float> err;
            calcOpticalFlowPyrLK(prevImg, nextImg, prevPts, nextPts,
                                 status, err, winSize_, maxLevel_, termCrit_);
        }

        void OpticalFlowTracker::drawFlow(Mat& img, const vector<Point2f>& prevPts,
                                               const vector<Point2f>& nextPts, const vector<uchar>& status,
                                               const Scalar& color = Scalar(0, 255, 0)) {
            for (size_t i = 0; i < prevPts.size(); i++) {
                if (status[i]) {
                    arrowedLine(img, prevPts[i], nextPts[i], color, 1);
                    circle(img, nextPts[i], 3, Scalar(0, 0, 255), -1);
                }
            }
        }


        void OpticalFlowTracker::updatePoints(vector<Point2f>& prevPts, vector<Point2f>& nextPts,
                          const vector<uchar>& status) {
            vector<Point2f> good_new;
            for (size_t i = 0; i < nextPts.size(); i++) {
                if (status[i]) {
                    good_new.push_back(nextPts[i]);
                }
            }
            prevPts = good_new;
        }

