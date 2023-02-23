#include <chrono>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) {
  if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) {
    std::cout << "Left button of the mouse is clicked - position (" << x << ", "
              << y << ")" << '\n';
    control_points.emplace_back(x, y);
  }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) {
  auto &p_0 = points[0];
  auto &p_1 = points[1];
  auto &p_2 = points[2];
  auto &p_3 = points[3];

  for (double t = 0.0; t <= 1.0; t += 0.001) {
    auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

    window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
  }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points,
                             float t) {
  // TODO: Implement de Casteljau's algorithm
  auto &A = control_points[0];
  auto &B = control_points[1];
  auto &C = control_points[2];
  auto &D = control_points[3];

  auto E = (1 - t) * A + t * B;
  auto F = (1 - t) * B + t * C;
  auto G = (1 - t) * C + t * D;

  auto H = (1 - t) * E + t * F;
  auto I = (1 - t) * F + t * G;

  auto J = (1 - t) * H + t * I;

  return J;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) {
  // TODO: Iterate through all t = 0 to t = 1 with small steps, and call
  // de Casteljau's recursive Bezier algorithm.

  for (double t = 0.0; t <= 1.0; t += 0.001) {
    auto point = recursive_bezier(control_points, t);

    auto nowP = cv::Point2f(std::floor(point.x), std::floor(point.y));
    float len = sqrt((nowP.x + 0.5 - point.x) * (nowP.x + 0.5 - point.x) +
                     (nowP.y + 0.5 - point.y) * (nowP.y + 0.5 - point.y));

    for (double dx = -0.5; dx <= 0.5; dx += 1.0) {
      for (double dy = -0.5; dy <= 0.5; dy += 1.0) {
        auto newP =
            cv::Point2f(std::floor(point.x + dx), std::floor(point.y + dy));
        float dis = sqrt((newP.x + 0.5 - point.x) * (newP.x + 0.5 - point.x) +
                         (newP.y + 0.5 - point.y) * (newP.y + 0.5 - point.y));

        float color = window.at<cv::Vec3b>(newP)[1];
        window.at<cv::Vec3b>(newP)[1] = std::max(255 * (len / dis), color);
      }
    }

    // window.at<cv::Vec3b>(point)[1] = 255;
  }
}

int main() {
  cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
  cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
  cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

  cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

  int key = -1;
  while (key != 27) {
    for (auto &point : control_points) {
      cv::circle(window, point, 3, {255, 255, 255}, 3);
    }

    if (control_points.size() == 4) {
      // naive_bezier(control_points, window);
      bezier(control_points, window);

      cv::imshow("Bezier Curve", window);
      cv::imwrite("my_bezier_curve.png", window);
      key = cv::waitKey(0);

      return 0;
    }

    cv::imshow("Bezier Curve", window);
    key = cv::waitKey(20);
  }

  return 0;
}
