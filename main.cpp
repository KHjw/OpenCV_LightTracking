#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

int main() {
  cv::VideoCapture cap(0);  // 카메라 열기
  if (!cap.isOpened()) {
    std::cerr << "Error: Webcam could not be opened." << std::endl;
    return -1;
  }

  cv::namedWindow("Flipped Video", cv::WINDOW_NORMAL);
  cv::resizeWindow("Flipped Video", 640, 480);
  cv::moveWindow("Flipped Video", 0, 0);

  cv::namedWindow("Grayscale Video", cv::WINDOW_NORMAL);
  cv::resizeWindow("Grayscale Video", 300, 200);
  cv::moveWindow("Grayscale Video", 700, 100);

  double minArea = 1000;  // 최소 객체 크기 설정

  while (true) {
    cv::Mat frame, flippedFrame, grayFrame, thresholdImage;
    cap >> frame;  // 프레임 캡처
    if (frame.empty()) break;

    // 좌우 반전
    cv::flip(frame, flippedFrame, 1);

    // 그레이스케일 변환 및 임계값 적용
    cv::cvtColor(flippedFrame, grayFrame, cv::COLOR_BGR2GRAY);
    cv::threshold(grayFrame, thresholdImage, 230, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholdImage, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    cv::Point maxLoc;
    double maxArea = 0;
    bool objectDetected = false;
    cv::Rect objectRect;
    std::string coords = "Coords: null";

    for (const auto& contour : contours) {
      double area = cv::contourArea(contour);
      if (area > minArea) {
        if (area > maxArea) {  // 최소 크기와 최대 크기 비교
          maxArea = area;
          objectRect = cv::boundingRect(contour);
          cv::Moments m = cv::moments(contour);
          maxLoc = cv::Point(static_cast<int>(m.m10 / m.m00),
                             static_cast<int>(m.m01 / m.m00));
          objectDetected = true;
        }
      }
    }

    cv::Point center(flippedFrame.cols / 2, flippedFrame.rows / 2);
    cv::line(flippedFrame, cv::Point(center.x - 10, center.y),
             cv::Point(center.x + 10, center.y), cv::Scalar(255, 255, 255), 2);
    cv::line(flippedFrame, cv::Point(center.x, center.y - 10),
             cv::Point(center.x, center.y + 10), cv::Scalar(255, 255, 255), 2);

    if (objectDetected) {
      cv::rectangle(flippedFrame, objectRect, cv::Scalar(255, 0, 0), 2);
      cv::Point transformed(maxLoc.x - center.x, center.y - maxLoc.y);
      coords = "Coords: (" + std::to_string(transformed.x) + ", " +
               std::to_string(transformed.y) + ")";
    }

    std::string detectionStatus =
        "ObjectDetected: " + std::string(objectDetected ? "true" : "false");
    cv::Scalar textColor =
        objectDetected ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);

    cv::putText(flippedFrame, coords, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, textColor, 2);
    cv::putText(flippedFrame, detectionStatus, cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, textColor, 2);

    cv::imshow("Flipped Video", flippedFrame);
    cv::imshow("Grayscale Video", thresholdImage);

    if (cv::waitKey(10) == 27) {
      break;  // ESC 키로 종료
    }
  }

  cap.release();
  cv::destroyAllWindows();
  return 0;
}