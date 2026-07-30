#include <arpa/inet.h>
#include <algorithm>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <stdexcept>
#include <vector>

#include <opencv2/opencv.hpp>

namespace {
constexpr int kJpegQuality = 80;
constexpr char kBoundary[] = "frame";
std::atomic_bool running{true};

struct Config {
  std::string camera = "/dev/v4l/by-id/usb-XHH-260128-A_2M-video-index0";
  int width = 640;
  int height = 480;
  int fps = 120;
  int stream_fps = 60;
  int port = 8080;
  cv::Rect roi;
  int threshold = 200;
  int min_area = 50;
  int max_area = 12000;
  int max_center_offset = 80;
  int pipe_threshold = 200;
  double pipe_min_aspect = 5.5;
  int hough_min_radius = 8;
  int hough_max_radius = 13;
  int edge_ignore = 16;
};

struct Frames {
  std::mutex mutex;
  std::vector<uchar> raw;
  std::vector<uchar> detected;
  bool found = false;
  double position_cm = 0.0;
  double processing_fps = 0.0;
  uint64_t sequence = 0;
  int64_t timestamp_ms = 0;
  uint64_t capture_time_us = 0;
  uint32_t processing_time_us = 0;
  float center_x_px = 0.0F;
  float center_y_px = 0.0F;
  float radius_px = 0.0F;
  float contour_area_px2 = 0.0F;
  cv::Rect roi;
};

struct PipeAxis {
  cv::Point2f centre;
  cv::Point2f direction;
  float length;
  float half_width;
};

struct TemplateMatch {
  cv::Vec3f circle;
  double score;
};

double calibrated_position_cm(double pipe_x_px) {
  // One-dimensional projective calibration of the current fixed camera view.
  // It is fitted from the measured -10/-7.5/-5/+5/+7.5/+10 cm marks.  A line
  // in the camera image is projective, so this is more accurate than forcing
  // a single linear centimetres-per-pixel scale before the four-point warp is
  // refined again.
  constexpr double kNumeratorSlope = 0.06281099943416173;
  constexpr double kNumeratorOffset = -15.450827222405008;
  constexpr double kDenominatorSlope = 0.0005801149048865943;
  return (kNumeratorSlope * pipe_x_px + kNumeratorOffset) /
         (kDenominatorSlope * pipe_x_px + 1.0);
}

cv::Mat make_ball_template(const cv::Mat& pipe, const cv::Vec3f& circle) {
  cv::Mat gray;
  cv::cvtColor(pipe, gray, cv::COLOR_BGR2GRAY);
  cv::Mat ball_template;
  cv::getRectSubPix(gray, cv::Size(24, 24), {circle[0], circle[1]}, ball_template);
  return ball_template;
}

std::optional<TemplateMatch> find_template_ball(const cv::Mat& pipe,
                                                const cv::Mat& ball_template,
                                                float expected_x,
                                                const PipeAxis& axis) {
  if (ball_template.empty() || ball_template.cols > pipe.cols || ball_template.rows > pipe.rows) {
    return std::nullopt;
  }
  cv::Mat gray;
  cv::cvtColor(pipe, gray, cv::COLOR_BGR2GRAY);
  const int half_width = ball_template.cols / 2;
  const int search_left = std::clamp(cvRound(expected_x) - 56 - half_width, 0,
                                     pipe.cols - ball_template.cols);
  const int search_right = std::clamp(cvRound(expected_x) + 56 + half_width,
                                      ball_template.cols, pipe.cols);
  const cv::Rect search(search_left, 0, search_right - search_left, pipe.rows);
  cv::Mat correlation;
  cv::matchTemplate(gray(search), ball_template, correlation, cv::TM_CCOEFF_NORMED);
  double minimum = 0.0;
  double maximum = 0.0;
  cv::Point maximum_point;
  cv::minMaxLoc(correlation, &minimum, &maximum, nullptr, &maximum_point);
  if (maximum < 0.66) return std::nullopt;
  const cv::Point2f point(static_cast<float>(search.x + maximum_point.x + half_width),
                          static_cast<float>(maximum_point.y + ball_template.rows / 2));
  const cv::Point2f delta = point - axis.centre;
  const double lateral = std::abs(delta.x * -axis.direction.y + delta.y * axis.direction.x);
  if (std::abs(point.x - expected_x) > 56.0F ||
      lateral > axis.half_width ||
      std::abs(delta.dot(axis.direction)) > axis.length / 2 - 16.0F) {
    return std::nullopt;
  }
  return TemplateMatch{cv::Vec3f(point.x, point.y, 10.0F), maximum};
}

void signal_handler(int) { running = false; }

bool send_all(int fd, const std::string& data) {
  const char* buffer = data.data();
  size_t remaining = data.size();
  while (remaining > 0) {
    const auto sent = send(fd, buffer, remaining, MSG_NOSIGNAL);
    if (sent <= 0) return false;
    buffer += sent;
    remaining -= static_cast<size_t>(sent);
  }
  return true;
}

bool send_all(int fd, const std::vector<uchar>& data) {
  const char* buffer = reinterpret_cast<const char*>(data.data());
  size_t remaining = data.size();
  while (remaining > 0) {
    const auto sent = send(fd, buffer, remaining, MSG_NOSIGNAL);
    if (sent <= 0) return false;
    buffer += sent;
    remaining -= static_cast<size_t>(sent);
  }
  return true;
}

std::optional<cv::Vec3f> find_ball(const cv::Mat& roi, const Config& cfg,
                                   const std::optional<double>& previous_fraction,
                                   const PipeAxis& axis, const cv::Rect& roi_rect) {
  cv::Mat gray, blurred;
  cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

  // The lighting on a steel ball changes with its position, so a fixed global
  // threshold alone is fragile.  In the rectified 45 px pipe strip, adaptive
  // thresholding extracts the locally darker ball while ignoring slow changes
  // in pipe brightness.  Shape scoring rejects the black chassis details.
  cv::Mat adaptive;
  cv::adaptiveThreshold(blurred, adaptive, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                        cv::THRESH_BINARY_INV, 21, 7);
  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
  cv::morphologyEx(adaptive, adaptive, cv::MORPH_CLOSE, kernel);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(adaptive, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::optional<cv::Vec3f> best;
  double best_score = std::numeric_limits<double>::max();
  for (const auto& contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < cfg.min_area || area > cfg.max_area) continue;
    const double perimeter = cv::arcLength(contour, true);
    if (perimeter <= 0.0) continue;
    const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
    const cv::Rect bounds = cv::boundingRect(contour);
    const double aspect = static_cast<double>(bounds.width) / std::max(bounds.height, 1);
    if (circularity < 0.38 || aspect < 0.45 || aspect > 1.9) continue;
    const cv::Moments moments = cv::moments(contour);
    if (moments.m00 == 0.0) continue;
    const cv::Point2f point(static_cast<float>(moments.m10 / moments.m00 + roi_rect.x),
                            static_cast<float>(moments.m01 / moments.m00 + roi_rect.y));
    const cv::Point2f delta = point - axis.centre;
    const double along = delta.dot(axis.direction);
    const double lateral = std::abs(delta.x * -axis.direction.y + delta.y * axis.direction.x);
    if (lateral > std::min<double>(axis.half_width, cfg.max_center_offset) ||
        std::abs(along) > axis.length / 2 - cfg.edge_ignore) continue;
    const double fraction = (along + axis.length / 2) / axis.length;
    if (previous_fraction && std::abs(fraction - *previous_fraction) > 0.12) continue;
    const double radius = std::sqrt(area / CV_PI);
    double score = 2.0 * lateral + 12.0 * std::abs(radius - 10.0) +
                   90.0 * (1.0 - circularity);
    if (previous_fraction) score += 140.0 * std::abs(fraction - *previous_fraction);
    if (score < best_score) {
      best_score = score;
      best = cv::Vec3f(point.x - roi_rect.x, point.y - roi_rect.y,
                        static_cast<float>(radius));
    }
  }
  if (best) return best;

  // Circular Hough detection remains a recovery path when a strong specular
  // highlight splits the dark contour into several pieces.
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1.2, 18,
                   80, 18, cfg.hough_min_radius, cfg.hough_max_radius);
  if (!circles.empty()) {
    std::optional<cv::Vec3f> hough_best;
    double hough_best_score = std::numeric_limits<double>::max();
    for (const auto& circle : circles) {
      const cv::Point2f point(circle[0] + roi_rect.x, circle[1] + roi_rect.y);
      const cv::Point2f delta = point - axis.centre;
      const double along = delta.dot(axis.direction);
      const double lateral = std::abs(delta.x * -axis.direction.y + delta.y * axis.direction.x);
      if (lateral > std::min<double>(axis.half_width, cfg.max_center_offset) ||
          std::abs(along) > axis.length / 2 - cfg.edge_ignore) continue;
      const double fraction = (along + axis.length / 2) / axis.length;
      if (previous_fraction && std::abs(fraction - *previous_fraction) > 0.12) continue;
      double score = lateral + 0.5 * std::abs(circle[2] - 10.0F);
      if (previous_fraction) score += 20.0 * std::abs(fraction - *previous_fraction);
      if (score < hough_best_score) {
        hough_best_score = score;
        hough_best = circle;
      }
    }
    if (hough_best) return hough_best;
  }

  // If a bright reflection breaks the circular edge, fall back to the darker
  // connected component inside the already-localised pipe ROI.
  cv::Mat binary;
  cv::threshold(blurred, binary, cfg.threshold, 255, cv::THRESH_BINARY_INV);
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

  cv::Mat labels, stats, centroids;
  const int label_count = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8);
  int best_label = -1;
  double threshold_best_score = std::numeric_limits<double>::max();
  for (int label = 1; label < label_count; ++label) {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    const double x = centroids.at<double>(label, 0);
    const double y = centroids.at<double>(label, 1);
    const cv::Point2f point(x + roi_rect.x, y + roi_rect.y);
    const cv::Point2f delta = point - axis.centre;
    const double along = delta.dot(axis.direction);
    const double lateral = std::abs(delta.x * -axis.direction.y + delta.y * axis.direction.x);
    if (area < cfg.min_area || area > cfg.max_area ||
        lateral > std::min<double>(axis.half_width, cfg.max_center_offset) ||
        std::abs(along) > axis.length / 2 - cfg.edge_ignore) continue;
    const double radius = std::sqrt(area / CV_PI);
    const double fraction = (along + axis.length / 2) / axis.length;
    if (previous_fraction && std::abs(fraction - *previous_fraction) > 0.12) continue;
    double score = lateral + 0.03 * std::abs(radius - 10.0);
    if (previous_fraction) {
      score += 20.0 * std::abs(fraction - *previous_fraction);
    }
    if (score < threshold_best_score) {
      threshold_best_score = score;
      best_label = label;
    }
  }
  if (best_label < 0) return std::nullopt;
  const int area = stats.at<int>(best_label, cv::CC_STAT_AREA);
  return cv::Vec3f(static_cast<float>(centroids.at<double>(best_label, 0)),
                   static_cast<float>(centroids.at<double>(best_label, 1)),
                   static_cast<float>(std::sqrt(area / CV_PI)));
}

const std::vector<cv::Point2f>& pipe_source() {
  static const std::vector<cv::Point2f> source{{59.0F, 199.0F}, {596.0F, 229.0F},
                                                {596.0F, 258.0F}, {59.0F, 236.0F}};
  return source;
}

const std::vector<cv::Point2f>& pipe_destination() {
  static const std::vector<cv::Point2f> destination{{0.0F, 0.0F}, {540.0F, 0.0F},
                                                     {540.0F, 45.0F}, {0.0F, 45.0F}};
  return destination;
}

cv::Mat rectify_pipe_band(const cv::Mat& image) {
  // Only transform the calibrated pipe strip, not the entire 640x480 frame.
  // This keeps the original full camera view for transmission and cuts the
  // per-frame perspective work from 307200 pixels to 24300 pixels.
  cv::Mat rectified;
  cv::warpPerspective(image, rectified,
                      cv::getPerspectiveTransform(pipe_source(), pipe_destination()),
                      cv::Size(540, 45), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
  return rectified;
}

cv::Point2f map_pipe_point(const cv::Point2f& point) {
  static const cv::Mat inverse =
      cv::getPerspectiveTransform(pipe_destination(), pipe_source());
  std::vector<cv::Point2f> points{point};
  cv::perspectiveTransform(points, points, inverse);
  return points.front();
}

cv::Rect pipe_display_roi() {
  return cv::boundingRect(pipe_source());
}

void draw_pipe_outline(cv::Mat& image) {
  std::vector<cv::Point> outline;
  outline.reserve(pipe_source().size());
  for (const auto& point : pipe_source()) {
    outline.emplace_back(cvRound(point.x), cvRound(point.y));
  }
  cv::polylines(image, outline, true, cv::Scalar(255, 180, 0), 2, cv::LINE_AA);
}

void annotate(cv::Mat& image, const cv::Mat& pipe, const Config& cfg, Frames& frames,
              double processing_fps, uint64_t capture_time_us) {
  const auto processing_started = std::chrono::steady_clock::now();
  const cv::Rect roi(0, 0, pipe.cols, pipe.rows);
  const cv::Rect display_roi = pipe_display_roi();
  const PipeAxis axis{{270.0F, 22.5F}, {1.0F, 0.0F}, 540.0F, 22.5F};
  static cv::KalmanFilter tracker(2, 1, 0, CV_32F);
  static bool tracker_ready = false;
  static std::optional<double> previous_fraction;
  static int missed_frames = 0;
  static bool configured = false;
  static cv::Mat ball_template;
  if (!configured) {
    tracker.transitionMatrix = (cv::Mat_<float>(2, 2) << 1.0F, 1.0F, 0.0F, 1.0F);
    tracker.measurementMatrix = (cv::Mat_<float>(1, 2) << 1.0F, 0.0F);
    cv::setIdentity(tracker.processNoiseCov, cv::Scalar::all(0.8));
    tracker.processNoiseCov.at<float>(1, 1) = 4.0F;
    cv::setIdentity(tracker.measurementNoiseCov, cv::Scalar::all(4.0));
    cv::setIdentity(tracker.errorCovPost, cv::Scalar::all(20.0));
    configured = true;
  }
  draw_pipe_outline(image);
  float predicted_x = 270.0F;
  if (tracker_ready) {
    predicted_x = tracker.predict().at<float>(0);
    previous_fraction = std::clamp(static_cast<double>(predicted_x) / axis.length, 0.0, 1.0);
  }
  const auto contour_circle = find_ball(pipe, cfg, previous_fraction, axis, roi);
  const auto template_match = tracker_ready
      ? find_template_ball(pipe, ball_template, predicted_x, axis)
      : std::nullopt;
  // A strong image correlation is substantially steadier than a changing
  // reflected contour while the ball is still.  On a real movement the old
  // template score drops and the normal contour detector takes over at once.
  const bool use_template = template_match && template_match->score >= 0.80;
  const auto circle = use_template
      ? std::optional<cv::Vec3f>(template_match->circle)
      : contour_circle ? contour_circle
      : template_match ? std::optional<cv::Vec3f>(template_match->circle) : std::nullopt;
  bool found = false;
  double position_cm = 0.0;
  float center_x_px = 0.0F;
  float center_y_px = 0.0F;
  float radius_px = 0.0F;
  float contour_area_px2 = 0.0F;
  bool rejected = false;
  if (circle) {
    const float measured_x = (*circle)[0];
    const bool innovation_ok = !tracker_ready || std::abs(measured_x - predicted_x) <= 55.0F;
    if (innovation_ok) {
      float tracked_x = measured_x;
      if (!tracker_ready) {
        tracker.statePost = (cv::Mat_<float>(2, 1) << measured_x, 0.0F);
        tracker.statePre = tracker.statePost.clone();
        tracker_ready = true;
      } else {
        cv::Mat measurement(1, 1, CV_32F);
        measurement.at<float>(0) = measured_x;
        tracked_x = tracker.correct(measurement).at<float>(0);
      }
      const double fraction = std::clamp(static_cast<double>(tracked_x) / axis.length, 0.0, 1.0);
      previous_fraction = fraction;
      missed_frames = 0;
      position_cm = calibrated_position_cm(tracked_x);
      if (!use_template) ball_template = make_ball_template(pipe, *circle);
      const cv::Point2f camera_point = map_pipe_point({tracked_x, axis.centre.y});
      const cv::Point display_point(cvRound(camera_point.x), cvRound(camera_point.y));
      cv::circle(image, display_point, 10, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
      cv::circle(image, display_point, 2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
      std::ostringstream text;
      text.setf(std::ios::fixed);
      text.precision(2);
      text << "steel ball: " << position_cm << " cm";
      cv::putText(image, text.str(), {display_roi.x + 8, std::max(28, display_roi.y - 10)},
                  cv::FONT_HERSHEY_SIMPLEX, 0.75, {0, 255, 0}, 2);
      found = true;
      center_x_px = camera_point.x;
      center_y_px = camera_point.y;
      radius_px = 10.0F;
      contour_area_px2 = static_cast<float>(CV_PI * radius_px * radius_px);
    } else {
      rejected = true;
      const cv::Point2f camera_point = map_pipe_point({measured_x, (*circle)[1]});
      cv::circle(image, {cvRound(camera_point.x), cvRound(camera_point.y)}, 10,
                 cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }
  }
  if (!found) {
    ++missed_frames;
    if (tracker_ready && missed_frames <= 18) {
      const cv::Point2f camera_point = map_pipe_point({predicted_x, axis.centre.y});
      cv::circle(image, {cvRound(camera_point.x), cvRound(camera_point.y)}, 10,
                 cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
      cv::putText(image, rejected ? "steel ball: rejected (not sent)"
                                  : "steel ball: predicted (not sent)",
                  {display_roi.x + 8, std::max(28, display_roi.y - 10)},
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 255, 255}, 2);
    } else {
      cv::putText(image, "steel ball: not found", {display_roi.x + 8, std::max(28, display_roi.y - 10)},
                  cv::FONT_HERSHEY_SIMPLEX, 0.75, {0, 0, 255}, 2);
    }
    if (missed_frames > 18) {
      tracker_ready = false;
      previous_fraction.reset();
    }
  }
  std::ostringstream fps_text;
  fps_text.setf(std::ios::fixed);
  fps_text.precision(1);
  fps_text << "Detect FPS: " << processing_fps;
  cv::putText(image, fps_text.str(), {display_roi.x + 8, std::min(image.rows - 10, display_roi.y + display_roi.height + 24)},
              cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 255, 255}, 2);
  std::lock_guard lock(frames.mutex);
  frames.found = found;
  frames.position_cm = position_cm;
  frames.processing_fps = processing_fps;
  frames.capture_time_us = capture_time_us;
  frames.processing_time_us = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - processing_started).count());
  frames.center_x_px = center_x_px;
  frames.center_y_px = center_y_px;
  frames.radius_px = radius_px;
  frames.contour_area_px2 = contour_area_px2;
  frames.roi = display_roi;
}

void capture_loop(const Config& cfg, Frames& frames) {
  cv::VideoCapture camera(cfg.camera, cv::CAP_V4L2);
  if (!camera.isOpened()) throw std::runtime_error("Cannot open camera: " + cfg.camera);
  camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  camera.set(cv::CAP_PROP_FRAME_WIDTH, cfg.width);
  camera.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.height);
  camera.set(cv::CAP_PROP_FPS, cfg.fps);
  const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, kJpegQuality};
  auto previous_time = std::chrono::steady_clock::now();
  auto next_frame_time = previous_time;
  double processing_fps = 0.0;
  int preview_frame_counter = 0;
  const int preview_divisor = std::max(1, (cfg.fps + cfg.stream_fps - 1) / cfg.stream_fps);
  while (running) {
    cv::Mat raw;
    if (!camera.read(raw) || raw.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    const uint64_t capture_time_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    const auto now = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(now - previous_time).count();
    previous_time = now;
    if (seconds > 0.0) {
      const double instant_fps = 1.0 / seconds;
      processing_fps = processing_fps == 0.0 ? instant_fps : 0.9 * processing_fps + 0.1 * instant_fps;
    }
    const cv::Mat pipe = rectify_pipe_band(raw);
    cv::Mat detected = raw.clone();
    annotate(detected, pipe, cfg, frames, processing_fps, capture_time_us);
    if (++preview_frame_counter >= preview_divisor) {
      preview_frame_counter = 0;
      std::vector<uchar> raw_jpeg, detected_jpeg;
      cv::imencode(".jpg", raw, raw_jpeg, params);
      cv::imencode(".jpg", detected, detected_jpeg, params);
      std::lock_guard lock(frames.mutex);
      frames.raw = std::move(raw_jpeg);
      frames.detected = std::move(detected_jpeg);
    }
    {
      std::lock_guard lock(frames.mutex);
      frames.sequence++;
      frames.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
    }
    next_frame_time += std::chrono::milliseconds(1000 / std::max(cfg.fps, 1));
    if (next_frame_time > std::chrono::steady_clock::now()) {
      std::this_thread::sleep_until(next_frame_time);
    } else {
      next_frame_time = std::chrono::steady_clock::now();
    }
  }
}

void serve_client(int client, Frames& frames, int stream_fps) {
  char request[1024]{};
  const auto received = recv(client, request, sizeof(request) - 1, 0);
  if (received <= 0) { close(client); return; }
  const std::string line(request, static_cast<size_t>(received));
  const bool detected = line.find("GET /detect.mjpg") == 0;
  const bool snapshot = line.find("GET /snapshot.jpg") == 0;
  const bool health = line.find("GET /health") == 0;
  const bool data_stream = line.find("GET /data") == 0;
  if (data_stream) {
    send_all(client, "HTTP/1.1 200 OK\r\nCache-Control: no-cache\r\nContent-Type: text/event-stream\r\n\r\n");
    uint64_t sent_sequence = 0;
    while (running) {
      bool found; double position; double fps; uint64_t sequence; int64_t timestamp; uint64_t capture_time_us;
      uint32_t processing_time_us; float center_x_px; float center_y_px; float radius_px; float contour_area_px2;
      cv::Rect roi;
      {
        std::lock_guard lock(frames.mutex);
        found = frames.found;
        position = frames.position_cm;
        fps = frames.processing_fps;
        sequence = frames.sequence;
        timestamp = frames.timestamp_ms;
        capture_time_us = frames.capture_time_us;
        processing_time_us = frames.processing_time_us;
        center_x_px = frames.center_x_px;
        center_y_px = frames.center_y_px;
        radius_px = frames.radius_px;
        contour_area_px2 = frames.contour_area_px2;
        roi = frames.roi;
      }
      if (sequence != 0 && sequence != sent_sequence) {
        std::ostringstream body;
        body << "{\"sequence\":" << sequence << ",\"timestamp_ms\":" << timestamp
             << ",\"found\":" << (found ? "true" : "false") << ",\"position_cm\":"
             << position << ",\"fps\":" << fps << ",\"capture_time_us\":" << capture_time_us
             << ",\"processing_time_us\":" << processing_time_us << ",\"center_x_px\":" << center_x_px
             << ",\"center_y_px\":" << center_y_px << ",\"radius_px\":" << radius_px
             << ",\"contour_area_px2\":" << contour_area_px2 << ",\"roi_x\":" << roi.x
             << ",\"roi_y\":" << roi.y << ",\"roi_w\":" << roi.width << ",\"roi_h\":" << roi.height << "}";
        if (!send_all(client, "data: " + body.str() + "\n\n")) break;
        sent_sequence = sequence;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    close(client); return;
  }
  if (health) {
    bool found; double position; double fps; uint64_t sequence; int64_t timestamp;
    {
      std::lock_guard lock(frames.mutex);
      found = frames.found;
      position = frames.position_cm;
      fps = frames.processing_fps;
      sequence = frames.sequence;
      timestamp = frames.timestamp_ms;
    }
    std::ostringstream body;
    body << "{\"status\":\"ok\",\"found\":" << (found ? "true" : "false")
         << ",\"position_cm\":" << position << ",\"fps\":" << fps
         << ",\"sequence\":" << sequence << ",\"timestamp_ms\":" << timestamp << "}";
    send_all(client, "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: " +
                     std::to_string(body.str().size()) + "\r\n\r\n" + body.str());
    close(client); return;
  }
  if (snapshot) {
    std::vector<uchar> image;
    { std::lock_guard lock(frames.mutex); image = frames.raw; }
    send_all(client, "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                     std::to_string(image.size()) + "\r\n\r\n");
    send_all(client, image); close(client); return;
  }
  send_all(client, "HTTP/1.1 200 OK\r\nCache-Control: no-cache\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");
  while (running) {
    std::vector<uchar> image;
    { std::lock_guard lock(frames.mutex); image = detected ? frames.detected : frames.raw; }
    if (!image.empty()) {
      if (!send_all(client, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                            std::to_string(image.size()) + "\r\n\r\n") || !send_all(client, image) ||
          !send_all(client, "\r\n")) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / std::max(stream_fps, 1)));
  }
  close(client);
}

void server_loop(const Config& cfg, Frames& frames) {
  const int server = socket(AF_INET, SOCK_STREAM, 0);
  if (server < 0) throw std::runtime_error("Cannot create HTTP socket");
  int yes = 1;
  setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(cfg.port);
  if (bind(server, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 || listen(server, 8) < 0) {
    close(server); throw std::runtime_error("Cannot listen on port " + std::to_string(cfg.port));
  }
  while (running) {
    const int client = accept(server, nullptr, nullptr);
    if (client >= 0) std::thread(serve_client, client, std::ref(frames), cfg.stream_fps).detach();
  }
  close(server);
}

Config parse_args(int argc, char** argv) {
  Config cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--camera") cfg.camera = argv[++i];
    else if (key == "--width") cfg.width = std::stoi(argv[++i]);
    else if (key == "--height") cfg.height = std::stoi(argv[++i]);
    else if (key == "--fps") cfg.fps = std::stoi(argv[++i]);
    else if (key == "--stream-fps") cfg.stream_fps = std::stoi(argv[++i]);
    else if (key == "--port") cfg.port = std::stoi(argv[++i]);
    else if (key == "--roi") { char comma; std::istringstream in(argv[++i]); in >> cfg.roi.x >> comma >> cfg.roi.y >> comma >> cfg.roi.width >> comma >> cfg.roi.height; }
    else if (key == "--threshold") cfg.threshold = std::stoi(argv[++i]);
    else if (key == "--min-area") cfg.min_area = std::stoi(argv[++i]);
    else if (key == "--max-area") cfg.max_area = std::stoi(argv[++i]);
    else if (key == "--max-center-offset") cfg.max_center_offset = std::stoi(argv[++i]);
    else if (key == "--pipe-threshold") cfg.pipe_threshold = std::stoi(argv[++i]);
    else if (key == "--pipe-min-aspect") cfg.pipe_min_aspect = std::stod(argv[++i]);
    else if (key == "--edge-ignore") cfg.edge_ignore = std::stoi(argv[++i]);
  }
  return cfg;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    const Config cfg = parse_args(argc, argv);
    Frames frames;
    std::thread capture(capture_loop, std::cref(cfg), std::ref(frames));
    server_loop(cfg, frames);
    running = false;
    capture.join();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
