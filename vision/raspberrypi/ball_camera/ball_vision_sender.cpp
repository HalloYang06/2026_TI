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
  int port = 8080;
  cv::Rect roi;
  double left_cm = -12.5;
  double right_cm = 12.5;
  int threshold = 200;
  int min_area = 50;
  int max_area = 12000;
  int max_center_offset = 80;
  int pipe_threshold = 200;
  double pipe_min_aspect = 5.5;
  int hough_min_radius = 6;
  int hough_max_radius = 18;
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
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1.2, 18,
                   80, 18, cfg.hough_min_radius, cfg.hough_max_radius);
  if (!circles.empty()) {
    std::optional<cv::Vec3f> best;
    double best_score = std::numeric_limits<double>::max();
    for (const auto& circle : circles) {
      const cv::Point2f point(circle[0] + roi_rect.x, circle[1] + roi_rect.y);
      const cv::Point2f delta = point - axis.centre;
      const double along = delta.dot(axis.direction);
      const double lateral = std::abs(delta.x * -axis.direction.y + delta.y * axis.direction.x);
      if (lateral > axis.half_width || std::abs(along) > axis.length / 2 - cfg.edge_ignore) continue;
      const double fraction = (along + axis.length / 2) / axis.length;
      double score = lateral + 0.5 * std::abs(circle[2] - 10.0F);
      if (previous_fraction) score += 20.0 * std::abs(fraction - *previous_fraction);
      if (score < best_score) {
        best_score = score;
        best = circle;
      }
    }
    if (best) return best;
  }

  // If a bright reflection breaks the circular edge, fall back to the darker
  // connected component inside the already-localised pipe ROI.
  cv::Mat binary;
  cv::threshold(blurred, binary, cfg.threshold, 255, cv::THRESH_BINARY_INV);
  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

  cv::Mat labels, stats, centroids;
  const int label_count = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8);
  int best_label = -1;
  double best_score = std::numeric_limits<double>::max();
  for (int label = 1; label < label_count; ++label) {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    const double x = centroids.at<double>(label, 0);
    const double y = centroids.at<double>(label, 1);
    const cv::Point2f point(x + roi_rect.x, y + roi_rect.y);
    const cv::Point2f delta = point - axis.centre;
    const double along = delta.dot(axis.direction);
    const double lateral = std::abs(delta.x * -axis.direction.y + delta.y * axis.direction.x);
    if (area < cfg.min_area || area > cfg.max_area || lateral > axis.half_width ||
        std::abs(along) > axis.length / 2 - cfg.edge_ignore) continue;
    const double radius = std::sqrt(area / CV_PI);
    const double fraction = (along + axis.length / 2) / axis.length;
    double score = lateral + 0.03 * std::abs(radius - 10.0);
    if (previous_fraction) {
      score += 20.0 * std::abs(fraction - *previous_fraction);
    }
    if (score < best_score) {
      best_score = score;
      best_label = label;
    }
  }
  if (best_label < 0) return std::nullopt;
  const int area = stats.at<int>(best_label, cv::CC_STAT_AREA);
  return cv::Vec3f(static_cast<float>(centroids.at<double>(best_label, 0)),
                   static_cast<float>(centroids.at<double>(best_label, 1)),
                   static_cast<float>(std::sqrt(area / CV_PI)));
}

std::optional<PipeAxis> find_pipe_axis(const cv::Mat& image, const Config& cfg) {
  cv::Mat gray, bright;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, bright, cfg.pipe_threshold, 255, cv::THRESH_BINARY);
  const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, {31, 7});
  cv::morphologyEx(bright, bright, cv::MORPH_CLOSE, close_kernel);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bright, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::optional<cv::RotatedRect> best;
  double best_score = 0.0;
  for (const auto& contour : contours) {
    const cv::RotatedRect rotated = cv::minAreaRect(contour);
    const double long_side = std::max(rotated.size.width, rotated.size.height);
    const double short_side = std::max(1.0F, std::min(rotated.size.width, rotated.size.height));
    const double aspect = long_side / short_side;
    if (aspect < cfg.pipe_min_aspect || long_side < image.cols * 0.45) continue;
    if (long_side * aspect > best_score) {
      best_score = long_side * aspect;
      best = rotated;
    }
  }
  if (!best) return std::nullopt;
  float angle = best->angle;
  if (best->size.width < best->size.height) angle += 90.0F;
  const float radians = angle * static_cast<float>(CV_PI / 180.0);
  return PipeAxis{best->center, {std::cos(radians), std::sin(radians)},
                  std::max(best->size.width, best->size.height),
                  std::min(best->size.width, best->size.height) / 2 + 8.0F};
}

std::optional<cv::Rect> find_pipe_roi(const cv::Mat& image, const Config& cfg) {
  cv::Mat gray, bright;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, bright, cfg.pipe_threshold, 255, cv::THRESH_BINARY);
  const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, {31, 7});
  cv::morphologyEx(bright, bright, cv::MORPH_CLOSE, close_kernel);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bright, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  cv::Rect best;
  double best_score = 0.0;
  for (const auto& contour : contours) {
    const cv::RotatedRect rotated = cv::minAreaRect(contour);
    const double long_side = std::max(rotated.size.width, rotated.size.height);
    const double short_side = std::max(1.0F, std::min(rotated.size.width, rotated.size.height));
    const double aspect = long_side / short_side;
    if (aspect < cfg.pipe_min_aspect || long_side < image.cols * 0.45) continue;
    const cv::Rect candidate = cv::boundingRect(contour) & cv::Rect(0, 0, image.cols, image.rows);
    const double score = long_side * aspect;
    if (score > best_score) {
      best_score = score;
      best = candidate;
    }
  }
  if (best.empty()) return std::nullopt;
  const int pad_x = 8;
  const int pad_y = 8;
  const cv::Rect padded(best.x - pad_x, best.y - pad_y,
                        best.width + pad_x * 2, best.height + pad_y * 2);
  return padded & cv::Rect(0, 0, image.cols, image.rows);
}

void annotate(cv::Mat& image, const Config& cfg, Frames& frames, double processing_fps,
              uint64_t capture_time_us) {
  const auto processing_started = std::chrono::steady_clock::now();
  const cv::Rect image_rect(0, 0, image.cols, image.rows);
  const cv::Rect fallback_roi = cfg.roi.area() ? (cfg.roi & image_rect) : image_rect;
  static cv::Rect tracked_roi;
  static std::optional<PipeAxis> tracked_axis;
  static std::optional<double> previous_fraction;
  static int missed_frames = 0;
  static int pipe_refresh_counter = 0;
  std::optional<cv::Rect> measured_roi;
  std::optional<PipeAxis> measured_axis;
  if (pipe_refresh_counter++ % 8 == 0 || tracked_roi.empty() || !tracked_axis) {
    measured_roi = find_pipe_roi(image, cfg);
    measured_axis = find_pipe_axis(image, cfg);
  }
  if (measured_roi) {
    if (tracked_roi.empty() || std::abs(measured_roi->x - tracked_roi.x) > tracked_roi.width / 3 ||
        std::abs(measured_roi->y - tracked_roi.y) > tracked_roi.height / 2) {
      tracked_roi = *measured_roi;
    } else {
      constexpr double alpha = 0.2;
      tracked_roi.x = cvRound((1 - alpha) * tracked_roi.x + alpha * measured_roi->x);
      tracked_roi.y = cvRound((1 - alpha) * tracked_roi.y + alpha * measured_roi->y);
      tracked_roi.width = cvRound((1 - alpha) * tracked_roi.width + alpha * measured_roi->width);
      tracked_roi.height = cvRound((1 - alpha) * tracked_roi.height + alpha * measured_roi->height);
    }
  } else if (tracked_roi.empty()) {
    tracked_roi = fallback_roi;
  }
  const cv::Rect roi = tracked_roi & image_rect;
  if (measured_axis) tracked_axis = measured_axis;
  const PipeAxis axis = tracked_axis.value_or(
      PipeAxis{{roi.x + roi.width / 2.0F, roi.y + roi.height / 2.0F}, {1.0F, 0.0F},
               static_cast<float>(roi.width), static_cast<float>(roi.height) / 2.0F});
  cv::rectangle(image, roi, cv::Scalar(255, 180, 0), 2);
  const cv::Point2f normal(-axis.direction.y, axis.direction.x);
  for (const float end : {-axis.length / 2 + cfg.edge_ignore, axis.length / 2 - cfg.edge_ignore}) {
    const cv::Point2f point = axis.centre + axis.direction * end;
    cv::line(image, point - normal * axis.half_width, point + normal * axis.half_width,
             cv::Scalar(0, 165, 255), 2);
  }
  const auto circle = find_ball(image(roi), cfg, previous_fraction, axis, roi);
  bool found = false;
  double position_cm = 0.0;
  float center_x_px = 0.0F;
  float center_y_px = 0.0F;
  float radius_px = 0.0F;
  float contour_area_px2 = 0.0F;
  if (circle) {
    const cv::Point centre(cvRound((*circle)[0]) + roi.x, cvRound((*circle)[1]) + roi.y);
    const int radius = cvRound((*circle)[2]);
    cv::circle(image, centre, radius, cv::Scalar(0, 255, 0), 3);
    cv::circle(image, centre, 2, cv::Scalar(0, 0, 255), 3);
    const cv::Point2f delta = cv::Point2f(centre) - axis.centre;
    const double fraction = std::clamp(static_cast<double>(delta.dot(axis.direction) + axis.length / 2) /
                                           axis.length,
                                       0.0, 1.0);
    position_cm = cfg.left_cm + fraction * (cfg.right_cm - cfg.left_cm);
    previous_fraction = previous_fraction ? 0.7 * *previous_fraction + 0.3 * fraction : fraction;
    missed_frames = 0;
    std::ostringstream text;
    text.setf(std::ios::fixed);
    text.precision(2);
    text << "steel ball: " << position_cm << " cm";
    cv::putText(image, text.str(), {roi.x + 8, std::max(28, roi.y - 10)},
                cv::FONT_HERSHEY_SIMPLEX, 0.75, {0, 255, 0}, 2);
    found = true;
    center_x_px = static_cast<float>(centre.x);
    center_y_px = static_cast<float>(centre.y);
    radius_px = (*circle)[2];
    contour_area_px2 = static_cast<float>(CV_PI * radius_px * radius_px);
  } else {
    if (++missed_frames > 5) previous_fraction.reset();
    cv::putText(image, "steel ball: not found", {roi.x + 8, std::max(28, roi.y - 10)},
                cv::FONT_HERSHEY_SIMPLEX, 0.75, {0, 0, 255}, 2);
  }
  std::ostringstream fps_text;
  fps_text.setf(std::ios::fixed);
  fps_text.precision(1);
  fps_text << "FPS: " << processing_fps;
  cv::putText(image, fps_text.str(), {roi.x + 8, std::min(image.rows - 10, roi.y + roi.height + 24)},
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
  frames.roi = roi;
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
    cv::Mat detected = raw.clone();
    annotate(detected, cfg, frames, processing_fps, capture_time_us);
    std::vector<uchar> raw_jpeg, detected_jpeg;
    cv::imencode(".jpg", raw, raw_jpeg, params);
    cv::imencode(".jpg", detected, detected_jpeg, params);
    std::lock_guard lock(frames.mutex);
    frames.raw = std::move(raw_jpeg);
    frames.detected = std::move(detected_jpeg);
    frames.sequence++;
    frames.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
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
    if (client >= 0) std::thread(serve_client, client, std::ref(frames), cfg.fps).detach();
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
    else if (key == "--port") cfg.port = std::stoi(argv[++i]);
    else if (key == "--roi") { char comma; std::istringstream in(argv[++i]); in >> cfg.roi.x >> comma >> cfg.roi.y >> comma >> cfg.roi.width >> comma >> cfg.roi.height; }
    else if (key == "--left-cm") cfg.left_cm = std::stod(argv[++i]);
    else if (key == "--right-cm") cfg.right_cm = std::stod(argv[++i]);
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
