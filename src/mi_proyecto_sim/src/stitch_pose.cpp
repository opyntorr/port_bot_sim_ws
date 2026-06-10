// stitch_pose.cpp — Port C++ del stitcher por pose (mision/stitch_pose.py).
//
// Mosaico aéreo incremental del dron: undistort -> proyección a suelo (por altitud)
// -> rotación por yaw -> máscara de peso (coseno) -> corrección fina de offset por
// features (SIFT/ORB + BFMatcher) -> blending multi-banda (pirámide Laplaciana) ->
// binarización HSV a occupancy -> mapa ROS (PGM+YAML) + visualizaciones.
//
// Es un reemplazo "drop-in" de `python3 -m mision.stitch_pose`: mismos argumentos
// y mismos archivos de salida, pero en C++ (mucho más rápido que las ops numpy/python).
//
// Uso:
//   stitch_pose --input DIR --output DIR [--camera YAML] [--resolution 0.002]
//               [--margin 0.5] [--levels 4] [--no-features] [--map-name occupancy_map]
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using json = nlohmann::json;

// ───────────────────────── etiquetas semánticas (= blend_vote) ──────────────
static constexpr unsigned char L_UNKNOWN = 0, L_FREE = 1, L_OCCUPIED = 2;

// ───────────────────────── parsing de argumentos ───────────────────────────
struct Args {
  std::string input, output, camera;
  double resolution = 0.002, margin = 0.5;
  int levels = 4;
  bool no_features = false;
  bool use_sift = false;   // --sift -> usa SIFT en vez de phaseCorrelate
  std::string map_name = "occupancy_map";
};

static Args parse_args(int argc, char** argv) {
  Args a;
  std::map<std::string, std::string> kv;
  std::vector<std::string> flags;
  for (int i = 1; i < argc; ++i) {
    std::string s = argv[i];
    if (s.rfind("--", 0) == 0) {
      if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
        kv[s] = argv[++i];
      } else {
        flags.push_back(s);  // flag booleana (o compat)
      }
    }
  }
  auto get = [&](const std::string& k, const std::string& d) {
    auto it = kv.find(k); return it == kv.end() ? d : it->second;
  };
  a.input  = get("--input", "");
  a.output = get("--output", "");
  a.camera = get("--camera", "");
  a.resolution = std::stod(get("--resolution", "0.002"));
  a.margin     = std::stod(get("--margin", "0.5"));
  a.levels     = std::stoi(get("--levels", "4"));
  a.map_name   = get("--map-name", "occupancy_map");
  for (auto& f : flags) { if (f == "--no-features") a.no_features = true; if (f == "--sift") a.use_sift = true; }
  if (a.input.empty() || a.output.empty()) {
    std::cerr << "[stitcher] faltan --input/--output\n";
    std::exit(1);
  }
  return a;
}

// ───────────────────────── cámara (YAML ROS) ────────────────────────────────
struct Camera { cv::Mat K, D; double cam_rot_rad = 0.0; };

static Camera load_camera_yaml(const std::string& path) {
  YAML::Node n = YAML::LoadFile(path);
  Camera c;
  auto km = n["camera_matrix"]["data"];
  c.K = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) c.K.at<double>(i / 3, i % 3) = km[i].as<double>();
  auto dd = n["distortion_coefficients"]["data"];
  c.D = cv::Mat(1, (int)dd.size(), CV_64F);
  for (size_t i = 0; i < dd.size(); ++i) c.D.at<double>(0, (int)i) = dd[i].as<double>();
  double rot_deg = n["camera_rotation_deg"] ? n["camera_rotation_deg"].as<double>() : 0.0;
  c.cam_rot_rad = rot_deg * CV_PI / 180.0;
  return c;
}

// ───────────────────────── etapas del pipeline ──────────────────────────────
static void undistort_img(const cv::Mat& img, const cv::Mat& K, const cv::Mat& D,
                          cv::Mat& out, cv::Mat& newK) {
  newK = cv::getOptimalNewCameraMatrix(K, D, img.size(), 0.0);
  cv::undistort(img, out, K, D, newK);
}

static cv::Mat project_to_ground(const cv::Mat& img, const cv::Mat& newK,
                                 double altitude, double m_per_px) {
  double fx = newK.at<double>(0, 0), fy = newK.at<double>(1, 1);
  double cx = newK.at<double>(0, 2), cy = newK.at<double>(1, 2);
  double width_m  = altitude * img.cols / fx;
  double height_m = altitude * img.rows / fy;
  int w_out = std::max(1, (int)std::lround(width_m  / m_per_px));
  int h_out = std::max(1, (int)std::lround(height_m / m_per_px));
  cv::Mat map_x(h_out, w_out, CV_32F), map_y(h_out, w_out, CV_32F);
  for (int v = 0; v < h_out; ++v) {
    float* mx = map_x.ptr<float>(v);
    float* my = map_y.ptr<float>(v);
    double yg = (v - h_out / 2.0) * m_per_px;
    float myv = (float)(fy * (yg / altitude) + cy);
    for (int u = 0; u < w_out; ++u) {
      double xg = (u - w_out / 2.0) * m_per_px;
      mx[u] = (float)(fx * (xg / altitude) + cx);
      my[u] = myv;
    }
  }
  cv::Mat ground;
  cv::remap(img, ground, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
  return ground;
}

static cv::Mat rotate_image(const cv::Mat& img, double yaw_rad) {
  int h = img.rows, w = img.cols;
  cv::Mat M = cv::getRotationMatrix2D(cv::Point2f(w / 2.0f, h / 2.0f),
                                      yaw_rad * 180.0 / CV_PI, 1.0);
  double cosv = std::abs(M.at<double>(0, 0)), sinv = std::abs(M.at<double>(0, 1));
  int nw = (int)(h * sinv + w * cosv), nh = (int)(h * cosv + w * sinv);
  M.at<double>(0, 2) += nw / 2.0 - w / 2.0;
  M.at<double>(1, 2) += nh / 2.0 - h / 2.0;
  cv::Mat rot;
  cv::warpAffine(img, rot, M, cv::Size(nw, nh), cv::INTER_LINEAR,
                 cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
  return rot;
}

static cv::Mat distance_weight_mask(int h, int w) {
  cv::Mat m(h, w, CV_32F);
  for (int j = 0; j < h; ++j) {
    float y = (h > 1) ? (-1.0f + 2.0f * j / (h - 1)) : 0.0f;
    float* row = m.ptr<float>(j);
    for (int i = 0; i < w; ++i) {
      float x = (w > 1) ? (-1.0f + 2.0f * i / (w - 1)) : 0.0f;
      float d = std::min(1.0f, std::sqrt(x * x + y * y));
      row[i] = std::cos(d * (float)(CV_PI / 2.0));
    }
  }
  return m;
}

// content mask (cualquier canal != 0) como float 0/1, y multiplica wmask
static void apply_content(cv::Mat& wmask, const cv::Mat& bgr) {
  for (int j = 0; j < bgr.rows; ++j) {
    const cv::Vec3b* p = bgr.ptr<cv::Vec3b>(j);
    float* w = wmask.ptr<float>(j);
    for (int i = 0; i < bgr.cols; ++i)
      if (p[i][0] == 0 && p[i][1] == 0 && p[i][2] == 0) w[i] = 0.0f;
  }
}

// ───────────────────────── corrección de offset por features ────────────────
static cv::Ptr<cv::Feature2D> g_det;
static cv::Ptr<cv::BFMatcher>  g_matcher;
static std::string g_det_name;
static bool g_use_sift = false;   // false = phaseCorrelate (rápido, default); true = SIFT (fiel al Python)

static void build_detector() {
  try {
    g_det = cv::SIFT::create(3000);
    g_matcher = cv::BFMatcher::create(cv::NORM_L2, false);
    g_det_name = "SIFT";
  } catch (...) {
    try {
      g_det = cv::ORB::create(3000);
      g_matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);
      g_det_name = "ORB";
    } catch (...) { g_det = nullptr; }
  }
}

static cv::Point feature_offset(const cv::Mat& canvas_f32, const cv::Mat& weight_sum,
                                const cv::Mat& patch, int u0, int v0) {
  const int MAX_OFFSET = 25, MIN_OVERLAP_PX = 2000, MIN_MATCHES = 10;
  int Hc = canvas_f32.rows, Wc = canvas_f32.cols, hp = patch.rows, wp = patch.cols;
  int cu0 = std::max(0, u0), cv0 = std::max(0, v0);
  int cu1 = std::min(Wc, u0 + wp), cv1 = std::min(Hc, v0 + hp);
  if (cu0 >= cu1 || cv0 >= cv1) return {0, 0};
  cv::Rect roi(cu0, cv0, cu1 - cu0, cv1 - cv0);
  cv::Mat overlap_w = weight_sum(roi);
  double wmax; cv::minMaxLoc(overlap_w, nullptr, &wmax);
  if (wmax < 0.05) return {0, 0};

  cv::Mat patch_crop = patch(cv::Rect(cu0 - u0, cv0 - v0, roi.width, roi.height));
  cv::Mat ref_crop;
  cv::min(cv::max(canvas_f32(roi), 0.0), 255.0, ref_crop);
  ref_crop.convertTo(ref_crop, CV_8UC3);

  // máscara overlap (patch con contenido) AND (peso de referencia > 0.05)
  cv::Mat mask_new(roi.height, roi.width, CV_8U), mask_ref(roi.height, roi.width, CV_8U);
  for (int j = 0; j < roi.height; ++j) {
    const cv::Vec3b* pc = patch_crop.ptr<cv::Vec3b>(j);
    const float* ow = overlap_w.ptr<float>(j);
    unsigned char* mn = mask_new.ptr<unsigned char>(j);
    unsigned char* mr = mask_ref.ptr<unsigned char>(j);
    for (int i = 0; i < roi.width; ++i) {
      mn[i] = (pc[i][0] || pc[i][1] || pc[i][2]) ? 255 : 0;
      mr[i] = (ow[i] > 0.05f) ? 255 : 0;
    }
  }
  cv::Mat mask_both; cv::bitwise_and(mask_new, mask_ref, mask_both);
  if ((int)(cv::countNonZero(mask_both)) < MIN_OVERLAP_PX) return {0, 0};

  cv::Mat gray_ref, gray_new;
  cv::cvtColor(ref_crop,   gray_ref, cv::COLOR_BGR2GRAY);
  cv::cvtColor(patch_crop, gray_new, cv::COLOR_BGR2GRAY);

  // --- ruta rápida: traslación por correlación de fase (FFT) sobre el solape ---
  if (!g_use_sift) {
    cv::Mat rf, nf;
    gray_ref.convertTo(rf, CV_32F);
    gray_new.convertTo(nf, CV_32F);
    rf.setTo(0, mask_both == 0);   // anula fuera del solape para no sesgar la fase
    nf.setTo(0, mask_both == 0);
    // downscale (lado mayor <= 256) -> FFT rápido; el offset es coarse (cap ±25px), low-res basta
    double scale = std::min(1.0, 256.0 / std::max(rf.rows, rf.cols));
    if (scale < 1.0) {
      cv::resize(rf, rf, cv::Size(), scale, scale, cv::INTER_AREA);
      cv::resize(nf, nf, cv::Size(), scale, scale, cv::INTER_AREA);
    }
    cv::Mat hann; cv::createHanningWindow(hann, rf.size(), CV_32F);
    cv::Point2d sh = cv::phaseCorrelate(nf, rf, hann);   // patch -> ref
    double du = sh.x / scale, dv = sh.y / scale;          // de vuelta a resolución completa
    if (std::abs(du) > MAX_OFFSET || std::abs(dv) > MAX_OFFSET) return {0, 0};
    return {(int)std::lround(du), (int)std::lround(dv)};
  }

  // --- ruta fiel: SIFT/ORB + BFMatcher (mediana de deltas), como el Python ---
  std::vector<cv::KeyPoint> kp1, kp2; cv::Mat des1, des2;
  g_det->detectAndCompute(gray_ref, mask_both, kp1, des1);
  g_det->detectAndCompute(gray_new, mask_both, kp2, des2);
  if (des1.empty() || des2.empty() || (int)kp1.size() < MIN_MATCHES ||
      (int)kp2.size() < MIN_MATCHES) return {0, 0};

  std::vector<std::vector<cv::DMatch>> knn;
  g_matcher->knnMatch(des1, des2, knn, 2);
  std::vector<float> dus, dvs;
  for (auto& mm : knn) {
    if (mm.size() < 2) continue;
    if (mm[0].distance < 0.75f * mm[1].distance) {
      const cv::Point2f& p1 = kp1[mm[0].queryIdx].pt;
      const cv::Point2f& p2 = kp2[mm[0].trainIdx].pt;
      dus.push_back(p1.x - p2.x);
      dvs.push_back(p1.y - p2.y);
    }
  }
  if ((int)dus.size() < MIN_MATCHES) return {0, 0};
  std::nth_element(dus.begin(), dus.begin() + dus.size() / 2, dus.end());
  std::nth_element(dvs.begin(), dvs.begin() + dvs.size() / 2, dvs.end());
  double du = dus[dus.size() / 2], dv = dvs[dvs.size() / 2];
  if (std::abs(du) > MAX_OFFSET || std::abs(dv) > MAX_OFFSET) return {0, 0};
  return {(int)std::lround(du), (int)std::lround(dv)};
}

// ───────────────────────── blending multi-banda (Laplaciano) ────────────────
static std::vector<cv::Mat> gaussian_pyr(const cv::Mat& img, int lvls) {
  std::vector<cv::Mat> pyr{img.clone()};
  cv::Mat cur = img.clone();
  for (int i = 0; i < lvls - 1; ++i) { cv::pyrDown(cur, cur); pyr.push_back(cur.clone()); }
  return pyr;
}
static std::vector<cv::Mat> laplacian_pyr(const cv::Mat& img, int lvls) {
  auto g = gaussian_pyr(img, lvls);
  std::vector<cv::Mat> lp;
  for (int i = 0; i < lvls - 1; ++i) {
    cv::Mat up; cv::pyrUp(g[i + 1], up, g[i].size());
    lp.push_back(g[i] - up);
  }
  lp.push_back(g[lvls - 1].clone());
  return lp;
}

static void multiband_blend(cv::Mat& canvas_f32, cv::Mat& weight_sum, const cv::Mat& patch,
                            const cv::Mat& weight_mask, int u0, int v0, int num_levels) {
  int Hc = canvas_f32.rows, Wc = canvas_f32.cols, hp = patch.rows, wp = patch.cols;
  int cu0 = std::max(0, u0), cv0 = std::max(0, v0);
  int cu1 = std::min(Wc, u0 + wp), cv1 = std::min(Hc, v0 + hp);
  if (cu0 >= cu1 || cv0 >= cv1) return;
  cv::Rect roi(cu0, cv0, cu1 - cu0, cv1 - cv0);
  cv::Rect proi(cu0 - u0, cv0 - v0, roi.width, roi.height);

  cv::Mat A = canvas_f32(roi).clone();
  cv::Mat B; patch(proi).convertTo(B, CV_32FC3);
  cv::Mat w_old = weight_sum(roi).clone();           // CV_32F
  cv::Mat w_new = weight_mask(proi).clone();         // CV_32F

  cv::Mat w_total = w_old + w_new;
  cv::Mat alpha(roi.height, roi.width, CV_32F);
  // A_init = where(w_old<1e-6, B, A)
  cv::Mat A_init = A.clone();
  for (int j = 0; j < roi.height; ++j) {
    const float* wt = w_total.ptr<float>(j);
    const float* wn = w_new.ptr<float>(j);
    const float* wo = w_old.ptr<float>(j);
    float* al = alpha.ptr<float>(j);
    const cv::Vec3f* b = B.ptr<cv::Vec3f>(j);
    cv::Vec3f* ai = A_init.ptr<cv::Vec3f>(j);
    for (int i = 0; i < roi.width; ++i) {
      al[i] = (wt[i] > 1e-8f) ? (wn[i] / wt[i]) : 0.0f;
      if (wo[i] < 1e-6f) ai[i] = b[i];
    }
  }
  cv::Mat alpha3; cv::cvtColor(alpha, alpha3, cv::COLOR_GRAY2BGR);  // 3 canales iguales

  int max_levels = std::max(1, (int)std::log2(std::min(roi.height, roi.width) + 1));
  int levels = std::min(num_levels, max_levels);

  auto lp_A = laplacian_pyr(A_init, levels);
  auto lp_B = laplacian_pyr(B, levels);
  auto gp_a = gaussian_pyr(alpha3, levels);

  std::vector<cv::Mat> blended(levels);
  for (int i = 0; i < levels; ++i)
    blended[i] = lp_A[i].mul(cv::Scalar::all(1.0) - gp_a[i]) + lp_B[i].mul(gp_a[i]);

  cv::Mat result = blended[levels - 1];
  for (int i = levels - 2; i >= 0; --i) {
    cv::Mat up; cv::pyrUp(result, up, blended[i].size());
    result = up + blended[i];
  }
  cv::min(cv::max(result, 0.0), 255.0, result);
  result.copyTo(canvas_f32(roi));
  cv::Mat ws = weight_sum(roi); ws += w_new;
}

// ───────────────────────── salida mapa ROS + grid vis ───────────────────────
static void write_ros_map(const cv::Mat& pgm, const fs::path& out_dir, const std::string& name,
                          double res, double ox, double oy) {
  cv::imwrite((out_dir / (name + ".pgm")).string(), pgm);
  std::ofstream f((out_dir / (name + ".yaml")).string());
  f << "image: " << name << ".pgm\n"
    << "mode: trinary\n"
    << "resolution: " << res << "\n"
    << "origin: [" << ox << ", " << oy << ", 0.0]\n"
    << "negate: 0\n"
    << "occupied_thresh: 0.65\n"
    << "free_thresh: 0.196\n";
}

static cv::Mat label_to_grid_vis(const cv::Mat& label) {
  cv::Mat out(label.size(), CV_8U, cv::Scalar(255));
  out.setTo(0, label == L_OCCUPIED);
  cv::Mat bgr; cv::cvtColor(out, bgr, cv::COLOR_GRAY2BGR);
  cv::rectangle(bgr, cv::Point(0, 0), cv::Point(bgr.cols - 1, bgr.rows - 1), cv::Scalar(0,0,0), 3);
  return bgr;
}

// ───────────────────────────────── main ─────────────────────────────────────
int main(int argc, char** argv) {
  Args args = parse_args(argc, argv);
  fs::path in_dir(args.input), out_dir(args.output);
  fs::create_directories(out_dir);

  std::vector<std::string> photos;
  for (auto& e : fs::directory_iterator(in_dir))
    if (e.path().extension() == ".png") photos.push_back(e.path().string());
  std::sort(photos.begin(), photos.end());
  if (photos.empty()) { std::cerr << "[stitcher] no .png en " << in_dir << "\n"; return 1; }

  // detectar config de cámara si no se pasó
  std::string camera_yaml = args.camera;
  if (camera_yaml.empty()) {
    fs::path meta0 = fs::path(photos[0]).replace_extension(".json");
    std::string pose_src;
    if (fs::exists(meta0)) { std::ifstream jf(meta0); json j; jf >> j; pose_src = j.value("pose_src", ""); }
    fs::path cfg = fs::path(args.input).has_parent_path() ? "" : "";
    // config/ vive en el paquete; lo deducimos relativo al cwd del proceso (src/mi_proyecto_sim)
    std::string base = (pose_src == "odometria" || args.input.find("sim") != std::string::npos)
                         ? "config/camera_tello_sim.yaml" : "config/camera_tello.yaml";
    camera_yaml = base;
  }
  std::cout << "[stitcher] Config de camara: " << camera_yaml << "\n";
  Camera cam = load_camera_yaml(camera_yaml);

  // ── 1. proyectar + rotar + weight masks ──
  std::vector<cv::Mat> projected, weight_masks;
  std::vector<std::array<double, 2>> centers;  // (x,y) mundo
  for (auto& ph : photos) {
    fs::path meta = fs::path(ph).replace_extension(".json");
    if (!fs::exists(meta)) continue;
    json m; { std::ifstream jf(meta); jf >> m; }
    cv::Mat img = cv::imread(ph);
    if (img.empty()) continue;
    double altitude = m.value("z", 0.0);
    if (altitude <= 0.1) continue;
    cv::Mat und, newK; undistort_img(img, cam.K, cam.D, und, newK);
    cv::Mat ground = project_to_ground(und, newK, altitude, args.resolution);
    // yaw puede venir null (poses "nominal"); en ese caso usa yaw_deg.
    double yaw = 0.0;
    if (m.contains("yaw") && !m["yaw"].is_null()) yaw = m["yaw"].get<double>();
    else if (m.contains("yaw_deg") && !m["yaw_deg"].is_null()) yaw = m["yaw_deg"].get<double>() * CV_PI / 180.0;
    cv::Mat rotated = rotate_image(ground, yaw + cam.cam_rot_rad);
    cv::Mat wmask = distance_weight_mask(rotated.rows, rotated.cols);
    apply_content(wmask, rotated);
    projected.push_back(rotated);
    weight_masks.push_back(wmask);
    centers.push_back({m.value("x", 0.0), m.value("y", 0.0)});
  }
  if (projected.empty()) { std::cerr << "[stitcher] sin fotos validas\n"; return 2; }

  // ── 2. canvas global ──
  double x_min = 1e18, x_max = -1e18, y_min = 1e18, y_max = -1e18;
  for (size_t i = 0; i < projected.size(); ++i) {
    double hw = projected[i].cols * args.resolution / 2.0;
    double hh = projected[i].rows * args.resolution / 2.0;
    x_min = std::min(x_min, centers[i][0] - hw); x_max = std::max(x_max, centers[i][0] + hw);
    y_min = std::min(y_min, centers[i][1] - hh); y_max = std::max(y_max, centers[i][1] + hh);
  }
  x_min -= args.margin; x_max += args.margin; y_min -= args.margin; y_max += args.margin;
  int W = (int)std::ceil((x_max - x_min) / args.resolution);
  int H = (int)std::ceil((y_max - y_min) / args.resolution);
  cv::Mat canvas_f32 = cv::Mat::zeros(H, W, CV_32FC3);
  cv::Mat weight_sum = cv::Mat::zeros(H, W, CV_32F);

  // ── 3. método de corrección de offset ──
  g_use_sift = args.use_sift;
  bool offsets = !args.no_features;
  if (offsets && g_use_sift) { build_detector(); if (g_det) std::cout << "[stitcher] Offset: feature detector " << g_det_name << "\n"; }
  else if (offsets) std::cout << "[stitcher] Offset: phase correlation (rapido)\n";

  // ── 4. pegar fotos ──
  for (size_t i = 0; i < projected.size(); ++i) {
    cv::Mat& img = projected[i];
    int cu     = (int)std::lround((centers[i][0] - x_min) / args.resolution);
    int cv_pix = (int)std::lround((y_max - centers[i][1]) / args.resolution);
    int u0 = cu - img.cols / 2, v0 = cv_pix - img.rows / 2;
    double wmax; cv::minMaxLoc(weight_sum, nullptr, &wmax);
    if (offsets && wmax > 0.05) {
      cv::Point d = feature_offset(canvas_f32, weight_sum, img, u0, v0);
      if (d.x || d.y) std::cout << "[stitcher] features offset (" << d.x << ", " << d.y << ") px\n";
      u0 += d.x; v0 += d.y;
    }
    multiband_blend(canvas_f32, weight_sum, img, weight_masks[i], u0, v0, args.levels);
  }

  // ── 5. mosaico + binarización ──
  cv::Mat canvas; cv::min(cv::max(canvas_f32, 0.0), 255.0, canvas); canvas.convertTo(canvas, CV_8UC3);
  cv::imwrite((out_dir / "mosaic_pose.png").string(), canvas);

  cv::Mat hsv; cv::cvtColor(canvas, hsv, cv::COLOR_BGR2HSV);
  cv::Mat covered = weight_sum > 0;  // CV_8U 0/255
  cv::Mat occ(canvas.size(), CV_8U, cv::Scalar(205));
  occ.setTo(254, covered);
  cv::Mat obs_box, obs_wall, obs_cage;
  // Cajas y paredes (vistas como cian/azul debido a RGB->BGR invertido)
  cv::inRange(hsv, cv::Scalar(90, 50, 40), cv::Scalar(130, 255, 255), obs_box);
  obs_wall = cv::Mat::zeros(obs_box.size(), CV_8U);
  obs_cage = cv::Mat::zeros(obs_box.size(), CV_8U);
  cv::Mat obstacles; cv::bitwise_or(obs_box, obs_wall, obstacles); cv::bitwise_or(obstacles, obs_cage, obstacles);

  cv::Mat k3 = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(obstacles, obstacles, cv::MORPH_OPEN, k3);
  cv::morphologyEx(obstacles, obstacles, cv::MORPH_CLOSE, k3, cv::Point(-1,-1), 2);

  cv::Mat occ_mask; cv::bitwise_and(covered, obstacles, occ_mask);
  occ.setTo(0, occ_mask);
  cv::Mat label = occ;

  // ── 6. mapas ROS + vis ──
  double world_x_left = x_min, world_y_bottom = y_max - H * args.resolution;
  cv::Mat pgm_flipped; cv::flip(label, pgm_flipped, 0);
  write_ros_map(pgm_flipped, out_dir, args.map_name, args.resolution, world_x_left, world_y_bottom);
  cv::imwrite((out_dir / (args.map_name + "_vis.png")).string(), label);

  cv::Mat semantic(label.size(), CV_8U, cv::Scalar(L_UNKNOWN));
  semantic.setTo(L_FREE,     label == 254);
  semantic.setTo(L_OCCUPIED, label == 0);
  cv::Mat grid = label_to_grid_vis(semantic);
  cv::imwrite((out_dir / (args.map_name + "_grid.png")).string(), grid);

  cv::Mat map_bgr; cv::cvtColor(label, map_bgr, cv::COLOR_GRAY2BGR);
  cv::Mat overlay; cv::addWeighted(map_bgr, 0.5, canvas, 0.5, 0, overlay);
  cv::imwrite((out_dir / (args.map_name + "_overlay.png")).string(), overlay);

  std::cout << "[stitcher] Wrote " << (out_dir / (args.map_name + ".pgm")).string()
            << " (" << W << "x" << H << ", res=" << args.resolution << " m/px)\n"
            << "[stitcher] Done. Outputs in " << out_dir.string() << "\n";
  return 0;
}
