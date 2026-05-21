#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>

// TF2 Headers
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;
using namespace cv;

struct RRTNode {
  Point pos;
  int parent_id;
  RRTNode(Point p, int id) : pos(p), parent_id(id) {}
};

double get_dist(Point p1, Point p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

bool check_collision(const Mat &map, Point p1, Point p2) {
  LineIterator it(map, p1, p2, 8);
  for (int i = 0; i < it.count; i++, ++it) {
    if (map.at<uchar>(it.pos()) < 200)
      return true;
  }
  return false;
}

vector<Point> smooth_path(const Mat &map, const vector<Point> &raw_path) {
  if (raw_path.size() <= 2)
    return raw_path;

  vector<Point> smoothed;
  smoothed.push_back(raw_path[0]);

  int current_idx = 0;
  while (current_idx < raw_path.size() - 1) {
    int furthest_visible = current_idx + 1;
    for (int j = raw_path.size() - 1; j > current_idx; j--) {
      if (!check_collision(map, raw_path[current_idx], raw_path[j])) {
        furthest_visible = j;
        break;
      }
    }
    smoothed.push_back(raw_path[furthest_visible]);
    current_idx = furthest_visible;
  }
  return smoothed;
}

vector<Point> densify_path(const vector<Point>& path, double max_dist_px) {
  if (path.size() < 2) return path;
  vector<Point> dense_path;
  for (size_t i = 0; i < path.size() - 1; ++i) {
    dense_path.push_back(path[i]);
    double d = get_dist(path[i], path[i+1]);
    if (d > max_dist_px) {
      int num_segments = ceil(d / max_dist_px);
      for (int j = 1; j < num_segments; ++j) {
        double t = static_cast<double>(j) / num_segments;
        dense_path.push_back(Point(
          path[i].x + t * (path[i+1].x - path[i].x),
          path[i].y + t * (path[i+1].y - path[i].y)
        ));
      }
    }
  }
  dense_path.push_back(path.back());
  return dense_path;
}

class RRTRosNode : public rclcpp::Node {
public:
  RRTRosNode() : Node("planificador_rrt", 
                      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/rrt_path", custom_qos);

    // Suscriptor al mapa dinámico (SLAM)
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", custom_qos, std::bind(&RRTRosNode::map_callback, this, std::placeholders::_1));

    // Suscriptor al mapa estático del dron
    map_dron_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map_dron", custom_qos, std::bind(&RRTRosNode::map_dron_callback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    replan_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/replan_request", 10, std::bind(&RRTRosNode::replan_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RRTRosNode::timer_callback, this));
  }

  // Conversión mundo → pixel (en Mat volteado de OpenCV)
  Point world_to_pixel(double wx, double wy) {
    int col = static_cast<int>((wx - map_origin_x_) / map_resolution_);
    int row = map_height_ - 1 - static_cast<int>((wy - map_origin_y_) / map_resolution_);
    return Point(col, row);
  }

  // Conversión pixel → mundo
  void pixel_to_world(const Point& p, double& wx, double& wy) {
    wx = p.x * map_resolution_ + map_origin_x_;
    wy = (map_height_ - 1 - p.y) * map_resolution_ + map_origin_y_;
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Guardar metadatos del mapa para conversiones
    map_resolution_ = msg->info.resolution;
    map_origin_x_ = msg->info.origin.position.x;
    map_origin_y_ = msg->info.origin.position.y;
    map_height_ = msg->info.height;

    RCLCPP_INFO(this->get_logger(), "Mapa recibido: %ux%u @ %.3f m/px, origen (%.2f, %.2f)",
      msg->info.width, msg->info.height, map_resolution_, map_origin_x_, map_origin_y_);
    
    int w = msg->info.width;
    int h = msg->info.height;
    Mat temp_map(h, w, CV_8UC1);

    for (int i = 0; i < h * w; i++) {
        int8_t val = msg->data[i];
        if (val >= 0 && val <= 30) temp_map.data[i] = 255;   // Libre
        else if (val > 60) temp_map.data[i] = 0;             // Ocupado
        else temp_map.data[i] = 128;                         // Desconocido
    }

    flip(temp_map, map_original_, 0);

    // Radio del robot en pixels (parametrizado desde el launch)
    double robot_radius_m = this->get_parameter_or("robot_radius_m", 0.19);
    int robot_radius = static_cast<int>(ceil(robot_radius_m / map_resolution_));
    Mat kernel = getStructuringElement(MORPH_RECT, Size(robot_radius * 2, robot_radius * 2));
    erode(map_original_, map_inflated_, kernel);
    
    // Mapa relajado (1/4 de las medidas originales)
    int relaxed_radius = max(1, static_cast<int>(ceil((robot_radius_m / 4.0) / map_resolution_)));
    Mat kernel_relaxed = getStructuringElement(MORPH_RECT, Size(relaxed_radius * 2, relaxed_radius * 2));
    erode(map_original_, map_relaxed_, kernel_relaxed);
    
    map_loaded_ = true;
    RCLCPP_INFO(this->get_logger(), "Mapa dinámico (SLAM) inflado (radio=%d px = %.2f m).", robot_radius, robot_radius_m);
  }

  void map_dron_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    (void)msg;
    map_dron_loaded_ = true;
    RCLCPP_INFO_ONCE(this->get_logger(), "Mapa estático del dron recibido en RRT.");
  }

  void publishPath(const vector<Point>& cv_path, double theta_goal = 0.0) {
    nav_msgs::msg::Path ros_path;
    ros_path.header.stamp = this->get_clock()->now();
    ros_path.header.frame_id = "map";

    for (size_t i = 0; i < cv_path.size(); i++) {
      geometry_msgs::msg::PoseStamped pose;
      double x_m, y_m;
      pixel_to_world(cv_path[i], x_m, y_m);

      pose.pose.position.x = x_m;
      pose.pose.position.y = y_m;
      pose.pose.position.z = 0.0;

      // Incluir orientacion objetivo en el ultimo punto
      if (i == cv_path.size() - 1) {
        tf2::Quaternion q_goal;
        q_goal.setRPY(0, 0, theta_goal);
        pose.pose.orientation.x = q_goal.x();
        pose.pose.orientation.y = q_goal.y();
        pose.pose.orientation.z = q_goal.z();
        pose.pose.orientation.w = q_goal.w();
      }

      ros_path.poses.push_back(pose);
    }

    path_pub_->publish(ros_path);
    RCLCPP_INFO(this->get_logger(), "Ruta RRT publicada (%zu puntos, yaw_goal=%.1f deg).", cv_path.size(), theta_goal * 180.0 / M_PI);
  }

private:
  bool map_loaded_ = false;
  bool map_dron_loaded_ = false;
  Mat map_original_;
  Mat map_inflated_;
  Mat map_relaxed_;
  double map_resolution_ = 0.05;
  double map_origin_x_ = 0.0;
  double map_origin_y_ = 0.0;
  int map_height_ = 0;
  double ros_yaw_goal_ = 0.0;  // Orientacion objetivo en frame ROS (para publicar en el Path)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_dron_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr replan_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback() {
    if (do_planning()) {
      timer_->cancel();
    }
  }

  void replan_callback(const std_msgs::msg::Empty::SharedPtr msg) {
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "Peticion de replanificacion recibida por topico.");
    do_planning();
  }

  bool do_planning() {
    if (!map_loaded_ || !map_dron_loaded_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Esperando mapas... SLAM recibido: %s, Dron recibido: %s", 
          map_loaded_ ? "SI" : "NO", map_dron_loaded_ ? "SI" : "NO");
        return false;
    }

    geometry_msgs::msg::TransformStamped start_tf;
    geometry_msgs::msg::TransformStamped goal_tf;

    try {
      // Intentar obtener la TF dinámica del robot primero
      start_tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      try {
        // Fallback a la snapshot estática del drone
        start_tf = tf_buffer_->lookupTransform("map", "carrito_aruco", tf2::TimePointZero);
      } catch (const tf2::TransformException & ex2) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Esperando la TF del carrito...");
        return false;
      }
    }

    try {
      goal_tf = tf_buffer_->lookupTransform("map", "meta_aruco", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Esperando la TF de meta_aruco...");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "TFs encontradas. Calculando RRT...");

    // Convertir transformacion de carrito a px usando resolución y origen del mapa
    Point start = world_to_pixel(
      start_tf.transform.translation.x,
      start_tf.transform.translation.y
    );

    tf2::Quaternion q_start(
      start_tf.transform.rotation.x,
      start_tf.transform.rotation.y,
      start_tf.transform.rotation.z,
      start_tf.transform.rotation.w);
    tf2::Matrix3x3 m_start(q_start);
    double r, p, y_start;
    m_start.getRPY(r, p, y_start);
    double theta_start = -y_start; // Invertir yaw para OpenCV

    // Convertir transformacion de meta a px
    Point goal = world_to_pixel(
      goal_tf.transform.translation.x,
      goal_tf.transform.translation.y
    );

    tf2::Quaternion q_goal(
      goal_tf.transform.rotation.x,
      goal_tf.transform.rotation.y,
      goal_tf.transform.rotation.z,
      goal_tf.transform.rotation.w);
    tf2::Matrix3x3 m_goal(q_goal);
    double rg, pg, y_goal;
    m_goal.getRPY(rg, pg, y_goal);
    double theta_goal = -y_goal;
    ros_yaw_goal_ = y_goal;  // Guardar yaw original ROS para publicar en el Path

    // Bucle de intentos
    bool success = false;
    for (int i = 0; i < 3; i++) {
        success = compute_rrt(start, theta_start, goal, theta_goal, map_inflated_, false);
        if (success) break;
    }

    if (!success) {
        RCLCPP_WARN(this->get_logger(), "Fallo tras 3 intentos en mapa inflado. Iniciando modo RELAJADO (1/4 de la medida original)...");
        success = compute_rrt(start, theta_start, goal, theta_goal, map_relaxed_, true);
    }

    return success;
  }

  bool compute_rrt(Point start, double theta_start, Point goal, double theta_goal, const Mat& map_in, bool is_relaxed) {
    Mat working_map = map_in.clone();
    // Despejar el area inicial del robot para permitir que el planificador 
    // encuentre una salida si el robot ya rozo la zona inflada de un obstaculo
    int escape_radius = static_cast<int>(ceil(0.20 / map_resolution_));
    circle(working_map, start, escape_radius, Scalar(255), -1);

    // Verificar que la meta no esta en colision
    bool goal_coll = check_collision(working_map, goal, goal);
    if (goal_coll) {
      if (!is_relaxed) {
        RCLCPP_WARN(this->get_logger(), "Error de colision en la META (mapa inflado).");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Error de colision en la META incluso con mapa relajado.");
      }
      return false;
    }

    vector<RRTNode> tree;
    // Empezar directamente desde el centro del robot
    tree.push_back(RRTNode(start, -1));

    int step_size = max(2, static_cast<int>(0.15 / map_resolution_)); // ~15cm
    int max_iter = 15000;
    double goal_margin = step_size * 1.5; 

    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> distX(0, working_map.cols - 1);
    uniform_int_distribution<> distY(0, working_map.rows - 1);
    uniform_real_distribution<> distProb(0.0, 1.0);

    bool goal_reached = false;
    int last_node_id = -1;

    Mat map_color;
    cvtColor(map_original_, map_color, COLOR_GRAY2BGR);

    circle(map_color, start, 5, Scalar(0, 255, 0), -1);
    circle(map_color, goal, 5, Scalar(255, 0, 0), -1);

    for (int i = 0; i < max_iter; i++) {
      // Sesgo del 10% hacia la meta directamente (sin punto de aproximacion)
      Point random_point = (distProb(gen) < 0.1) ? goal : Point(distX(gen), distY(gen));

      int nearest_idx = 0;
      double min_dist = get_dist(tree[0].pos, random_point);
      for (size_t j = 1; j < tree.size(); j++) {
        double d = get_dist(tree[j].pos, random_point);
        if (d < min_dist) {
          min_dist = d;
          nearest_idx = j;
        }
      }
      Point nearest_point = tree[nearest_idx].pos;

      double theta = atan2(random_point.y - nearest_point.y, random_point.x - nearest_point.x);
      Point new_point(nearest_point.x + step_size * cos(theta), nearest_point.y + step_size * sin(theta));

      if (new_point.x < 0 || new_point.x >= working_map.cols ||
          new_point.y < 0 || new_point.y >= working_map.rows)
        continue;

      if (!check_collision(working_map, nearest_point, new_point)) {
        tree.push_back(RRTNode(new_point, nearest_idx));
        line(map_color, nearest_point, new_point, Scalar(255, 220, 220), 1);

        if (get_dist(new_point, goal) <= goal_margin) {
          if (!check_collision(working_map, new_point, goal)) {
            tree.push_back(RRTNode(goal, tree.size() - 1));
            last_node_id = tree.size() - 1;
            goal_reached = true;
            RCLCPP_INFO(this->get_logger(), "Meta encontrada en %d iteraciones!", i);
            break;
          }
        }
      }
    }

    if (goal_reached) {
      vector<Point> inner_path;
      int current_id = last_node_id;
      while (current_id != -1) {
        inner_path.push_back(tree[current_id].pos);
        current_id = tree[current_id].parent_id;
      }
      reverse(inner_path.begin(), inner_path.end());

      vector<Point> smoothed_inner = smooth_path(working_map, inner_path);
      double densify_dist = max(1.0, 0.05 / map_resolution_); // ~5cm entre puntos
      vector<Point> dense_inner = densify_path(smoothed_inner, densify_dist);

      vector<Point> final_path;
      final_path.push_back(start);
      for (Point p : dense_inner) final_path.push_back(p);
      final_path.push_back(goal);

      for (size_t i = 0; i < final_path.size() - 1; i++) {
        line(map_color, final_path[i], final_path[i + 1], Scalar(0, 0, 255), 3);
        circle(map_color, final_path[i], 3, Scalar(0, 0, 0), -1);
      }

      publishPath(final_path, ros_yaw_goal_);

      // Usar _imwrite_ para que la visualizacion no bloquee el hilo de ROS 2 si no se cierra CV
      imwrite("resultado_rrt_pro.png", map_color);
      // Opcional: mostrar la ventana y cerrarla tras 5 segundos. 
      // Si no cierra, mantendra la instancia congelada, por lo tanto mejor que no se ejecute imshow/waitKey si es automatico
      // imshow("RRT Suavizado y Orientado", map_color);
      // waitKey(1000); 
      return true;
    } else {
      if (is_relaxed) {
        RCLCPP_ERROR(this->get_logger(), "Fallo la busqueda incluso en modo relajado.");
      }
      return false;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<RRTRosNode>();

  // Spin mantendrá vivo este planificador para que escuche el tópico /map,
  // atrape los TFs y publique la ruta.
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
