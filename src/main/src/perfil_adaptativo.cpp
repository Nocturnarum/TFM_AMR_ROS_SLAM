#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

struct Perfil {
  double max_vel_x, max_vel_trans, max_vel_theta, occdist_scale;
  double inflation_radius, cost_scaling_factor;
};

class PerfilAdaptativo {
public:
  PerfilAdaptativo() : pnh_("~"), actual_(""), tiene_goal_(false), reintentos_(0) {
    cargarPerfil("estrecho", 0.10, 0.10, 0.7, 0.01, 0.15, 10.0);
    cargarPerfil("amplio",   0.18, 0.18, 1.0, 0.05, 0.30, 3.0);
    pnh_.param("ancho_estrecho", ancho_estrecho_, 0.90);
    pnh_.param("ancho_amplio", ancho_amplio_, 1.09);
    pnh_.param("frontal_estrecho", frontal_estrecho_, 0.45);
    pnh_.param("rango_min", rango_min_, 0.15);
    pnh_.param("tiempo_min_perfil", t_min_, 2.0);
    pnh_.param("ratio_ruta_larga", ratio_larga_, 2.5);
    pnh_.param("tiempo_forzado", t_forzado_, 20.0);
    pnh_.param("reintentos_max", reintentos_max_, 1);
    std::string ns;
    pnh_.param<std::string>("move_base_ns", ns, "/move_base");

    srv_dwa_ = ns + "/DWAPlannerROS/set_parameters";
    srv_inf_.push_back(ns + "/local_costmap/inflation_layer/set_parameters");
    srv_inf_.push_back(ns + "/global_costmap/inflation_layer/set_parameters");

    ROS_INFO("Esperando dynamic_reconfigure de move_base...");
    ros::service::waitForService(srv_dwa_, ros::Duration(30.0));
    for (size_t i = 0; i < srv_inf_.size(); ++i)
      ros::service::waitForService(srv_inf_[i], ros::Duration(30.0));

    pub_perfil_ = nh_.advertise<std_msgs::String>("/perfil_navegacion", 1, true);
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    t_cambio_ = ros::Time(0);
    forzado_hasta_ = ros::Time(0);

    aplicar("amplio", "inicio");
    sub_scan_   = nh_.subscribe("/scan", 1, &PerfilAdaptativo::cbScan, this);
    sub_plan_   = nh_.subscribe(ns + "/NavfnROS/plan", 1, &PerfilAdaptativo::cbPlan, this);
    sub_goal_   = nh_.subscribe(ns + "/goal", 1, &PerfilAdaptativo::cbGoal, this);
    sub_result_ = nh_.subscribe(ns + "/result", 1, &PerfilAdaptativo::cbResult, this);
  }

private:
  // ---------- perfiles ----------
  void cargarPerfil(const std::string& n, double vx, double vt, double vth,
                    double occ, double infl, double csf) {
    Perfil p;
    std::string b = "perfiles/" + n + "/";
    pnh_.param(b + "max_vel_x", p.max_vel_x, vx);
    pnh_.param(b + "max_vel_trans", p.max_vel_trans, vt);
    pnh_.param(b + "max_vel_theta", p.max_vel_theta, vth);
    pnh_.param(b + "occdist_scale", p.occdist_scale, occ);
    pnh_.param(b + "inflation_radius", p.inflation_radius, infl);
    pnh_.param(b + "cost_scaling_factor", p.cost_scaling_factor, csf);
    perfiles_[n] = p;
  }

  static void addDouble(dynamic_reconfigure::Reconfigure& s, const std::string& k, double v) {
    dynamic_reconfigure::DoubleParameter d;
    d.name = k; d.value = v;
    s.request.config.doubles.push_back(d);
  }

  bool llamar(const std::string& srv, dynamic_reconfigure::Reconfigure& s) {
    if (!ros::service::call(srv, s)) {
      ROS_WARN("Fallo reconfigurando %s", srv.c_str());
      return false;
    }
    return true;
  }

  void aplicar(const std::string& nombre, const std::string& motivo) {
    if (nombre == actual_) return;
    const Perfil& p = perfiles_[nombre];

    dynamic_reconfigure::Reconfigure dwa;
    addDouble(dwa, "max_vel_x", p.max_vel_x);
    addDouble(dwa, "max_vel_trans", p.max_vel_trans);
    addDouble(dwa, "max_vel_theta", p.max_vel_theta);
    addDouble(dwa, "occdist_scale", p.occdist_scale);
    llamar(srv_dwa_, dwa);

    for (size_t i = 0; i < srv_inf_.size(); ++i) {
      dynamic_reconfigure::Reconfigure inf;
      addDouble(inf, "inflation_radius", p.inflation_radius);
      addDouble(inf, "cost_scaling_factor", p.cost_scaling_factor);
      llamar(srv_inf_[i], inf);
    }

    actual_ = nombre;
    t_cambio_ = ros::Time::now();
    std_msgs::String m; m.data = nombre;
    pub_perfil_.publish(m);
    ROS_INFO("Perfil -> %s (%s)", nombre.c_str(), motivo.c_str());
  }

  void forzarEstrecho(const std::string& motivo) {
    forzado_hasta_ = ros::Time::now() + ros::Duration(t_forzado_);
    aplicar("estrecho", motivo);
  }

  // ---------- detección de zona ----------
  double sectorMin(const sensor_msgs::LaserScan& s, double a0, double a1) const {
    double m = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < s.ranges.size(); ++i) {
      double a = s.angle_min + i * s.angle_increment;
      a = std::atan2(std::sin(a), std::cos(a));
      double r = s.ranges[i];
      if (a >= a0 && a <= a1 && std::isfinite(r) && r > rango_min_ && r < s.range_max)
        m = std::min(m, r);
    }
    return m;
  }

  void cbScan(const sensor_msgs::LaserScan::ConstPtr& s) {
    ros::Time ahora = ros::Time::now();
    if (ahora < forzado_hasta_) return;
    if ((ahora - t_cambio_).toSec() < t_min_) return;
    const double d2r = M_PI / 180.0;
    double izq = sectorMin(*s, 60 * d2r, 120 * d2r);
    double der = sectorMin(*s, -120 * d2r, -60 * d2r);
    double frente = sectorMin(*s, -25 * d2r, 25 * d2r);
    double ancho = izq + der;
    char buf[96];
    snprintf(buf, sizeof(buf), "ancho %.2f m, frente %.2f m", ancho, frente);
    if (ancho < ancho_estrecho_ || frente < frontal_estrecho_)
      aplicar("estrecho", buf);
    else if (ancho > ancho_amplio_ && frente > 2.0 * frontal_estrecho_)
      aplicar("amplio", buf);
  }

  // ---------- recuperación de planificación ----------
  void cbGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& g) {
    const geometry_msgs::PoseStamped& n = g->goal.target_pose;
    if (!tiene_goal_ ||
        std::hypot(n.pose.position.x - goal_.pose.position.x,
                   n.pose.position.y - goal_.pose.position.y) > 0.05)
      reintentos_ = 0;
    goal_ = n;
    tiene_goal_ = true;
  }

  void cbPlan(const nav_msgs::Path::ConstPtr& path) {
    if (path->poses.size() < 2) return;
    double L = 0.0;
    for (size_t k = 1; k < path->poses.size(); ++k)
      L += std::hypot(path->poses[k].pose.position.x - path->poses[k-1].pose.position.x,
                      path->poses[k].pose.position.y - path->poses[k-1].pose.position.y);
    double recta = std::hypot(path->poses.back().pose.position.x - path->poses.front().pose.position.x,
                              path->poses.back().pose.position.y - path->poses.front().pose.position.y);
    if (recta > 0.5 && L / recta > ratio_larga_ && actual_ != "estrecho") {
      char buf[96];
      snprintf(buf, sizeof(buf), "ruta larga: %.1f m frente a %.1f m en recta", L, recta);
      forzarEstrecho(buf);
    }
  }

  void cbResult(const move_base_msgs::MoveBaseActionResult::ConstPtr& r) {
    if (r->status.status != actionlib_msgs::GoalStatus::ABORTED || !tiene_goal_) return;
    if (reintentos_ < reintentos_max_) {
      ++reintentos_;
      char buf[64];
      snprintf(buf, sizeof(buf), "move_base abortó, reintento %d", reintentos_);
      forzarEstrecho(buf);
      ros::Duration(1.0).sleep();
      goal_.header.stamp = ros::Time::now();
      pub_goal_.publish(goal_);
    } else {
      ROS_WARN("Objetivo no alcanzable tras %d reintentos", reintentos_);
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_perfil_, pub_goal_;
  ros::Subscriber sub_scan_, sub_plan_, sub_goal_, sub_result_;
  std::map<std::string, Perfil> perfiles_;
  std::string srv_dwa_;
  std::vector<std::string> srv_inf_;
  std::string actual_;
  ros::Time t_cambio_, forzado_hasta_;
  double ancho_estrecho_, ancho_amplio_, frontal_estrecho_, rango_min_, t_min_;
  double ratio_larga_, t_forzado_;
  int reintentos_max_;
  geometry_msgs::PoseStamped goal_;
  bool tiene_goal_;
  int reintentos_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "perfil_adaptativo");
  PerfilAdaptativo nodo;
  ros::spin();
  return 0;
}
