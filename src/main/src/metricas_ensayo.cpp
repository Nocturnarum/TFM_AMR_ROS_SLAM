#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <boost/foreach.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Pose  { double t, x, y, th; };
struct Evento { double t; int v; std::string s; };

static double yawDe(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
static double ang(double a) { return std::atan2(std::sin(a), std::cos(a)); }
static std::string num(double v, int d) {
  char b[32]; snprintf(b, sizeof(b), "%.*f", d, v); return b;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Uso: metricas_ensayo ensayo.bag [--pose /amcl_pose] [--real X Y YAW] [--csv f.csv]\n";
    return 1;
  }
  std::string bagfile = argv[1], topic_pose = "/amcl_pose", csv = "resultados.csv";
  double tol_xy = 0.10, tol_yaw = 0.15;
  bool hay_real = false; double rx = 0, ry = 0, rth = 0;
  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--pose" && i + 1 < argc) topic_pose = argv[++i];
    else if (a == "--csv" && i + 1 < argc) csv = argv[++i];
    else if (a == "--tol" && i + 2 < argc) { tol_xy = atof(argv[++i]); tol_yaw = atof(argv[++i]); }
    else if (a == "--real" && i + 3 < argc) {
      hay_real = true; rx = atof(argv[++i]); ry = atof(argv[++i]); rth = atof(argv[++i]);
    }
  }

  std::vector<Pose> poses, goals, goals_rviz;
  std::vector<Evento> results, modos, perf;

  rosbag::Bag bag;
  bag.open(bagfile, rosbag::bagmode::Read);
  rosbag::View view(bag);
  BOOST_FOREACH (const rosbag::MessageInstance& m, view) {
    double t = m.getTime().toSec();
    const std::string& tp = m.getTopic();
    if (tp == topic_pose) {
      geometry_msgs::PoseWithCovarianceStamped::ConstPtr a = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
      nav_msgs::Odometry::ConstPtr o = m.instantiate<nav_msgs::Odometry>();
      const geometry_msgs::Pose* p = a ? &a->pose.pose : (o ? &o->pose.pose : NULL);
      if (p) { Pose q = {t, p->position.x, p->position.y, yawDe(p->orientation)}; poses.push_back(q); }
    } else if (tp == "/move_base/goal") {
      move_base_msgs::MoveBaseActionGoal::ConstPtr g = m.instantiate<move_base_msgs::MoveBaseActionGoal>();
      if (g) {
        const geometry_msgs::Pose& p = g->goal.target_pose.pose;
        Pose q = {t, p.position.x, p.position.y, yawDe(p.orientation)}; goals.push_back(q);
      }
    } else if (tp == "/move_base_simple/goal") {
      geometry_msgs::PoseStamped::ConstPtr g = m.instantiate<geometry_msgs::PoseStamped>();
      if (g) { Pose q = {t, g->pose.position.x, g->pose.position.y, yawDe(g->pose.orientation)}; goals_rviz.push_back(q); }
    } else if (tp == "/move_base/result") {
      move_base_msgs::MoveBaseActionResult::ConstPtr r = m.instantiate<move_base_msgs::MoveBaseActionResult>();
      if (r) { Evento e = {t, r->status.status, ""}; results.push_back(e); }
    } else if (tp == "/mode_manual") {
      std_msgs::Bool::ConstPtr b = m.instantiate<std_msgs::Bool>();
      if (b) { Evento e = {t, b->data ? 1 : 0, ""}; modos.push_back(e); }
    } else if (tp == "/perfil_navegacion") {
      std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
      if (s) { Evento e = {t, 0, s->data}; perf.push_back(e); }
    }
  }
  bag.close();
  if (poses.empty()) { std::cerr << "No hay mensajes en " << topic_pose << "\n"; return 1; }

  struct Tramo { int n; double t0, t1; int st; bool hay_goal; Pose g; };
  std::vector<Tramo> tramos;
  // Modo manual: hay goals de RViz pero move_base no los gestionó (no corre).
  // El tramo termina al entrar en la tolerancia del goal (éxito) o al final del bag.
  bool manual = goals.empty() && !goals_rviz.empty();
  if (manual) {
    for (size_t i = 0; i < goals_rviz.size(); ++i) {
      Tramo tr; tr.n = i + 1; tr.t0 = goals_rviz[i].t; tr.st = 0; tr.hay_goal = true; tr.g = goals_rviz[i];
      tr.t1 = (i + 1 < goals_rviz.size()) ? goals_rviz[i + 1].t : poses.back().t;
      for (size_t k = 0; k < poses.size(); ++k) {
        if (poses[k].t < tr.t0 || poses[k].t > tr.t1) continue;
        if (std::hypot(poses[k].x - tr.g.x, poses[k].y - tr.g.y) <= tol_xy &&
            std::fabs(ang(poses[k].th - tr.g.th)) <= tol_yaw) { tr.t1 = poses[k].t; tr.st = 3; break; }
      }
      tramos.push_back(tr);
    }
  } else if (!goals.empty()) {
    for (size_t i = 0; i < goals.size(); ++i) {
      Tramo tr; tr.n = i + 1; tr.t0 = goals[i].t; tr.t1 = poses.back().t; tr.st = -1;
      tr.hay_goal = true; tr.g = goals[i];
      for (size_t k = 0; k < results.size(); ++k)
        if (results[k].t >= tr.t0) { tr.t1 = results[k].t; tr.st = results[k].v; break; }
      tramos.push_back(tr);
    }
  } else {
    Tramo tr; tr.n = 1; tr.t0 = poses.front().t; tr.t1 = poses.back().t; tr.st = -2; tr.hay_goal = false;
    tramos.push_back(tr);
  }

  std::string cab = "bag,tramo,tiempo_s,longitud_m,exito,status,intervenciones,cambios_perfil,pct_estrecho,"
                    "err_llegada_xy_m,err_llegada_yaw_rad,eficiencia,err_pose_xy_m,err_pose_yaw_rad,"
                    "x_final,y_final,yaw_final";
  std::ifstream existe(csv.c_str());
  bool nuevo = !existe.good(); existe.close();
  std::ofstream out(csv.c_str(), std::ios::app);
  if (nuevo) out << cab << "\n";
  std::string base = bagfile.substr(bagfile.find_last_of('/') + 1);

  for (size_t i = 0; i < tramos.size(); ++i) {
    const Tramo& tr = tramos[i];
    std::vector<Pose> seg;
    for (size_t k = 0; k < poses.size(); ++k)
      if (poses[k].t >= tr.t0 && poses[k].t <= tr.t1) seg.push_back(poses[k]);
    if (seg.size() < 2) continue;

    double L = 0;
    for (size_t k = 1; k < seg.size(); ++k)
      L += std::hypot(seg[k].x - seg[k-1].x, seg[k].y - seg[k-1].y);
    const Pose& f = seg.back();

    int inter = 0;
    for (size_t k = 1; k < modos.size(); ++k)
      if (modos[k].t >= tr.t0 && modos[k].t <= tr.t1 && modos[k].v == 1 && modos[k-1].v == 0) ++inter;

    int cambios = 0; double t_estr = 0; std::string estado; double tk = tr.t0;
    for (size_t k = 0; k < perf.size(); ++k) if (perf[k].t <= tr.t0) estado = perf[k].s;
    for (size_t k = 0; k < perf.size(); ++k) {
      if (perf[k].t <= tr.t0 || perf[k].t > tr.t1) continue;
      ++cambios;
      if (estado == "estrecho") t_estr += perf[k].t - tk;
      tk = perf[k].t; estado = perf[k].s;
    }
    if (estado == "estrecho") t_estr += tr.t1 - tk;
    double T = tr.t1 - tr.t0;

    std::ostringstream l;
    l << base << "," << tr.n << "," << num(T, 2) << "," << num(L, 3) << ",";
    if (tr.hay_goal) l << (tr.st == 3 ? 1 : 0) << "," << tr.st; else l << ",";
    l << "," << inter << ",";
    if (!perf.empty()) l << cambios << "," << num(T > 0 ? 100.0 * t_estr / T : 0, 1); else l << ",";
    l << ",";
    if (tr.hay_goal) {
      double recta = std::hypot(tr.g.x - seg.front().x, tr.g.y - seg.front().y);
      l << num(std::hypot(tr.g.x - f.x, tr.g.y - f.y), 3) << ","
        << num(std::fabs(ang(tr.g.th - f.th)), 3) << ","
        << (L > 0 ? num(recta / L, 3) : std::string(""));
    } else l << ",,";
    l << ",";
    if (hay_real && i + 1 == tramos.size())
      l << num(std::hypot(rx - f.x, ry - f.y), 3) << "," << num(std::fabs(ang(rth - f.th)), 3);
    else l << ",";
    l << "," << num(f.x, 3) << "," << num(f.y, 3) << "," << num(f.th, 3);

    out << l.str() << "\n";
    std::cout << l.str() << "\n";
  }
  std::cout << (manual ? "[modo manual] " : "") << "Añadido a " << csv << "\n";
  return 0;
}

