#ifndef NURBS_HPP
#define NURBS_HPP
#include <Eigen/Eigen>
#include <vector>
namespace nurbs {
using Point = Eigen::Vector3f;

inline double base(int i, int k, double u, std::vector<double> knots) {
  if (k == 0) {
    if (u >= knots[i] && u < knots[i + 1]) {
      return 1.0;
    }
    return 0.0;
  }
  float a = 0.0, b = 0.0;
  if (knots[i + k] - knots[i] != 0.0) {
    a = (u - knots[i]) / (knots[i + k] - knots[i]);
  }
  if (knots[i + k + 1] - knots[i + 1] != 0.0) {
    b = (knots[i + k + 1] - u) / (knots[i + k + 1] - knots[i + 1]);
  }
  return a * base(i, k - 1, u, knots) + b * base(i + 1, k - 1, u, knots);
}
inline Point calculate_point(double u, std::vector<Point> ctrl_ps,
                             std::vector<double> knots,
                             std::vector<double> weights, int degree) {
  int n = ctrl_ps.size() - 1;
  Point result{0, 0, 0};
  double denominator = 0.0;
  for (int i = 0; i <= n; i++) {
    double factor = base(i, degree, u, knots) * weights[i];
    denominator += factor;
    result.x() += factor * ctrl_ps[i].x();
    result.y() += factor * ctrl_ps[i].y();
    result.z() += factor * ctrl_ps[i].z();
  }
  // std::cout << result << " " << denominator << std::endl;

  result.x() /= denominator;
  result.y() /= denominator;
  result.z() /= denominator;
  return result;
};
inline std::vector<Point> calculate_nurbs(std::vector<Point> ctrl_ps,
                                          std::vector<double> weights,
                                          int degree, double step) {
  int num = ctrl_ps.size() + degree + 1;
  double delta = 1.0 / (num - 2 * degree - 1);
  std::vector<double> knots;
  for (int i = 0; i < num; ++i) {
    if (i < degree + 1) {
      knots.push_back(0.0);
    } else if (i >= num - degree) {
      knots.push_back(1.0);
    } else {
      knots.push_back(knots.back() + delta);
    }
    //   std::cout << knots.back() << std::endl;
  }
  std::vector<Point> res;
  for (double u = 0; u <= 1; u += step) {
    Point point = calculate_point(u, ctrl_ps, knots, weights, degree);
    res.push_back(point);
  }
  return res;
}
inline std::vector<Eigen::Vector3f> calculate_normal(
    const std::vector<Point>& ps) {
  if (ps.size() < 2) {
    return {};
  }
  std::vector<Eigen::Vector3f> res;
  for (int i = 0; i < ps.size(); ++i) {
    if (i == 0) {
      Eigen::Vector3f Normal = ps[1] - ps[0];
      res.push_back(Normal.normalized());
    } else if (i == ps.size() - 1) {
      Eigen::Vector3f Normal = ps.back() - ps[ps.size() - 2];
      res.push_back(Normal.normalized());
    } else {
      Eigen::Vector3f Normal = ps[i + 1] - ps[i - 1];
      res.push_back(Normal.normalized());
    }
  }
  return res;
}

}  // namespace nurbs
#endif