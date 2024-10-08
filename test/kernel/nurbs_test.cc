#include "../../include/component/util/nurbs.hpp"

#include <fstream>
#include <iostream>
int main() {
  std::vector<nurbs::Point> ctrl_ps{
      {17, 8.5, 0},  {17.4, 4.4, 0},  {18.15, 5.65, 0}, {19.9, 4.9, 0},
      {21, 4.45, 0}, {21.5, 3.95, 0}, {21.5, 2, 0}};
  auto res = nurbs::calculate_nurbs(ctrl_ps, {1, 1, 1, 1, 1, 1, 1}, 6, 0.01);
  std::fstream fs;
  fs.open("data.txt", std::ios::out);
  assert(fs.is_open());
  for (auto& x : res) {
    fs << x.x() << "\t" << x.y() << std::endl;
  }
}