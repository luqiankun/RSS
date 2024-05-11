#ifndef ENVELOPE_HPP
#define ENVELOPE_HPP
#include <fcl/fcl.h>

#include "../../tcsresource.hpp"
namespace data {
namespace model {
class Envelope : public TCSResource {
 public:
  enum class Type { _2D = 0, _3D = 1 };
  using TCSResource ::TCSResource;
  using Vertex = fcl::Vector3<double>;
  Envelope(const std::string& name, int type = 0)
      : TCSResource(name), type(static_cast<Type>(type)) {}
  Envelope(const std::string& name, const std::string& type = "2D")
      : TCSResource(name) {
    if (type == "2D") {
      this->type = Type::_2D;
    } else {
      this->type = Type::_3D;
    }
  }
  inline void add_vertex(double x, double y, double z) {
    vertexs.push_back(fcl::Vector3d(x / 1000, y / 1000, z / 1000));
  }
  inline void compute_mesh() {
    if (type == Type::_2D) {
      // 2D按棱柱体计算，高度设置为1,面数量=顶点数+上下两个面
      int face_num = vertexs.size() + 2;
      std::vector<Vertex> tmep_v;
      tmep_v.assign(vertexs.begin(), vertexs.end());
      for (auto& x : vertexs) {
        tmep_v.push_back(Vertex(x.x(), x.y(), x.z() + 1));
      }
      {
        // 上
        faces.push_back(vertexs.size());
        for (int i = 0; i < vertexs.size(); i++) {
          faces.push_back(i);
        }
        // 下
        faces.push_back(vertexs.size());
        for (int i = 0; i < vertexs.size(); i++) {
          faces.push_back(i + vertexs.size());
        }
        // 侧面
        for (int i = 0; i < vertexs.size(); i++) {
          if (i == vertexs.size() - 1) {
            faces.insert(faces.end(), {4, i, 0, i + (int)vertexs.size(),
                                       0 + (int)vertexs.size()});
          } else {
            faces.insert(faces.end(), {4, i, i + 1, i + (int)vertexs.size(),
                                       i + 1 + (int)vertexs.size()});
          }
        }
      }
      std::vector<int> temp;
      temp.assign(faces.begin(), faces.end());

      auto face_ptr = std::make_shared<const std::vector<int>>(temp);
      auto ps_ptr = std::make_shared<const std::vector<fcl::Vector3d>>(tmep_v);
      convex = std::make_shared<fcl::Convexd>(ps_ptr, face_num, face_ptr);
      convex->computeLocalAABB();
    } else {
      // TODO
    }
  }

 public:
  Type type;
  std::vector<Vertex> vertexs;
  std::vector<int> faces;
  std::shared_ptr<fcl::Convexd> convex;
};
}  // namespace model
}  // namespace data
#endif