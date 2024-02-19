#ifndef VEC_HPP_
#define VEC_HPP_
#include <cmath>
namespace data {
struct Vector2i {
  int x{0};
  int y{0};
  Vector2i() = default;
  bool operator==(Vector2i a) {
    if (a.x != x) {
      return false;
    }
    if (a.y != y) {
      return false;
    }
    return true;
  }
  Vector2i operator-(const Vector2i& t) { return Vector2i{x - t.x, y - t.y}; }
  Vector2i operator+(const Vector2i& t) { return Vector2i{x + t.x, y + t.y}; }

  Vector2i(int a, int b) {
    x = a;
    y = b;
  }
  Vector2i(const Vector2i& r) {
    x = r.x;
    y = r.y;
  }
  int norm() { return sqrt(x * x + y * y); }
  bool operator!=(Vector2i t) { return !(&t == this); }
};
struct Vector3i {
  int x{0};
  int y{0};
  int z{0};
  Vector3i() = default;
  bool operator==(Vector2i a) {
    if (a.x != x) {
      return false;
    }
    if (a.y != y) {
      return false;
    }
    return true;
  }
  Vector3i operator+(const Vector3i& t) {
    return Vector3i{x + t.x, y + t.y, z + t.z};
  }
  Vector3i operator-(const Vector3i& t) {
    return Vector3i{x - t.x, y - t.y, z - t.z};
  }
  Vector3i(int a, int b, int c) {
    x = a;
    y = b;
    z = c;
  }
  Vector3i(const Vector3i& r) {
    x = r.x;
    y = r.y;
    z = r.z;
  }
  int norm() { return sqrt(x * x + y * y + z * z); }
};
struct Vector2f {
  float x{0.0};
  float y{0.0};
  Vector2f() = default;
  bool operator==(Vector2f a) {
    if (a.x != x) {
      return false;
    }
    if (a.y != y) {
      return false;
    }
    return true;
  }
  Vector2f operator-(const Vector2f& t) { return Vector2f{x - t.x, y - t.y}; }
  Vector2f operator+(const Vector2f& t) { return Vector2f{x + t.x, y + t.y}; }
  Vector2f(float a, float b) {
    x = a;
    y = b;
  }
  Vector2f(const Vector2f& r) {
    x = r.x;
    y = r.y;
  }
  float norm() { return sqrt(x * x + y * y); }
  float dot(Vector2f t) { return x * t.x + y * t.y; }
};
struct Vector3f {
  float x{0.0};
  float y{0.0};
  float z{0.0};
  Vector3f() = default;
  float norm() { return sqrt(x * x + y * y); }
};

}  // namespace data

#endif