#pragma once


#include <initializer_list>
#include <string>
#include <sstream>
#include <cmath>
#include <iostream>
#include <iomanip>

namespace ekumen {
  namespace math {

class Vector3 {
 public:
  static const Vector3 kUnitX;
  static const Vector3 kUnitY;
  static const Vector3 kUnitZ;
  static const Vector3 kZero;

  Vector3(double x, double y, double z): x_(x), y_(y), z_(z) {};
  explicit Vector3(std::initializer_list<double> row) {
    if(row.size() != 3) {
      throw std::runtime_error("List has invalid amount of elements");
    }
    std::initializer_list<double>::iterator it = row.begin();
    x_ = *it++;
    y_ = *it++;
    z_ = *it;
  };
  Vector3(const Vector3 & obj): x_(obj[0]), y_(obj[1]), z_(obj[2]) {};
  Vector3(): x_(0.), y_(0.), z_(0.) {};

  const double & x() const {return x_;};
  const double & y() const {return y_;};
  const double & z() const {return z_;};
  double & x() {return x_;};
  double & y() {return y_;};
  double & z() {return z_;};

  const double & operator [] (const int index) const;
  double & operator [] (const int index);
  Vector3 & operator += (const Vector3 & obj);
  Vector3 & operator -= (const Vector3 & obj);
  Vector3 & operator *= (const Vector3 & obj);
  Vector3 & operator /= (const Vector3 & obj);
  Vector3 operator + (const Vector3 & obj) const {return Vector3(*this) += obj;};
  Vector3 operator - (const Vector3 & obj) const {return Vector3(*this) -= obj;};
  Vector3 operator * (const Vector3 & obj) const {return Vector3(*this) *= obj;};
  Vector3 operator / (const Vector3 & obj) const {return Vector3(*this) /= obj;};
  Vector3 & operator = (const Vector3 & obj);
  bool operator == (const std::initializer_list<double> & obj) const;
  bool operator == (const Vector3 & obj) const;
  bool operator != (const std::initializer_list<double> & obj) const {return !(*this == obj);};
  bool operator != (const Vector3 & obj) const {return !(*this == obj);};
  Vector3 & operator *= (const double & obj);
  Vector3 & operator /= (const double & obj);
  Vector3 operator * (const double & obj) const {return Vector3(*this) *= obj;};
  Vector3 operator / (const double & obj) const {return Vector3(*this) /= obj;};

  Vector3 cross(const Vector3 & obj) const;
  double dot(const Vector3 & obj) const;
  double norm() const;

  friend std::ostream& operator << (std::ostream & os, const Vector3 & v) {
            os << std::string("(x: ") << v.x() << ", y: " << v.y() << ", z: " << v.z() << ")";
            return os;
        };
 private:
   double x_,y_,z_;
};

Vector3 operator * (const double & obj1, const Vector3 & obj2);

class Matrix3 {
  public:
    static const Matrix3 kIdentity;
    static const Matrix3 kZero;
    static const Matrix3 kOnes;
    Matrix3(std::initializer_list<double> row1, std::initializer_list<double> row2,
       std::initializer_list<double> row3): row1_(row1), row2_(row2), row3_(row3) {};
    Matrix3(double x1, double y1, double z1, double x2, double y2, double z2,
       double x3, double y3, double z3): row1_(x1, y1, z1), row2_(x2, y2, z2), row3_(x3, y3, z3) {};
    Matrix3(const Vector3 & vec1, const Vector3 & vec2, const Vector3 & vec3): row1_(vec1), row2_(vec2), row3_(vec3) {};
    Matrix3(const Matrix3 & obj): row1_(obj[0]), row2_(obj[1]), row3_(obj[2]) {};
    Matrix3(): row1_(0., 0., 0.), row2_(0., 0., 0.), row3_(0., 0., 0.) {};
    Vector3 row(const int index) const;
    Vector3 col(const int index) const;
    bool operator == (const Matrix3 & obj) const;
    bool operator != (const Matrix3 & obj) const {return !(*this == obj);};
    bool operator == (const std::initializer_list<double> & obj) const;
    bool operator != (const std::initializer_list<double> & obj) const {return !(*this == obj);};
    Matrix3 & operator += (const Matrix3 & obj);
    Matrix3 & operator -= (const Matrix3 & obj);
    Matrix3 & operator *= (const Matrix3 & obj);
    Matrix3 & operator /= (const Matrix3 & obj);
    Matrix3 operator + (const Matrix3 & obj) const {return Matrix3(*this) += obj;};
    Matrix3 operator - (const Matrix3 & obj) const {return Matrix3(*this) -= obj;};
    Matrix3 operator * (const Matrix3 & obj) const {return Matrix3(*this) *= obj;};
    Matrix3 operator / (const Matrix3 & obj) const {return Matrix3(*this) /= obj;};
    Matrix3 & operator *= (const double & obj);
    Matrix3 & operator /= (const double & obj);
    Matrix3 operator * (const double & obj) const {return Matrix3(*this) *= obj;};
    Matrix3 operator / (const double & obj) const {return Matrix3(*this) /= obj;};
    Vector3 operator * (const Vector3 & obj) const;
    Matrix3 & operator = (const Matrix3 & obj);
    Vector3 operator [] (const int index) const;
    Vector3 & operator [] (const int index);
    double det() const;
    Matrix3 inverse() const;
    friend std::ostream& operator << (std::ostream & os, const Matrix3 & v) {
              os << std::setprecision(9);
              os << std::string("[[") << v[0][0] << ", " << v[0][1] << ", " << v[0][2] << "], [";
              os << v[1][0] << ", " << v[1][1] << ", " << v[1][2] << "], [";
              os << v[2][0] << ", " << v[2][1] << ", " << v[2][2] << "]]";
              return os;
          };
  private:
    Vector3 row1_, row2_, row3_;
};

Matrix3 operator * (const double & obj1, const Matrix3 & obj2);

class Isometry {
  public:
    Isometry(const Vector3 & translation, const Matrix3 & rotation): translation_(translation), rotation_(rotation) {};
    Isometry(const Isometry & obj): translation_(obj.translation()), rotation_(obj.rotation()) {};
    static Isometry FromTranslation(const Vector3 & translation){return Isometry(translation, Matrix3::kIdentity);};
    static Isometry RotateAround(const Vector3 & translation, double angle);
    static Isometry FromEulerAngles(double roll, double pitch, double yaw);

    const Matrix3 & rotation() const {return rotation_;};
    const Vector3 & translation() const {return translation_;};

    bool operator == (const Isometry & obj) const;
    Vector3 operator * (const Vector3 & obj) const;
    Isometry operator *= (const Isometry & obj);
    Isometry operator * (const Isometry & obj) const {return Isometry(*this) *= obj;};
    Isometry & operator = (const Isometry & obj);
    Vector3 transform(const Vector3 & obj) const;
    Isometry inverse() const;
    Isometry compose(const Isometry & obj) const {return Isometry(*this) *= obj;};

    friend std::ostream& operator << (std::ostream & os, const Isometry & v) {
              os << std::string("[T: ") << v.translation() << ", R:" << v.rotation() << "]";
              return os;
          };
  private:
    Vector3 translation_;
    Matrix3 rotation_;
};

}  // math
}  // ekumen
