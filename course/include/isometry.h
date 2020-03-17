#pragma once


#include <initializer_list>
#include <string>
#include <sstream>
#include <cmath>
#include <iostream>

namespace ekumen {
  namespace math {

class Vector3 {
 public:
  static const Vector3 kUnitX;
  static const Vector3 kUnitY;
  static const Vector3 kUnitZ;
  static const Vector3 kZero;

  Vector3(double x, double y, double z): x_(x), y_(y), z_(z) {};
  Vector3(std::initializer_list<double> row): x_(row.begin()[0]), y_(row.begin()[1]), z_(row.begin()[2]) {};
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
  Vector3 operator += (const Vector3 & obj) const;
  Vector3 operator -= (const Vector3 & obj) const;
  Vector3 operator *= (const Vector3 & obj) const;
  Vector3 operator /= (const Vector3 & obj) const;
  Vector3 operator + (const Vector3 & obj) const {return *this += obj;};
  Vector3 operator - (const Vector3 & obj) const {return *this -= obj;};
  Vector3 operator * (const Vector3 & obj) const {return *this *= obj;};
  Vector3 operator / (const Vector3 & obj) const {return *this /= obj;};
  bool operator == (const std::initializer_list<double> & obj) const;
  bool operator == (const Vector3 & obj) const;
  bool operator != (const std::initializer_list<double> & obj) const {return !(*this == obj);};
  bool operator != (const Vector3 & obj) const {return !(*this == obj);};
  Vector3 operator *= (const double & obj) const;
  Vector3 operator /= (const double & obj) const;
  Vector3 operator * (const double & obj) const {return *this *= obj;};
  Vector3 operator / (const double & obj) const {return *this /= obj;};

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

}  // math
}  // ekumen
