#include <isometry/isometry.hpp>
// Copyright 2020 <Jose Tomas Lorente>

namespace ekumen {
namespace math {

  // Vector3

  const Vector3 Vector3::kUnitX(1., 0., 0.);
  const Vector3 Vector3::kUnitY(0., 1., 0.);
  const Vector3 Vector3::kUnitZ(0., 0., 1.);
  const Vector3 Vector3::kZero(0., 0., 0.);

  Vector3 & Vector3::operator *= (const double & obj) {
    x_ *= obj;
    y_ *= obj;
    z_ *=  obj;
    return *this;
  }

  Vector3 & Vector3::operator /= (const double & obj) {
    x_ /= obj;
    y_ /= obj;
    z_ /=  obj;
    return *this;
  }

  Vector3 & Vector3::operator += (const Vector3 & obj) {
    x_ += obj.x();
    y_ += obj.y();
    z_ +=  obj.z();
    return *this;
  }

  Vector3 & Vector3::operator -= (const Vector3 & obj) {
    x_ -= obj.x();
    y_ -= obj.y();
    z_ -=  obj.z();
    return *this;
  }

  bool Vector3::operator == (const std::initializer_list<double> & obj) const {
    if (obj.size() != 3) {
      throw std::runtime_error("Size of list not valid");
    }
    auto it = obj.begin();
    double x_list, y_list, z_list;
    x_list = *it++;
    y_list = *it++;
    z_list = *it;
    return !( (x_list != x_) || (y_list != y_) || (z_list != z_) );
  }

  bool Vector3::operator == (const Vector3 & obj) const {
    return !( (obj[0] != x_) || (obj[1] != y_) || (obj[2] != z_) );
  }

  Vector3 & Vector3::operator *= (const Vector3 & obj) {
    x_ *= obj.x();
    y_ *= obj.y();
    z_ *=  obj.z();
    return *this;
  }

  Vector3 & Vector3::operator /= (const Vector3 & obj) {
    x_ /= obj.x();
    y_ /= obj.y();
    z_ /=  obj.z();
    return *this;
  }

  Vector3 & Vector3::operator = (const Vector3& obj) {
    x_ = obj.x();
    y_ = obj.y();
    z_ = obj.z();
    return *this;
  }

  double Vector3::norm() const {
    return sqrt( std::pow(x_, 2.0) + std::pow(y_, 2.0) + std::pow(z_, 2.0) );
  }

  Vector3 operator * (const double & obj1, const Vector3 & obj2) {
    return Vector3(obj2.x() * obj1, obj2.y() * obj1, obj2.z() * obj1);
  }

  const double & Vector3::operator[] (const int index) const {
    if (index > 2 || index < 0) {
      throw std::runtime_error("Index not valid");
    } else if (index == 0) {
      return x_;
    } else if (index == 1) {
      return y_;
    } else {
      return z_;
    }
  }

  double & Vector3::operator[] (const int index) {
    if (index > 2 || index < 0) {
      throw std::runtime_error("Index not valid");
    } else if (index == 0) {
      return x_;
    } else if (index == 1) {
      return y_;
    } else {
      return z_;
    }
  }

  Vector3 Vector3::cross(const Vector3 & obj) const {
    return Vector3(
                y_ * obj.z_ - z_ * obj.y_,
                z_ * obj.x_ - x_ * obj.z_,
                x_ * obj.y_ - y_ * obj.x_);
  }

  double Vector3::dot(const Vector3 & obj) const {
    return (x_ * obj.x_) + (y_ * obj.y_) + (z_ * obj.z_);
  }

  // Matrix3

  const Matrix3 Matrix3::kIdentity({1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.});
  const Matrix3 Matrix3::kZero({0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.});
  const Matrix3 Matrix3::kOnes({1., 1., 1.}, {1., 1., 1.}, {1., 1., 1.});

  bool Matrix3::operator == (const Matrix3 & obj) const {
    return !(obj[0] != row1_ || obj[1] != row2_ || obj[2] != row3_);
  }

  bool Matrix3::operator == (const std::initializer_list<double> & obj) const {
    if (obj.size() != 9) {
      throw std::runtime_error("Size of list not valid");
    }
    if (obj.begin()[0] != row1_[0] || obj.begin()[1] != row1_[1] ||
      obj.begin()[2] != row1_[2]) {
      return false;
    } else if (obj.begin()[3] != row2_[0] || obj.begin()[4] != row2_[1] ||
      obj.begin()[5] != row2_[2]) {
      return false;
    } else if (obj.begin()[6] != row3_[0] || obj.begin()[7] != row3_[1] ||
      obj.begin()[8] != row3_[2]) {
      return false;
    }
    return true;
  }

  Vector3 Matrix3::operator[] (const int index) const {
    if (index > 2 || index < 0) {
      throw std::runtime_error("Index not valid");
    } else if (index == 0) {
      return row1_;
    } else if (index == 1) {
      return row2_;
    }
    return row3_;
  }

  Vector3 & Matrix3::operator[] (const int index) {
    if (index > 2 || index < 0) {
      throw std::runtime_error("Index not valid");
    } else if (index == 0) {
      return row1_;
    } else if (index == 1) {
      return row2_;
    }
    return row3_;
  }

  Matrix3 & Matrix3::operator += (const Matrix3 & obj) {
    row1_ += obj[0];
    row2_ += obj[1];
    row3_ += obj[2];
    return *this;
  }

  Matrix3 & Matrix3::operator -= (const Matrix3 & obj) {
    row1_ -= obj[0];
    row2_ -= obj[1];
    row3_ -= obj[2];
    return *this;
  }

  Matrix3 & Matrix3::operator *= (const Matrix3 & obj) {
    row1_ *= obj[0];
    row2_ *= obj[1];
    row3_ *= obj[2];
    return *this;
  }

  Matrix3 & Matrix3::operator /= (const Matrix3 & obj) {
    row1_ /= obj[0];
    row2_ /= obj[1];
    row3_ /= obj[2];
    return *this;
  }

  Matrix3 & Matrix3::operator *= (const double & obj) {
    row1_ *= obj;
    row2_ *= obj;
    row3_ *= obj;
    return *this;
  }

  Matrix3 & Matrix3::operator /= (const double & obj) {
    row1_ /= obj;
    row2_ /= obj;
    row3_ /= obj;
    return *this;
  }

  Vector3 Matrix3::operator * (const Vector3 & obj) const {
    double new_x = (row1_[0] * obj[0]) +
      (row1_[1] * obj[1]) + (row1_[2] * obj[2]);
    double new_y = (row2_[0] * obj[0]) +
      (row2_[1] * obj[1]) + (row2_[2] * obj[2]);
    double new_z = (row3_[0] * obj[0]) +
      (row3_[1] * obj[1]) + (row3_[2] * obj[2]);
    return Vector3(new_x, new_y, new_z);
  }

  Matrix3 operator * (const double & obj1, const Matrix3 & obj2) {
    return Matrix3(obj2[0] * obj1, obj2[1] * obj1, obj2[2] * obj1);
  }

  Matrix3 & Matrix3::operator = (const Matrix3& obj) {
    row1_ = obj.row(0);
    row2_ = obj.row(1);
    row3_ = obj.row(2);
    return *this;
  }

  double Matrix3::det() const {
    return ((row1_[0] * row2_[1] * row3_[2]) +
      (row1_[1] * row2_[2] * row3_[0]) +
      (row1_[2] * row2_[0] * row3_[1]) -
      (row1_[2] * row2_[1] * row3_[0]) -
      (row1_[0] * row2_[2] * row3_[1]) -
      (row1_[1] * row2_[0] * row3_[2]));
  }

  Vector3 Matrix3::row(const int index) const {
    if (index > 2 || index < 0) {
      throw std::runtime_error("Index not valid");
    } else if (index == 0) {
      return row1_;
    } else if (index == 1) {
      return row2_;
    }
    return row3_;
  }

  Vector3 Matrix3::col(const int index) const {
    if (index > 2 || index < 0) {
      throw std::runtime_error("Index not valid");
    }
    return Vector3(row1_[index], row2_[index], row3_[index]);
  }

  Matrix3 Matrix3::inverse() const {
    double det = this->det();
    if (det == 0) {
      throw std::runtime_error("Matrix is not invertible");
    }
    return Matrix3(
        (row2_[1] * row3_[2] - row2_[2] * row3_[1]) / det,
        (row2_[0] * row3_[2] - row2_[2] * row3_[0]) / det * -1,
        (row2_[0] * row3_[1] - row2_[1] * row3_[0]) / det,
        (row1_[1] * row3_[2] - row1_[2] * row3_[1]) / det * -1,
        (row1_[0] * row3_[2] - row1_[2] * row3_[0]) / det,
        (row1_[0] * row3_[1] - row1_[1] * row3_[0]) / det * -1,
        (row1_[1] * row2_[2] - row1_[2] * row2_[1]) / det,
        (row1_[0] * row2_[2] - row1_[2] * row2_[0]) / det * -1,
        (row1_[0] * row2_[1] - row1_[1] * row2_[0]) / det);
  }

  // Isometry

  bool Isometry::operator == (const Isometry & obj) const {
    return ((rotation_ == obj.rotation()) &&
      (translation_ == obj.translation()));
  }

  Vector3 Isometry::operator * (const Vector3 & obj) const {
    return rotation_ * obj + translation_;
  }

  Isometry Isometry::operator *= (const Isometry & obj) {
    rotation_ *= obj.rotation();
    translation_ = rotation_ * obj.translation() + translation_;
    return *this;
  }

  Isometry & Isometry::operator = (const Isometry& obj) {
    rotation_ = obj.rotation();
    translation_ = obj.translation();
    return *this;
  }

  Vector3 Isometry::transform(const Vector3 & obj) const {
    return rotation_ * obj + translation_;
  }

  Isometry Isometry::inverse() const {
    Matrix3 new_rotation{rotation_.inverse()};
    Vector3 new_translation{new_rotation * translation_};
    return Isometry(-1 * new_translation, new_rotation);
  }

  Isometry Isometry::RotateAround(const Vector3 & translation, double angle) {
    double cos{std::cos(angle)};
    double sin{std::sin(angle)};

    Matrix3 new_rotation;

    new_rotation[0][0] = cos + (translation.x() * translation.x()) * (1 - cos);
    new_rotation[0][1] =
      translation.x() * translation.y() * (1 - cos) - translation.z() * sin;
    new_rotation[0][2] =
      translation.x() * translation.z() * (1 - cos) + translation.y() * sin;
    new_rotation[1][0] =
      translation.y() * translation.x() * (1 - cos) + translation.z() * sin;
    new_rotation[1][1] = cos + (translation.y() * translation.y()) * (1 - cos);
    new_rotation[1][2] =
      translation.y() * translation.z() * (1 - cos) - translation.x() * sin;
    new_rotation[2][0] =
      translation.z() * translation.x() * (1 - cos) + translation.y() * sin;
    new_rotation[2][1] =
      translation.z() * translation.y() * (1 - cos) + translation.x() * sin;
    new_rotation[2][2] = cos + (translation.z() * translation.z()) * (1 - cos);

    return Isometry(Vector3(), new_rotation);
  }

  Isometry Isometry::FromEulerAngles(double roll, double pitch, double yaw) {
    return Isometry(Isometry::RotateAround(Vector3::kUnitX, roll) *
                    Isometry::RotateAround(Vector3::kUnitY, pitch) *
                    Isometry::RotateAround(Vector3::kUnitZ, yaw));
  }

}  // namespace math
}  // namespace ekumen
