#include <isometry.h>

namespace ekumen {
  namespace math {

  const Vector3 Vector3::kUnitX(1.,0,0);
  const Vector3 Vector3::kUnitY(0,1.,0);
  const Vector3 Vector3::kUnitZ(0,0,1.);
  const Vector3 Vector3::kZero(0,0,0);

  Vector3 Vector3::operator + (const Vector3 & obj) const
  {
    return Vector3( x_ + obj.x(), y_ + obj.y(),z_ +  obj.z());
  }

  Vector3 Vector3::operator - (const Vector3 & obj) const
  {
    return Vector3( x_ - obj.x(), y_ - obj.y(),z_ -  obj.z());
  }

  bool Vector3::operator == (const std::initializer_list<double> & obj) const
  {
    if (obj.size() != 3) return false;
    int i = 0;
    for (auto & e : obj)
    {
      if (*data_[i] != e) return false;
      i++;
    }
    return true;
  }

  bool Vector3::operator == (const Vector3 & obj) const
  {
    if (obj.x() != x_ || obj.y() != y_ || obj.z() != z_) return false;
    return true;
  }

  bool Vector3::operator != (const std::initializer_list<double> & obj) const
  {
    if (obj.size() != 3) return true;
    int i = 0;
    for (auto & e : obj)
    {
      if (*data_[i] != e) return true;
      i++;
    }
    return false;
  }

  bool Vector3::operator != (const Vector3 & obj) const
  {
    if (obj.x() != x_ || obj.y() != y_ || obj.z() != z_) return true;
    return false;
  }

  Vector3 Vector3::operator * (const Vector3 & obj) const
  {
    return Vector3(x_ * obj.x(), y_ * obj.y(), z_ * obj.z());
  }

  Vector3 Vector3::operator / (const Vector3 & obj) const
  {
    return Vector3(x_ / obj.x(), y_ / obj.y(), z_ / obj.z());
  }

  double Vector3::norm() const
  {
    return sqrt( std::pow(x_, 2.0) + std::pow(y_, 2.0) + std::pow(z_, 2.0) );
  }

  Vector3 operator * (const int obj1, const Vector3 & obj2)
  {
    return Vector3(obj2.x() * obj1, obj2.y() * obj1, obj2.z() * obj1);
  }

  Vector3 operator * (const Vector3 & obj1, const double obj2)
  {
    return Vector3(obj1.x() * obj2, obj1.y() * obj2, obj1.z() * obj2);
  }

  double & Vector3::operator [] (const int index) const
  {
    if (index > 2 || index < 0)
    {
      throw std::runtime_error("Index not valid");
    }
    return *(data_[index]);
  }

  double & Vector3::operator [] (const int index)
  {
    if (index > 2 || index < 0)
    {
      throw std::runtime_error("Index not valid");
    }
    return *(data_[index]);
  }

  Vector3 Vector3::cross(const Vector3 & obj) const
  {
    return Vector3(
    y_ * obj.z_ - z_ * obj.y_,
    z_ * obj.x_ - x_ * obj.z_,
    x_ * obj.y_ - y_ * obj.x_);
  }

  double Vector3::dot(const Vector3 & obj) const
  {
    return (x_ * obj.x_) + (y_ * obj.y_) + (z_ * obj.z_);
  }


}
}
