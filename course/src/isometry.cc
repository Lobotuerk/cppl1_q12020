#include <isometry.h>

namespace ekumen {
  namespace math {

  //Vector3

  const Vector3 Vector3::kUnitX(1., 0., 0.);
  const Vector3 Vector3::kUnitY(0., 1., 0.);
  const Vector3 Vector3::kUnitZ(0., 0., 1.);
  const Vector3 Vector3::kZero(0., 0., 0.);

  Vector3 Vector3::operator += (const Vector3 & obj) const
  {
    return Vector3( x_ + obj.x(), y_ + obj.y(),z_ +  obj.z());
  }

  Vector3 Vector3::operator -= (const Vector3 & obj) const
  {
    return Vector3( x_ - obj.x(), y_ - obj.y(),z_ -  obj.z());
  }

  bool Vector3::operator == (const std::initializer_list<double> & obj) const
  {
    if (obj.size() != 3)
    {
      throw std::runtime_error("Size of list not valid");
    }
    if ( (obj.begin()[0] != x_) || (obj.begin()[1] != y_) || (obj.begin()[2] != z_) )
    {
      return false;
    }
    return true;
  }

  bool Vector3::operator == (const Vector3 & obj) const
  {
    if ( (obj[0] != x_) || (obj[1] != y_) || (obj[2] != z_) )
    {
      return false;
    }
    return true;
  }

  Vector3 Vector3::operator *= (const Vector3 & obj) const
  {
    return Vector3(x_ * obj.x(), y_ * obj.y(), z_ * obj.z());
  }

  Vector3 Vector3::operator /= (const Vector3 & obj) const
  {
    return Vector3(x_ / obj.x(), y_ / obj.y(), z_ / obj.z());
  }

  Vector3 Vector3::operator *= (const double & obj) const
  {
    return Vector3(x_ * obj, y_ * obj, z_ * obj);
  }

  Vector3 Vector3::operator /= (const double & obj) const
  {
    return Vector3(x_ / obj, y_ / obj, z_ / obj);
  }


  double Vector3::norm() const
  {
    return sqrt( std::pow(x_, 2.0) + std::pow(y_, 2.0) + std::pow(z_, 2.0) );
  }

  Vector3 operator * (const double & obj1, const Vector3 & obj2)
  {
    return Vector3(obj2.x() * obj1, obj2.y() * obj1, obj2.z() * obj1);
  }

  const double & Vector3::operator [] (const int index) const
  {
    if (index > 2 || index < 0)
    {
      throw std::runtime_error("Index not valid");
    }
    else if (index == 0)
    {
      return x_;
    }
    else if (index == 1)
    {
      return y_;
    }
    else
    {
      return z_;
    }
  }

  double & Vector3::operator [] (const int index)
  {
    if (index > 2 || index < 0)
    {
      throw std::runtime_error("Index not valid");
    }
    else if (index == 0) return x_;
    else if (index == 1) return y_;
    else return z_;
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
