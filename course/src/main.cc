#include <iostream>
#include "isometry.h"

int main(int argc, char **argv) {
  (void) argc;
  (void) argv;
  std::cout << m1 * m2 << std::endl;
  const ekumen::math::Isometry t5{ekumen::math::Isometry::RotateAround(ekumen::math::Vector3::kUnitZ, M_PI / 8.)};

  std::cout << t5 << std::endl;
  return 0;
}
