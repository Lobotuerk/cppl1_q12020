#include <iostream>
#include "isometry.h"

int main(int argc, char **argv) {
  (void) argc;
  (void) argv;

  ekumen::math::Matrix3 m1{{1., 2., 3.}, {4., 5., 6.}, {7., 8., 9.}};
  const ekumen::math::Matrix3 m2{1., 2., 3., 4., 5., 6., 7., 8., 9.};

  std::cout << m1 * m2 << std::endl;
  return 0;
}
