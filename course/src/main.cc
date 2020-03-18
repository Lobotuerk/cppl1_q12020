#include <iostream>
#include "isometry.h"

int main(int argc, char **argv) {
  (void) argc;
  (void) argv;

  ekumen::math::Vector3 v1{1., 2., 3.};
  const ekumen::math::Vector3 q{4., 5., 6.};

  v1 += q;
  std::cout << v1 << std::endl;
  return 0;
}
