#ifndef VECTOR_OPS_HPP
#define VECTOR_OPS_HPP

#pragma once
#include <vector>

namespace util {

std::vector<int> arange(int n) {
  std::vector<int> out(n);
  for (int i=0; i < n; ++i) out[i] = i;
  return out;
}


}

#endif // VECTOR_OPS_HPP