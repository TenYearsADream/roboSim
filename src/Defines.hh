#pragma once

#include <iostream>

#define DEBUG_VAR(a) std::cout << #a << ": " << (a) << std::endl
#define DEBUG_VEC(a) std::cout << #a << ": "; for (size_t asdfasdfasdfasdf = 0; asdfasdfasdfasdf < (a).size(); asdfasdfasdfasdf++) std::cout << (a)[asdfasdfasdfasdf] << ", "; std::cout << "\b\b" << std::endl
#define DEBUG_MAT(a) std::cout << #a << ": " << std::endl << (a) << std::endl
#define DEBUG_SET(a) std::cout << #a << ": "; for (auto itr = (a).begin(); itr != (a).end(); ++itr) { std::cout << *itr << ", "; } std::cout << std::endl
