#include "ik.hpp"
#define ZEROERR_IMPLEMENTATION
#include "zeroerr.hpp"

using namespace zeroerr;

TEST_CASE("1. basic") {
    ik::vec3 v1 = {1, 2, 3};
    ik::vec3 v2 = {4, 5, 6};
    
    CHECK_EQ(v1 + v2, (ik::vec3{5, 7, 9}));
    CHECK_EQ(v1 - v2, (ik::vec3{-3, -3, -3}));

    CHECK_EQ(v1 * 2.0f, (ik::vec3{2, 4, 6}));
    CHECK_EQ(2.0f * v1, (ik::vec3{2, 4, 6}));

    CHECK_EQ(v1 / 2.0f, (ik::vec3{0.5, 1, 1.5}));
    CHECK_EQ(2.0f / v1, (ik::vec3{2, 1, 2.0/3}));

    
    
}