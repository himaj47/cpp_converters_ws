#pragma once

#include <string>
#include <vector>

#include "msg_pkg_three/headers/num.hpp"

namespace nspace {
    struct num
    {   
        float number;
        std::vector<ns1::num> arr;
    };
}