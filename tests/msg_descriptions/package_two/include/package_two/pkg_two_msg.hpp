#pragma once

#include <string>
#include <vector>

#include "package_one/package_one/headers/pkg_one_msg.hpp"

namespace nspace {
    struct pkg_two_msg
    {   
        float c;
        std::vector<ns::pkg_one_msg> d;
    };
}