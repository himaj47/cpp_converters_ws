#pragma once
#include "type_from_two.hpp"
#include <vector>

namespace nspace {
    struct type_from_two
    {
        int data;
        std::vector<ns::type_from_two> vec;
    };
    
}