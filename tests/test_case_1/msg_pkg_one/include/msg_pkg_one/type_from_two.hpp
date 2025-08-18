#pragma once
#include "type_from_one.hpp"
#include <vector>

namespace ns {
    struct type_from_two
    {
        int data;
        ns1::type_from_one type_from_one;
    };
    
}