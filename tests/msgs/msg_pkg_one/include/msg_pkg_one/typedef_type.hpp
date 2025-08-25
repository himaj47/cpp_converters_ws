#pragma once
#include <vector>
#include <cstdint>
#include "type_from_one.hpp"

namespace ns {

    using funType = int;
    using funType2 = float;

    struct typedef_type
    {
        funType a;
        funType2 b;
        std::vector<funType2> c;
        ns1::type_from_one d;
    };
}
