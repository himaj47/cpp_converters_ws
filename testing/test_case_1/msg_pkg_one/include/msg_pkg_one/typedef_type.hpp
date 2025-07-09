#pragma once
#include <vector>
#include <cstdint>

namespace ns {

    using funType = int;
    using funType2 = float;

    struct type_from_one
    {
        funType a;
        funType2 b;
        std::vector<funType2> c;
    };
}
