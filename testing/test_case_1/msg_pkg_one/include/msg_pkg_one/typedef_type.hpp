#pragma once
#include <vector>
#include <cstdint>

namespace ns {

    using funType = int;
    using funType2 = float;

    struct typedef_type
    {
        funType a;
        funType2 b;
        std::vector<funType2> c;
    };
}
