
// Standard.
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include <dirent.h>
#include <memory>

// Utility.
#include "global_constants.hpp"
#include "global_header.hpp"
#include "utility_header.hpp"
#include "util.h"

// Namespace.
using namespace std;

// Include.
#include "secondOrderActuator.h"
#include "randomNumbers.h"
#include "atm1976_metric.h"
#include "atmNASA_imperial.h"

int main()
{

    class x
    {
    public:
        x() {};
        double arr[3] = {0.0, 0.0, 0.0};
    };

    x one = x{};
    consolePrintArray("one", one.arr);

    x two = one;
    two.arr[0] = 1.0;
    consolePrintArray("two", two.arr);

    return 0;

}
