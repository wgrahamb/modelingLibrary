
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

    Matrix ecef(3, 1);
    double lat;
    double lon;
    double alt;

    ecef.assign_loc(0, 0, 1113739.6669);
    ecef.assign_loc(1, 0, 4845855.8020);
    ecef.assign_loc(2, 0, 3981710.342);

    cad_geo84_in(lat, lon, alt, ecef, 0.0);

    cout << lat << "\n";
    cout << lon << "\n";
    cout << alt << "\n";

    return 0;

}
