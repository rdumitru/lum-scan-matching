#ifndef _COMMON_H
#define _COMMON_H
//==============================================================================
// Includes.
//==============================================================================
// C++ includes.
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>

// C includes.
#include <stdlib.h>
#include <time.h>
#include <math.h>

//==============================================================================
// Functions.
//==============================================================================
static inline std::string int2String(const int &toConvert, const int &nrDigits)
{
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(nrDigits) << toConvert;
    return ss.str();
}

static inline void die(const std::string &msg) {
    std::cerr << msg << std::endl;
    exit(EXIT_FAILURE);
}

static inline double deg2Rad(const double &val) {
    double rad = (val / 180.0) * M_PI;
    return fmod(rad, 2 * M_PI);
}

static inline double rad2Deg(const double &val) {
    double deg = (val / M_PI) * 180.0;
    return fmod(deg, 360.0);
}

#endif
