//==============================================================================
// Includes.
//==============================================================================
// C++ includes.
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
using namespace std;

// C includes.
#include <stdlib.h>

//==============================================================================
// Functions.
//==============================================================================
static inline string int2String(const int &toConvert, const int &nrDigits)
{
    stringstream ss;
    ss << setfill('0') << setw(nrDigits) << toConvert;
    return ss.str();
}

static inline void die(const string &msg) {
    cerr << msg << endl;
    exit(EXIT_FAILURE);
}
