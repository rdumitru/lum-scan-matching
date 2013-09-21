//==============================================================================
// Includes.
//==============================================================================
// User includes.
#include <reader/PclReader.h>

// C++ includes.
#include <iostream>
using namespace std;

//==============================================================================
// Main.
//==============================================================================
int main(int argc, char* argv[]) {
    PclReader reader;
    reader.read("/media/Mobile/Scans/lum", 0, 3);
    reader.run();
    reader.printPc("/home/rdumitru/Downloads/scan000.3d");

    cout << "Program end..." << endl;
}

