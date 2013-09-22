//==============================================================================
// Includes.
//==============================================================================
// User includes.
#include <reader/TdtkReader.h>

// C++ includes.
#include <iostream>
using namespace std;

//==============================================================================
// Main.
//==============================================================================
int main(int argc, char* argv[]) {
    TdtkReader reader;
    reader.read("/home/cprodescu/Dropbox/PhotosRemus/lum/", 0, 3);
    reader.run();

    cout << "Program end..." << endl;
}

