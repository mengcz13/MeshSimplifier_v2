#include <iostream>
#include <cstdlib>
#include "simplifier.h"
using namespace std;

int main(int argc, char** argv)
{
    if (argc != 4) {
        cout << "Usage: ./main <input> <output> rate" << endl;
    }
    Simplifier simplifier;
    simplifier.parser(argv[1]);
    simplifier.init();
    simplifier.simplify(atof(argv[3]));
    simplifier.writer(argv[2]);
    return 0;
}

