#include "iotests.h"
#include "Localisation/SelfLocalisation.h"

#include <fstream>
#include <string>
void runTests()
{
    const std::string filename = "temp.bin";
    SelfLocalisation* testLoc = new SelfLocalisation(2);
    testLoc->doInitialReset(GameInformation::RedTeam);

    std::ofstream outFile(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    testLoc->writeStreamBinary(outFile);
    outFile.close();

    SelfLocalisation* copyLoc = new SelfLocalisation(2);
    std::ifstream inFile(filename.c_str());
    copyLoc->readStreamBinary(inFile);
    inFile.close();
    bool equal = (*testLoc == *copyLoc);

    std::cout << "Localisation is " << (equal?"":"NOT ") << "equal.";

    delete testLoc;
    delete copyLoc;
}
