#include <list>
#include <fstream>
#include <iostream>

using namespace std;

void writeParams(list<float> params);

int main(int argc, char *argv[])
{
    list<float> params;
    float next;
    ifstream params_in("walkparams");
    while(params_in.good()) {
        params_in >> next;
        params.push_back(next);
        cout << next << endl;
    }
    
    writeParams(params);
    
    return 0;
}

void writeParams(list<float> params) {
    
    params.pop_front();
    params.pop_front();
    params.pop_front();
    params.pop_front();
    params.pop_front();
    params.pop_front();
    ofstream file("BWalk.cfg");
    file << "BWalk" << endl;
    file << "Max Speeds (x cm/s, y cm/s, yaw rad/s): [10, 10, 0.5]" << endl;
    file << "Max Accelerations (x cm/s/s, y cm/s/s, yaw rad/s/s): [0.5, 0.5, 0.1]" << endl;
    file << "standComPositionZ: " << params.front() << " [110, 210] Blah" << endl;
    params.pop_front();
    file << "walkRefX: " << params.front() << " [5, 25] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkRefXAtFullSpeedX: " << params.front() << " [-3, 17] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkRefY: " << params.front() << " [30, 50] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkRefYAtFullSpeedX: " << params.front() << " [28, 48] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkRefYAtFullSpeedY: " << params.front() << " [30, 50] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkStepDuration: " << params.front() << " [350, 800] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkStepDurationAtFullSpeedX: " << params.front() << " [250, 1100] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkStepDurationAtFullSpeedY: " << params.front() << " [250, 1100] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkLiftOffset0: " << params.front() << " [-20, 20] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkLiftOffset1: " << params.front() << " [-25, 15] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkLiftOffset2: " << params.front() << " [5, 45] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkLiftOffsetJerk: " << params.front() << " [0, 5] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkLiftOffsetAtFullSpeedY0: " << params.front() << " [-20, 20] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkLiftOffsetAtFullSpeedY1: " << params.front() << " [-30, 10] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkLiftOffsetAtFullSpeedY2: " << params.front() << " [7, 47] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkLiftRotation0: " << params.front() << " [-0.25, 0.15] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkLiftRotation1: " << params.front() << " [-0.25, 0.15] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkLiftRotation2: " << params.front() << " [-0.2, 0.2]  Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkAntiLiftOffset0: " << params.front() << " [-20, 20] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkAntiLiftOffset1: " << params.front() << " [-20, 20] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkAntiLiftOffset2: " << params.front() << " [-20, 20] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkAntiLiftOffsetAtFullSpeedY0: " << params.front() << " [-20, 20] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkAntiLiftOffsetAtFullSpeedY1: " << params.front() << " [-20, 20] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkAntiLiftOffsetAtFullSpeedY2: " << params.front() << " [-20, 20] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "walkComBodyRotation: " << params.front() << " [-0.15, 0.25] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "speedMaxRot: " << params.front() << " [0, 1] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "speedMaxX: " << params.front() << " [10, 800] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "speedMaxY: " << params.front() << " [10, 400] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "speedMaxMinRot: " << params.front() << " [0, 1] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "speedMaxMinX: " << params.front() << " [0, 800] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "speedMaxMinY: " << params.front() << " [0, 400] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "speedMaxBackwards: " << params.front() << " [0, 600] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "speedMaxChangeRot: " << params.front() << " [0, 2] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "speedMaxChangeX: " << params.front() << " [0, 90] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "speedMaxChangeY: " << params.front() << " [0, 90] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "balanceComXP: " << params.front() << " [0, 0.3] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "balanceComXD: " << params.front() << " [-0.3, 0] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "balanceComYP: " << params.front() << " [0, 0.3] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "balanceComYD: " << params.front() << " [-0.3, 0] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "balanceComZP: " << params.front() << " [0, 0.3] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "balanceComZD: " << params.front() << " [-0.3, 0] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "balanceBodyRotationXP: " << params.front() << " [0, 0.3] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "balanceBodyRotationXD: " << params.front() << " [-0.3, 0] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "balanceBodyRotationYP: " << params.front() << " [0, 0.3] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "balanceBodyRotationYD: " << params.front() << " [-0.3, 0] Blahdeblah blah blah" << endl;
    params.pop_front();
    file << "ArmGains (%): [[30, 30, 30]]" << endl;
    file << "TorsoGains (%): []" << endl;
    file << "LegGains (%): [[" << params.front() << ", ";
    params.pop_front();
    file << params.front() << ", ";
    params.pop_front();
    file << params.front() << ", ";
    params.pop_front();
    file << params.front() << ", ";
    params.pop_front();
    file << params.front() << ", ";
    params.pop_front();
    file << params.front() << "]]" << endl;
}



