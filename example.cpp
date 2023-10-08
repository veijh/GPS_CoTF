#include <iostream>
#include "GPS_CoTF.h"

using namespace std;
int main() {
    Eigen::Vector2d ENU_LALO(39.955225, 116.263203);
    vector<Eigen::Vector2d> MSN_LALO;
    vector<Eigen::Vector2d> MSN_XY;
    MSN_LALO.push_back(Eigen::Vector2d(39.955336, 116.263293));
    MSN_XY.push_back(Eigen::Vector2d(0.0, 20.0));
    MSN_LALO.push_back(Eigen::Vector2d(39.955315, 116.263518));
    MSN_XY.push_back(Eigen::Vector2d(20.0, 20.0));
    MSN_LALO.push_back(Eigen::Vector2d(39.955166, 116.263271));
    MSN_XY.push_back(Eigen::Vector2d(0.0, 0.0));
    MSN_LALO.push_back(Eigen::Vector2d(39.955142, 116.263495));
    MSN_XY.push_back(Eigen::Vector2d(20.0, 0.0));

    GPS_CoTF CoTF(5.0, ENU_LALO, MSN_LALO, MSN_XY);
    cout << "result: " << endl << frame_tfer.MSN_to_ENU(10.0, 10.0) << endl;
    return 0;
}
