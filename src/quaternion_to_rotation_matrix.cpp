#include <iostream>

using namespace std;

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

//#include "utils/PoseManipUtils.h"
#include "utils/ImuPoseUtils.h"

int main()
{
    //---- INPUT
    double qx,qy,qz,qw;
    double tx,ty,tz;



    qx= 1.1583577112806141e-03;
    qy= 1.0518165433133224e-03;
    qz= -4.1365397131674534e-04;
    qw= 9.9999869038902467e-01;
    tx= -6.0656642301473683e-02;
    ty= -3.3383976377090237e-04;
    tz= 7.8073133976738231e-04;



    //---- Process
    Matrix4d dst;
    ImuPoseUtils::raw_wxyz_to_eigenmat( Vector4d(qw,qx,qy,qz), Vector3d(tx,ty,tz), dst );



    //---- Output
    cout << "dst:=" << ImuPoseUtils::prettyPrint( dst ) << endl;
    cout << "dst:=\n" << dst << endl;

}
