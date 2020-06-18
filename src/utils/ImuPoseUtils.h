#pragma once
#include <iostream>
#include <string>
#include <queue>

using namespace std;

#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>

class ImuPoseUtils
{
public:
    static void geometry_msgs_Pose_to_eigenmat( const geometry_msgs::Pose& pose, Matrix4d& dstT )
    {
        Quaterniond q = Quaterniond( pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z  );

        dstT = Matrix4d::Zero();
        dstT.topLeftCorner<3,3>() = q.toRotationMatrix();

        dstT(0,3) = pose.position.x;
        dstT(1,3) = pose.position.y;
        dstT(2,3) = pose.position.z;
        dstT(3,3) = 1.0;
    }

    static void eigenmat_to_geometry_msgs_Pose( const Matrix4d& T, geometry_msgs::Pose& pose )
    {
        assert( T(3,3) == 1 );
        Quaterniond q( T.topLeftCorner<3,3>() );

        pose.position.x = T(0,3);
        pose.position.y = T(1,3);
        pose.position.z = T(2,3);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

    }

    static void sensor_msgs_Imu_to_eigenmat( sensor_msgs::Imu::ConstPtr imu_msg,
        Vector3d& ang_vel, Vector3d& lin_acc )
    {
        ang_vel = Vector3d( imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z );
        lin_acc = Vector3d( imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z );
    }



    /** @brief converts a rotation matrix to ZXY Euler angles.
    output rpy need to be in radians.
    */
    static Eigen::Vector3d rotation2EulerZXY(const Eigen::Matrix3d& R)
    {
      #if 0
      double phi = asin(R(2, 1));
      // double theta = atan(-R(2, 0) / R(2, 2));
      // double psi = atan(-R(0, 1) / R(1, 1));
      double theta = atan2(-R(2, 0), R(2, 2));
      double psi = atan2(-R(0, 1), R(1, 1));

      return Eigen::Vector3d(phi, theta, psi);
      #endif


      #if 1
      const double pi = 3.14159265359;
      double thetaX, thetaY, thetaZ;
      if( R(2,1)< +1 ){

      	if( R(2,1) > -1 )
      	{
      		thetaX=asin(R(2,1)) ;
      		thetaZ=atan2(-R(0,1) , R(1,1) ) ;
      		thetaY=atan2(-R(2,0) , R(2,2) ) ;
      	}
      	else //  r21 =−1
      	{
      		// Not a  unique  s o l u t i o n :   thetaY−thetaZ = atan2 ( r02 , r00 )
      		thetaX =-pi /2;
      		thetaZ =-atan2( R(0,2) , R(0,0) ) ;
      		thetaY = 0;
      	}
      }
      else //  r21 = +1
      {
      	// Not a  unique  solution:   thetaY + thetaZ = atan2 ( r02 , r00 )
      	thetaX = +pi /2;
      	thetaZ = atan2(R(0,2),R(0,0)) ;
      	thetaY = 0;
      }

      double phi = thetaX;
      double theta = thetaY;
      double psi = thetaZ;
      return Eigen::Vector3d(phi, theta, psi);

      #endif


    }

    /** @brief converts a euler angle representation to rotation matrix
    output rpy need to be in radians.
    */
    static Eigen::Matrix3d eulerZXY2Rotation( const Eigen::Vector3d& e  )
    {

        double phi   = e(0);
        double theta = e(1);
        double psi   = e(2);


        Eigen::Quaterniond q = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY());

        Eigen::Matrix3d Rzxy = q.toRotationMatrix();

        return Rzxy;
    }


    /** @brief given a roll-pitch-yaw returns a transformation matrix (4x4).
    input rpy need to be in radians.
    */
    static Eigen::Matrix4d rpyt_to_transform( const Eigen::Vector3d& rpy, const Eigen::Vector3d& t )
    {
        Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
        M.topLeftCorner<3,3>() = eulerZXY2Rotation( rpy );
        M.col(3) << t, 1.0;
        return M;
    }


    /** @brief given a transform returns a roll-pitch-yaw
    output rpy need to be in radians.
    */
    static void transform_to_rpyt( const Eigen::Matrix4d& transform, Eigen::Vector3d& rpy, Eigen::Vector3d& t )
    {
        Eigen::Matrix3d R = transform.topLeftCorner<3,3>();
        rpy = rotation2EulerZXY( R );
        t = transform.col(3).topRows(3);
    }



    static Eigen::Vector3d deg2rads( Eigen::Vector3d& rpy_deg )
    {
        const double _PI_ = 3.14159265359;
        return rpy_deg / 180. * _PI_;
    }

    static double deg2rads( double ang_deg )
    {
        const double _PI_ = 3.14159265359;
        return ang_deg / 180. * _PI_;
    }

    static Eigen::Vector3d rads2deg( Eigen::Vector3d& rpy )
    {
        const double _PI_ = 3.14159265359;
        return rpy / _PI_ * 180.;
    }

    static double rads2deg( double ang_rads )
    {
        const double _PI_ = 3.14159265359;
        return ang_rads / _PI_ * 180.;
    }



    static std::string prettyPrint( const Eigen::Matrix4d& M, bool ang_in_deg=true, bool ang_in_rads=false )
    {
        Eigen::Vector3d rpy_cap, t_cap;
        transform_to_rpyt( M, rpy_cap, t_cap );

        std::stringstream ss;

        if( ang_in_deg )
            ss << "rpy(deg):\t" << rads2deg(rpy_cap).transpose() << ";\t";

        if( ang_in_rads )
            ss << "rpy(rads):\t" << rpy_cap.transpose() << ";\t";

        ss << "t:\t" << t_cap.transpose() << ";";

        return ss.str();
    }

    static std::string prettyPrint( const Eigen::Matrix3d& R, bool ang_in_deg=true, bool ang_in_rads=false )
    {
        Eigen::Vector3d rpy_cap;
        rpy_cap = rotation2EulerZXY( R );

        std::stringstream ss;

        if( ang_in_deg )
            ss << "rpy(deg):\t" << rads2deg(rpy_cap).transpose() << ";\t";

        if( ang_in_rads )
            ss << "rpy(rads):\t" << rpy_cap.transpose() << ";\t";


        return ss.str();
    }


    //---------------
    // quaternion <---> Matrix4d
    static void raw_wxyz_to_eigenmat( const double * quat, const double * t, Matrix4d& dstT )
    {
      Quaterniond q = Quaterniond( quat[0], quat[1], quat[2], quat[3] );

      dstT = Matrix4d::Zero();
      dstT.topLeftCorner<3,3>() = q.toRotationMatrix();

      dstT(0,3) = t[0];
      dstT(1,3) = t[1];
      dstT(2,3) = t[2];
      dstT(3,3) = 1.0;
    }


    static void eigenmat_to_raw_wxyz( const Matrix4d& T, double * quat, double * t)
    {
      assert( T(3,3) == 1 );
      Quaterniond q( T.topLeftCorner<3,3>() );
      quat[0] = q.w();
      quat[1] = q.x();
      quat[2] = q.y();
      quat[3] = q.z();
      t[0] = T(0,3);
      t[1] = T(1,3);
      t[2] = T(2,3);
    }




    static void raw_xyzw_to_eigenmat( const double * quat, const double * t, Matrix4d& dstT )
    {
      Quaterniond q = Quaterniond( quat[3], quat[0], quat[1], quat[2] );

      dstT = Matrix4d::Zero();
      dstT.topLeftCorner<3,3>() = q.toRotationMatrix();

      dstT(0,3) = t[0];
      dstT(1,3) = t[1];
      dstT(2,3) = t[2];
      dstT(3,3) = 1.0;
    }

    static void eigenmat_to_raw_xyzw( const Matrix4d& T, double * quat, double * t)
    {
      assert( T(3,3) == 1 );
      Quaterniond q( T.topLeftCorner<3,3>() );
      quat[0] = q.x();
      quat[1] = q.y();
      quat[2] = q.z();
      quat[3] = q.w();
      t[0] = T(0,3);
      t[1] = T(1,3);
      t[2] = T(2,3);
    }



//-------------eigen

    static void raw_wxyz_to_eigenmat( const Vector4d& quat, const Vector3d& t, Matrix4d& dstT )
    {
      Quaterniond q = Quaterniond( quat(0), quat(1), quat(2), quat(3) );

      dstT = Matrix4d::Zero();
      dstT.topLeftCorner<3,3>() = q.toRotationMatrix();

      dstT(0,3) = t(0);
      dstT(1,3) = t(1);
      dstT(2,3) = t(2);
      dstT(3,3) = 1.0;
    }


    static void eigenmat_to_raw_wxyz( const Matrix4d& T, Vector4d& quat, Vector3d& t)
    {
      assert( T(3,3) == 1 );
      Quaterniond q( T.topLeftCorner<3,3>() );
      quat(0) = q.w();
      quat(1) = q.x();
      quat(2) = q.y();
      quat(3) = q.z();
      t(0) = T(0,3);
      t(1) = T(1,3);
      t(2) = T(2,3);
    }



    static void raw_xyzw_to_eigenmat( const Vector4d& quat, const Vector3d& t, Matrix4d& dstT )
    {
      Quaterniond q = Quaterniond( quat(3), quat(0), quat(1), quat(2) );

      dstT = Matrix4d::Zero();
      dstT.topLeftCorner<3,3>() = q.toRotationMatrix();

      dstT(0,3) = t(0);
      dstT(1,3) = t(1);
      dstT(2,3) = t(2);
      dstT(3,3) = 1.0;
    }

    static void eigenmat_to_raw_xyzw( const Matrix4d& T, Vector4d& quat, Vector3d& t)
    {
      assert( T(3,3) == 1 );
      Quaterniond q( T.topLeftCorner<3,3>() );
      quat(0) = q.x();
      quat(1) = q.y();
      quat(2) = q.z();
      quat(3) = q.w();
      t(0) = T(0,3);
      t(1) = T(1,3);
      t(2) = T(2,3);
    }

};
