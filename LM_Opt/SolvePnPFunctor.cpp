#include "SolvePnPFunctor.h"

int SolvePnPFunctor::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
  // Projection = Cam_Calib * [R|T] * 3D Position
  // The incoming vector x has the following composition:
  // x[5],[4],[3] are the angles of rotation in XYZ Extrinsic Euler notation
  // x[0],[1],[2] describe the translation vector

  Eigen::Matrix3f rotation_mat;
     rotation_mat = Eigen::AngleAxisf(x(3), Eigen::Vector3f::UnitZ())
     * Eigen::AngleAxisf(x(4), Eigen::Vector3f::UnitY())
     * Eigen::AngleAxisf(x(5), Eigen::Vector3f::UnitX());

  Eigen::Translation3f translation_mat(x(0),x(1),x(2));
  Eigen::Affine3f transformation_mat;
  transformation_mat= translation_mat * rotation_mat;

 for(int i=0; i<values()/2;i++)
  {
     //The length of fvec is twice the number of correspondances input
     //We iterate through the correspondences and construct fvec

     //Get object point from the object_points vector of vectors
     Eigen::Vector3f object_point;
     object_point= object_points.row(i);
     //Compute the coordintates of the object point in camera coordinates
     Eigen::Vector3f object_point_cam_coord;
     object_point_cam_coord= transformation_mat * object_point;

     //Get the expected projection point from the projection_points vector
     Eigen::Vector3f projection_point = projection_points.row(i);
     //fvec carries the difference between x coordinates and y coordinates
     //of the projection and the expected projection
     fvec[2*i] = (projection_point[0] - object_point_cam_coord[0]/object_point_cam_coord[2]);
     fvec[2*i+1] = (projection_point[1] - object_point_cam_coord[1]/object_point_cam_coord[2]);
  }
  return 0;
}
int SolvePnPFunctor::df(const Eigen::VectorXd &x, JacobianType &fjac) const
{
    //Let the angle of rotation about X, Y and Z be theta, phi and psi
    Eigen::AngleAxisf temp_aa;
    temp_aa = Eigen::AngleAxisf(x(5), Eigen::Vector3f::UnitX());
    Eigen::Matrix3f R_theta = temp_aa.toRotationMatrix();
    temp_aa = Eigen::AngleAxisf(x(4), Eigen::Vector3f::UnitY());
    Eigen::Matrix3f R_phi = temp_aa.toRotationMatrix();
    temp_aa = Eigen::AngleAxisf(x(3), Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f R_psi = temp_aa.toRotationMatrix();

    Eigen::Matrix3f R_theta_dot;
    R_theta_dot << 0, 0, 0,
                  0, -sin(x(5)), -cos(x(5)),
                  0,cos(x(5)), -sin(x(5));

    Eigen::Matrix3f R_phi_dot;
    R_phi_dot << -sin(x(4)), 0,  cos(x(4)),
                  0 , 0 , 0,
                  -cos(x(4)), 0, -sin(x(4));

    Eigen::Matrix3f R_psi_dot;
    R_psi_dot << -sin(x(3)), -cos(x(3)), 0,
                  cos(x(3)), -sin(x(3)), 0,
                   0, 0, 0;

    Eigen::RowVector3f R1,R2,R3; //Row vectors of the rotation Matrix
    Eigen::RowVector3f R1_dtheta, R2_dtheta, R3_dtheta; //Row vectors of dR/dtheta
    Eigen::RowVector3f R1_dphi, R2_dphi, R3_dphi;
    Eigen::RowVector3f R1_dpsi, R2_dpsi, R3_dpsi;

    Eigen::Matrix3f temp_rot_matrix = R_psi * R_phi * R_theta;
    R1=temp_rot_matrix.row(0);
    R2=temp_rot_matrix.row(1);
    R3=temp_rot_matrix.row(2);

    temp_rot_matrix = R_psi * R_phi * R_theta_dot;
    R1_dtheta=temp_rot_matrix.row(0);
    R2_dtheta=temp_rot_matrix.row(1);
    R3_dtheta=temp_rot_matrix.row(2);

    temp_rot_matrix = R_psi * R_phi_dot * R_theta;
    R1_dphi=temp_rot_matrix.row(0);
    R2_dphi=temp_rot_matrix.row(1);
    R3_dphi=temp_rot_matrix.row(2);

    temp_rot_matrix = R_psi_dot * R_phi * R_theta;
    R1_dpsi=temp_rot_matrix.row(0);
    R2_dpsi=temp_rot_matrix.row(1);
    R3_dpsi=temp_rot_matrix.row(2);

    for(int i=0; i<values()/2;i++)
     {
        //Get object point from the object_points vector of vectors
        Eigen::Vector3f object_point;
        object_point= object_points.row(i);

        double denom = R3 * object_point + x(2);

        fjac(2*i,0) = - 1.0/ denom;
        fjac(2*i,1) = 0;
        fjac(2*i+1,0) = 0;
        fjac(2*i+1,1) = - 1.0/denom;

        double r1t1 = R1 * object_point + x(0);
        double r2t2 = R2 * object_point + x(1);

        fjac(2*i,2) = r1t1/(denom*denom);
        fjac(2*i+1,2) = r2t2/(denom*denom);

        //More temp variables
        double temp1, temp2, temp3;
        temp1 = R1_dpsi * object_point;
        temp2 = R2_dpsi * object_point;
        temp3 = R3_dpsi * object_point;       //Psi
        fjac(2*i,3) = (r1t1 * temp3) /(denom*denom) - (temp1)/denom;
        fjac(2*i+1,3) =(r2t2 * temp3) /(denom*denom) - (temp2)/denom;
        temp1 = R1_dphi * object_point;
        temp2 = R2_dphi * object_point;
        temp3 = R3_dphi * object_point;       //Phi
        fjac(2*i,4) = (r1t1 * temp3) /(denom*denom) - (temp1)/denom;
        fjac(2*i+1,4) =(r2t2 * temp3) /(denom*denom) - (temp1)/denom;
        temp1 = R1_dtheta * object_point;
        temp2 = R2_dtheta * object_point;
        temp3 = R3_dtheta * object_point;        //Theta
        fjac(2*i,5) = (r1t1 * temp3) /(denom*denom) - (temp1)/denom;
        fjac(2*i+1,5) =(r2t2 * temp3) /(denom*denom) - (temp1)/denom;

    }
     return 0;
}
