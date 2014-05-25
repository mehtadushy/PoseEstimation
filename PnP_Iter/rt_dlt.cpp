#include "rt_dlt.h"
#include <Eigen/SVD>
#include <iostream>
#include <Eigen/Core>

using namespace Eigen;
//A modified form of DLT that estimates the Rotation and Translation components of
//the projection matrix (the extrinsic part), when provided with 3D points and their
//normalized projections in camera coordinates
//We are operating under the assumption that the input points are non coplanar.

Eigen::Matrix<double,6,1> rt_dlt(const Eigen::MatrixX3f & obj_pts,const Eigen::MatrixX3f & proj_pts)
{
  //Check the number of points that we have available as inputs
  //Demand that there be atleast 6 points input.
    if(obj_pts.rows()<6)
    {
        std::cout << "Need a minimum of 6 points\n";
    }
    if(obj_pts.rows()!=proj_pts.rows())
    {
        std::cout << "Need and equal nummber of Object Points and Normalized Projection Points\n";
    }

    //To ensure numerical stability, we expect the model coordinates to be well behaved as well
    //(Small values, close to 1)

    //Create a vector of size 12, that would hold the rotation and translation elements, estimate
    //it using DLT (SVD and other magic)
    Eigen::VectorXf v(12);
    Eigen::MatrixXf M(2*obj_pts.rows(),12);
    //
    //See http://www.maths.lth.se/matematiklth/personal/calle/datorseende13/notes/forelas3.pdf for
    //details
    //Populate M
    for(int i=0; i< obj_pts.rows(); i++)
    {
        //Eigen::RowVectorXf & Xrow(12);
        //Xrow = M.row(2*i);
        //Eigen::RowVectorXf & Yrow(12);
        //Yrow= M.row(2*i+1);
        Eigen::Vector3f ob_pt = obj_pts.row(i);
        Eigen::Vector3f pr_pt = proj_pts.row(i);

        M(2*i,0) = -ob_pt(0);            //-X
        M(2*i,1) = -ob_pt(1);            //-Y
        M(2*i,2) = -ob_pt(2);            //-Z
        M(2*i,3) = 0;
        M(2*i,4) = 0;
        M(2*i,5) = 0;
        M(2*i,6) = ob_pt(0)*pr_pt(0);    //Xx
        M(2*i,7) = ob_pt(1)*pr_pt(0);    //Yx
        M(2*i,8) = ob_pt(2)*pr_pt(0);    //Zx
        M(2*i,9) = -1;
        M(2*i,10) = 0;
        M(2*i,11) = pr_pt(0);            //x

        M(2*i+1,3) = -ob_pt(0);            //-X
        M(2*i+1,4) = -ob_pt(1);            //-Y
        M(2*i+1,5) = -ob_pt(2);            //-Z
        M(2*i+1,0) = 0;
        M(2*i+1,1) = 0;
        M(2*i+1,2) = 0;
        M(2*i+1,6) = ob_pt(0)*pr_pt(1);    //Xy
        M(2*i+1,7) = ob_pt(1)*pr_pt(1);    //Yy
        M(2*i+1,8) = ob_pt(2)*pr_pt(1);    //Zy
        M(2*i+1,9) = 0;
        M(2*i+1,10) = -1;
        M(2*i+1,11) = pr_pt(1);            //y

    }

    //Compute M'M
    Eigen::MatrixXf MTM;
    MTM = M.transpose()*M;

    //Do SVD of MTM
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(MTM, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //Get the eigenvector corresponding to the smallest eigenvalue
    Eigen::MatrixXf Matv = svd.matrixV();
    v = Matv.col(Matv.cols()-1);

    Eigen::Matrix3f rot_mat;
    rot_mat<< v(0),v(1),v(2),v(3),v(4),v(5),v(6),v(7),v(8);
    Eigen::Vector3f euler = rot_mat.eulerAngles(3,2,1);
     //The vector to be returned carries 3 euler angles and 3 translation elements
    Eigen::VectorXd x(6);
    x<< v(9),v(10),v(11),euler(0),euler(1),euler(2);
    return(x);

}
