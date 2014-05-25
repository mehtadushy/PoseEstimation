#include <QtCore/QCoreApplication>
#include <iostream>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<unsupported/Eigen/NonLinearOptimization>
#include "SolvePnPFunctor.h"
#include "rt_dlt.h"

#define USE_DLT false  //Use provided guess or use DLT Algorithm to initialize optimizer

using namespace Eigen;

int main(int argc, char *argv[])
{

    Eigen::MatrixX3f obj_pts;
    Eigen::MatrixX3f proj_pts;
    Matrix3f camMatrix;

    camMatrix.resize(3,3);
    obj_pts.resize(7,3);
    proj_pts.resize(7,3);

    obj_pts.row(0) << -0.61832,-1.68686, -0.49279;	// l eye (v 314)
    obj_pts.row(1) << 0.67145,-1.66963, -0.500858;	// r eye (v 0)
    obj_pts.row(2) << 0.038243,-2.23306,-1.10262;	//nose (v 1087)
    obj_pts.row(3) << -0.371717, -1.86736, -1.60827;	// l mouth (v 1537)
    obj_pts.row(4) << 0.415854, -1.83616, -1.61628;	// r mouth (v 704)
    obj_pts.row(5) << -1.88161, 0.316872, -0.215308;	// l ear (v )
    obj_pts.row(6) << 1.80743, 0.227834, -0.23995;	// r ear (v 1315)

    proj_pts.row(0) << 120, 200, 0;
    proj_pts.row(1) << 235, 210, 0;
    proj_pts.row(2) << 170, 297, 0;
    proj_pts.row(3) << 130, 330, 0;
    proj_pts.row(4) << 201, 335, 0;
    proj_pts.row(5) << 41, 141, 0;
    proj_pts.row(6) << 317, 156, 0;

    int imgRows = 474, imgCols=368;
    camMatrix << std::max(imgRows,imgCols), 0 , imgCols/2.0,
              0 , std::max(imgRows,imgCols), imgRows/2.0,
      0, 0, 1.0 ;

    int num_constraints = 2 * obj_pts.rows();

    //Bring projected points back to camera coordinate system
    MatrixX3f normalisedProjPoints= proj_pts;
    for(int i=0; i< num_constraints/2; i++)
    {
    normalisedProjPoints(i,0) = (normalisedProjPoints(i,0) - camMatrix(0,2))/camMatrix(0,0);
    normalisedProjPoints(i,1) = (normalisedProjPoints(i,1) - camMatrix(1,2))/camMatrix(1,1);
    }

    //The length 6 vector that we are going to minimize
    VectorXd x(6);

    //Initialize x to something.
#if USE_DLT
    x = rt_dlt(obj_pts,normalisedProjPoints);
#else
    //x << 0,0,0,0.0,0.0,1.5;
     x << -0.035,-1.25,5.8,0.0,0.0,1.5;
#endif

    Matrix3f init_rotation_mat;
    init_rotation_mat = Eigen::AngleAxisf(x(3), Vector3f::UnitZ())
    * Eigen::AngleAxisf(x(4), Vector3f::UnitY())
    * Eigen::AngleAxisf(x(5), Vector3f::UnitX());
    std::cout <<"Init Rot Mat " <<init_rotation_mat<<"\n";
    std::cout <<"Init X Vector "<<x.transpose()<<"\n";

    SolvePnPFunctor the_functor(num_constraints,obj_pts, normalisedProjPoints);
    Eigen::LevenbergMarquardt<SolvePnPFunctor,double> lm(the_functor);
    int ret = lm.minimize(x);
    //Check if minimization was properly done.
    if(ret == Eigen::LevenbergMarquardtSpace::ImproperInputParameters ||
               ret == Eigen::LevenbergMarquardtSpace::TooManyFunctionEvaluation)
    {
      std::cout<< "Levenburg Marquardt Minimization failed: "<<ret<<"\n";
      std::cout<< "FEvals:" <<lm.nfev<< " Iter:"<<lm.iter<<"\n";
    }
    else
    {
      std::cout<< "Levenburg Marquardt Minimization returned: "<<ret<<"\n";
      std::cout<< "FEvals:" <<lm.nfev<< " Iter:"<<lm.iter<<"\n";
      std::cout<< "Minimized X " <<x.transpose()<<"\n";

      Matrix3f rotation_mat;
      rotation_mat = Eigen::AngleAxisf(x(3), Vector3f::UnitZ())
      * Eigen::AngleAxisf(x(4), Vector3f::UnitY())
      * Eigen::AngleAxisf(x(5), Vector3f::UnitX());
      std::cout<<"Rotation Matrix" <<rotation_mat<<"\n";
    }

}
