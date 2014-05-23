#ifndef SOLVEPNPFUNCTOR_H
#define SOLVEPNPFUNCTOR_H
#include "GenericFunctor.h"
struct SolvePnPFunctor: GFunctor<double>
{
//Matrix3f camera_matrix;
Eigen::MatrixX3f object_points;
Eigen::MatrixX3f projection_points;
SolvePnPFunctor(int n_fvec, Eigen::MatrixX3f ob_pts, Eigen::MatrixX3f proj_pts)
: GFunctor<double>(6, n_fvec), /*camera_matrix(cam_mat),*/
object_points(ob_pts), projection_points(proj_pts)
{}

int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;
int df(const Eigen::VectorXd &x, JacobianType &fjac) const;
};

#endif
