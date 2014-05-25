#ifndef RT_DLT_H
#define RT_DLT_H
#include <Eigen/Dense>

//A modified form of DLT that estimates the Rotation and Translation components of
//the projection matrix (the extrinsic part), when provided with 3D points and their
//normalized projections in camera coordinates

Eigen::Matrix<double,6,1> rt_dlt(const Eigen::MatrixX3f & obj_pts,const Eigen::MatrixX3f & proj_pts);

#endif // RT_DLT_H
