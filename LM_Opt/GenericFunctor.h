#ifndef GENERIFUNCTOR_H
#define GENERIFUNCTOR_H
#include<Eigen/Core>
#include<Eigen/Dense>

template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct GFunctor
{
typedef _Scalar Scalar;
enum {
   InputsAtCompileTime = NX,
   ValuesAtCompileTime = NY
};
typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

int m_inputs, m_values;

GFunctor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
GFunctor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

int inputs() const { return m_inputs; }
int values() const { return m_values; }

};


#endif // GENERIFUNCTOR_H
