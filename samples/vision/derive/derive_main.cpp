﻿/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "saiga/core/Core.h"
#include "saiga/vision/VisionTypes.h"
#include "saiga/vision/ceres/CeresArap.h"


#define LSD_REL

namespace Saiga
{
using Delta = Vec6;
using JType = Eigen::Matrix<double, 6, 6>;

struct Node
{
    SE3 v;

    void plusDelta(const Delta& dV) { v = SE3::exp(dV) * v; }
};

struct Edge
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    SE3 meassurement;



    void setRel(Node& nA, Node& nB)
    {
        auto& A = nA.v;
        auto& B = nB.v;
#ifdef LSD_REL
        meassurement = A.inverse() * B;
#else
        meassurement = B * A.inverse();
#endif
    }

    Delta residual(Node& nA, Node& nB)
    {
        auto& A = nA.v;
        auto& B = nB.v;
#ifdef LSD_REL
        auto error_ = A.inverse() * B * meassurement.inverse();
#else
        auto error_  = meassurement * A * B.inverse();
#endif
        return error_.log();
    }
};

Node n1, n2;
Edge e;

void numericDeriv(JType& JA, JType& JB)
{
    auto delta  = 1e-9;
    auto scalar = 1 / (2 * delta);

    for (auto i : Range(0, 6))
    {
        // add small step along the unit vector in each dimension
        Vec6 unitDelta = Vec6::Zero();
        unitDelta[i]   = delta;

        Node n1cpy = n1;
        Node n2cpy = n2;

        n1cpy.plusDelta(unitDelta);
        auto errorPlus = e.residual(n1cpy, n2cpy);

        n1cpy        = n1;
        unitDelta[i] = -delta;
        n1cpy.plusDelta(unitDelta);
        auto errorMinus = e.residual(n1cpy, n2cpy);

        JA.col(i) = scalar * (errorPlus - errorMinus);
    }

    for (auto i : Range(0, 6))
    {
        // add small step along the unit vector in each dimension
        Vec6 unitDelta = Vec6::Zero();
        unitDelta[i]   = delta;

        Node n1cpy = n1;
        Node n2cpy = n2;

        n2cpy.plusDelta(unitDelta);
        auto errorPlus = e.residual(n1cpy, n2cpy);


        n2cpy        = n2;
        unitDelta[i] = -delta;
        n2cpy.plusDelta(unitDelta);
        auto errorMinus = e.residual(n1cpy, n2cpy);

        JB.col(i) = scalar * (errorPlus - errorMinus);
    }
}

void analyticDeriv(JType& JA, JType& JB)
{
    auto A    = n1.v;
    auto B    = n2.v;
    auto meas = e.meassurement;
#ifdef LSD_REL
    JB = A.inverse().Adj();
    JA = -JB;
#endif
}

void test()
{
    n1.v = SE3(Quat(0.998305, -0.00736573, -0.0537998, -0.0209538), Vec3(0.477353, -0.131007, 2.78683));
    n2.v = SE3(Quat(0.998204, -0.000715942, -0.0553556, -0.0229123), Vec3(0.738355, -1.0208, 2.02948));
    e.setRel(n1, n2);
    e.meassurement =
        SE3(Quat(0.999975, 0.00656501, -0.00140489, -0.00232645), Vec3(0.0394367, -0.0333284, -0.00711748));

    //    cout << n1.v << endl;

    JType JA, JB, JA2, JB2;
    //
    numericDeriv(JA, JB);
    analyticDeriv(JA2, JB2);

    cout << endl;
    cout << "Numeric: " << endl;
    cout << JA << endl << endl;
    cout << JB << endl << endl;

    cout << endl;
    cout << "Analytic: " << endl;
    cout << JA2 << endl << endl;
    cout << JB2 << endl << endl;

    cout << "Error 1: " << (JA - JA2).norm() << endl;
    cout << "Error 2: " << (JB - JB2).norm() << endl;
}



}  // namespace Saiga

using namespace Saiga;

int main(int, char**)
{
    Saiga::SaigaParameters saigaParameters;
    Saiga::initSample(saigaParameters);
    Saiga::initSaiga(saigaParameters);

    Saiga::Random::setSeed(93865023985);
    test();

    return 0;
}
