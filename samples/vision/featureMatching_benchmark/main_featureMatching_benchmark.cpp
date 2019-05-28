﻿/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "saiga/core/framework/framework.h"
#include "saiga/core/image/image.h"
#include "saiga/core/time/all.h"
#include "saiga/core/util/fileChecker.h"
#include "saiga/core/util/random.h"
#include "saiga/vision/Eigen_Compile_Checker.h"
#include "saiga/vision/Features.h"
#include "saiga/vision/VisionIncludes.h"


using namespace Saiga;


std::vector<DescriptorORB> randomDescriptors(int n)
{
    std::vector<DescriptorORB> des(n);
    for (auto& d : des)
    {
        for (auto& dd : d) dd = Random::urand64();
    }
    return des;
}

int main(int, char**)
{
    Saiga::SaigaParameters saigaParameters;
    Saiga::initSample(saigaParameters);
    Saiga::initSaiga(saigaParameters);

    Saiga::EigenHelper::checkEigenCompabitilty<15357>();
    Saiga::Random::setSeed(45786045);


    int N = 1000;

    auto des1 = randomDescriptors(N);
    auto des2 = randomDescriptors(N);



    std::vector<int> d_matchMatrix(N * N);
    ImageView<int> matchMatrix(N, N, d_matchMatrix.data());

    cout << "Bruteforce matching " << N << "x" << N << " ORB Descriptors..." << endl;
    {
        SAIGA_BLOCK_TIMER("Matching");
        for (auto i : Range(0, N))
        {
            for (auto j : Range(0, N))
            {
                matchMatrix(i, j) = distance(des1[i], des2[j]);
            }
        }
    }


    auto sum = std::accumulate(d_matchMatrix.begin(), d_matchMatrix.end(), 0);
    cout << "distance sum: " << sum << " avg: " << double(sum) / (N * N) << endl;

    return 0;
}
