﻿/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "CeresBA.h"

#include "saiga/core/time/timer.h"

#include "Eigen/Sparse"
#include "Eigen/SparseCholesky"
#include "ceres/ceres.h"
#include "ceres/problem.h"
#include "ceres/rotation.h"
#include "ceres/solver.h"

#include "CeresKernel_BA_Intr4.h"
#include "local_parameterization_se3.h"

namespace Saiga
{
OptimizationResults CeresBA::solve()
{
    Scene& scene = *_scene;

    SAIGA_OPTIONAL_BLOCK_TIMER(optimizationOptions.debugOutput);

    ceres::Problem::Options problemOptions;
    problemOptions.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problemOptions.cost_function_ownership          = ceres::DO_NOT_TAKE_OWNERSHIP;
    problemOptions.loss_function_ownership          = ceres::DO_NOT_TAKE_OWNERSHIP;

    ceres::Problem problem(problemOptions);



    //    Sophus::test::LocalParameterizationSE3* camera_parameterization = new Sophus::test::LocalParameterizationSE3;
    Sophus::test::LocalParameterizationSE32 camera_parameterization;
    for (size_t i = 0; i < scene.extrinsics.size(); ++i)
    {
        problem.AddParameterBlock(scene.extrinsics[i].se3.data(), 7, &camera_parameterization);
    }

    ceres::HuberLoss lossFunctionMono(baOptions.huberMono);
    ceres::HuberLoss lossFunctionStereo(baOptions.huberStereo);



    int monoCount   = 0;
    int stereoCount = 0;
    for (auto& img : scene.images)
    {
        for (auto& ip : img.stereoPoints)
        {
            if (!ip) continue;
            if (ip.depth > 0)
            {
                stereoCount++;
            }
            else
            {
                monoCount++;
            }
        }
    }

    auto ordering = std::make_shared<ceres::ParameterBlockOrdering>();


    std::vector<std::unique_ptr<CostBAStereoAnalytic>> monoCostFunctions;
    std::vector<std::unique_ptr<CostBAStereoAnalytic>> stereoCostFunctions;

    monoCostFunctions.reserve(monoCount);
    stereoCostFunctions.reserve(stereoCount);


    for (auto& img : scene.images)
    {
        auto& extr   = scene.extrinsics[img.extr].se3;
        auto& camera = scene.intrinsics[img.intr];

        for (auto& ip : img.stereoPoints)
        {
            if (!ip) continue;
            auto& wp = scene.worldPoints[ip.wp].p;
            double w = ip.weight * scene.scale();


            if (ip.depth > 0)
            {
#if 0
                auto stereoPoint   = ip.point(0) - scene.bf / ip.depth;
                auto cost_function = CostBAStereo<>::create(camera, ip.point, stereoPoint, scene.bf, Vec2(w, w));
                ceres::LossFunction* lossFunction =
                    options.huberStereo > 0 ? new ceres::HuberLoss(options.huberStereo) : nullptr;
                problem.AddResidualBlock(cost_function, lossFunction, extr.data(), wp.data());
#endif
            }
            else
            {
#if 0
                auto cost_function = CostBAMono::create(camera, ip.point, w);
                ceres::LossFunction* lossFunction =
                    options.huberMono > 0 ? new ceres::HuberLoss(options.huberMono) : nullptr;
                problem.AddResidualBlock(cost_function, lossFunction, extr.data(), wp.data());
#else
                CostBAStereoAnalytic* cost = new CostBAStereoAnalytic(camera, ip.point, w);
                monoCostFunctions.emplace_back(cost);

                problem.AddResidualBlock(cost, baOptions.huberMono > 0 ? &lossFunctionMono : nullptr, extr.data(),
                                         wp.data());

                // With this ordering the schur complement is computed in the correct order
                ordering->AddElementToGroup(wp.data(), 0);
                ordering->AddElementToGroup(extr.data(), 1);
#endif
            }
        }
    }


    //    double costInit = 0;
    //    ceres::Problem::EvaluateOptions defaultEvalOptions;
    //    problem.Evaluate(defaultEvalOptions, &costInit, nullptr, nullptr, nullptr);

    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_progress_to_stdout = optimizationOptions.debugOutput;
    //    ceres_options.minimizer_progress_to_stdout = true;
    ceres_options.max_num_iterations           = optimizationOptions.maxIterations;
    ceres_options.max_linear_solver_iterations = optimizationOptions.maxIterativeIterations;
    ceres_options.min_linear_solver_iterations = optimizationOptions.maxIterativeIterations;
    ceres_options.min_relative_decrease        = 1e-50;
    ceres_options.function_tolerance           = 1e-50;

    ceres_options.initial_trust_region_radius = 1.0 / optimizationOptions.initialLambda;

    ceres_options.linear_solver_ordering = ordering;


    switch (optimizationOptions.solverType)
    {
        case OptimizationOptions::SolverType::Direct:
            ceres_options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
            break;
        case OptimizationOptions::SolverType::Iterative:
            ceres_options.linear_solver_type = ceres::LinearSolverType::ITERATIVE_SCHUR;
            break;
    }

    ceres::Solver::Summary summaryTest;

#if 0

    ceres::Problem::EvaluateOptions defaultEvalOptions;
    ceres::CRSMatrix matrix;
    double costFinal = 0;
    problem.Evaluate(defaultEvalOptions, &costFinal, nullptr, nullptr, &matrix);

    cout << "num residuals: " << problem.NumResiduals() << endl;

    cout << matrix.num_rows << "x" << matrix.num_cols << endl;

    {
        Eigen::SparseMatrix<double, Eigen::RowMajor> ematrix(matrix.num_rows, matrix.num_cols);
        ematrix.resizeNonZeros(matrix.values.size());

        for (int i = 0; i < matrix.num_rows + 1; ++i)
        {
            ematrix.outerIndexPtr()[i] = matrix.rows[i];
        }
        for (int i = 0; i < matrix.values.size(); ++i)
        {
            ematrix.valuePtr()[i]      = matrix.values[i];
            ematrix.innerIndexPtr()[i] = matrix.cols[i];
        }
        cout << ematrix.toDense() << endl;
    }
#endif

    //    exit(0);


    {
        SAIGA_OPTIONAL_BLOCK_TIMER(optimizationOptions.debugOutput);
        ceres::Solve(ceres_options, &problem, &summaryTest);
    }

    //    cout << "linear solver time " << summaryTest.linear_solver_time_in_seconds << "s." << endl;
    //    cout << summaryTest.FullReport() << endl;

    OptimizationResults result;
    result.name               = name;
    result.cost_initial       = summaryTest.initial_cost * 2.0;
    result.cost_final         = summaryTest.final_cost * 2.0;
    result.linear_solver_time = summaryTest.linear_solver_time_in_seconds * 1000;
    result.total_time         = summaryTest.total_time_in_seconds * 1000;
    return result;

    //    std::cout << "optimizePoints residual: " << costInit << " -> " << costFinal << endl;
}

}  // namespace Saiga
