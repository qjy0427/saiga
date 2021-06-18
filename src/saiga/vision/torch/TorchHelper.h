/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/image/image.h"
#include "saiga/core/util/table.h"

#include "torch/torch.h"


namespace Saiga
{
template <typename T>
std::pair<std::vector<T>, std::vector<T>> SplitDataset(std::vector<T> data, float ratio)
{
    int total_n = data.size();
    int first_n = std::round(total_n * ratio);
    std::vector<T> a(data.begin(), data.begin() + first_n);
    std::vector<T> b(data.begin() + first_n, data.end());
    return {a, b};
}

/**
 * Writes some information of the given tensor to std::cout.
 */
inline void PrintTensorInfo(at::Tensor t)
{
    if (!t.defined())
    {
        std::cout << "[undefined tensor]" << std::endl;
        return;
    }
    auto mi = t.min().item().toFloat();
    auto ma = t.max().item().toFloat();

    float mean = 0;
    if (t.dtype() == at::kFloat)
    {
        mean = t.mean().item().toFloat();
    }
    std::cout << "Tensor " << t.sizes() << " " << t.dtype() << " " << t.device() << " Min/Max " << mi << " " << ma
              << " Mean " << mean << " req-grad " << t.requires_grad() << std::endl;
}


inline void PrintModelParams(torch::nn::Module module)
{
    Table tab({40, 25, 10, 15});
    size_t sum = 0;

    tab << "Name"
        << "Params"
        << "Params"
        << "Sum";
    for (auto& t : module.named_parameters())
    {
        size_t local_sum = 1;
        for (auto i : t.value().sizes())
        {
            local_sum *= i;
        }
        sum += local_sum;
        std::stringstream strm;
        strm << t.value().sizes();
        tab << t.key() << strm.str() << local_sum << sum;
    }
    std::cout << std::endl;
}

inline torch::nn::AnyModule NormFromString(const std::string& str, int channels)
{
    if (str == "bn")
    {
        return torch::nn::AnyModule(torch::nn::BatchNorm2d(torch::nn::BatchNorm2dOptions(channels).momentum(0.01)));
    }
    else if (str == "bn_no_stat")
    {
        return torch::nn::AnyModule(
            torch::nn::InstanceNorm2d(torch::nn::InstanceNorm2dOptions(channels).track_running_stats(false)));
    }
    else if (str == "in")
    {
        return torch::nn::AnyModule(torch::nn::InstanceNorm2d(torch::nn::InstanceNorm2dOptions(channels)));
    }
    else if (str == "id")
    {
        return torch::nn::AnyModule(torch::nn::Identity());
    }
    else
    {
        SAIGA_EXIT_ERROR("Unknown norm " + str);
        return {};
    }
}

inline torch::nn::AnyModule ActivationFromString(const std::string& str)
{
    if (str == "id" || str.empty())
    {
        return torch::nn::AnyModule(torch::nn::Identity());
    }
    else if (str == "sigmoid")
    {
        return torch::nn::AnyModule(torch::nn::Sigmoid());
    }
    else if (str == "tanh")
    {
        return torch::nn::AnyModule(torch::nn::Tanh());
    }
    else if (str == "elu")
    {
        return torch::nn::AnyModule(torch::nn::ELU());
    }
    else if (str == "relu")
    {
        return torch::nn::AnyModule(torch::nn::ReLU());
    }
    else
    {
        SAIGA_EXIT_ERROR("Unknown activation " + str);
        return {};
    }
}


inline torch::nn::AnyModule Pooling2DFromString(const std::string& str, torch::ExpandingArray<2> kernel_size)
{
    if (str == "average")
    {
        return torch::nn::AnyModule(torch::nn::AvgPool2d(torch::nn::AvgPool2dOptions(kernel_size)));
    }
    else if (str == "max")
    {
        return torch::nn::AnyModule(torch::nn::MaxPool2d(torch::nn::MaxPool2dOptions(kernel_size)));
    }
    else
    {
        SAIGA_EXIT_ERROR("Unknown activation " + str);
        return {};
    }
}


//  After each epoch we mulitply the learning rate by the decay
inline double ExponentialDecay(int epoch_id, int max_epochs, double decay)
{
    SAIGA_ASSERT(decay >= 0 && decay <= 1);
    return decay;
}

// After <step_size> steps the learning rate is multiplied by decay
inline double SteppedExponentialDecay(int epoch_id, int max_epochs, int step_size, double decay)
{
    SAIGA_ASSERT(epoch_id > 0);
    SAIGA_ASSERT(decay >= 0 && decay <= 1);

    if (epoch_id % step_size == 0)
    {
        return decay;
    }
    else
    {
        return 1;
    }
}


// Set the LR of all param groups in this optimizer
template <typename OptionsType>
void SetLR(torch::optim::Optimizer* optimizer, double lr)
{
    for (auto& pg : optimizer->param_groups())
    {
        auto opt = dynamic_cast<OptionsType*>(&pg.options());
        SAIGA_ASSERT(opt);
        opt->lr() = lr;
    }
}

// Multiplies the LR of all param groups by the given value
template <typename OptionsType>
void UpdateLR(torch::optim::Optimizer* optimizer, double factor)
{
    for (auto& pg : optimizer->param_groups())
    {
        auto opt = dynamic_cast<OptionsType*>(&pg.options());
        SAIGA_ASSERT(opt);
        opt->lr() = opt->lr() * factor;
    }
}

}  // namespace Saiga
