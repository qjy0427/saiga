#pragma once
#include "saiga/core/math/math.h"

#include "torch/torch.h"

// This is a helper function so that we can use
// tensor.data_ptr<vec3>() on tensors with the shape
//   [x,x,...,x,3]
template <>
inline Saiga::vec3* at::Tensor::data_ptr<Saiga::vec3>() const
{
    CHECK_EQ(size(dim() - 1), 3);
    CHECK_EQ(stride(dim() - 1), 1);
    return (Saiga::vec3*)data_ptr<float>();
}
template <>
inline Saiga::vec2* at::Tensor::data_ptr<Saiga::vec2>() const
{
    CHECK_EQ(size(dim() - 1), 2);
    CHECK_EQ(stride(dim() - 1), 1);
    return (Saiga::vec2*)data_ptr<float>();
}