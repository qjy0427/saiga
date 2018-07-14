﻿/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


#pragma once

#include "Buffer.h"
#include "saiga/vulkan/Base.h"

namespace Saiga {
namespace Vulkan {


class SAIGA_GLOBAL UniformBuffer : public Buffer
{
public:

    void init(VulkanBase& base, const void* data, size_t size)
    {
        createBuffer(base,size,vk::BufferUsageFlagBits::eUniformBuffer | vk::BufferUsageFlagBits::eTransferDst);
        allocateMemoryBuffer(base,vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
        if(data)
            DeviceMemory::mappedUpload(0,size,data);
    }

    vk::DescriptorBufferInfo getDescriptorInfo()
    {
        vk::DescriptorBufferInfo descriptorInfo;
        descriptorInfo.buffer = buffer;
        descriptorInfo.offset = 0;
        descriptorInfo.range = size;
        return descriptorInfo;

    }
};

}
}