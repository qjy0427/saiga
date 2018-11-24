//
// Created by Peter Eichinger on 15.10.18.
//

#pragma once

#include <saiga/util/assert.h>
#include "MemoryLocation.h"
namespace Saiga{
namespace Vulkan{
namespace Memory{

struct SAIGA_LOCAL BaseMemoryAllocator {

    explicit BaseMemoryAllocator(bool _mapped) : mapped(_mapped) {}

    virtual MemoryLocation allocate(vk::DeviceSize size) = 0;
    virtual void deallocate(MemoryLocation& location) = 0;
    bool mapped = false;
};

}
}
}