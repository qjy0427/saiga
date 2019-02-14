//
// Created by Peter Eichinger on 2018-11-30.
//

#include "VulkanMemory.h"

#include "saiga/core/imgui/imgui.h"
#include "saiga/core/util/tostring.h"
#include "saiga/vulkan/Base.h"

#include "ImageCopyComputeShader.h"

#include <memory>
namespace Saiga::Vulkan::Memory
{
void VulkanMemory::init(VulkanBase* base)
{
    m_pDevice      = base->physicalDevice;
    m_device       = base->device;
    m_queue        = base->transferQueue;
    strategy       = std::make_unique<FirstFitStrategy<BufferMemoryLocation>>();
    image_strategy = std::make_unique<FirstFitStrategy<ImageMemoryLocation>>();

    img_copy_shader = std::make_unique<ImageCopyComputeShader>();
    img_copy_shader->init(base);

    chunkCreator.init(m_pDevice, m_device);

    auto props = m_pDevice.getMemoryProperties();
    memoryTypes.resize(props.memoryTypeCount);
    for (auto i = 0U; i < props.memoryTypeCount; ++i)
    {
        memoryTypes[i] = props.memoryTypes[i];
    }

    auto vertIndexType = BufferType{vk::BufferUsageFlagBits::eVertexBuffer | vk::BufferUsageFlagBits::eIndexBuffer |
                                        vk::BufferUsageFlagBits::eTransferDst,
                                    vk::MemoryPropertyFlagBits::eDeviceLocal};
    auto vertIndexHostType =
        BufferType{vk::BufferUsageFlagBits::eVertexBuffer | vk::BufferUsageFlagBits::eIndexBuffer |
                       vk::BufferUsageFlagBits::eTransferDst,
                   vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent};

    get_allocator_exact(vertIndexType);
    get_allocator_exact(vertIndexHostType);

    auto stagingType = BufferType{vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eTransferDst,
                                  vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent};

    auto effectiveFlags   = getEffectiveFlags(stagingType.memoryFlags);
    auto effectiveStaging = BufferType{stagingType.usageFlags, effectiveFlags};
    get_allocator_exact(effectiveStaging);

    fallbackAllocator = std::make_unique<FallbackAllocator>(m_device, m_pDevice);
}

VulkanMemory::BufferIter VulkanMemory::createNewBufferAllocator(VulkanMemory::BufferMap& map,
                                                                const VulkanMemory::BufferDefaultMap& defaultSizes,
                                                                const BufferType& type)
{
    auto effectiveFlags = getEffectiveFlags(type.memoryFlags);



    auto effectiveType = BufferType{type.usageFlags, effectiveFlags};

    bool allow_defragger =
        (effectiveType.usageFlags & vk::BufferUsageFlagBits::eUniformBuffer) != vk::BufferUsageFlagBits::eUniformBuffer;

    if (allow_defragger)
    {
        effectiveType.usageFlags |= vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eTransferSrc;
    }

    auto found = find_default_size<BufferDefaultMap, BufferType>(default_buffer_chunk_sizes, effectiveType);


    auto chunk_alloc = std::make_unique<BufferChunkAllocator>(m_device, &chunkCreator, effectiveType, *strategy,
                                                              m_queue, found->second);

    std::unique_ptr<BufferDefragger> defragger;
    if (allow_defragger)
    {
        defragger = std::make_unique<BufferDefragger>(base, m_device, chunk_alloc.get());
    }
    auto new_alloc = map.emplace(effectiveType, BufferContainer{std::move(chunk_alloc), std::move(defragger)});
    SAIGA_ASSERT(new_alloc.second, "Allocator was already present.");


    return new_alloc.first;
}

VulkanMemory::ImageIter VulkanMemory::createNewImageAllocator(VulkanMemory::ImageMap& map,
                                                              const VulkanMemory::ImageDefaultMap& defaultSizes,
                                                              const ImageType& type)
{
    auto effectiveFlags = getEffectiveFlags(type.memoryFlags);

    auto effectiveType = ImageType{type.usageFlags, effectiveFlags};

    auto found = find_default_size<ImageDefaultMap, ImageType>(default_image_chunk_sizes, type);


    bool allow_defragger = (effectiveType.usageFlags & vk::ImageUsageFlagBits::eDepthStencilAttachment) !=
                           vk::ImageUsageFlagBits::eDepthStencilAttachment;

    if (allow_defragger)
    {
        effectiveType.usageFlags |= vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eTransferSrc;
    }

    std::unique_ptr<ImageDefragger> defragger;

    auto chunk_alloc = std::make_unique<ImageChunkAllocator>(m_device, &chunkCreator, effectiveType, *image_strategy,
                                                             m_queue, found->second);
    if (allow_defragger)
    {
        defragger = std::make_unique<ImageDefragger>(base, m_device, chunk_alloc.get(), img_copy_shader.get());
    }
    auto emplaced = map.emplace(effectiveType, ImageContainer{std::move(chunk_alloc), std::move(defragger)});
    SAIGA_ASSERT(emplaced.second, "Allocator was already present.");
    return emplaced.first;
}

VulkanMemory::BufferContainer& VulkanMemory::getAllocator(const BufferType& type)
{
    auto foundAllocator = findAllocator<BufferMap, vk::BufferUsageFlags>(bufferAllocators, type);
    if (foundAllocator == bufferAllocators.end())
    {
        foundAllocator = createNewBufferAllocator(bufferAllocators, default_buffer_chunk_sizes, type);
    }
    return foundAllocator->second;
}



VulkanMemory::ImageContainer& VulkanMemory::getImageAllocator(const ImageType& type)
{
    auto foundAllocator = findAllocator<ImageMap, vk::ImageUsageFlags>(imageAllocators, type);

    if (foundAllocator == imageAllocators.end())
    {
        foundAllocator = createNewImageAllocator(imageAllocators, default_image_chunk_sizes, type);
    }

    return foundAllocator->second;
}

VulkanMemory::BufferContainer& VulkanMemory::get_allocator_exact(const BufferType& type)
{
    auto foundAllocator = find_allocator_exact<BufferMap, vk::BufferUsageFlags>(bufferAllocators, type);
    if (foundAllocator == bufferAllocators.end())
    {
        foundAllocator = createNewBufferAllocator(bufferAllocators, default_buffer_chunk_sizes, type);
    }
    return foundAllocator->second;
}


VulkanMemory::ImageContainer& VulkanMemory::get_image_allocator_exact(const ImageType& type)
{
    auto foundAllocator = find_allocator_exact<ImageMap, vk::ImageUsageFlags>(imageAllocators, type);

    if (foundAllocator == imageAllocators.end())
    {
        foundAllocator = createNewImageAllocator(imageAllocators, default_image_chunk_sizes, type);
    }

    return foundAllocator->second;
}

void VulkanMemory::destroy()
{
    chunkCreator.destroy();

    for (auto& allocator : bufferAllocators)
    {
        allocator.second.allocator->destroy();
    }

    for (auto& allocator : imageAllocators)
    {
        allocator.second.allocator->destroy();
    }

    if (img_copy_shader)
    {
        img_copy_shader->destroy();
        img_copy_shader = nullptr;
    }
}


BufferMemoryLocation* VulkanMemory::allocate(const BufferType& type, vk::DeviceSize size)
{
    auto& allocator = getAllocator(type);

    if (size > allocator.allocator->m_chunkSize)
    {
        return fallbackAllocator->allocate(type, size);
    }

    if (allocator.defragger)
    {
        allocator.defragger->stop();
    }
    auto memoryLocation = allocator.allocator->allocate(size);

    if (allocator.defragger)
    {
        allocator.defragger->invalidate(memoryLocation->memory);
        allocator.defragger->start();
    }

    return memoryLocation;
}

ImageMemoryLocation* VulkanMemory::allocate(const ImageType& type, ImageData& image_data)
{
    auto& allocator = getImageAllocator(type);

    if (image_data.image_requirements.size > allocator.allocator->m_chunkSize)
    {
        return fallbackAllocator->allocate(type, image_data);
    }

    if (allocator.defragger)
    {
        allocator.defragger->stop();
    }
    auto location = allocator.allocator->allocate(image_data);

    if (allocator.defragger)
    {
        allocator.defragger->invalidate(location->memory);
        allocator.defragger->start();
    }
    return location;
}

void VulkanMemory::deallocateBuffer(const BufferType& type, BufferMemoryLocation* location)
{
    auto& allocator = getAllocator(type);
    if (location->size > allocator.allocator->m_chunkSize)
    {
        fallbackAllocator->deallocate(location);

        return;
    }

    if (allocator.defragger)
    {
        allocator.defragger->stop();
        allocator.defragger->invalidate(location->memory);
        allocator.defragger->invalidate(location);
    }

    allocator.allocator->deallocate(location);

    if (allocator.defragger)
    {
        allocator.defragger->start();
    }
}

void VulkanMemory::deallocateImage(const ImageType& type, ImageMemoryLocation* location)
{
    auto& allocator = getImageAllocator(type);
    if (location->size > allocator.allocator->m_chunkSize)
    {
        fallbackAllocator->deallocate(location);
        return;
    }
    if (allocator.defragger)
    {
        allocator.defragger->stop();
        allocator.defragger->invalidate(location->memory);
        allocator.defragger->invalidate(location);
    }

    allocator.allocator->deallocate(location);

    if (allocator.defragger)
    {
        allocator.defragger->start();
    }
}

void VulkanMemory::enable_defragmentation(const BufferType& type, bool enable)
{
    getAllocator(type).defragger->setEnabled(enable);
}

void VulkanMemory::start_defrag(const BufferType& type)
{
    getAllocator(type).defragger->start();
}

void VulkanMemory::stop_defrag(const BufferType& type)
{
    getAllocator(type).defragger->stop();
}

void VulkanMemory::enable_defragmentation(const ImageType& type, bool enable)
{
    getImageAllocator(type).defragger->setEnabled(enable);
}

void VulkanMemory::start_defrag(const ImageType& type)
{
    getImageAllocator(type).defragger->start();
}

void VulkanMemory::stop_defrag(const ImageType& type)
{
    getImageAllocator(type).defragger->stop();
}


}  // namespace Saiga::Vulkan::Memory