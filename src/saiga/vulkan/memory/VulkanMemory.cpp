//
// Created by Peter Eichinger on 2018-11-30.
//

#include "VulkanMemory.h"

#include "saiga/imgui/imgui.h"
#include "saiga/util/tostring.h"

#include <memory>
void VulkanMemory::init(vk::PhysicalDevice _pDevice, vk::Device _device)
{
    m_pDevice = _pDevice;
    m_device  = _device;
    strategy  = FirstFitStrategy();
    chunkAllocator.init(_pDevice, _device);

    auto props = _pDevice.getMemoryProperties();
    memoryTypes.resize(props.memoryTypeCount);
    for (int i = 0; i < props.memoryTypeCount; ++i)
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

    getAllocator(vertIndexType.usageFlags, vertIndexType.memoryFlags);
    getAllocator(vertIndexHostType.usageFlags, vertIndexHostType.memoryFlags);

    auto stagingType = BufferType{vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eTransferDst,
                                  vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent};

    auto effectiveFlags = getEffectiveFlags(stagingType.memoryFlags);
    bufferAllocators.emplace(
        BufferType{stagingType.usageFlags, effectiveFlags},
        std::make_shared<SimpleMemoryAllocator>(m_device, m_pDevice, effectiveFlags, stagingType.usageFlags, true));
}

VulkanMemory::BufferIter VulkanMemory::createNewBufferAllocator(VulkanMemory::BufferMap& map,
                                                                const VulkanMemory::BufferDefaultMap& defaultSizes,
                                                                const BufferType& type)
{
    auto effectiveFlags = getEffectiveFlags(type.memoryFlags);

    auto effectiveType = BufferType{type.usageFlags, effectiveFlags};

    auto found = find_default_size<BufferDefaultMap, BufferType>(default_buffer_chunk_sizes, effectiveType);


    bool mapped = (effectiveType.memoryFlags & vk::MemoryPropertyFlagBits::eHostVisible) ==
                  vk::MemoryPropertyFlagBits::eHostVisible;
    auto emplaced = map.emplace(effectiveType, std::make_shared<BufferChunkAllocator>(
                                                   m_device, &chunkAllocator, effectiveType.memoryFlags,
                                                   effectiveType.usageFlags, strategy, found->second, mapped));
    SAIGA_ASSERT(emplaced.second, "Allocator was already present.");
    return emplaced.first;
}

VulkanMemory::ImageIter VulkanMemory::createNewImageAllocator(VulkanMemory::ImageMap& map,
                                                              const VulkanMemory::ImageDefaultMap& defaultSizes,
                                                              const ImageType& type)
{
    auto found = find_default_size<ImageDefaultMap, ImageType>(default_image_chunk_sizes, type);
    bool mapped =
        (type.memoryFlags & vk::MemoryPropertyFlagBits::eHostVisible) == vk::MemoryPropertyFlagBits::eHostVisible;

    auto emplaced = map.emplace(
        type, ImageChunkAllocator(m_device, &chunkAllocator, type.memoryFlags, strategy, found->second, mapped));
    SAIGA_ASSERT(emplaced.second, "Allocator was already present.");
    return emplaced.first;
}

BaseMemoryAllocator& VulkanMemory::getAllocator(const vk::BufferUsageFlags& usage, const vk::MemoryPropertyFlags& flags)
{
    auto foundAllocator = findAllocator<BufferMap, vk::BufferUsageFlags>(bufferAllocators, {usage, flags});
    if (foundAllocator == bufferAllocators.end())
    {
        foundAllocator = createNewBufferAllocator(bufferAllocators, default_buffer_chunk_sizes, {usage, flags});
    }
    return *(foundAllocator->second);
}

BaseMemoryAllocator& VulkanMemory::getImageAllocator(const vk::MemoryPropertyFlags& flags,
                                                     const vk::ImageUsageFlags& usage)
{
    auto foundAllocator = findAllocator<ImageMap, vk::ImageUsageFlags>(imageAllocators, {usage, flags});

    if (foundAllocator == imageAllocators.end())
    {
        foundAllocator = createNewImageAllocator(imageAllocators, default_image_chunk_sizes, {usage, flags});
    }

    return foundAllocator->second;
}

void VulkanMemory::destroy()
{
    chunkAllocator.destroy();

    for (auto& allocator : bufferAllocators)
    {
        allocator.second->destroy();
    }

    for (auto& allocator : imageAllocators)
    {
        allocator.second.destroy();
    }
}

void VulkanMemory::renderGUI()
{
    if (!ImGui::CollapsingHeader("Memory Stats"))
    {
        return;
    }
    static std::unordered_map<vk::MemoryPropertyFlags, MemoryStats> memoryTypeStats;


    for (auto& entry : memoryTypeStats)
    {
        entry.second = MemoryStats();
    }


    for (auto& allocator : bufferAllocators)
    {
        if (memoryTypeStats.find(allocator.first.memoryFlags) == memoryTypeStats.end())
        {
            memoryTypeStats[allocator.first.memoryFlags] = MemoryStats();
        }
        memoryTypeStats[allocator.first.memoryFlags] += allocator.second->collectMemoryStats();
    }
    for (auto& allocator : imageAllocators)
    {
        if (memoryTypeStats.find(allocator.first.memoryFlags) == memoryTypeStats.end())
        {
            memoryTypeStats[allocator.first.memoryFlags] = MemoryStats();
        }
        memoryTypeStats[allocator.first.memoryFlags] += allocator.second.collectMemoryStats();
    }

    static std::vector<ImGui::ColoredBar> bars;

    bars.resize(memoryTypeStats.size(),
                ImGui::ColoredBar({0, 16}, {{0.1f, 0.1f, 0.1f, 1.0f}, {0.4f, 0.4f, 0.4f, 1.0f}}, true));
    int index = 0;
    for (auto& memStat : memoryTypeStats)
    {
        ImGui::Text("%s", vk::to_string(memStat.first).c_str());
        // ImGui::SameLine(150.0f);
        auto& bar = bars[index];

        ImGui::Indent();
        bar.renderBackground();
        bar.renderArea(0.0f, static_cast<float>(memStat.second.used) / memStat.second.allocated,
                       ImGui::ColoredBar::BarColor{{0.0f, 0.2f, 0.2f, 1.0f}, {0.133f, 0.40f, 0.40f, 1.0f}});

        ImGui::Text("%s / %s (%s fragmented free)", sizeToString(memStat.second.used).c_str(),
                    sizeToString(memStat.second.allocated).c_str(), sizeToString(memStat.second.fragmented).c_str());

        ImGui::Unindent();
        index++;
    }

    ImGui::Spacing();


    if (!ImGui::CollapsingHeader("Detailed Memory Stats"))
    {
        return;
    }

    ImGui::Indent();

    for (auto& allocator : bufferAllocators)
    {
        allocator.second->showDetailStats();
    }
    for (auto& allocator : imageAllocators)
    {
        allocator.second.showDetailStats();
    }

    ImGui::Unindent();
}
