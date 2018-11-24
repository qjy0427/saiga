//
// Created by Peter Eichinger on 08.10.18.
//

#include "ChunkType.h"

using Saiga::Vulkan::Memory::ChunkType;
using Saiga::Vulkan::Memory::Chunk;
std::shared_ptr<Chunk> ChunkType::allocate(vk::DeviceSize chunkSize) {
    vk::MemoryAllocateInfo info(chunkSize, m_memoryTypeIndex);
    auto chunk = std::make_shared<Chunk>(m_device.allocateMemory(info), chunkSize, propertyFlags);
    m_chunks.push_back(chunk);
    return chunk;
}

void ChunkType::deallocate(std::shared_ptr<Chunk> chunk) {
    m_device.free(chunk->memory);
    m_chunks.erase(std::remove(m_chunks.begin(), m_chunks.end(), chunk), m_chunks.end());
}