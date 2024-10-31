//
// Created by jingye on 24-10-31.
//

#ifndef SHAREDMEMORY_H
#define SHAREDMEMORY_H

#include "saiga/vision/camera/EuRoCDataset.h"

// The meta data is given in .yaml files so we need that dependency.
#ifdef SAIGA_USE_YAML_CPP

namespace Saiga
{

class SAIGA_VISION_API SharedMemory : public EuRoCDataset {
    std::vector<Imu::Data> last_imu_data;
    double last_time = 0;
    double last_imu_time = 0;

public:
    explicit SharedMemory(const DatasetParameters& params, Sequence sequence = UNKNOWN);

    void Load();

    void LoadImageData(FrameData& data) override;

    int LoadMetaData() override;

    bool getImageSync(FrameData& data) override;

    std::vector<Imu::Data> GetImuSample(double curr_time);
};

}  // namespace Saiga

#endif


#endif //SHAREDMEMORY_H
