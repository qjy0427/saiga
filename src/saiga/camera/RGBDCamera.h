/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include <saiga/config.h>
#include <chrono>
#include "saiga/image/image.h"

namespace Saiga {



class SAIGA_GLOBAL RGBDCamera
{
public:

    struct FrameData
    {
          TemplatedImage<ucvec3> colorImg;
          TemplatedImage<unsigned short> depthImg;
          int frameId;
          std::chrono::steady_clock::time_point  captureTime;
    };

    int colorW, colorH;
    int depthW, depthH;



//    TemplatedImage<ucvec4> colorImg;
//    TemplatedImage<unsigned short> depthImg;

    virtual bool readFrame(FrameData& data) = 0;

    std::shared_ptr<FrameData> makeFrameData();

protected:
    int currentId = 0;

    void setNextFrame(FrameData& data);
};

}