/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


#include "ICPDepthMap.h"
#include "saiga/time/timer.h"

namespace Saiga {

using namespace Depthmap;

namespace ICP {


DepthMapExtended::DepthMapExtended(DepthMap depth, Intrinsics4 camera, SE3 pose)
    : depth(depth), points(depth.h,depth.w), normals(depth.h,depth.w), camera(camera), pose(pose)
{
    Saiga::Depthmap::toPointCloud(depth,points,camera);
    Saiga::Depthmap::normalMap(points,normals);
}

std::vector<Correspondence> projectiveCorrespondences(const DepthMapExtended& ref, const DepthMapExtended& src,
                                                      double distanceThres, double cosNormalThres
                                                      , int searchRadius, bool useInvDepthAsWeight, bool scaleDistanceThresByDepth)
{
    std::vector<Correspondence> result;
    result.reserve(ref.depth.h * ref.depth.w);


    auto T = ref.pose.inverse() * src.pose; // A <- B

    for(int i = 0; i < src.depth.h; ++i)
    {
        for(int j = 0; j < src.depth.w; ++j)
        {
            Vec3 p0 = src.points(i,j);
            Vec3 n0 = src.normals(i,j);

            Vec3 p = p0;
            Vec3 n = n0;

            if(!p.allFinite() || !n.allFinite())
                continue;

            // transform point and normal to reference frame
            p = T * p;
            n = T.so3() * n;

            // project point to reference to find correspondences
            Vec2 ip = ref.camera.project(p);

            // round to nearest integer
            ip = ip.array().round();

            int sx = ip(0);
            int sy = ip(1);




            double bestDist = std::numeric_limits<double>::infinity();
            Correspondence corr;

            // search in a small neighbourhood of the projection
            int S = searchRadius;
            for(int dy = -S; dy <= S; ++dy)
            {
                for(int dx = -S; dx <= S; ++dx)
                {
                    int x = sx + dx;
                    int y = sy + dy;

                    if(!ref.points.getConstImageView().inImage(y,x))
                        continue;

                    Vec3 p2 = ref.points(y,x);
                    Vec3 n2 = ref.normals(y,x);


                    if(!p2.allFinite() || !n2.allFinite())
                        continue;

                    auto distance = (p2-p).norm();

                    auto depth = p2(2);
                    auto invDepth = 1.0 / depth;

                    auto disTh = scaleDistanceThresByDepth ? distanceThres * depth : distanceThres;

                    if( distance < bestDist && distance < disTh && n.dot(n2) > cosNormalThres )
                    {
                        corr.refPoint = p2;
                        corr.refNormal = n2;
                        corr.srcPoint = p0;
                        corr.srcNormal = n0;
                        corr.weight = useInvDepthAsWeight ? invDepth * invDepth : 1;
                        bestDist = distance;
                    }
                }
            }

            if(std::isfinite(bestDist))
            {
                result.push_back(corr);
            }

        }
    }

    return result;
}

SE3 alignDepthMaps(DepthMap referenceDepthMap, DepthMap sourceDepthMap, SE3 refPose, SE3 srcPose, Intrinsics4 camera, int iterations)
{
    DepthMapExtended ref(referenceDepthMap,camera,refPose);
    DepthMapExtended src(sourceDepthMap,camera,srcPose);


    std::vector<Saiga::ICP::Correspondence> corrs;

    for(int k = 0; k < iterations; ++k)
    {
        corrs = Saiga::ICP::projectiveCorrespondences(ref,src);
        src.pose = Saiga::ICP::pointToPlane(corrs,ref.pose,src.pose);
    }
    return src.pose;
}


}
}