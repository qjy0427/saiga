#pragma once

#include "libhello/util/glm.h"
#include "libhello/opengl/vertexBuffer.h"


class Particle
{
public:
    enum Orientation{
        BILLBOARD = 0,
        VELOCITY
    };

    vec3 position = vec3(0);
    vec4 velocity = vec4(0); //normalized velocity x,y,z in worldspace. w is the scale factor
    vec3 force = vec3(0); //force on the particle. for example gravity
    float radius=1;
    float lifetime = 0; //lifetime in seconds
    float scale = 0; //upscaling over time. increases the radius by 'scale' per second

    int start = 0; //start tick
    int image = 0; //texture from texture array
    int orientation = BILLBOARD;
    Particle();
}/*__attribute__((packed))*/;



template<>
void VertexBuffer<Particle>::setVertexAttributes();
