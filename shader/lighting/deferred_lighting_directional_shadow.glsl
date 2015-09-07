
##GL_VERTEX_SHADER

#version 400
layout(location=0) in vec3 in_position;
layout(location=1) in vec3 in_normal;
layout(location=2) in vec2 in_tex;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

uniform mat4 MV;
uniform mat4 MVP;




void main() {
    gl_Position = vec4(in_position.x,in_position.y,0,1);

}





##GL_FRAGMENT_SHADER
#version 400


#ifdef SHADOWS
uniform sampler2DShadow depthTex;
#endif

uniform sampler2D ssaoTex;

uniform vec3 direction;
uniform float ambientIntensity;

#include "deferred_lighting_fs.glsl"

layout(location=0) out vec4 out_color;

void main() {
    vec3 diffColor,vposition,normal,data;
    float depth;
    getGbufferData(diffColor,vposition,depth,normal,data);
    vec3 specColor = vec3(1);
    vec3 ambColor = diffColor;
    vec3 lightDir = direction;

    float ssao = texture(ssaoTex,CalcTexCoord()).r;

    float intensity = color.w;
    vec3 lightColor = color.rgb;

    float visibility = 1.0f;
#ifdef SHADOWS
//    visibility = calculateShadow(depthTex,vposition,1.0f);
    visibility = calculateShadow(depthTex,vposition);
#endif


    float localIntensity = intensity*visibility; //amount of light reaching the given point


    float Iamb = intensity*ambientIntensity*ssao;
    float Idiff = localIntensity * intensityDiffuse(normal,lightDir);
    float Ispec = localIntensity * intensitySpecular(vposition,normal,lightDir,40);

//    float Iemissive = data.y ;
//    vec3 Iemissive = diffColor*data.y ;

//    out_color = vec4(lightColor*( Idiff*diffColor + Ispec*specColor + Iamb*ambColor) + Iemissive,1);
    out_color = vec4(lightColor*(Idiff+Iamb) ,Ispec); //accumulation
}

