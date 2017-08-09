/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "saiga/rendering/deferred_renderer.h"
#include "saiga/util/error.h"
#include "saiga/geometry/triangle_mesh_generator.h"
#include "saiga/opengl/shader/shaderLoader.h"

#include "saiga/camera/camera.h"
#include "saiga/rendering/renderer.h"
#include "saiga/imgui/imgui.h"

namespace Saiga {

Deferred_Renderer::Deferred_Renderer(int windowWidth, int windowHeight, RenderingParameters _params) :
    ddo(windowWidth,windowHeight),
    windowWidth(windowWidth), windowHeight(windowHeight),
    width(windowWidth*_params.renderScale), height(windowHeight*_params.renderScale),
    params(_params),lighting(gbuffer)
{
    cameraBuffer.createGLBuffer(nullptr,sizeof(CameraDataGLSL),GL_DYNAMIC_DRAW);

    //    setSize(windowWidth,windowHeight);

    if(params.useSMAA){
      smaa = std::make_shared<SMAA>(width, height);
      smaa->loadShader(params.smaaQuality);
    }

    {
        //create a 2x2 grayscale black dummy texture
        blackDummyTexture = std::make_shared<Texture>();
        std::vector<int> data(2*2,0);
        blackDummyTexture->createTexture(2,2,GL_RED,GL_R8,GL_UNSIGNED_BYTE, (GLubyte*)data.data());

    }
    if(params.useSSAO){
        ssao = std::make_shared<SSAO>(width, height);
    }
    lighting.ssaoTexture = ssao ? ssao->bluredTexture : blackDummyTexture;
//        ssao.init(windowWidth*params.renderScale, windowHeight*params.renderScale);
//    else
//        ssao.init(2,2);

    if(params.srgbWrites){

        //intel graphics drivers on windows do not define this extension but srgb still works..
        //SAIGA_ASSERT(hasExtension("GL_EXT_framebuffer_sRGB"));

        //Mesa drivers do not respect the spec when blitting with srgb framebuffers.
        //https://lists.freedesktop.org/archives/mesa-dev/2015-February/077681.html

        //TODO check for mesa
        //If this is true some recording softwares record the image too dark :(
        blitLastFramebuffer = false;
    }



    gbuffer.init(width, height, params.gbp);

    lighting.init(width, height, params.useGPUTimers);
    lighting.shadowSamples = params.shadowSamples;
    lighting.clearColor = params.lightingClearColor;
    lighting.loadShaders();



    postProcessor.init(width, height, &gbuffer, params.ppp, lighting.lightAccumulationTexture, params.useGPUTimers);


    auto qb = TriangleMeshGenerator::createFullScreenQuadMesh();
    qb->createBuffers(quadMesh);

    int numTimers = DeferredTimings::COUNT;
    if (!params.useGPUTimers)
        numTimers = 1; //still use one rendering timer :)
    timers.resize(numTimers);
    for (auto &t : timers) {
        t.create();
    }



    blitDepthShader = ShaderLoader::instance()->load<MVPTextureShader>("lighting/blitDepth.glsl");

    ddo.setDeferredFramebuffer(&gbuffer,ssao ? ssao->bluredTexture : blackDummyTexture);

    cout << "Deferred Renderer initialized. Render resolution: " << width << "x" << height << endl;

}

Deferred_Renderer::~Deferred_Renderer()
{

}



void Deferred_Renderer::resize(int windowWidth, int windowHeight)
{


    if (windowWidth <= 0 || windowHeight <= 0) {
        cout << "Warning: The window size must be greater than zero to be complete." << endl;
        windowWidth = glm::max(windowWidth, 1);
        windowHeight = glm::max(windowHeight, 1);
    }
    this->windowWidth = windowWidth;
    this->windowHeight = windowHeight;
    this->width = windowWidth * params.renderScale;
    this->height = windowHeight * params.renderScale;
    cout << "Resizing Window to : " << windowWidth << "," << windowHeight << endl;
    cout << "Framebuffer size: " << width << " " << height << endl;
    postProcessor.resize(width, height);
    gbuffer.resize(width, height);
    lighting.resize(width, height);

    if(ssao)
        ssao->resize(width, height);

    if(smaa){
        smaa->resize(width,height);
    }
}






void Deferred_Renderer::render_intern() {

    if (params.srgbWrites)
        glEnable(GL_FRAMEBUFFER_SRGB);

    startTimer(TOTAL);

    // When GL_FRAMEBUFFER_SRGB is disabled, the system assumes that the color written by the fragment shader
    // is in whatever colorspace the image it is being written to is. Therefore, no colorspace correction is performed.
    // If GL_FRAMEBUFFER_SRGB is enabled however, then if the destination image is in the sRGB colorspace
    // (as queried through glGetFramebufferAttachmentParameter(GL_FRAMEBUFFER_ATTACHMENT_COLOR_ENCODING)​),
    // then it will assume the shader's output is in the linear RGB colorspace.
    // It will therefore convert the output from linear RGB to sRGB.
    //    if (params.srgbWrites)
    //        glEnable(GL_FRAMEBUFFER_SRGB); //no reason to switch it off

    (*currentCamera)->recalculatePlanes();
    bindCamera(*currentCamera);
    renderGBuffer(*currentCamera);
    assert_no_glerror();


    renderSSAO(*currentCamera);
    //    return;

    lighting.initRender();
    lighting.cullLights(*currentCamera);
    renderDepthMaps();


    //    glDisable(GL_DEPTH_TEST);
    //    glViewport(0,0,width,height);


    //copy depth to lighting framebuffer. that is needed for stencil culling




    //    mix_framebuffer.bind();
    //    glClear( GL_COLOR_BUFFER_BIT );


    bindCamera(*currentCamera);
    renderLighting(*currentCamera);


    //    startTimer(LIGHTACCUMULATION);
    //    postProcessor.nextFrame();
    //    postProcessor.bindCurrentBuffer();

    //    lighting.renderLightAccumulation();
    //    stopTimer(LIGHTACCUMULATION);

    if (params.writeDepthToOverlayBuffer) {
        //        writeGbufferDepthToCurrentFramebuffer();
    }
    else {
        glClear(GL_DEPTH_BUFFER_BIT);
    }

    startTimer(OVERLAY);

    bindCamera(*currentCamera);
    renderer->renderOverlay(*currentCamera);
    stopTimer(OVERLAY);



    postProcessor.nextFrame();
    postProcessor.bindCurrentBuffer();
    //    postProcessor.switchBuffer();


    startTimer(POSTPROCESSING);
    //postprocessor's 'currentbuffer' will still be bound after render
    postProcessor.render();
    stopTimer(POSTPROCESSING);

    //    deferred_framebuffer.blitDepth(0);




    if(params.useSMAA){
        startTimer(SMAATIME);
        smaa->render(postProcessor.getCurrentTexture(),postProcessor.getTargetBuffer());
        postProcessor.switchBuffer();
        postProcessor.bindCurrentBuffer();
        stopTimer(SMAATIME);
    }

    //write depth to default framebuffer
    if (params.writeDepthToDefaultFramebuffer) {
        //        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        postProcessor.bindCurrentBuffer();
        writeGbufferDepthToCurrentFramebuffer();
    }


    //glBindFramebuffer(GL_FRAMEBUFFER, 0);
    //glClear(GL_COLOR_BUFFER_BIT);
    startTimer(FINAL);
    if(renderDDO){
        bindCamera(&ddo.layout.cam);
        ddo.render();
    }
    renderer->renderFinal(*currentCamera);
    stopTimer(FINAL);

    glDisable(GL_BLEND);

    if(blitLastFramebuffer)
        postProcessor.blitLast(windowWidth, windowHeight);
    else
        postProcessor.renderLast(windowWidth, windowHeight);

    //    if (params.srgbWrites)
    //        glDisable(GL_FRAMEBUFFER_SRGB);

    if (params.useGlFinish)
        glFinish();

    stopTimer(TOTAL);



    //    std::cout<<"Time spent on the GPU: "<< getTime(TOTAL) <<std::endl;

    //    printTimings();


    //    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);   // Make sure no FBO is set as the draw framebuffer
    //     glBindFramebuffer(GL_READ_FRAMEBUFFER, lighting.lightAccumulationBuffer.getId()); // Make sure your multisampled FBO is the read framebuffer
    //     glDrawBuffer(GL_BACK);                       // Set the back buffer as the draw buffer
    //     glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);

    assert_no_glerror();

}

void Deferred_Renderer::renderGBuffer(Camera *cam) {
    startTimer(GEOMETRYPASS);

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);


    gbuffer.bind();
    glViewport(0, 0, width, height);
    glClearColor(params.gbufferClearColor.x, params.gbufferClearColor.y, params.gbufferClearColor.z, params.gbufferClearColor.w);

    if (params.maskUsedPixels) {
        glClearStencil(0xFF); //sets stencil buffer to 255
        //mark all written pixels with 0 in the stencil buffer
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 0, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    }
    else {
        glClearStencil(0x00);
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);



    if (offsetGeometry) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(offsetFactor, offsetUnits);
    }

    if (wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(wireframeLineSize);
    }
    renderer->render(cam);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


    if (offsetGeometry) {
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    glDisable(GL_STENCIL_TEST);

    gbuffer.unbind();


    stopTimer(GEOMETRYPASS);

    assert_no_glerror();

}

void Deferred_Renderer::renderDepthMaps() {

    startTimer(DEPTHMAPS);


    lighting.renderDepthMaps(renderer);


    stopTimer(DEPTHMAPS);

    assert_no_glerror();

}

void Deferred_Renderer::renderLighting(Camera *cam) {
    startTimer(LIGHTING);

    lighting.render(cam);

    stopTimer(LIGHTING);

    assert_no_glerror();
}

void Deferred_Renderer::renderSSAO(Camera *cam)
{

    startTimer(SSAOT);

    if(params.useSSAO)
        ssao->render(cam, &gbuffer);


    stopTimer(SSAOT);

    assert_no_glerror();

}

void Deferred_Renderer::writeGbufferDepthToCurrentFramebuffer()
{
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_ALWAYS);
    blitDepthShader->bind();
    blitDepthShader->uploadTexture(gbuffer.getTextureDepth());
    quadMesh.bindAndDraw();
    blitDepthShader->unbind();
    glDepthFunc(GL_LESS);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    assert_no_glerror();
}


void Deferred_Renderer::bindCamera(Camera *cam)
{
    CameraDataGLSL cd(cam);
    cameraBuffer.updateBuffer(&cd,sizeof(CameraDataGLSL),0);
    cameraBuffer.bind(CAMERA_DATA_BINDING_POINT);
}

void Deferred_Renderer::printTimings()
{
    cout << "====================================" << endl;
    cout << "Geometry pass: " << getTime(GEOMETRYPASS) << "ms" << endl;
    cout << "SSAO: " << getTime(SSAOT) << "ms" << endl;
    cout << "Depthmaps: " << getTime(DEPTHMAPS) << "ms" << endl;
    cout << "Lighting: " << getTime(LIGHTING) << "ms" << endl;
    lighting.printTimings();
    //    cout<<"Light accumulation: "<<getTime(LIGHTACCUMULATION)<<"ms"<<endl;
    cout << "Overlay pass: " << getTime(OVERLAY) << "ms" << endl;
    cout << "Postprocessing: " << getTime(POSTPROCESSING) << "ms" << endl;
    postProcessor.printTimings();
    cout << "SMAA: " << getTime(SMAATIME) << "ms" << endl;
    cout << "Final pass: " << getTime(FINAL) << "ms" << endl;
    float total = getTime(TOTAL);
    cout << "Total: " << total << "ms (" << 1000 / total << " fps)" << endl;
    cout << "====================================" << endl;

}


void Deferred_Renderer::renderImGui()
{
    ImGui::SetNextWindowPos(ImVec2(400, 20), ImGuiSetCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400,600), ImGuiSetCond_FirstUseEver);
    ImGui::Begin("Deferred Renderer");

    ImGui::Checkbox("renderDDO", &renderDDO);
    ImGui::Checkbox("wireframe", &wireframe);
    ImGui::Checkbox("offsetGeometry", &offsetGeometry);

    ImGui::Text("Render Time");
    ImGui::Text("%fms - Geometry pass",getTime(GEOMETRYPASS));
    ImGui::Text("%fms - SSAO",getTime(SSAOT));
    ImGui::Text("%fms - Depthmaps",getTime(DEPTHMAPS));
    ImGui::Text("%fms - Lighting",getTime(LIGHTING));
    ImGui::Text("%fms - Overlay pass",getTime(OVERLAY));
    ImGui::Text("%fms - Postprocessing",getTime(POSTPROCESSING));
    ImGui::Text("%fms - SMAA",getTime(SMAATIME));
    ImGui::Text("%fms - Final pass",getTime(FINAL));
    ImGui::Text("%fms - Total",getTime(TOTAL));

    ImGui::Separator();

   if(ImGui::Checkbox("SMAA",&params.useSMAA)){
       if(params.useSMAA){
           smaa = std::make_shared<SMAA>(width, height);
           smaa->loadShader(params.smaaQuality);
       }else{
           smaa.reset();
       }
   }
   if(smaa){
        smaa->renderImGui();
    }


   if(ImGui::Checkbox("SSAO",&params.useSSAO)){
       if(params.useSSAO){
           ssao = std::make_shared<SSAO>(width, height);
       }else{
           ssao.reset();
       }
       lighting.ssaoTexture = ssao ? ssao->bluredTexture : blackDummyTexture;
       ddo.setDeferredFramebuffer(&gbuffer,ssao ? ssao->bluredTexture : blackDummyTexture);
   }
   if(ssao){
        ssao->renderImGui();
    }

    ImGui::End();
}

}
