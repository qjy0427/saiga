/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */



#include "saiga/core/geometry/triangle_mesh_generator.h"
#include "saiga/opengl/rendering/forwardRendering/forward_renderer.h"
#include "saiga/opengl/shader/shaderLoader.h"
#include "saiga/opengl/window/SampleWindowDeferred.h"

using namespace Saiga;

class Sample : public StandaloneWindow<WindowManagement::EGL, Forward_Renderer>
{
   public:
    PerspectiveCamera camera;

    SimpleAssetObject cube1, cube2;
    SimpleAssetObject groundPlane;
    SimpleAssetObject sphere;

    ProceduralSkybox skybox;


    Sample() : StandaloneWindow("config.ini")
    {
        // create a perspective camera
        float aspect = window->getAspectRatio();
        camera.setProj(60.0f, aspect, 0.1f, 50.0f);
        camera.setView(vec3(0, 5, 10), vec3(0, 0, 0), vec3(0, 1, 0));

        // Set the camera from which view the scene is rendered
        window->setCamera(&camera);



        // This simple AssetLoader can create assets from meshes and generate some generic debug assets
        AssetLoader assetLoader;

        // First create the triangle mesh of a cube
        auto cubeMesh = TriangleMeshGenerator::createMesh(AABB(make_vec3(-1), make_vec3(1)));

        // To render a triangle mesh we need to wrap it into an asset. This creates the required OpenGL buffers and
        // provides render functions.
        auto cubeAsset = assetLoader.assetFromMesh(*cubeMesh, Colors::blue);

        // Rendering an asset at a user defined location is done most efficiently with a 4x4 transformation matrix,
        // that is passed to the shader as a uniform. The SimpleAssetObject does exactly this. It contains a
        // transformation matrix and simple transformation methods for example 'translate' 'rotate'. The 'render'
        // methods of a SimpleAssetObject will bind the correct shaders, upload the matrix to the correct uniform and
        // call the raw 'render' of the referenced asset.
        cube1.asset = cubeAsset;

        // An asset can be referenced by multiple SimpleAssetObject, because each SimpleAssetObject has its own
        // transformation matrix and therefore they all can be drawn at different locations.
        cube2.asset = cubeAsset;

        // Translate the first cube
        cube1.translateGlobal(vec3(3, 1, 0));
        // Compute the 4x4 transformation matrix. This has to be done before rendering when a 'transform method' was
        // called.
        cube1.calculateModel();

        cube2.translateGlobal(vec3(3, 1, 5));
        cube2.calculateModel();


        auto sphereMesh  = TriangleMeshGenerator::createMesh(Sphere(make_vec3(0), 1), 2);
        auto sphereAsset = assetLoader.assetFromMesh(*sphereMesh, Colors::green);
        sphere.asset     = sphereAsset;
        sphere.translateGlobal(vec3(-2, 1, 0));
        sphere.calculateModel();

        groundPlane.asset = assetLoader.loadDebugPlaneAsset(vec2(20, 20), 1.0f, Colors::lightgray, Colors::gray);



        std::cout << "Program Initialized!" << std::endl;
    }
    ~Sample()
    {
        // We don't need to delete anything here, because objects obtained from saiga are wrapped in smart pointers.
    }

    void update(float dt) override {}
    void interpolate(float dt, float interpolation) override {}

    void render(Camera* cam, RenderPass render_pass) override
    {
        skybox.render(cam);
        groundPlane.renderForward(cam);
        cube1.renderForward(cam);
        cube2.renderForward(cam);
        sphere.renderForward(cam);


        window->ScreenshotDefaultFramebuffer().save("render.png");
        window->close();
    }
};



int main(int argc, char* args[])
{
    // This should be only called if this is a sample located in saiga/samples
    initSaigaSample();

    Sample window;
    window.run();

    return 0;
}