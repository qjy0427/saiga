/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


#include "saiga/sdl/sdl.h"
#include "saiga/util/crash.h"

#include "advancedWindow.h"


int main(int argc, char* args[])
{
    // Add a signal handler for SIGSEGV and print the stack trace when a SIGSEGV is caught
    catchSegFaults();

    WindowParameters windowParameters;
    initSample(windowParameters.saigaParameters);
    windowParameters.fromConfigFile("config.ini");

    windowParameters.height = 150;
    windowParameters.width  = 1000;

    OpenGLParameters openglParameters;
    openglParameters.fromConfigFile("config.ini");

    // 1. Create an SDL window.
    // This also creates the required OpenGL context.
    SDLWindow window(windowParameters, openglParameters);

    // 2. Create the OpenGL renderer
    Forward_Renderer renderer(window);

    // 3. Create an object of our class, which is both renderable and updateable
    Sample simpleWindow(window, renderer);

    // Everyhing is initilalized, we can run the main loop now!
    MainLoopParameters mainLoopParameters;
    mainLoopParameters.fromConfigFile("config.ini");
    mainLoopParameters.framesPerSecond = 0;
    window.startMainLoop(mainLoopParameters);
    return 0;
}