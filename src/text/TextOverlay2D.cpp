#include "saiga/text/TextOverlay2D.h"
#include "saiga/text/textShader.h"
#include "saiga/text/text.h"
#include "saiga/opengl/shader/shaderLoader.h"

#include <algorithm>

TextOverlay2D::TextOverlay2D(int width, int height):width(width),height(height){
    proj = glm::ortho(0.0f,(float)width,0.0f,(float)height,1.0f,-1.0f);

    loadShader();

}

void TextOverlay2D::render(){


    textShader->bind();
    textShader->uploadProj(proj);
    for(Text* &text : texts){
        if(text->visible)
            text->render(textShader);
    }
    textShader->unbind();
}

void TextOverlay2D::addText(Text* text){
    texts.push_back(text);
}

void TextOverlay2D::removeText(Text *text)
{
    texts.erase( std::remove(texts.begin(),texts.end(),text),texts.end());
}

void TextOverlay2D::loadShader()
{
    if(textShader!=nullptr)
        return;
//    textShader = ShaderLoader::instance()->load<TextShader>("deferred_text.glsl");
    textShader = ShaderLoader::instance()->load<TextShader>("sdf_text.glsl");

}


