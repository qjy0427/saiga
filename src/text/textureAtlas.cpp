#include "saiga/text/textureAtlas.h"
#include "saiga/opengl/texture/texture.h"
#include "saiga/opengl/texture/textureLoader.h"
#include "saiga/geometry/triangle_mesh.h"
#include "saiga/text/fontLoader.h"
#include <algorithm>
#include <FreeImagePlus.h>
#include <ft2build.h>
#include <ftstroke.h>
#include "saiga/util/assert.h"

#include FT_FREETYPE_H

#define NOMINMAX
#undef max
#undef min

FT_Library TextureAtlas::ft = nullptr;

TextureAtlas::TextureAtlas(){
    if(ft==nullptr){
        if(FT_Init_FreeType(&ft)) {
            std::cerr<< "Could not init freetype library"<<std::endl;
            assert(0);
        }
    }
}

TextureAtlas::~TextureAtlas()
{
    FT_Done_Face(face);
    delete textureAtlas;
}


void TextureAtlas::loadFont(const std::string &font, int font_size, int stroke_size){

    FontLoader fl(font);
    fl.loadMonochromatic(30);
    fl.writeGlyphsToFiles("debug/fonts/");
    this->font = font;
//    font_size*=6;
    this->font_size = font_size;
    this->stroke_size = stroke_size;

    //    face = std::make_shared<FT_Face>();
    if(FT_New_Face(ft, font.c_str(), 0, &face)) {
        std::cerr<<"Could not open font "<<font<<std::endl;
        assert(0);
        return;
    }
    FT_Set_Pixel_Sizes(face, 0, font_size);

    Image img, monoImg, sdf;
    charPaddingX = 20;
    charPaddingY = 20;
    createTextureAtlasMono(img);
    createTextureAtlasSDF(img,sdf);

//    charPaddingX = 5;
//    charPaddingY = 5;
//    createTextureAtlas(img);


    std::string str = "debug/ta_normal_"+std::to_string(atlasWidth)+"x"+std::to_string(atlasHeight)+".png";
    if(!TextureLoader::instance()->saveImage(str,img)){
        cout<<"could not save "<<str<<endl;
    }

    str = "debug/ta_sdf_"+std::to_string(atlasWidth)+"x"+std::to_string(atlasHeight)+".png";
    if(!TextureLoader::instance()->saveImage(str,sdf)){
        cout<<"could not save "<<str<<endl;
    }

    textureAtlas = new Texture();

    textureAtlas->fromImage(sdf);

    textureAtlas->bind();
    // The allowable values are 1 (byte-alignment), 2 (rows aligned to even-numbered bytes), Default: 4 (word-alignment), and 8 (rows start on double-word boundaries)
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    textureAtlas->unbind();

    //create mipmaps to reduce aliasing effects
    textureAtlas->generateMipmaps();
}

void TextureAtlas::createTextureAtlas(Image &outImg){
    FT_Render_Mode renderMode = FT_RENDER_MODE_NORMAL; //it corresponds to 8-bit anti-aliased bitmaps.
    //    FT_Render_Mode renderMode = FT_RENDER_MODE_MONO; //This mode corresponds to 1-bit bitmaps (with 2 levels of opacity).

    FT_Error error;
    FT_Stroker stroker = 0;

    if(stroke_size>0){
        // Set up a stroker.
        FT_Stroker_New(ft, &stroker);
        FT_Stroker_Set(stroker,
                       stroke_size,
                       FT_STROKER_LINECAP_ROUND,
                       FT_STROKER_LINEJOIN_ROUND,
                       0);
    }


    charNum = 0;

    const int count = 128-32;
    FT_Glyph glyphs[count], glyphs_bitmaps[count];
    FT_Glyph glyph_strokes[count], glyph_strokes_bitmaps[count];

    FT_GlyphSlot slot = (face)->glyph;
    for(int i = 32; i < 128; i++) {
        int id = i-32;
        FT_UInt  glyph_index;

        /* retrieve glyph index from character code */
        glyph_index = FT_Get_Char_Index( face, i );

        /* load glyph image into the slot (erase previous one) */
        //        error = FT_Load_Glyph( face, glyph_index, FT_LOAD_MONOCHROME );
        error = FT_Load_Glyph( face, glyph_index, FT_LOAD_DEFAULT );
        //        error = FT_Load_Glyph( face, glyph_index, FT_LOAD_TARGET_MONO );
        if ( error )
            continue;  /* ignore errors */

        error = FT_Get_Glyph( slot, &glyphs[id]);
        /* render the glyph to a bitmap, don't destroy original */
        glyphs_bitmaps[id] = glyphs[id];
        error = FT_Glyph_To_Bitmap( &glyphs_bitmaps[id], renderMode, NULL, 0 );

        FT_Glyph glyph = glyphs_bitmaps[id];

        if(stroke_size>0){
            error = FT_Get_Glyph( slot, &glyph_strokes[id] );
            error = FT_Glyph_Stroke( &glyph_strokes[id], stroker, 1 );

            glyph_strokes_bitmaps[id] = glyph_strokes[id];
            error = FT_Glyph_To_Bitmap( &glyph_strokes_bitmaps[id], renderMode, NULL, 0 );
            //stroke bitmap should be bigger than normal bitmap
            glyph = glyph_strokes_bitmaps[id];
        }

        if ( glyph->format != FT_GLYPH_FORMAT_BITMAP )
            cout<< "invalid glyph format returned!" <<endl;

        FT_BitmapGlyph bitmap = (FT_BitmapGlyph)glyph;
        FT_Bitmap* source = &bitmap->bitmap;

        character_info &info = characters[i];

        info.ax = ( glyph->advance.x + 0x8000 ) >> 16;
        info.ay = ( glyph->advance.y + 0x8000 ) >> 16;
        info.ax += stroke_size/64; //????

        info.bw = source->width;
        info.bh = source->rows;

        info.bl = bitmap->left;
        info.bt = bitmap->top;


        maxCharacter.min = glm::min(maxCharacter.min,vec3(info.bl,info.bt-info.bh,0));
        maxCharacter.max = glm::max(maxCharacter.max,vec3(info.bl+info.bw,info.bt-info.bh+info.bh,0));

        charNum++;


    }

    calculateTextureAtlasPositions();
    cout<<"AtlasWidth "<<atlasWidth<<" AtlasHeight "<<atlasHeight<<endl;

    outImg.bitDepth = 8;
    outImg.channels = 2;
    outImg.width = atlasWidth;
    outImg.height = atlasHeight;
    outImg.create();
    outImg.makeZero();

    for(int i = 32; i < 128; i++) {

        FT_BitmapGlyph bitmap = (FT_BitmapGlyph)glyphs_bitmaps[i-32];
        FT_Bitmap* source = &bitmap->bitmap;
//        cout<<"FT_Bitmap "<<source->width<<","<<source->rows<<endl;

        character_info &info = characters[i];

        //offset from normal glyph relative to stroke glyph
        int offsetX=0,offsetY=0;
        if(stroke_size>0){
            FT_BitmapGlyph bitmapstroke = (FT_BitmapGlyph)glyph_strokes_bitmaps[i-32];
            FT_Bitmap* sourceStroke = &bitmapstroke->bitmap;
            offsetX = bitmap->left - bitmapstroke->left;
            offsetY = -bitmap->top + bitmapstroke->top;

            //        cout<<offsetX<<" "<<offsetY<<endl;

            for(int y = 0 ; y < info.bh  ; ++y){
                for(int x = 0 ; x < info.bw ; ++x){
                    unsigned char c = sourceStroke->buffer[y*(info.bw) + x];
                    uint16_t s = c;
                    outImg.setPixel(info.atlasX+x ,info.atlasY+y,s);
                }
            }
        }

        for(int y = 0 ; y < source->rows  ; ++y){
            for(int x = 0 ; x < source->width ; ++x){
                unsigned char c;
                if(renderMode==FT_RENDER_MODE_MONO){
                    int byteIndex = y*source->pitch + x/8;
                    int bitIndex = 7 - (x % 8);
                    unsigned char c = source->buffer[byteIndex];
                    c = (c>>bitIndex) & 0x1;
                    if(c)
                        c = 255;
                }else{
                    c = source->buffer[y*(source->width) + x];
                }

                uint16_t s = c;

                int ox = x + offsetX;
                int oy = y + offsetY;

                uint16_t old = outImg.getPixel<uint16_t>(info.atlasX+ox ,info.atlasY+oy);
                old = old + (s<<8);
                outImg.setPixel(info.atlasX+ox ,info.atlasY+oy,old);
            }
        }

    }


    //cleanup freetype stuff
    if(stroke_size>0){
        FT_Stroker_Done(stroker);
        for(int i = 32; i < 128; i++) {
            FT_Done_Glyph(glyph_strokes[i-32]);
            FT_Done_Glyph(glyph_strokes_bitmaps[i-32]);
        }
    }
    for(int i = 32; i < 128; i++) {
        FT_Done_Glyph(glyphs[i-32]);
        FT_Done_Glyph(glyphs_bitmaps[i-32]);
    }
    outImg.addChannel();
}


void TextureAtlas::createTextureAtlasMono(Image &outImg){
    //    FT_Render_Mode renderMode = FT_RENDER_MODE_NORMAL; //it corresponds to 8-bit anti-aliased bitmaps.
    FT_Render_Mode renderMode = FT_RENDER_MODE_MONO; //This mode corresponds to 1-bit bitmaps (with 2 levels of opacity).

    FT_Error error;


    charNum = 0;

    const int count = 128-32;
    FT_Glyph glyphs[count], glyphs_bitmaps[count];

    FT_GlyphSlot slot = (face)->glyph;
    for(int i = 32; i < 128; i++) {
        int id = i-32;
        FT_UInt  glyph_index;

        /* retrieve glyph index from character code */
        glyph_index = FT_Get_Char_Index( face, i );

        /* load glyph image into the slot (erase previous one) */
        //        error = FT_Load_Glyph( face, glyph_index, FT_LOAD_MONOCHROME );
        //        error = FT_Load_Glyph( face, glyph_index, FT_LOAD_DEFAULT );
        error = FT_Load_Glyph( face, glyph_index, FT_LOAD_TARGET_MONO );
        if ( error )
            continue;  /* ignore errors */

        error = FT_Get_Glyph( slot, &glyphs[id]);
        /* render the glyph to a bitmap, don't destroy original */
        glyphs_bitmaps[id] = glyphs[id];
        error = FT_Glyph_To_Bitmap( &glyphs_bitmaps[id], renderMode, NULL, 0 );

        FT_Glyph glyph = glyphs_bitmaps[id];

        if ( glyph->format != FT_GLYPH_FORMAT_BITMAP )
            cout<< "invalid glyph format returned!" <<endl;

        FT_BitmapGlyph bitmap = (FT_BitmapGlyph)glyph;
        FT_Bitmap* source = &bitmap->bitmap;

        character_info &info = characters[i];

        info.ax = ( glyph->advance.x + 0x8000 ) >> 16;
        info.ay = ( glyph->advance.y + 0x8000 ) >> 16;

        info.bw = source->width;
        info.bh = source->rows;

        info.bl = bitmap->left;
        info.bt = bitmap->top;


        maxCharacter.min = glm::min(maxCharacter.min,vec3(info.bl,info.bt-info.bh,0));
        maxCharacter.max = glm::max(maxCharacter.max,vec3(info.bl+info.bw,info.bt-info.bh+info.bh,0));

        charNum++;


    }

    calculateTextureAtlasPositions();
    cout<<"AtlasWidth "<<atlasWidth<<" AtlasHeight "<<atlasHeight<<endl;

    outImg.bitDepth = 8;
    outImg.channels = 1;
    outImg.width = atlasWidth;
    outImg.height = atlasHeight;
    outImg.create();
    outImg.makeZero();

    for(int i = 32; i < 128; i++) {

        FT_BitmapGlyph bitmap = (FT_BitmapGlyph)glyphs_bitmaps[i-32];
        FT_Bitmap* source = &bitmap->bitmap;
//        cout<<"FT_Bitmap "<<source->width<<","<<source->rows<<endl;

        character_info &info = characters[i];

        for(int y = 0 ; y < source->rows  ; ++y){
            for(int x = 0 ; x < source->width ; ++x){

                int byteIndex = y*source->pitch + x/8;
                int bitIndex = 7 - (x % 8);
                unsigned char c = source->buffer[byteIndex];
                c = (c>>bitIndex) & 0x1;
                if(c)
                    c = 255;
                int ox = x;
                int oy = y;
                outImg.setPixel(info.atlasX+ox ,info.atlasY+oy,c);
            }
        }

    }


    for(int i = 32; i < 128; i++) {
        FT_Done_Glyph(glyphs[i-32]);
        FT_Done_Glyph(glyphs_bitmaps[i-32]);
    }


}

void TextureAtlas::createTextureAtlasSDF(Image &moneImage, Image &outImg)
{
    int w = moneImage.width;
    int h = moneImage.height;

    outImg.bitDepth = 8;
    outImg.channels = 1;
    outImg.width = w;
    outImg.height = h;
    outImg.create();
    outImg.makeZero();

    int searchRadius = 7;

    std::vector<glm::ivec2> samplePositions;
    for(int x = -searchRadius ; x <= searchRadius ; ++x){
        for(int y = -searchRadius ; y <= searchRadius ; ++y){
            if(x!=0 || y!=0)
            samplePositions.emplace_back(x,y);
        }
    }
    std::sort(samplePositions.begin(),samplePositions.end(),[](const glm::ivec2 &a,const glm::ivec2 &b)->bool
    {
        return (a.x*a.x+a.y*a.y)<(b.x*b.x+b.y*b.y);
    });

    //remove samples further away then searchRadius
    int samples = 0;
    for(;samples<samplePositions.size();samples++){
        glm::ivec2 a = samplePositions[samples];
        if(glm::sqrt((float)(a.x*a.x+a.y*a.y))>searchRadius){
            break;
        }
    }
    samplePositions.resize(samples);

    for(glm::ivec2 s : samplePositions){
//        cout<<"sample "<<s.x<<","<<s.y<<endl;
    }

    for(int y = 0 ; y < h ; ++y){
        for(int x = 0 ; x < w ; ++x){
            float d = 12345;
            unsigned char current = moneImage.getPixel<unsigned char>(x,y);
            for(glm::ivec2 s : samplePositions){
                glm::ivec2 ps = glm::ivec2(x,y) + s;
                ps = glm::clamp(ps,glm::ivec2(0),glm::ivec2(w-1,h-1));
                unsigned char other = moneImage.getPixel<unsigned char>(ps.x,ps.y);
                if(current!=other){
                    d = glm::sqrt((float)(s.x*s.x+s.y*s.y));
                    break;
                }
            }

            //map to 0-1
            d = d / (float)searchRadius;
            d = glm::clamp(d,0.0f,1.0f);
            d = d * 0.5f;

            //set 0.5 to border and >0.5 to inside and <0.5 to outside
            if(current){
                d = d + 0.5f;
            }else{
                d = 0.5f - d;
            }

            unsigned char out = d * 255.0f;
            outImg.setPixel(x,y,out);
        }
    }

    for(int i = 32; i < 128; i++) {
        character_info &info = characters[i];
//        info.bl -= charPaddingX;
//        info.bt -= charPaddingY;
//        cout<<"char "<<(char)i<<" "<<info.bw<<","<<info.bh<<endl;
                info.atlasX -= charPaddingX/2;
                info.atlasY -= charPaddingY/2;
        info.bw += charPaddingX;
        info.bh += charPaddingY;

    }

    //calculate the texture coordinates
    for(int i = 32; i < 128; i++) {
        character_info &info = characters[i];
        float tx = (float)info.atlasX / (float)atlasWidth;
        float ty = (float)info.atlasY / (float)atlasHeight;

        info.tcMin = vec2(tx,ty);
        info.tcMax = vec2(tx+(float)info.bw/(float)atlasWidth,ty+(float)info.bh/(float)atlasHeight);
    }
}
void TextureAtlas::calculateTextureAtlasPositions()
{
    int charsPerRow = glm::ceil(glm::sqrt((float)charNum));

    atlasHeight = charPaddingY;
    atlasWidth = 0;

    for(int cy = 0 ; cy < charsPerRow ; ++cy){
        int currentW = charPaddingX;
        int currentH = charPaddingY;

        for(int cx = 0 ; cx < charsPerRow ; ++cx){
            int i = cy * charsPerRow + cx;
            if(i>=charNum)
                break;
            character_info &info = characters[i+32];

            info.atlasX = currentW;
            info.atlasY = atlasHeight;

            currentW += info.bw+charPaddingX;
            currentH = std::max(currentH, info.bh);
        }
        atlasWidth = std::max(currentW, atlasWidth);
        atlasHeight += currentH+charPaddingY;
    }

    //increase width to a number dividable by 8 to fix possible alignment issues
    while(atlasWidth%8!=0)
        atlasWidth++;

    //calculate the texture coordinates
    for(int i = 32; i < 128; i++) {
        character_info &info = characters[i];
        float tx = (float)info.atlasX / (float)atlasWidth;
        float ty = (float)info.atlasY / (float)atlasHeight;

        info.tcMin = vec2(tx,ty);
        info.tcMax = vec2(tx+(float)info.bw/(float)atlasWidth,ty+(float)info.bh/(float)atlasHeight);
    }
}

