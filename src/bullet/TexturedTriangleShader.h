#ifndef Magnum_Examples_TexturedTriangle_TexturedTriangleShader_h
#define Magnum_Examples_TexturedTriangle_TexturedTriangleShader_h
/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020 —
            Vladimír Vondruš <mosra@centrum.cz>

    This is free and unencumbered software released into the public domain.

    Anyone is free to copy, modify, publish, use, compile, sell, or distribute
    this software, either in source code form or as a compiled binary, for any
    purpose, commercial or non-commercial, and by any means.

    In jurisdictions that recognize copyright laws, the author or authors of
    this software dedicate any and all copyright interest in the software to
    the public domain. We make this dedication for the benefit of the public
    at large and to the detriment of our heirs and successors. We intend this
    dedication to be an overt act of relinquishment in perpetuity of all
    present and future rights to this software under copyright law.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/Math/Color.h>

using namespace Magnum::Math::Literals;

namespace Magnum { namespace LabFinal {

class TexturedTriangleShader: public GL::AbstractShaderProgram {
    public:
        typedef GL::Attribute<0, Vector2> Position;
        typedef GL::Attribute<1, Vector2> TextureCoordinates;

        explicit TexturedTriangleShader();
        explicit TexturedTriangleShader(NoCreateT);

        Int _transformationMatrixUniform{1},
            _projectionMatrixUniform{2},
            _ambientColorUniform{3},
            _normalMatrixUniform{4},
            TextureUnit;

        TexturedTriangleShader& setColor(const Color3& color) {
            setUniform(_colorUniform, color);
            return *this;
        }

        TexturedTriangleShader& bindTexture(GL::CubeMapTexture& texture) {
            texture.bind(TextureUnit);
            return *this;
        }
        TexturedTriangleShader& setProjectionMatrix(const Matrix4& matrix) {
          setUniform((int) _projectionMatrixUniform, (Math::RectangularMatrix<4,4,Float> &) matrix);
          return *this;
        }
        TexturedTriangleShader& setTransformationMatrix(const Matrix4& matrix) {
          setUniform((int) _transformationMatrixUniform, (Math::RectangularMatrix<4,4,Float> &) matrix);
          return *this;
        }
        TexturedTriangleShader& setNormalMatrix(const Matrix3x3& matrix) {
          setUniform((int) _normalMatrixUniform, matrix);
          return *this;
        }
    private:
        Int _colorUniform;
};

}}

#endif
