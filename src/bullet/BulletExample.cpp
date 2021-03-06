/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020 —
            Vladimír Vondruš <mosra@centrum.cz>
        2013 — Jan Dupal <dupal.j@gmail.com>
        2019 — Max Schwarz <max.schwarz@online.de>

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

#include <btBulletDynamicsCommon.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Pointer.h>
#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/DebugStl.h>
#include <Magnum/Timeline.h>
#include <Magnum/ImageView.h>
#include <Magnum/Mesh.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <Magnum/BulletIntegration/DebugDraw.h>
#include <Magnum/DebugTools/FrameProfiler.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Math/Constants.h>
#include <Magnum/Math/Color.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/UVSphere.h>
#include <Magnum/Primitives/Icosphere.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Trade/Trade.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>
#include <Magnum/ImGuiIntegration/Context.hpp>

// Smooth Arcball Camera
#include "../arcball/ArcBall.h"
#include "../arcball/ArcBallCamera.h"

#include "TexturedTriangleShader.h"

// Learn opengl stuff
#include "learnopengl/filesystem.h"
#include "learnopengl/shader_m.h"
#include "stb_image.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using std::cout;
using std::endl;

#ifdef CORRADE_TARGET_ANDROID
#include <Magnum/Platform/AndroidApplication.h>
#elif defined(CORRADE_TARGET_EMSCRIPTEN)
#include <Magnum/Platform/EmscriptenApplication.h>
#else
#include <Magnum/Platform/Sdl2Application.h>
#endif

namespace Magnum {
  namespace LabFinal {

    using namespace Math::Literals;
    using namespace Magnum::Examples;
    using Trade::PhongMaterialData;

    typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
    typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D>  Scene3D;

    struct InstanceData {
      Matrix4   transformationMatrix;
      Matrix3x3 normalMatrix;
      Color3    color;
    };

    /* Basic Camera & Shaders */
    Vector3                  _lightRealPosition;

    class RigidBody : public Object3D {
    public:
      RigidBody(Object3D *parent, Float mass, btCollisionShape *bShape,
                btDynamicsWorld &bWorld) : Object3D{parent}, _bWorld(bWorld) {
        /* Calculate inertia so the object reacts as it should with
           rotation and everything */
        btVector3 bInertia(0.0f, 0.0f, 0.0f);
        if (!Math::TypeTraits<Float>::equals(mass, 0.0f)) {
          bShape->calculateLocalInertia(mass, bInertia);
        }

        /* Bullet rigid body setup */
        auto *motionState = new BulletIntegration::MotionState{*this};
        _bRigidBody.emplace(btRigidBody::btRigidBodyConstructionInfo{
          mass, &motionState->btMotionState(), bShape, bInertia});
        _bRigidBody->forceActivationState(DISABLE_DEACTIVATION);
        bWorld.addRigidBody(_bRigidBody.get());
      }

      ~RigidBody() {
        _bWorld.removeRigidBody(_bRigidBody.get());
      }

      btRigidBody &rigidBody() { return *_bRigidBody; }

      bool isAvailable() {
        return _bRigidBody != NULL;
      };

      /* needed after changing the pose from Magnum side */
      void syncPose() {
        _bRigidBody->setWorldTransform(btTransform(transformationMatrix()));
      }

    private:
      btDynamicsWorld                  &_bWorld;
      Containers::Pointer<btRigidBody> _bRigidBody;
    };

    class BulletExample : public Platform::Application {
    public:
      explicit BulletExample(const Arguments &arguments);
      ~BulletExample() {
        exit(0);
      }

    private:
      GL::Mesh                        _box{NoCreate}, _sphere{NoCreate};
      GL::Buffer                      _boxInstanceBuffer{
        NoCreate},                    _sphereInstanceBuffer{NoCreate};
      Shaders::Phong                  _shader{NoCreate};
      TexturedTriangleShader          _customShader{NoCreate};
      BulletIntegration::DebugDraw    _debugDraw{NoCreate};
      Containers::Array<InstanceData> _boxInstanceData, _sphereInstanceData;

      btDbvtBroadphase                    _bBroadphase;
      btDefaultCollisionConfiguration     _bCollisionConfig;
      btCollisionDispatcher               _bDispatcher{&_bCollisionConfig};
      btSequentialImpulseConstraintSolver _bSolver;

      /* boolean-valued options */
      bool _collisionDetectionByOctree = true;
      bool _isPausing                  = false;
      bool _drawBoundingBoxes          = false;
      bool _animation                  = false;
      bool _directionsPressed[5]       = {false};
      bool _enableAutoShoot            = false;
      const bool ENABLE_BOX_CENTRE     = false;

      /* Viewer related */
      Shaders::Phong    _coloredShader{NoCreate},
                        _texturedShader{NoCreate};
      Shaders::Flat2D   _2dShader{NoCreate};
      Containers::Array<Containers::Optional<GL::Mesh>>      _meshes;
      std::vector<Containers::Array<Containers::Optional<GL::Texture2D>> *> _allTextures;
      Object3D _manipulator;
      Containers::Pointer<Trade::AbstractImporter> _importer = nullptr;
      Containers::Pointer<PluginManager::Manager<Trade::AbstractImporter>> _manager = nullptr;

      /* SKybox */
      Shader *_skyboxShader;
      unsigned int _cubeMapTexture, _skyboxVao;

      /* Shadow Map */
      unsigned int _planeVAO;
      unsigned int _woodTexture, _depthMapFBO;
      unsigned int _cubeVAO, _cubeVBO;
      unsigned int _depthMap;

      Shader *_mappingShader, *_simpleDepthShaderPtr, *_debugDepthQuadPtr;

      /* Imgui */
      ImGuiIntegration::Context _imgui{NoCreate};
      bool _showDemoWindow = true;
      bool _showAnotherWindow = false;
      Color4 _clearColor = 0x72909aff_rgbaf;
      Float _floatValue = 0.0f;

      /* Robo tank related */
      Vector3                  _basePosition;
      std::vector<Matrix4 *>   _stackedRoboPos;
      std::vector<RigidBody *> _stackedRoboTank;
      int                      _currentTankLevel;
      float                    _currentScale;
      Float _basePositionX = 5.0f;
      Float _basePositionY = 0.0f;
      Float _basePositionZ = 5.0f;
      Float _basePositionVerticalSpeed = 0.0f;
      Float _hueVal = 45.0f;

      /* Custom shader object */
      GL::Mesh      _cubeMesh{NoCreate};
      GL::Texture2D _stoneTexture{NoCreate};
      GL::CubeMapTexture _skyboxTexture{NoCreate};

      Long _lastChecked = 0;

      /* Octree and boundary boxes */
      Containers::Pointer<int> _octree;

      /* Profiling */
      DebugTools::GLFrameProfiler _profiler{
        DebugTools::GLFrameProfiler::Value::FrameTime |
        DebugTools::GLFrameProfiler::Value::CpuDuration, 180};

      /* The world has to live longer than the scene because RigidBody
         instances have to remove themselves from it on destruction */
      btDiscreteDynamicsWorld _bWorld{&_bDispatcher, &_bBroadphase, &_bSolver,
                                      &_bCollisionConfig};

      Scene3D                             _scene;
      Containers::Optional<ArcBallCamera> _arcballCamera;
      Matrix4                             _projectionMatrix;
      SceneGraph::DrawableGroup3D         _drawables;
      SceneGraph::DrawableGroup3D         _skyboxGroup;
      Timeline                            _timeline;

      btBoxShape    _bBoxShape{{0.5f, 0.5f, 0.5f}};
      btSphereShape _bSphereShape{0.25f};
      btBoxShape    _bGroundShape{{20.0f, 0.5f, 20.0f}};
      btBoxShape    _roboTankShape{{1.0f, 0.5f, 0.5f}};
      btBoxShape    _xwallShape{{5.0f, 5.0f, 0.5f}};
      btBoxShape    _zwallShape{{0.5f, 5.0f, 5.0f}};


      bool _drawCubes{true}, _drawDebug{true}, _shootBox{true};

      // Display Parts & Event Handling
      void drawEvent() override;
      void keyPressEvent(KeyEvent &event) override;
      void keyReleaseEvent(KeyEvent &event) override;
      void mousePressEvent(MouseEvent &event) override;
      void drawTreeNodeBoundingBoxes();
      void mouseScrollEvent(MouseScrollEvent& event) override;
      void mouseMoveEvent(MouseMoveEvent& event) override;
      void mouseReleaseEvent(MouseEvent&) override;
      void viewportEvent(ViewportEvent& event) override;

      /* IMGUI related*/
      void drawImgui();
      void initializeGui();

      // Robo tank related.
      void calculateStackDemoPositions();
      void initializeRoboTank();
      void playerControlStart(KeyEvent &event);
      void playerControlEnd(KeyEvent &event);
      void playerMovement(Float duration);
      RigidBody *getRootRigidBody();

      /* Bullet related */
      void shootItem(Vector2i startLocation, bool isBox);
      void shootOnClick(MouseEvent &event);

      /* Viewer related */
      void addObject(Trade::AbstractImporter &importer,
                     Containers::Array<Containers::Optional<GL::Texture2D>> &textures,
                     Containers::ArrayView
                       <const Containers::Optional<PhongMaterialData>> materials,
                     Object3D &parent, UnsignedInt i);
      Vector3 positionOnSphere(const Vector2i& position) const;
      void prepareShaders();
      void setupCamera();
      void importFile(const Arguments &arguments);

      void configureWindow();
      void initializeCustomModel();
      void autoShoot();
      void createWall(const Vector3 &_xshape, const Vector3 &pos);
      void importRealFile(const std::string filename, Object3D &parent);

      void initializeSkybox();

      void loadTextureFromImage(const std::string &filename);

      void drawSkybox();

      unsigned int loadCubemap(std::vector<std::string> vector);
      void initializeShadowMapping();
      void drawShadowMapping();
      void renderScene(const Shader &shader);
      void renderCube();
    };

    class ColoredDrawable : public SceneGraph::Drawable3D {
    public:
      explicit ColoredDrawable(Object3D &object,
                               Containers::Array<InstanceData> &instanceData,
                               const Color3 &color,
                               const Matrix4 &primitiveTransformation,
                               SceneGraph::DrawableGroup3D &drawables)
        : SceneGraph::Drawable3D{object, &drawables},
          _instanceData(instanceData), _color{color},
          _primitiveTransformation{primitiveTransformation} {}

    private:
      void
      draw(const Matrix4 &transformation, SceneGraph::Camera3D &) override {
        const Matrix4 t = transformation * _primitiveTransformation;
        arrayAppend(_instanceData, Containers::InPlaceInit,
                    t, t.normalMatrix(), _color);
      }
      Containers::Array<InstanceData> &_instanceData;
      Color3                          _color;
      Matrix4                         _primitiveTransformation;
    };

    class NewColoredDrawable: public SceneGraph::Drawable3D {
    public:
      explicit NewColoredDrawable(Object3D &object, Shaders::Phong &shader,
                                  GL::Mesh &mesh, const Color4 &color,
                                  SceneGraph::DrawableGroup3D &group)
        : SceneGraph::Drawable3D{object, &group}, _localShader(shader), _mesh(mesh),
          _color{color} {}

    private:
      void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override;

      Shaders::Phong& _localShader;
      GL::Mesh& _mesh;
      Color4 _color;
    };

    class TexturedDrawable: public SceneGraph::Drawable3D {
    public:
      explicit TexturedDrawable(Object3D &object, Shaders::Phong &shader,
                                GL::Mesh &mesh, GL::Texture2D &texture,
                                SceneGraph::DrawableGroup3D &group)
        : SceneGraph::Drawable3D{object, &group}, _localShader(shader),
          _mesh(mesh), _texture(texture) {}
    private:
      void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override;
      Shaders::Phong& _localShader;
      GL::Mesh& _mesh;
      GL::Texture2D& _texture;
    };

    class TexturedWithPrimitiveDrawable: public SceneGraph::Drawable3D {
    public:
      explicit TexturedWithPrimitiveDrawable(Object3D &object, TexturedTriangleShader &shader,
                                GL::Mesh &mesh, GL::CubeMapTexture &texture, const Matrix4 &primitive,
                                SceneGraph::DrawableGroup3D &group)
        : SceneGraph::Drawable3D{object, &group}, _localShader(shader),
          _mesh(mesh), _texture(texture),_primitiveTransform(primitive) {}
    private:
      void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override;
      TexturedTriangleShader& _localShader;
      GL::Mesh& _mesh;
      GL::CubeMapTexture& _texture;
      Matrix4 _primitiveTransform;
    };

    void NewColoredDrawable::draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) {
      _localShader
        .setTransformationMatrix(transformationMatrix)
        .setNormalMatrix(transformationMatrix.normalMatrix())
        .setProjectionMatrix(camera.projectionMatrix())
        .setDiffuseColor(_color)
        .draw(_mesh);
    }

    void TexturedDrawable::draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) {
      _localShader
        .setTransformationMatrix(transformationMatrix)
        .setNormalMatrix(transformationMatrix.normalMatrix())
        .setProjectionMatrix(camera.projectionMatrix())
        .bindDiffuseTexture(_texture)
        .draw(_mesh);
    }

    void TexturedWithPrimitiveDrawable::draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) {
      _localShader
        .setTransformationMatrix(transformationMatrix * _primitiveTransform)
        .setNormalMatrix(transformationMatrix.normalMatrix())
        .setProjectionMatrix(camera.projectionMatrix())
        .bindTexture(_texture)
        .draw(_mesh);
    }

    BulletExample::BulletExample(const Arguments &arguments)
      : Platform::Application(arguments, NoCreate) {
      /* Try 8x MSAA, fall back to zero samples if not possible. Enable only 2x
         MSAA if we have enough DPI. */
      configureWindow();
      setupCamera();
      prepareShaders();
      importFile(arguments);

      /* Box and sphere mesh, with an (initially empty) instance buffer */
      _box                  = MeshTools::compile(Primitives::cubeSolid());
      _sphere               = MeshTools::compile(Primitives::uvSphereSolid(16, 32));
      _boxInstanceBuffer    = GL::Buffer{};
      _sphereInstanceBuffer = GL::Buffer{};
      _box.addVertexBufferInstanced(_boxInstanceBuffer, 1, 0,
                                    Shaders::Phong::TransformationMatrix{},
                                    Shaders::Phong::NormalMatrix{},
                                    Shaders::Phong::Color3{});
      _sphere.addVertexBufferInstanced(_sphereInstanceBuffer, 1, 0,
                                       Shaders::Phong::TransformationMatrix{},
                                       Shaders::Phong::NormalMatrix{},
                                       Shaders::Phong::Color3{});

      /* Setup the renderer so we can draw the debug lines on top */
      GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
      GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
      GL::Renderer::enable(GL::Renderer::Feature::PolygonOffsetFill);
      GL::Renderer::setPolygonOffset(2.0f, 0.5f);

      /* Bullet setup */
      _debugDraw = BulletIntegration::DebugDraw{};
      _debugDraw.setMode(BulletIntegration::DebugDraw::Mode::DrawWireframe);
      _bWorld.setGravity({0.0f, -10.0f, 0.0f});
      _bWorld.setDebugDrawer(&_debugDraw);

      /* Create the ground */
      auto *ground = new RigidBody{&_scene, 0.0f, &_bGroundShape, _bWorld};
      new ColoredDrawable{*ground, _boxInstanceData, 0xffffff_rgbf,
                          Matrix4::scaling({20.0f, 0.5f, 20.0f}), _drawables};

      initializeRoboTank();
      // initializeCustomModel();
      initializeShadowMapping();
      initializeSkybox();

      // _profiler.enable();

      /* Create boxes with random colors */
      Deg      hue = 42.0_degf;
      for (Int i   = 0; i != 5; ++i) {
        for (Int j = 0; j != 5; ++j) {
          for (Int k = 0; k != 5; ++k) {
            if (ENABLE_BOX_CENTRE) {
              auto *o = new RigidBody{&_scene, 1.0f, &_bBoxShape, _bWorld};
              o->translate({i - 2.0f, j + 4.0f, k - 2.0f});
              o->syncPose();
              new ColoredDrawable{*o, _boxInstanceData,
                                  Color3::fromHsv(
                                    {hue += 137.5_degf, 0.75f, 0.9f}),
                                  Matrix4::scaling(Vector3{0.5f}), _drawables};
            }
          }
        }
      }

      initializeGui();

      /* Loop at 60 Hz max */
      setSwapInterval(1);
      setMinimalLoopPeriod(16);
      _timeline.start();
    }

    void BulletExample::drawEvent() {
      GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

      /* Setup the renderer so we can draw the debug lines on top */
      GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
      GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
      GL::Renderer::enable(GL::Renderer::Feature::PolygonOffsetFill);
      GL::Renderer::setPolygonOffset(2.0f, 0.5f);

      /* Housekeeping: remove any objects which are far away from the origin */
      for (Object3D *obj = _scene.children().first(); obj;) {
        Object3D *next = obj->nextSibling();
        if (obj->transformation().translation().dot() > 200 * 200) {
          // obj->transformation().translation() = Vector3{5.0, 1.0, 5.0};
          if (_arcballCamera->getCameraObjectPtr() == (void *) obj) {
          } else {
            //delete obj;
          }
        }

        obj = next;
      }

      /* Step bullet simulation */
      _bWorld.stepSimulation(_timeline.previousFrameDuration(), 5);
      _shader.setLightPosition(_arcballCamera->camera()
                               .cameraMatrix()
                               .transformPoint(_lightRealPosition));

      playerMovement(_timeline.previousFrameDuration());
      calculateStackDemoPositions();

      /* Populate instance data with transformations and colors */
      arrayResize(_boxInstanceData, 0);
      arrayResize(_sphereInstanceData, 0);
      bool camChanged = _arcballCamera->update();

      _texturedShader
        .setLightPosition(_arcballCamera->camera().cameraMatrix().transformPoint(_lightRealPosition));
      _coloredShader
        .setLightPosition(_arcballCamera->camera().cameraMatrix().transformPoint(_lightRealPosition));
      _arcballCamera->draw(_drawables);


      if (_drawCubes) {
        /* Call arcball update in every frame. This will do nothing if the camera
         has not been changed. Otherwise, camera transformation will be
         propagated into the camera objects. */
        _shader.setLightPosition(_arcballCamera->camera()
                                   .cameraMatrix()
                                   .transformPoint(_lightRealPosition));
        _shader.setProjectionMatrix(_projectionMatrix)
          .setTransformationMatrix(_arcballCamera->viewMatrix() * _arcballCamera->transformationMatrix())
          .setNormalMatrix(_arcballCamera->viewMatrix().normalMatrix());

        /* Upload instance data to the GPU (orphaning the previous buffer
           contents) and draw all cubes in one call, and all spheres (if any)
           in another call */
        _boxInstanceBuffer.setData(_boxInstanceData,
                                   GL::BufferUsage::DynamicDraw);
        _box.setInstanceCount(_boxInstanceData.size());
        _shader.draw(_box);

        _sphereInstanceBuffer.setData(_sphereInstanceData,
                                      GL::BufferUsage::DynamicDraw);
        _sphere.setInstanceCount(_sphereInstanceData.size());
        _shader.draw(_sphere);
      }

      /* Debug draw. If drawing on top of cubes, avoid flickering by setting
         depth function to <= instead of just <. */
      if (_drawDebug) {
        if (_drawCubes) {
          GL::Renderer::setDepthFunction(
            GL::Renderer::DepthFunction::LessOrEqual);
        }

        _debugDraw.setTransformationProjectionMatrix(
          _projectionMatrix * _arcballCamera->camera().cameraMatrix());
        _bWorld.debugDrawWorld();

        if (_drawCubes) {
          GL::Renderer::setDepthFunction(GL::Renderer::DepthFunction::Less);
        }
      }

      // No need for this now.
      autoShoot();

      // Blending
      // Shadow Mapping
      //drawShadowMapping();

      /* Draw the skybox */
      drawSkybox();

      /* Imgui*/
      drawImgui();

      /* Update camera before drawing instances */
      bool moving = _arcballCamera->updateTransformation();

      swapBuffers();
      if (!_isPausing) {
        _timeline.nextFrame();
        redraw();
      }

      /* If the camera is moving or the animation is running, redraw immediately */
      if(moving || _animation) redraw();
    }

    void BulletExample::keyPressEvent(KeyEvent &event) {
      redraw();
      if (_imgui.handleKeyPressEvent(event)) return;

      playerControlStart(event);

      /* Toggling draw modes */
      if(event.key() == KeyEvent::Key::Zero|| event.key() == KeyEvent::Key::NumZero) {
        _currentTankLevel = 0;
        _currentScale = 1.0;
      } else if(event.key() == KeyEvent::Key::One|| event.key() == KeyEvent::Key::NumOne) {
        _currentTankLevel = 1;
        _currentScale = 1.0;
      } else if(event.key() == KeyEvent::Key::Two || event.key() == KeyEvent::Key::NumTwo) {
        _currentTankLevel = 2;
        _currentScale = 1.0;
      } else if(event.key() == KeyEvent::Key::N) {
        _animation = true;
        int i = _currentTankLevel;
        Matrix4 *cur_relative_position = _stackedRoboPos.at(i);
        if (event.modifiers() & MouseMoveEvent::Modifier::Alt) {
          if (event.modifiers() & MouseMoveEvent::Modifier::Shift) {
            _currentScale -= 0.2;
          } else {
            _currentScale += 0.2;
          }
        } else {
          if (event.modifiers() & MouseMoveEvent::Modifier::Shift) {
            _currentScale -= 0.2;
          } else {
            _currentScale += 0.2;
          }
        }
        *cur_relative_position = Matrix4::translation({0.0, _currentScale, 0.0});
      }
      event.setAccepted();
      redraw();
    }

    void BulletExample::shootOnClick(MouseEvent &event) {
      /* Shoot an object on click */
      redraw();
      if (event.button() == MouseEvent::Button::Left) {
        shootItem(event.position(), _shootBox);
        event.setAccepted();
      }
    }

    void BulletExample::autoShoot() {
      if (time(NULL) - _lastChecked > 1 && _enableAutoShoot) {
        _lastChecked = time(NULL);
        /* Shoot an object on click */
        Vector2i center = windowSize() / 2;
        shootItem(center, false);
        redraw();
      }
    }


    void BulletExample::drawTreeNodeBoundingBoxes() {
    }

    void BulletExample::viewportEvent(ViewportEvent& event) {
      GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});
      _arcballCamera->reshape(event.windowSize(), event.framebufferSize());

      _projectionMatrix = Matrix4::perspectiveProjection(_arcballCamera->fov(),
                                                         Vector2{event.framebufferSize()}.aspectRatio(),
                                                         0.01f, 100.0f);
      _imgui.relayout(Vector2{event.windowSize()}/event.dpiScaling(),
                      event.windowSize(), event.framebufferSize());
      redraw();
    }

    void BulletExample::mousePressEvent(MouseEvent& event) {
      redraw();
      if(_imgui.handleMousePressEvent(event)) return;
      /* Enable mouse capture so the mouse can drag outside of the window */
      /** @todo replace once https://github.com/mosra/magnum/pull/419 is in */
      SDL_CaptureMouse(SDL_TRUE);
      _arcballCamera->initTransformation(event.position());
      if (!(event.modifiers() & MouseMoveEvent::Modifier::Shift) &&
          (event.modifiers() & MouseMoveEvent::Modifier::Alt)) {
        shootOnClick(event);
      }
      event.setAccepted();
      redraw(); /* camera has changed, redraw! */
    }

    void BulletExample::mouseReleaseEvent(MouseEvent &event) {
      /* Disable mouse capture again */
      /** @todo replace once https://github.com/mosra/magnum/pull/419 is in */
      if(_imgui.handleMouseReleaseEvent(event)) return;
      SDL_CaptureMouse(SDL_FALSE);
      redraw();
    }

    void BulletExample::mouseMoveEvent(MouseMoveEvent& event) {
      if(_imgui.handleMouseMoveEvent(event)) return;
      if(!event.buttons()) return;

      if((event.modifiers() & MouseMoveEvent::Modifier::Shift) &&
         !(event.modifiers() & MouseMoveEvent::Modifier::Alt)) {
        _arcballCamera->translate(event.position());
      } else if (!(event.modifiers() & MouseMoveEvent::Modifier::Shift) &&
        (event.modifiers() & MouseMoveEvent::Modifier::Alt)) {
        // ...
      } else _arcballCamera->rotate(event.position());

      event.setAccepted();
      redraw(); /* camera has changed, redraw! */
    }

    void BulletExample::mouseScrollEvent(MouseScrollEvent& event) {
      if (_imgui.handleMouseMoveEvent(event)) return;
      const Float delta = event.offset().y();
      if(Math::abs(delta) < 1.0e-2f) return;

      _arcballCamera->zoom(delta);

      event.setAccepted();
      redraw(); /* camera has changed, redraw! */
    }

    void BulletExample::drawImgui() {

      _imgui.newFrame();

      /* Enable text input, if needed */
      if(ImGui::GetIO().WantTextInput && !isTextInputActive())
        startTextInput();
      else if(!ImGui::GetIO().WantTextInput && isTextInputActive())
        stopTextInput();

      /* 1. Show a simple window.
         Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets appear in
         a window called "Debug" automatically */
      {
        ImGui::Text("Here are some controls over the tiered drawing scene.");
        ImGui::Separator();
        ImGui::SliderFloat("basePositionX", &_basePositionX, -8.0f, 8.0f);
        ImGui::SliderFloat("basePositionY", &_basePositionY, -8.0f, 8.0f);
        ImGui::SliderFloat("basePositionZ", &_basePositionZ, -8.0f, 8.0f);
        ImGui::SliderFloat("Hue val", &_hueVal, 0.0f, 360.0f);
        ImGui::SliderInt("currentTankLevel", &_currentTankLevel, 0, 4);
        ImGui::SliderFloat("lightPositionX", _lightRealPosition.data(), -100.0f, 100.0f);
        ImGui::SliderFloat("lightPositionY", _lightRealPosition.data() + 1, -100.0f, 100.0f);
        ImGui::SliderFloat("lightPositionZ", _lightRealPosition.data() + 2, -100.0f, 100.0f);
        if(ImGui::ColorEdit3("Clear Color", _clearColor.data()))
          GL::Renderer::setClearColor(_clearColor);
        if(ImGui::Button("Test Window"))
          _showDemoWindow ^= true;
        if(ImGui::Button("Another Window"))
          _showAnotherWindow ^= true;
        if(ImGui::Button("Reset camera position")) {
          _arcballCamera->reset();
        }
        if(ImGui::Button("Shoot a ball")) {
          Vector2i center = windowSize() / 2;
          shootItem(center, false);
        }
        ImGui::SameLine();
        if(ImGui::Button("Shoot a box")) {
          Vector2i center = windowSize() / 2;
          shootItem(center, true);
        }
        if(ImGui::Button("Choose what to shoot when click")) {
          _shootBox ^= true;
        }
        if(ImGui::Button("Show wireframes")) {
          if (_drawCubes && _drawDebug) {
            _drawDebug = false;
          } else if (_drawCubes && !_drawDebug) {
            _drawCubes = false;
            _drawDebug = true;
          } else if (!_drawCubes && _drawDebug) {
            _drawCubes = true;
            _drawDebug = true;
          }
          /* What to shoot */
        }
        ImGui::Checkbox("Enable bounding box", &_drawBoundingBoxes);
        ImGui::Checkbox("Enable auto shooting", &_enableAutoShoot);
        if(ImGui::Button("Profiler collision")) {
          if((_collisionDetectionByOctree ^= true))
            Debug{} << "Collision detection using octree";
          else
            Debug{} << "Collision detection using brute force";
          /* Reset the profiler to avoid measurements of the two methods mixed
             together */
        }
        if(ImGui::Button("Profiler enabling")) {
          if(_profiler.isEnabled()) _profiler.disable();
          else _profiler.enable();
        }
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                    1000.0/Double(ImGui::GetIO().Framerate), Double(ImGui::GetIO().Framerate));
      }

      /**
       * Show scene control window.
       */
      if(true) {
        ImGui::SetNextWindowPos(ImVec2(650, 30), ImGuiCond_FirstUseEver);
        ImGui::Begin("Scene control window", &_showAnotherWindow);
        if(ImGui::Button("Import nanosuit")) {
          importRealFile("/Users/xc5/CLionProjects/opengl/magnum-examples/resources/objects/nanosuit/nanosuit.obj", _manipulator);
        }
        if(ImGui::Button("Import city")) {
          importRealFile("/Users/xc5/CLionProjects/opengl/magnum-examples/resources/3dmodel/city-circular/city.obj", _manipulator);
        }
        ImGui::End();
      }

      /* 2. Show another simple window, now using an explicit Begin/End pair */
      if(_showAnotherWindow) {
        ImGui::SetNextWindowSize(ImVec2(500, 100), ImGuiCond_FirstUseEver);
        ImGui::Begin("Another Window", &_showAnotherWindow);
        ImGui::Text("Hello");
        ImGui::End();
      }

      /* 3. Show the ImGui demo window. Most of the sample code is in
         ImGui::ShowDemoWindow() */
      if(_showDemoWindow) {
        ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiCond_FirstUseEver);
        ImGui::ShowDemoWindow();
      }

      /* Update application cursor */
      _imgui.updateApplicationCursor(*this);

      /* Set appropriate states. If you only draw ImGui, it is sufficient to
         just enable blending and scissor test in the constructor. */
      GL::Renderer::enable(GL::Renderer::Feature::Blending);
      GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);
      GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);
      GL::Renderer::disable(GL::Renderer::Feature::DepthTest);

      _imgui.drawFrame();

      /* Reset state. Only needed if you want to draw something else with
         different state after. */
      GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
      GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
      GL::Renderer::disable(GL::Renderer::Feature::ScissorTest);
      GL::Renderer::disable(GL::Renderer::Feature::Blending);

    }

    void BulletExample::initializeGui() {
      _imgui = ImGuiIntegration::Context(Vector2{windowSize()}/dpiScaling(),
                                         windowSize(), framebufferSize());
      /* Set up proper blending to be used by ImGui. There's a great chance
       you'll need this exact behavior for the rest of your scene. If not, set
       this only for the drawFrame() call. */
      GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add,
                                     GL::Renderer::BlendEquation::Add);
      GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
                                     GL::Renderer::BlendFunction::OneMinusSourceAlpha);
    }

    void
    BulletExample::keyReleaseEvent(Platform::Sdl2Application::KeyEvent &event) {
      if(_imgui.handleKeyReleaseEvent(event)) return;
      playerControlEnd(event);
    }

    void BulletExample::calculateStackDemoPositions() {
      RigidBody *rootRigidBody = (RigidBody*) _stackedRoboTank[0];
      _basePositionX = rootRigidBody->transformation().translation().x();
      _basePositionY = rootRigidBody->transformation().translation().y();
      _basePositionZ = rootRigidBody->transformation().translation().z();

      _basePosition     = Vector3{_basePositionX, _basePositionY, _basePositionZ};
      Vector3 parent_position = _basePosition;
      CORRADE_ASSERT(_stackedRoboTank.size() == 5,
                     "The list should contain 5 robot tank items.", );

      for (int i = 0; i < 5; i++) {
        Matrix4 *cur_relative_position = _stackedRoboPos.at(i);
        Vector3 cur_position = (*cur_relative_position).transformPoint(parent_position);
        /* Create the 'robotank' */
        RigidBody *robotank = (RigidBody *) _stackedRoboTank.at(i);
        CORRADE_ASSERT(robotank != NULL, "Should not be null", );
        if (robotank && robotank->isAvailable() && i > 0) {
          robotank->resetTransformation();
          robotank->translate(cur_position);
          robotank->setDirty();
          robotank->syncPose();
        }
        parent_position = cur_position;
      }
    }

    void BulletExample::shootItem(Vector2i startLocation, bool isBox) {
      /* First scale the position from being relative to window size to being
           relative to framebuffer size as those two can be different on HiDPI
           systems */
      const Vector2i position   = startLocation * Vector2{framebufferSize()} /
                       Vector2{windowSize()};
      const Vector2  clickPoint = Vector2::yScale(-1.0f) *
                                  (Vector2{position} /
                                   Vector2{framebufferSize()} -
                                   Vector2{0.5f}) * 1.0f;
      // Then calculate the click point ?->yes
      const Vector3  direction  =
                       (_arcballCamera->camera().object().absoluteTransformationMatrix().rotationScaling() *
                        Vector3{clickPoint, -1.0f}).normalized();

      auto *object = new RigidBody{
        &_scene,
        isBox ? 1.0f : 5.0f,
        isBox ? static_cast<btCollisionShape *>(&_bBoxShape)
                  : &_bSphereShape,
        _bWorld};

      object->translate(
        _arcballCamera->transformation().translation());
      /* Has to be done explicitly after the translate() above, as Magnum ->
         Bullet updates are implicitly done only for kinematic bodies */
      object->syncPose();

      Deg      hue = Deg(_hueVal);
      Color3   color = Color3::fromHsv({hue, 0.75f, 0.9f});
      _hueVal += 137.5;
      /* Create either a box or a sphere */
      new ColoredDrawable{*object,
                          isBox ? _boxInstanceData : _sphereInstanceData,
                          color,
                          Matrix4::scaling(Vector3{isBox ? 0.5f : 0.25f}),
                          _drawables};

      /* Give it an initial velocity */
      object->rigidBody().setLinearVelocity(btVector3{direction * 25.f});

    }

    void BulletExample::initializeRoboTank() {
      _basePosition     = Vector3{_basePositionX, _basePositionY, _basePositionZ};
      _currentTankLevel = 0;
      _currentScale = 1.0;
      Vector3 parent_position = _basePosition;
      Deg      hue = 42.0_degf;
      for (int i = 0; i < 5; i++) {
        Matrix4 *cur_relative_position = new Matrix4{0.0f};
        *cur_relative_position = Matrix4::translation({0.0, ((i == 0) ? (0.0f) : 1.0f), 0.0});
        Vector3 cur_position = (*cur_relative_position).transformVector(parent_position);
        /* Create the 'robotank' */
        RigidBody *robotank = new RigidBody{&_scene, 80.0f, &_roboTankShape, _bWorld};
        //        robotank->rigidBody().setCollisionFlags( robotank->rigidBody().getCollisionFlags() |
        //                                                 btCollisionObject::CF_KINEMATIC_OBJECT);
        //        robotank->rigidBody().setActivationState(DISABLE_DEACTIVATION);
//        new ColoredDrawable{*robotank, _boxInstanceData, Color3::fromHsv(
//                            {hue += 137.5_degf, 0.75f, 0.9f}),
//                            Matrix4::scaling({1.0f, 0.5f, 0.5f}), _drawables};

        robotank->translate(cur_position);
        parent_position = cur_position;
        robotank->syncPose();
        _stackedRoboPos.push_back(cur_relative_position);
        _stackedRoboTank.push_back(robotank);
      }

      // initialize some walls;
      Vector3 _xshape = {5.0, 5.0, 0.5};
      createWall(_xshape, Vector3{2.5f, 5.0f, -20.0f});
      createWall(_xshape, Vector3{2.5f, 5.0f, -30.0f});
      createWall(_xshape, Vector3{6.5f, 5.0f, 12.0f});
      createWall(_xshape, Vector3{6.5f, 5.0f, 16.0f});

    }

    void BulletExample::createWall(const Vector3 &_xshape,
                                   const Vector3 &pos) {
      RigidBody *onewall = new RigidBody{&_scene, 0.0f, &_xwallShape, _bWorld};
      onewall->rigidBody().setCollisionFlags( onewall->rigidBody().getCollisionFlags() |
                                               btCollisionObject::CF_KINEMATIC_OBJECT);
      onewall->rigidBody().setActivationState(DISABLE_DEACTIVATION);
      new ColoredDrawable{*onewall, _boxInstanceData, 0xffffff_rgbf,
                          Matrix4::scaling(_xshape), _drawables};
      onewall->translate(pos);
      onewall->syncPose();
    }

    void BulletExample::playerControlStart(KeyEvent &event) {
      if (event.key() == KeyEvent::Key::W) {
        _directionsPressed[0] = true;
      } else if (event.key() == KeyEvent::Key::A) {
        _directionsPressed[1] = true;
      } else if (event.key() == KeyEvent::Key::S) {
        _directionsPressed[2] = true;
      } else if (event.key() == KeyEvent::Key::D) {
        _directionsPressed[3] = true;
      } else if (event.key() == KeyEvent::Key::Space) {
        _directionsPressed[4] = true;
      } else if (event.key() == KeyEvent::Key::K) {
        RigidBody *rootRigidBody = getRootRigidBody();
        rootRigidBody->rigidBody().setLinearVelocity(btVector3{0.0f, 0.0f, .0f});
      }
    }
    void BulletExample::playerControlEnd(KeyEvent &event) {
      if (event.key() == KeyEvent::Key::W) {
        _directionsPressed[0] = false;
      } else if (event.key() == KeyEvent::Key::A) {
        _directionsPressed[1] = false;
      } else if (event.key() == KeyEvent::Key::S) {
        _directionsPressed[2] = false;
      } else if (event.key() == KeyEvent::Key::D) {
        _directionsPressed[3] = false;
      } else if (event.key() == KeyEvent::Key::Space) {
        _directionsPressed[4] = false;
      }
    }
    void BulletExample::playerMovement(Float) {
      Float step = 10.0f;
      Float yspeed = 0.0f;
      Float zspeed = 0.0f;
      Float xspeed = 0.0f;
      if(_directionsPressed[0]) {
        zspeed = -step;
      }
      if(_directionsPressed[2]) {
        zspeed = step;
      }
      if(_directionsPressed[1]) {
        xspeed = -step;
      }
      if(_directionsPressed[3]) {
        xspeed = step;
      }
      if (_directionsPressed[4]) {
        // Jump now.
        yspeed = 15.0f;
      } else {
        yspeed = 0.0;
      }
      RigidBody *rigidBody = (RigidBody*) _stackedRoboTank[0];
      if (_directionsPressed[0] || _directionsPressed[1]
       || _directionsPressed[2] || _directionsPressed[3] || _directionsPressed[4]) {
        rigidBody->rigidBody().setLinearVelocity(btVector3{xspeed, yspeed, zspeed});
      }
      _manipulator.resetTransformation();
      _manipulator.setTransformation(Matrix4::translation(rigidBody->transformation().translation()));
    }

    RigidBody *BulletExample::getRootRigidBody() {
      return (RigidBody*) _stackedRoboTank[0];
    }

    void BulletExample::addObject(Trade::AbstractImporter &importer,
                                  Containers::Array<Containers::Optional<GL::Texture2D>> &currentTextures,
                                  Containers::ArrayView<const Containers::Optional<Trade::PhongMaterialData>> materials,
                                  Object3D &parent, UnsignedInt i) {
      Debug{} << "Importing object" << i << importer.object3DName(i);
      Containers::Pointer<Trade::ObjectData3D> objectData = importer.object3D(i);
      if(!objectData) {
        Error{} << "Cannot import object, skipping";
        return;
      }

      /* Add the object to the scene and set its transformation */
      auto* object = new Object3D{&parent};
      object->setTransformation(objectData->transformation());

      /* Add a drawable if the object has a mesh and the mesh is loaded */
      if(objectData->instanceType() == Trade::ObjectInstanceType3D::Mesh
      && objectData->instance() != -1
      && _meshes[objectData->instance()]) {
        const Int materialId = static_cast<Trade::MeshObjectData3D*>(objectData.get())->material();

        /* Material not available / not loaded, use a default material */
        if(materialId == -1 || !materials[materialId]) {
           new NewColoredDrawable{*object, _coloredShader, *_meshes[objectData->instance()], 0xffffff_rgbf, _drawables};

          /* Textured material. If the texture failed to load, again just use a
             default colored material. */
        } else if(materials[materialId]->flags() & Trade::PhongMaterialData::Flag::DiffuseTexture) {
          Containers::Optional<GL::Texture2D>& texture = currentTextures[materials[materialId]->diffuseTexture()];
          if(texture)
            new TexturedDrawable{*object, _texturedShader, *_meshes[objectData->instance()], *texture, _drawables};
          else
            new NewColoredDrawable{*object, _coloredShader, *_meshes[objectData->instance()], 0xffffff_rgbf, _drawables};

          /* Color-only material */
        } else {
          new NewColoredDrawable{*object, _coloredShader, *_meshes[objectData->instance()], materials[materialId]->diffuseColor(), _drawables};
        }
      }

      /* Recursively add children */
      for(std::size_t id: objectData->children())
        addObject(importer, currentTextures, materials, *object, id);
    }

    void BulletExample::prepareShaders() {
      _lightRealPosition = {0.0f, 80.0f, 80.0f};

      /* Create an instanced shader */
      _shader = Shaders::Phong{
        Shaders::Phong::Flag::VertexColor |
        Shaders::Phong::Flag::InstancedTransformation};

      _shader.setAmbientColor(0x404040_rgbf)
        .setSpecularColor(0xffffff_rgbf)
        .setLightPosition(_arcballCamera->camera()
                            .cameraMatrix()
                            .transformPoint(_lightRealPosition));

      _coloredShader = Shaders::Phong{};
      _coloredShader
        .setAmbientColor(0x111111_rgbf)
        .setSpecularColor(0xffffff_rgbf)
        .setShininess(80.0f);

      _texturedShader = Shaders::Phong{Shaders::Phong::Flag::DiffuseTexture, 1};
      _texturedShader
        .setLightPosition({0.0f, 0.0f, 0.0f})
        .setAmbientColor(0x111111_rgbf)
        .setSpecularColor(0xffffff_rgbf)
        .setShininess(100.0f);

      _customShader = TexturedTriangleShader();
    }
    void BulletExample::setupCamera() {
      /* Setup camera */
      {
        const Vector3 eye = Vector3::zAxis(5.0f);
        const Vector3 viewCenter;
        const Vector3 up = Vector3::yAxis();
        const Deg fov = 45.0_degf;
        _arcballCamera.emplace(_scene, eye, viewCenter, up, fov,
                               windowSize(), framebufferSize());
        _arcballCamera->setLagging(0.85f);

        _projectionMatrix = Matrix4::perspectiveProjection(fov,
                                                           Vector2{framebufferSize()}.aspectRatio(),
                                                           0.01f, 100.0f);
      }
    }

    void BulletExample::importFile(const Arguments &arguments) {
      Utility::Arguments args;
      args.addArgument("file").setHelp("file", "file to load")
        .addOption("importer", "AnySceneImporter").setHelp("importer",
                                                           "importer plugin to use")
        .addSkippedPrefix("magnum", "engine-specific options")
        .setGlobalHelp("Displays a 3D scene file provided on command line.")
        .parse(arguments.argc, arguments.argv);

      /* Base object, parent of all (for easy manipulation) */
      _manipulator.setParent(&_scene);
      const std::string importerName = args.value("importer");
      const std::string filename = args.value("file");
      PluginManager::Manager<Trade::AbstractImporter> *importer = new PluginManager::Manager<Trade::AbstractImporter>();
      _manager = Containers::Pointer<PluginManager::Manager<Trade::AbstractImporter>> (importer);
      _importer = _manager->loadAndInstantiate(importerName);
      if(!_importer) std::exit(1);
      //importRealFile(filename, _manipulator);
    }

    /**
     * Import a file into the scene.
     */
    void BulletExample::importRealFile(const std::string filename, Object3D &parent) {
      /* Load a scene importer plugin */

      _importer = _manager->loadAndInstantiate("AnySceneImporter");
      if(!_importer) std::exit(1);

      Debug{} << "Opening file" << filename;

      /* Load file */
      if(!_importer->openFile(filename))
        std::exit(4);

      /* Load all textures. Textures that fail to load will be NullOpt. */
      //      Int originalSize = 0;
      //      if (_textures.size() > 0) {
      //        originalSize = _textures.size();
      //      }

      // Textures
      Containers::Array<Containers::Optional<GL::Texture2D>> &_textures =
        *new Containers::Array<Containers::Optional<GL::Texture2D>>{_importer->textureCount()};
      _allTextures.push_back(&_textures);
      for(UnsignedInt i = 0; i != _importer->textureCount(); ++i) {
        Debug{} << "Importing texture" << i << _importer->textureName(i);

        Containers::Optional<Trade::TextureData> textureData = _importer->texture(i);
        if(!textureData || textureData->type() != Trade::TextureData::Type::Texture2D) {
          Warning{} << "Cannot load texture properties, skipping";
          continue;
        }

        UnsignedInt imageId = textureData->image();
        Debug{} << "Importing image" << imageId << _importer->image2DName(imageId);

        Containers::Optional<Trade::ImageData2D> imageData = _importer->image2D(imageId);
        GL::TextureFormat format;
        if(imageData && imageData->format() == PixelFormat::RGB8Unorm)
          format = GL::TextureFormat::RGB8;
        else if(imageData && imageData->format() == PixelFormat::RGBA8Unorm)
          format = GL::TextureFormat::RGBA8;
        else {
          Warning{} << "Cannot load texture image, skipping";
          continue;
        }

        /* Configure the texture */
        GL::Texture2D &texture = *new GL::Texture2D();
        texture
          .setMagnificationFilter(textureData->magnificationFilter())
          .setMinificationFilter(textureData->minificationFilter(), textureData->mipmapFilter())
          .setWrapping(textureData->wrapping().xy())
          .setStorage(Math::log2(imageData->size().max()) + 1, format, imageData->size())
          .setSubImage(0, {}, *imageData)
          .generateMipmap();

        _textures[i] = std::move(texture);
      }

      /* Load all materials. Materials that fail to load will be NullOpt. The
         data will be stored directly in objects later, so save them only
         temporarily. */
      Containers::Array<Containers::Optional<Trade::PhongMaterialData>> materials{_importer->materialCount()};
      for(UnsignedInt i = 0; i != _importer->materialCount(); ++i) {
        Debug{} << "Importing material" << i << _importer->materialName(i);

        Containers::Pointer<Trade::AbstractMaterialData> materialData = _importer->material(i);
        if(!materialData || !(materialData->type() == Trade::MaterialType::Phong)) {
          Warning{} << "Cannot load material, skipping";
          continue;
        }

        materials[i] = std::move(static_cast<Trade::PhongMaterialData&>(*materialData));
      }

      /* Load all meshes. Meshes that fail to load will be NullOpt. */
      _meshes = Containers::Array<Containers::Optional<GL::Mesh>>{_importer->meshCount()};
      for(UnsignedInt i = 0; i != _importer->meshCount(); ++i) {
        Debug{} << "Importing mesh" << i << _importer->meshName(i);

        Containers::Optional<Trade::MeshData> meshData = _importer->mesh(i);
        if(!meshData || !meshData->hasAttribute(Trade::MeshAttribute::Normal) || meshData->primitive() != MeshPrimitive::Triangles) {
          Warning{} << "Cannot load the mesh, skipping";
          continue;
        }

        /* Compile the mesh */
        _meshes[i] = MeshTools::compile(*meshData);
      }

      /* Load the scene */
      if(_importer->defaultScene() != -1) {
        Debug{} << "Adding default scene" << _importer->sceneName(_importer->defaultScene());

        Containers::Optional<Trade::SceneData> sceneData = _importer->scene(_importer->defaultScene());
        if(!sceneData) {
          Error{} << "Cannot load scene, exiting";
          return;
        }

        /* Recursively add all children */
        for(UnsignedInt objectId: sceneData->children3D())
          addObject(*_importer, _textures, materials, parent, objectId);

        /* The format has no scene support, display just the first loaded mesh with
           a default material and be done with it */
      } else if(!_meshes.empty() && _meshes[0])
        new NewColoredDrawable{parent, _coloredShader, *_meshes[0], 0xffffff_rgbf, _drawables};
    }

    void BulletExample::configureWindow() {
      const Vector2 dpiScaling = this->dpiScaling({});
      Configuration conf;
      conf.setTitle("Guanting Lu 2017152003 Final Project")
        .setSize(conf.size(), dpiScaling);
      conf.setWindowFlags(Configuration::WindowFlag::Resizable);
      GLConfiguration glConf;
      // Work with DPI-awareness.
      glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
      if (!tryCreate(conf, glConf)) {
        create(conf, glConf.setSampleCount(0));
      }
    }

    void BulletExample::initializeCustomModel() {
    }

    void BulletExample::loadTextureFromImage(const std::string & filename) {
      Containers::Pointer<Trade::AbstractImporter> importer = _manager->loadAndInstantiate("TgaImporter");
      if(!importer) std::exit(1);

      /* Load the texture */
      const Utility::Resource rs{"textured-triangle-data"};
      if(!importer->openData(rs.getRaw("stone.tga")))
        std::exit(2);

      /* Set texture data and parameters */
      Containers::Optional<Trade::ImageData2D> image = importer->image2D(0);
      CORRADE_INTERNAL_ASSERT(image);
      _stoneTexture.setWrapping(GL::SamplerWrapping::ClampToEdge)
        .setMagnificationFilter(GL::SamplerFilter::Linear)
        .setMinificationFilter(GL::SamplerFilter::Linear)
        .setStorage(1, GL::textureFormat(image->format()), image->size())
        .setSubImage(0, {}, *image);

      std::vector<std::string> faces
       {
         FileSystem::getPath("resources/textures/skybox/right.jpg"),
         FileSystem::getPath("resources/textures/skybox/left.jpg"),
         FileSystem::getPath("resources/textures/skybox/top.jpg"),
         FileSystem::getPath("resources/textures/skybox/bottom.jpg"),
         FileSystem::getPath("resources/textures/skybox/front.jpg"),
         FileSystem::getPath("resources/textures/skybox/back.jpg")
       };

      loadCubemap(faces);
    }

    template <typename RESTYPE, typename INTYPE, int size>
    RESTYPE getGlmMat4(const INTYPE &mat4) {
      RESTYPE result(0.0f);
      for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
          result[i][j] = mat4[i][j];
        }
      }
      return result;
    }


// utility function for loading a 2D texture from file
// ---------------------------------------------------
    unsigned int loadTexture(char const * path)
    {
      unsigned int textureID;
      glGenTextures(1, &textureID);

      int width, height, nrComponents;
      unsigned char *data = stbi_load(path, &width, &height, &nrComponents, 0);
      if (data)
      {
        GLenum format;
        if (nrComponents == 1)
          format = GL_RED;
        else if (nrComponents == 3)
          format = GL_RGB;
        else if (nrComponents == 4)
          format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
      }
      else
      {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
      }

      return textureID;
    }

    // loads a cubemap texture from 6 individual texture faces
    // order:
    // +X (right)
    // -X (left)
    // +Y (top)
    // -Y (bottom)
    // +Z (front)
    // -Z (back)
    // -------------------------------------------------------
    unsigned int loadSimpleCubeMap(const std::vector<std::string> &faces)
    {
      unsigned int textureID;
      glGenTextures(1, &textureID);
      glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

      int width, height, nrChannels;
      for (unsigned int i = 0; i < faces.size(); i++)
      {
        unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
        if (data)
        {
          glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
          stbi_image_free(data);
        }
        else
        {
          std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
          stbi_image_free(data);
        }
      }
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

      return textureID;
    }


    void BulletExample::initializeSkybox() {
      loadTextureFromImage("");
      Vector3                       _skyboxScaling{20.0f, 20.0f, 20.0f};

      struct TriangleVertex {
        Vector3 position;
        Vector2 textureCoordinates;
      };
      // set up vertex data (and buffer(s)) and configure vertex attributes
      // ------------------------------------------------------------------
      float cubeVertices[] = {
        // positions          // texture Coords
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
        0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
        0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
        0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
        0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
        0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
        0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

        -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

        0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
        0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
        0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
        0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
        0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
        0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
        0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
        0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
        0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
        0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
        0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
        0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f
      };
      float skyboxVertices[] = {
        // positions
        -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f,
        1.0f, -1.0f, -1.0f,
        1.0f, -1.0f, -1.0f,
        1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,

        -1.0f, -1.0f,  1.0f,
        -1.0f, -1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,

        1.0f, -1.0f, -1.0f,
        1.0f, -1.0f,  1.0f,
        1.0f,  1.0f,  1.0f,
        1.0f,  1.0f,  1.0f,
        1.0f,  1.0f, -1.0f,
        1.0f, -1.0f, -1.0f,

        -1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f,  1.0f,
        1.0f,  1.0f,  1.0f,
        1.0f,  1.0f,  1.0f,
        1.0f, -1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,

        -1.0f,  1.0f, -1.0f,
        1.0f,  1.0f, -1.0f,
        1.0f,  1.0f,  1.0f,
        1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,

        -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
        1.0f, -1.0f, -1.0f,
        1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
        1.0f, -1.0f,  1.0f
      };


      // build and compile shaders
      // -------------------------
      _skyboxShader = new Shader("/Users/xc5/CLionProjects/opengl/magnum-examples/src/bullet/6.1.skybox.vs",
                                 "/Users/xc5/CLionProjects/opengl/magnum-examples/src/bullet/6.1.skybox.fs");
      _skyboxShader->use();
      _skyboxShader->setInt("skybox", 0);
      // skybox VAO
      unsigned int skyboxVAO, skyboxVBO;
      glGenVertexArrays(1, &skyboxVAO);
      glGenBuffers(1, &skyboxVBO);
      glBindVertexArray(skyboxVAO);
      glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
      glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
      _skyboxVao = skyboxVAO;

      GL::Buffer buffer;
      buffer.setData(skyboxVertices);
      GL::Mesh &mesh = *(new GL::Mesh());
      mesh.setCount(36)
        .addVertexBuffer(std::move(buffer), 0,
                         TexturedTriangleShader::Position{});

      // GL::Mesh                      skybox          = MeshTools::compile(Primitives::cubeSolid());
      const Matrix4                 matrix4        = Matrix4::scaling(_skyboxScaling);
      TexturedWithPrimitiveDrawable *skyboxDrawable = new TexturedWithPrimitiveDrawable
        {_manipulator, _customShader, mesh, _skyboxTexture, matrix4, _skyboxGroup};
    }

    void BulletExample::drawSkybox() {
      GL::Renderer::setDepthFunction(GL::Renderer::DepthFunction::LessOrEqual);
//      GL::Renderer::setFaceCullingMode(GL::Renderer::PolygonFacing::Front);

      // draw skybox as last
//      glDepthFunc(GL_LEQUAL);  // change depth function so depth test passes when values are equal to depth buffer's content
      _skyboxShader->use();
      Matrix4 view = _arcballCamera->viewMatrix(); //* _arcballCamera->transformationMatrix(); // remove translation from the view matrix
      Matrix4 proj = _projectionMatrix;

      view = Matrix4(view.rotationScaling());
      proj = Matrix4(proj.rotationScaling());

      glm::mat4 viewglm = getGlmMat4<glm::mat4, Matrix4, 4>(view);
      glm::mat4 projglm = getGlmMat4<glm::mat4, Matrix4, 4>(proj); // glm::mat4(1.0f);//getGlmMat4<glm::mat4, Matrix4, 4>(proj);
      _skyboxShader->setMat4("view", viewglm);
      _skyboxShader->setMat4("projection", projglm);

      // skybox cube
      glBindVertexArray(_skyboxVao);
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_CUBE_MAP, _cubeMapTexture);
      glDrawArrays(GL_TRIANGLES, 0, 36);
      // glDepthFunc(GL_LESS); // set depth function back to default
      GL::Renderer::setDepthFunction(GL::Renderer::DepthFunction::Less);

      // _arcballCamera->draw(_skyboxGroup);
      //GL::Renderer::setFaceCullingMode(GL::Renderer::PolygonFacing::Back);
    }

    unsigned int BulletExample::loadCubemap(std::vector<std::string> faces) {
      _cubeMapTexture = loadSimpleCubeMap(faces);
      Containers::Pointer<Trade::AbstractImporter> importer = _manager->loadAndInstantiate("JpegImporter");
      if(!importer) std::exit(1);
      _skyboxTexture = GL::CubeMapTexture{};
      _skyboxTexture.setWrapping(GL::SamplerWrapping::ClampToEdge)
        .setMagnificationFilter(GL::SamplerFilter::Linear)
        .setMinificationFilter(GL::SamplerFilter::Linear);

      GL::CubeMapCoordinate idx[] = {GL::CubeMapCoordinate::PositiveX,
                                     GL::CubeMapCoordinate::NegativeX,
                                     GL::CubeMapCoordinate::NegativeY,
                                     GL::CubeMapCoordinate::PositiveY,
                                     GL::CubeMapCoordinate::PositiveZ,
                                     GL::CubeMapCoordinate::NegativeZ};
      int i = 0;
      for (auto oneFile: faces) {
        if (!importer->openFile(oneFile))
          std::exit(2);
        /* Set texture data and parameters */
        Containers::Optional<Trade::ImageData2D> image = importer->image2D(0);
        CORRADE_INTERNAL_ASSERT(image);
//          .setStorage(Math::log2(256)+1, GL::TextureFormat::RGBA8, {256, 256})
        _skyboxTexture.setStorage(1, GL::textureFormat(image->format()), image->size())
          .setSubImage(idx[i], 0, {}, *image);
        i++;
      }
      return 0;
    }

    void BulletExample::initializeShadowMapping() {

      // build and compile shaders
      // -------------------------
      _mappingShader = new Shader(FileSystem::getPath("b/3.1.3.shadow_mapping.vs").c_str(),
                                  FileSystem::getPath("b/3.1.3.shadow_mapping.fs").c_str());
      Shader &shader = *_mappingShader;
      Shader &simpleDepthShader = *new Shader(FileSystem::getPath("b/3.1.3.shadow_mapping_depth.vs").c_str(),
                                              FileSystem::getPath("b/3.1.3.shadow_mapping_depth.fs").c_str());
      Shader &debugDepthQuad = *new Shader(FileSystem::getPath("b/3.1.3.debug_quad.vs").c_str(),
                                           FileSystem::getPath("b/3.1.3.debug_quad_depth.fs").c_str());
      _simpleDepthShaderPtr = &simpleDepthShader;
      _debugDepthQuadPtr = &debugDepthQuad;

      // set up vertex data (and buffer(s)) and configure vertex attributes
      // ------------------------------------------------------------------
      float planeVertices[] = {
        // positions            // normals         // texcoords
        25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,  25.0f,  0.0f,
        -25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,   0.0f,  0.0f,
        -25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,   0.0f, 25.0f,

        25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,  25.0f,  0.0f,
        -25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,   0.0f, 25.0f,
        25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,  25.0f, 25.0f
      };
      // plane VAO
      unsigned int planeVBO;
      glGenVertexArrays(1, &_planeVAO);
      glGenBuffers(1, &planeVBO);
      glBindVertexArray(_planeVAO);
      glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
      glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
      glEnableVertexAttribArray(2);
      glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
      glBindVertexArray(0);

      // load textures
      // -------------
      unsigned int _woodTexture = loadTexture(FileSystem::getPath("resources/textures/wood.png").c_str());

      // configure depth map FBO
      // -----------------------
      const unsigned int SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;
      glGenFramebuffers(1, &_depthMapFBO);
      // create depth texture
      unsigned int &depthMap = _depthMap;
      glGenTextures(1, &depthMap);
      glBindTexture(GL_TEXTURE_2D, depthMap);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
      float borderColor[] = { 1.0, 1.0, 1.0, 1.0 };
      glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
      // attach depth texture as FBO's depth buffer
      glBindFramebuffer(GL_FRAMEBUFFER, _depthMapFBO);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
      glDrawBuffer(GL_NONE);
      glReadBuffer(GL_NONE);
      glBindFramebuffer(GL_FRAMEBUFFER, 0);


      // shader configuration
      // --------------------
      shader.use();
      shader.setInt("diffuseTexture", 0);
      shader.setInt("shadowMap", 1);
      debugDepthQuad.use();
      debugDepthQuad.setInt("depthMap", 0);

    }

    // renders the 3D scene
    // --------------------
    void BulletExample::renderScene(const Shader &shader)
    {
      // floor
      glm::mat4 model = glm::mat4(1.0f);
      shader.setMat4("model", model);
      glBindVertexArray(_planeVAO);
      glDrawArrays(GL_TRIANGLES, 0, 6);
      // cubes
      model = glm::mat4(1.0f);
      model = glm::translate(model, glm::vec3(0.0f, 1.5f, 0.0));
      model = glm::scale(model, glm::vec3(0.5f));
      shader.setMat4("model", model);
      renderCube();
      model = glm::mat4(1.0f);
      model = glm::translate(model, glm::vec3(2.0f, 0.0f, 1.0));
      model = glm::scale(model, glm::vec3(0.5f));
      shader.setMat4("model", model);
      renderCube();
      model = glm::mat4(1.0f);
      model = glm::translate(model, glm::vec3(-1.0f, 0.0f, 2.0));
      model = glm::rotate(model, glm::radians(60.0f), glm::normalize(glm::vec3(1.0, 0.0, 1.0)));
      model = glm::scale(model, glm::vec3(0.25));
      shader.setMat4("model", model);
      renderCube();
    }

    void BulletExample::renderCube()
    {
      // initialize (if necessary)
      if (_cubeVAO == 0)
      {
        float vertices[] = {
          // back face
          -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
          1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
          1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, // bottom-right
          1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
          -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
          -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f, // top-left
          // front face
          -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
          1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f, // bottom-right
          1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
          1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
          -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, // top-left
          -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
          // left face
          -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
          -1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-left
          -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
          -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
          -1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-right
          -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
          // right face
          1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
          1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
          1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-right
          1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
          1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
          1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-left
          // bottom face
          -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
          1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 1.0f, // top-left
          1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
          1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
          -1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f, // bottom-right
          -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
          // top face
          -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
          1.0f,  1.0f , 1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
          1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f, // top-right
          1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
          -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
          -1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f  // bottom-left
        };
        glGenVertexArrays(1, &_cubeVAO);
        glGenBuffers(1, &_cubeVBO);
        // fill buffer
        glBindBuffer(GL_ARRAY_BUFFER, _cubeVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        // link vertex attributes
        glBindVertexArray(_cubeVAO);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
      }
      // render Cube
      glBindVertexArray(_cubeVAO);
      glDrawArrays(GL_TRIANGLES, 0, 36);
      glBindVertexArray(0);
    }

    void BulletExample::drawShadowMapping() {
      Shader &shader = *_mappingShader;
      Shader &simpleDepthShader = *_simpleDepthShaderPtr;
      Shader &debugDepthQuad = *_debugDepthQuadPtr;

      // 1. render depth of scene to texture (from light's perspective)
      // --------------------------------------------------------------
      glm::mat4 lightProjection, lightView;
      glm::mat4 lightSpaceMatrix;
      float near_plane = 1.0f, far_plane = 7.5f;

      //lightProjection = glm::perspective(glm::radians(45.0f), (GLfloat)SHADOW_WIDTH / (GLfloat)SHADOW_HEIGHT, near_plane, far_plane);
      // note that if you use a perspective projection matrix you'll
      // have to change the light position as the current light position
      // isn't enough to reflect the whole scene
      lightProjection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, near_plane, far_plane);
      lightView = glm::lookAt(glm::make_vec3(_lightRealPosition.data()), glm::vec3(0.0f), glm::vec3(0.0, 1.0, 0.0));
      lightSpaceMatrix = lightProjection * lightView;
      // render scene from light's point of view
      simpleDepthShader.use();
      simpleDepthShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

      glBindFramebuffer(GL_FRAMEBUFFER, _depthMapFBO);
//      glClear(GL_DEPTH_BUFFER_BIT);

      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, _woodTexture);
      renderScene(simpleDepthShader);
      glBindFramebuffer(GL_FRAMEBUFFER, 0);

      // reset viewport
//      glViewport(0, 0, windowSize().x(), windowSize().y());
//      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      // 2. render scene as normal using the generated depth/shadow map
      // --------------------------------------------------------------
//      glViewport(0, 0, windowSize().x(), windowSize().y());
//      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      shader.use();
      glm::mat4 projection = getGlmMat4<glm::mat4, Matrix4, 4>(_projectionMatrix);
      glm::mat4 view = getGlmMat4<glm::mat4, Matrix4, 4>(_arcballCamera->viewMatrix());
      shader.setMat4("projection", projection);
      shader.setMat4("view", view);
      // set light uniforms
      shader.setVec3("viewPos", glm::make_vec3(_arcballCamera->transformation().translation().data()));
      shader.setVec3("lightPos", glm::make_vec3(_lightRealPosition.data()));
      shader.setMat4("lightSpaceMatrix", lightSpaceMatrix);
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, _woodTexture);
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, _depthMap);
      renderScene(shader);

      // render Depth map to quad for visual debugging
      // ---------------------------------------------
      debugDepthQuad.use();
      debugDepthQuad.setFloat("near_plane", near_plane);
      debugDepthQuad.setFloat("far_plane", far_plane);
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, _depthMap);
      //renderQuad();

    }
  }
}

MAGNUM_APPLICATION_MAIN(Magnum::LabFinal::BulletExample)
