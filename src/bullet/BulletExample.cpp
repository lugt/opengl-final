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
      Shaders::Phong _coloredShader{NoCreate}, _texturedShader{NoCreate};
      Containers::Array<Containers::Optional<GL::Mesh>>      _meshes;
      Containers::Array<Containers::Optional<GL::Texture2D>> _textures;
      Object3D _manipulator;
      std::string _importer;

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
      GL::Mesh      _mesh{NoCreate};
      GL::Texture2D _texture{NoCreate};

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
      initializeCustomModel();

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
      GL::defaultFramebuffer.clear(
        GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

      /* Housekeeping: remove any objects which are far away from the origin */
      for (Object3D *obj = _scene.children().first(); obj;) {
        Object3D *next = obj->nextSibling();
        if (obj->transformation().translation().dot() > 200 * 200) {
          // obj->transformation().translation() = Vector3{5.0, 1.0, 5.0};
          delete obj;
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

      drawImgui();
      autoShoot();

      /* Update camera before drawing instances */
      bool moving = _arcballCamera->updateTransformation();

      swapBuffers();
      if (!_isPausing) {
        _timeline.nextFrame();
        moving = true;
      }

      /* If the camera is moving or the animation is running, redraw immediately */
      if(moving || _animation) redraw();
    }

    void BulletExample::keyPressEvent(KeyEvent &event) {
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
        _animation ^= true;
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
      }
    }


    void BulletExample::drawTreeNodeBoundingBoxes() {
//      arrayResize(_boxInstanceData, 0);
//
//      /* Always draw the root node */
//      arrayAppend(_boxInstanceData, Containers::InPlaceInit,
//                  _arcballCamera->viewMatrix()*
//                  Matrix4::translation(_octree->center())*
//                  Matrix4::scaling(Vector3{_octree->halfWidth()}), 0x00ffff_rgbf);
//
//      /* Draw the remaining non-empty nodes */
//      if(_drawBoundingBoxes) {
//        const auto& activeTreeNodeBlocks = _octree->activeTreeNodeBlocks();
//        for(OctreeNodeBlock* const pNodeBlock : activeTreeNodeBlocks) {
//          for(std::size_t childIdx = 0; childIdx < 8; ++childIdx) {
//            const OctreeNode& pNode = pNodeBlock->_nodes[childIdx];
//
//            /* Non-empty node */
//            if(!pNode.isLeaf() || pNode.pointCount() > 0) {
//              const Matrix4 t = _arcballCamera->viewMatrix() *
//                                Matrix4::translation(pNode.center())*
//                                Matrix4::scaling(Vector3{pNode.halfWidth()});
//              arrayAppend(_boxInstanceData, Containers::InPlaceInit, t,
//                          0x197f99_rgbf);
//            }
//          }
//        }
//      }
//
//      _boxInstanceBuffer.setData(_boxInstanceData, GL::BufferUsage::DynamicDraw);
//      _boxMesh.setInstanceCount(_boxInstanceData.size());
//      _boxShader.setTransformationProjectionMatrix(_projectionMatrix)
//        .draw(_boxMesh);
    }

    void BulletExample::viewportEvent(ViewportEvent& event) {
      GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});
      _arcballCamera->reshape(event.windowSize(), event.framebufferSize());

      _projectionMatrix = Matrix4::perspectiveProjection(_arcballCamera->fov(),
                                                         Vector2{event.framebufferSize()}.aspectRatio(), 0.01f, 100.0f);
      _imgui.relayout(Vector2{event.windowSize()}/event.dpiScaling(),
                      event.windowSize(), event.framebufferSize());
    }

    void BulletExample::mousePressEvent(MouseEvent& event) {
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
          if(_profiler.isEnabled()) _profiler.enable();
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
        if(ImGui::Button("Import object")) {
          importRealFile("/Users/xc5/CLionProjects/opengl/magnum-examples/resources/objects/nanosuit/nanosuit.obj", _manipulator);
        }
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
        new ColoredDrawable{*robotank, _boxInstanceData, Color3::fromHsv(
                            {hue += 137.5_degf, 0.75f, 0.9f}),
                            Matrix4::scaling({1.0f, 0.5f, 0.5f}), _drawables};

        robotank->translate(cur_position);
        parent_position = cur_position;
        robotank->syncPose();
        _stackedRoboPos.push_back(cur_relative_position);
        _stackedRoboTank.push_back(robotank);
      }

      // initialize some walls;
      Vector3 _xshape = {5.0, 5.0, 0.5};
      createWall(_xshape, Vector3{2.5f, 5.0f, 5.0f});
      createWall(_xshape, Vector3{2.5f, 5.0f, 10.0f});
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
      if (_directionsPressed[0] || _directionsPressed[1]
       || _directionsPressed[2] || _directionsPressed[3] || _directionsPressed[4]) {
        RigidBody *rigidBody = (RigidBody*) _stackedRoboTank[0];
        rigidBody->rigidBody().setLinearVelocity(btVector3{xspeed, yspeed, zspeed});
      }
    }

    RigidBody *BulletExample::getRootRigidBody() {
      return (RigidBody*) _stackedRoboTank[0];
    }

    void BulletExample::addObject(Trade::AbstractImporter &importer,
                                  Containers::ArrayView<const
                                  Containers::Optional<Trade::PhongMaterialData>
                                  > materials,
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
      if(objectData->instanceType() == Trade::ObjectInstanceType3D::Mesh && objectData->instance() != -1 && _meshes[objectData->instance()]) {
        const Int materialId = static_cast<Trade::MeshObjectData3D*>(objectData.get())->material();

        /* Material not available / not loaded, use a default material */
        if(materialId == -1 || !materials[materialId]) {
           new NewColoredDrawable{*object, _coloredShader, *_meshes[objectData->instance()], 0xffffff_rgbf, _drawables};

          /* Textured material. If the texture failed to load, again just use a
             default colored material. */
        } else if(materials[materialId]->flags() & Trade::PhongMaterialData::Flag::DiffuseTexture) {
          Containers::Optional<GL::Texture2D>& texture = _textures[materials[materialId]->diffuseTexture()];
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
        addObject(importer, materials, *object, id);
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

      _texturedShader = Shaders::Phong{
        Shaders::Phong::Flag::DiffuseTexture};
      _texturedShader
        .setAmbientColor(0x111111_rgbf)
        .setSpecularColor(0x111111_rgbf)
        .setShininess(80.0f);

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
                                                           Vector2{framebufferSize()}.aspectRatio(), 0.01f, 100.0f);
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
      _importer = args.value("importer");
      const std::string filename = args.value("file");
      //importRealFile(filename, _manipulator);
    }

    /**
     * Import a file into the scene.
     */
    void BulletExample::importRealFile(const std::string filename, Object3D &parent) {
      /* Load a scene importer plugin */
      PluginManager::Manager<Trade::AbstractImporter> manager;
      Containers::Pointer<Trade::AbstractImporter> importer = manager.loadAndInstantiate(_importer);
      if(!importer) std::exit(1);

      Debug{} << "Opening file" << filename;

      /* Load file */
      if(!importer->openFile(filename))
        std::exit(4);

      /* Load all textures. Textures that fail to load will be NullOpt. */
      _textures = Containers::Array<Containers::Optional<GL::Texture2D>>{importer->textureCount()};
      for(UnsignedInt i = 0; i != importer->textureCount(); ++i) {
        Debug{} << "Importing texture" << i << importer->textureName(i);

        Containers::Optional<Trade::TextureData> textureData = importer->texture(i);
        if(!textureData || textureData->type() != Trade::TextureData::Type::Texture2D) {
          Warning{} << "Cannot load texture properties, skipping";
          continue;
        }

        Debug{} << "Importing image" << textureData->image() << importer->image2DName(textureData->image());

        Containers::Optional<Trade::ImageData2D> imageData = importer->image2D(textureData->image());
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
        GL::Texture2D texture;
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
      Containers::Array<Containers::Optional<Trade::PhongMaterialData>> materials{importer->materialCount()};
      for(UnsignedInt i = 0; i != importer->materialCount(); ++i) {
        Debug{} << "Importing material" << i << importer->materialName(i);

        Containers::Pointer<Trade::AbstractMaterialData> materialData = importer->material(i);
        if(!materialData || !(materialData->type() == Trade::MaterialType::Phong)) {
          Warning{} << "Cannot load material, skipping";
          continue;
        }

        materials[i] = std::move(static_cast<Trade::PhongMaterialData&>(*materialData));
      }

      /* Load all meshes. Meshes that fail to load will be NullOpt. */
      _meshes = Containers::Array<Containers::Optional<GL::Mesh>>{importer->meshCount()};
      for(UnsignedInt i = 0; i != importer->meshCount(); ++i) {
        Debug{} << "Importing mesh" << i << importer->meshName(i);

        Containers::Optional<Trade::MeshData> meshData = importer->mesh(i);
        if(!meshData || !meshData->hasAttribute(Trade::MeshAttribute::Normal) || meshData->primitive() != MeshPrimitive::Triangles) {
          Warning{} << "Cannot load the mesh, skipping";
          continue;
        }

        /* Compile the mesh */
        _meshes[i] = MeshTools::compile(*meshData);
      }

      /* Load the scene */
      if(importer->defaultScene() != -1) {
        Debug{} << "Adding default scene" << importer->sceneName(importer->defaultScene());

        Containers::Optional<Trade::SceneData> sceneData = importer->scene(importer->defaultScene());
        if(!sceneData) {
          Error{} << "Cannot load scene, exiting";
          return;
        }

        /* Recursively add all children */
        for(UnsignedInt objectId: sceneData->children3D())
          addObject(*importer, materials, parent, objectId);

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
      struct TriangleVertex {
        Vector2 position;
        Vector2 textureCoordinates;
      };
      const TriangleVertex data[]{
        {{-0.5f, -0.5f}, {0.0f, 0.0f}}, /* Left position and texture coordinate */
        {{ 0.5f, -0.5f}, {1.0f, 0.0f}}, /* Right position and texture coordinate */
        {{ 0.0f,  0.5f}, {0.5f, 1.0f}}  /* Top position and texture coordinate */
      };

      GL::Buffer buffer;
      buffer.setData(data);
      _mesh = GL::Mesh{};
      _mesh.setCount(3)
        .addVertexBuffer(std::move(buffer), 0,
                         TexturedTriangleShader::Position{},
                         TexturedTriangleShader::TextureCoordinates{});

      /* Load TGA importer plugin */
      PluginManager::Manager<Trade::AbstractImporter> manager;
      Containers::Pointer<Trade::AbstractImporter> importer =
                                                        manager.loadAndInstantiate("TgaImporter");
      if(!importer) std::exit(1);

      /* Load the texture */
      const Utility::Resource rs{"textured-triangle-data"};
      if(!importer->openData(rs.getRaw("stone.tga")))
        std::exit(2);

      /* Set texture data and parameters */
      Containers::Optional<Trade::ImageData2D> image = importer->image2D(0);
      CORRADE_INTERNAL_ASSERT(image);
      _texture = GL::Texture2D{};
      _texture.setWrapping(GL::SamplerWrapping::ClampToEdge)
        .setMagnificationFilter(GL::SamplerFilter::Linear)
        .setMinificationFilter(GL::SamplerFilter::Linear)
        .setStorage(1, GL::textureFormat(image->format()), image->size())
        .setSubImage(0, {}, *image);
    }
  }
}

MAGNUM_APPLICATION_MAIN(Magnum::LabFinal::BulletExample)
