/*
 * Copyright 2019 Google LLC. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "hello_cardboard_app.h"

#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <android/log.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

namespace ndk_hello_cardboard {

namespace {

// The objects are about 1 meter in radius, so the min/max target distance are
// set so that the objects are always within the room (which is about 5 meters
// across) and the reticle is always closer than any objects.
constexpr float kMinTargetDistance = 2.5f;
constexpr float kMaxTargetDistance = 3.5f;
constexpr float kMinTargetHeight = 0.5f;
constexpr float kMaxTargetHeight = kMinTargetHeight + 3.0f;

constexpr float kDefaultFloorHeight = -1.7f;

constexpr uint64_t kPredictionTimeWithoutVsyncNanos = 25000000;

// Angle threshold for determining whether the controller is pointing at the
// object.
constexpr float kAngleLimit = 0.2f;

// Number of different possible targets
constexpr int kTargetMeshCount = 3;

// Simple shaders to render .obj files without any lighting.
constexpr const char* kObjVertexShaders =
    R"glsl(
    uniform mat4 u_MVP;
    attribute vec4 a_Position;
    attribute vec2 a_UV;
    varying vec2 v_UV;

    void main() {
      v_UV = a_UV;
      gl_Position = u_MVP * a_Position;
    })glsl";

constexpr const char* kObjFragmentShaders =
    R"glsl(
    precision mediump float;
    varying vec2 v_UV;
    uniform sampler2D u_Texture;

    void main() {
      // The y coordinate of this sample's textures is reversed compared to
      // what OpenGL expects, so we invert the y coordinate.
      gl_FragColor = texture2D(u_Texture, vec2(v_UV.x, 1.0 - v_UV.y));
    })glsl";

}  // anonymous namespace

HelloCardboardApp::HelloCardboardApp(JavaVM* vm, jobject obj, jobject asset_mgr_obj)
    : head_tracker_(nullptr),
      lens_distortion_(nullptr),
      distortion_renderer_(nullptr),
      screen_params_changed_(false),
      device_params_changed_(false),
      screen_width_(0),
      screen_height_(0),
      depthRenderBuffer_(0),
      framebuffer_(0),
      texture_(0),
      obj_program_(0),
      obj_position_param_(0),
      obj_uv_param_(0),
      obj_modelview_projection_param_(0),
      target_object_meshes_(kTargetMeshCount),
      target_object_not_selected_textures_(kTargetMeshCount),
      target_object_selected_textures_(kTargetMeshCount),
      cur_target_object_(1) {
  JNIEnv* env;
  vm->GetEnv((void**)&env, JNI_VERSION_1_6);
  java_asset_mgr_ = env->NewGlobalRef(asset_mgr_obj);
  asset_mgr_ = AAssetManager_fromJava(env, asset_mgr_obj);

  Cardboard_initializeAndroid(vm, obj);
  head_tracker_ = CardboardHeadTracker_create();
  float* eulerAngle = GetEulerAngle(head_view_);
  for (int i = 0; i < 3; ++i) {
    iniAngle[i] = *(eulerAngle+i);
    abAngle[i] = iniAngle[i];
  }
  //lAngle = *(eulerAngle+1);
  firstIn = true;
  rotated_angle_ = 0.0f;
  head_view_ = GetPose();
  first_rotate_ = true;
  out_ = 0.0;
  cnt_ = 0;
  x_offset = 0.0;
  z_offset = 0.0;
}

HelloCardboardApp::~HelloCardboardApp() {
  CardboardHeadTracker_destroy(head_tracker_);
  CardboardLensDistortion_destroy(lens_distortion_);
  CardboardDistortionRenderer_destroy(distortion_renderer_);
}

void HelloCardboardApp::OnSurfaceCreated(JNIEnv* env) {
  const int obj_vertex_shader =
      LoadGLShader(GL_VERTEX_SHADER, kObjVertexShaders);
  const int obj_fragment_shader =
      LoadGLShader(GL_FRAGMENT_SHADER, kObjFragmentShaders);

  obj_program_ = glCreateProgram();
  glAttachShader(obj_program_, obj_vertex_shader);
  glAttachShader(obj_program_, obj_fragment_shader);
  glLinkProgram(obj_program_);
  glUseProgram(obj_program_);

  CHECKGLERROR("Obj program");

  obj_position_param_ = glGetAttribLocation(obj_program_, "a_Position");
  obj_uv_param_ = glGetAttribLocation(obj_program_, "a_UV");
  obj_modelview_projection_param_ = glGetUniformLocation(obj_program_, "u_MVP");

  CHECKGLERROR("Obj program params");

  HELLOCARDBOARD_CHECK(room_.Initialize(env, asset_mgr_, "CubeRoom.obj",
                                 obj_position_param_, obj_uv_param_));
  HELLOCARDBOARD_CHECK(
      room_tex_.Initialize(env, java_asset_mgr_, "CubeRoom_BakedDiffuse.png"));
  HELLOCARDBOARD_CHECK(target_object_meshes_[0].Initialize(
      env, asset_mgr_, "Icosahedron.obj", obj_position_param_, obj_uv_param_));
  HELLOCARDBOARD_CHECK(target_object_not_selected_textures_[0].Initialize(
      env, java_asset_mgr_, "Icosahedron_Blue_BakedDiffuse.png"));
  HELLOCARDBOARD_CHECK(target_object_selected_textures_[0].Initialize(
      env, java_asset_mgr_, "Icosahedron_Pink_BakedDiffuse.png"));
  HELLOCARDBOARD_CHECK(target_object_meshes_[1].Initialize(
      env, asset_mgr_, "QuadSphere.obj", obj_position_param_, obj_uv_param_));
  HELLOCARDBOARD_CHECK(target_object_not_selected_textures_[1].Initialize(
      env, java_asset_mgr_, "QuadSphere_Blue_BakedDiffuse.png"));
  HELLOCARDBOARD_CHECK(target_object_selected_textures_[1].Initialize(
      env, java_asset_mgr_, "QuadSphere_Pink_BakedDiffuse.png"));
  HELLOCARDBOARD_CHECK(target_object_meshes_[2].Initialize(
      env, asset_mgr_, "TriSphere.obj", obj_position_param_, obj_uv_param_));
  HELLOCARDBOARD_CHECK(target_object_not_selected_textures_[2].Initialize(
      env, java_asset_mgr_, "TriSphere_Blue_BakedDiffuse.png"));
  HELLOCARDBOARD_CHECK(target_object_selected_textures_[2].Initialize(
      env, java_asset_mgr_, "TriSphere_Pink_BakedDiffuse.png"));

  // Target object first appears directly in front of user.

//  float angle = -PI/2 + (speed[speed_idx_] / 180.0f * PI * MULTIPLER + OFFSET) * dir;
//  float distance = (kMinTargetDistance + kMaxTargetDistance) / 2;
//  float height = 1.5f;
//  std::array<float, 3> target_position = {std::cos(angle) * distance, height,
//                                          std::sin(angle) * distance};
//  model_target_ = GetTranslationMatrix(target_position);

  float angle = -M_PI/2 + 0.4*M_PI;
  float distance = kMinTargetDistance;
  float height = 1.5f;
  std::array<float, 3> target_position = {std::cos(angle) * distance, height,
                                          std::sin(angle) * distance};
  model_target_ = GetTranslationMatrix(target_position);

  angle = -M_PI/2 - 0.4*M_PI;
  target_position = {std::cos(angle) * distance, height,
                     std::sin(angle) * distance};
  model_target1_ = GetTranslationMatrix(target_position);

  float a = - 1.5 * M_PI + 0.1 * M_PI;
  float interval = 1.8 * M_PI / NUM_TARGETS;
//  float interval = 0;
  model_target_vec_.reserve(NUM_TARGETS);
  for(int i = 0; i < NUM_TARGETS; i++) {
    float real_a = a + interval / 2 + RandomUniformFloat(-interval/2, interval/2);
//    float real_a = a;
    height = RandomUniformFloat(kMinTargetHeight, kMaxTargetHeight);
    distance = RandomUniformFloat(kMinTargetDistance, kMaxTargetDistance);
    target_position = {std::cos(real_a) * distance, height,
                       std::sin(real_a) * distance};
    Matrix4x4 trans = GetTranslationMatrix(target_position);
    model_target_vec_[i] = trans;
    a += interval;
  }

  model_obstacle_vec_.reserve(OBSTACLE_CNT);
  for(int i = 0; i < OBSTACLE_CNT; i++) {
    a = RandomUniformFloat(-M_PI, M_PI);
//    float real_a = a;
    height = RandomUniformFloat(kMinTargetHeight, kMaxTargetHeight);
    distance = RandomUniformFloat(3.0f, kMaxTargetDistance);
    target_position = {std::cos(a) * distance, height,
                       std::sin(a) * distance};
    Matrix4x4 trans = GetTranslationMatrix(target_position);
    model_obstacle_vec_[i] = trans;
  }

  CHECKGLERROR("OnSurfaceCreated");
}

void HelloCardboardApp::SetScreenParams(int width, int height) {
  screen_width_ = width;
  screen_height_ = height;
  screen_params_changed_ = true;
}

float HelloCardboardApp::GetAmp(float speed) {
    speed = std::abs(speed);
    if(0 <= speed && speed < 22) {
        return 2.95;
    } else if(speed < 48.3) {
        return 2.55 + (48.3 - speed) / (48.3 - 22) * (2.95 - 2.55);
    } else if(speed < 74.6) {
        return 2.22 + (74.6 - speed) / (74.6 - 48.3) * (2.55 - 2.22);
    } else {
        return 2.22;
    }
}


//normal VR
void HelloCardboardApp::realization0(float mainAngle) {
    rotateM(rotated_head_view_, 0, head_view_, 0, 1, 0);
}


//non-linear amp
void HelloCardboardApp::realizationA(float mainAngle) {
  float alpha = (float)abs(cos(-mainAngle*PI/180));
  mainAngle = (2.0f - alpha) * (-mainAngle);
  rotateM(rotated_head_view_, mainAngle, head_view_, 0, 1, 0);
}


//auto rotation
void HelloCardboardApp::realizationB(float mainAngle) {
//  if (mainAngle > aAngle) {
//    flag = (aStep+flag) > 1 ? 1 : (aStep+flag);
//    theta -= flag * aSpeed;
//    //theta = std::max(-180.0f, theta);
//  }
//  else if (mainAngle < -aAngle) {
//    flag = (flag-aStep) < -1 ? -1 : (flag-aStep);
//    theta -= flag * aSpeed;
//    //theta = std::min(180.0f, theta);
//  }
//  else if (flag > 0) {
//    if (mainAngle > bAngle) {
//      theta -= flag * bSpeed;
//      //theta = std::max(-180.0f, theta);
//    }
//    else {
//      flag = (flag-bStep) > 0 ? (flag-bStep) : 0;
//      theta -= flag * bSpeed;
//    }
//  }
//  else if (flag < 0) {
//    if (mainAngle < -bAngle) {
//      theta -= flag * bSpeed;
//      //theta = std::min(180.0f, theta);
//    }
//    else {
//      flag = (flag+bStep) < 0 ? (flag+bStep) : 0;
//      theta -= flag * bSpeed;
//    }
//  }

//  float delta = 0;
//  if(mainAngle > 10.0) {
//    delta = 1.0f;
//  } else if(mainAngle < -10.0) {
//    delta = -1.0f;
//  } else {
//    delta = 0;
//  }
  if(mainAngle < -aAngle) {
    theta -= speed[speed_idx_]/60.0f;
  } else if(mainAngle > aAngle) {
    theta += speed[speed_idx_]/60.0f;
  }
  if(theta > 200 || theta < -200) {
    rotateM(rotated_head_view_, 0, head_view_, 0, 1, 0);
  }
  else rotateM(rotated_head_view_, -theta, head_view_, 0, 1, 0);
}


//save for test, also can be used as linear amp
void HelloCardboardApp::realizationC(float mainAngle) {
  //theta -= amp * std::max((float)(std::abs(mainAngle - lAngle) * 180 / PI / lowSpeed), 1.0f) * (mainAngle - lAngleUse);
  if (firstIn) offset = theta;
  theta = offset - (amp-1) * mainAngle; firstIn = false;
  rotateM(rotated_head_view_, theta, head_view_, 0, 1, 0);
  //rotateM(rotated_head_view_, angle[0], rotated_head_view_, 1, 0, 0);

}

float HelloCardboardApp::realizationD() {
    tamp = GetAmp(bqueue_.speed_());

    if(cnt_ < 5) {
        cnt_++;
        bqueue_.insertHead(angle[1]);
        bqueue_.insertView(angle[1]);
        rotateM(rotated_head_view_, 0, head_view_, 0, 1, 0);
        return 1;
    }
    //state changed, record this last key angle，以此作为上一个关键角度来进行旋转
    // 否则利用rotateM会转不动
//    if (tamp != amp4) {
//    for (int i = 0; i < 3; ++i) lastKeyAngles[i] = *(tmp+i);
    startTurningBack = false;
//    }

    bqueue_.insertHead(angle[1]);
    float last_view = bqueue_.view_angle_.back();
    if(angle[1] < 0 && bqueue_.view_angle_.back() > 100) {
      last_view = -360 + last_view;
    } else if(angle[1] > 0 && bqueue_.view_angle_.back() < -100) {
      last_view = 360 + last_view;
    }
    //turning back，这里有bug，屏幕会黑，首先检查viewAngle是否正确
    if (isTurningBack) {
        tamp = last_view / angle[1];
        float mainAngle = -(angle[1] - bqueue_.last_()) * tamp;
        rotated_angle_ = angle[1] - last_view - (angle[1] - bqueue_.last_()) * tamp;
        rotateM(rotated_head_view_, rotated_angle_, head_view_, 0, 1, 0);
        bqueue_.insertView(*(GetEulerAngle(rotated_head_view_)+1)*180/PI);
    }
    // 继续向更大角度转头，则使用tamp作为增益
    else {
        float mainAngle = -(angle[1] - bqueue_.last_()) * tamp;
        rotated_angle_ = -(last_view - angle[1]) + mainAngle;

//        if(std::abs(angle[1]) + std::abs(rotated_angle_) >= 178.0) {
//            if(angle[1] < 0) {
//                rotateM(rotated_head_view_, (178.0 + angle[1]), head_view_, 0, 1, 0);
//            } else {
//                rotateM(rotated_head_view_, -(178.0 - angle[1]), head_view_, 0, 1, 0);
//            }
//        } else {
            rotateM(rotated_head_view_, rotated_angle_, head_view_, 0, 1, 0);
//        }
        bqueue_.insertView(*(GetEulerAngle(rotated_head_view_)+1)*180/PI);

    }
    return tamp;
}

std::vector<float> HelloCardboardApp::ReturnVector() {
    std::vector<float> res;
    res.push_back(amp);
    res.push_back(angleDiff);
    res.push_back(maxAngle);
    res.push_back(angle[1]);
    res.push_back(direction);
    res.push_back(viewAngle);
    res.push_back(tamp);
    if(isTurningBack) res.push_back(1);
    else res.push_back(0);
    res.push_back(rotated_angle_);

    res.push_back(deleted_ball_);
    if(deleted_ball_ != -1) {
      deleted_ball_ = -1;
    }
    res.push_back(asin((double)head_view_.m[1][2]));
    res.push_back(asin((double)rotated_head_view_.m[1][2]));
//    for(int i = 0; i < bqueue_.q2.size(); i++) {
//        res.push_back(bqueue_.q2[i]);
//    }
//    for (int i = 0; i < 4; ++i) {
//        for (int j = 0; j < 4; ++j) {
//            res.push_back(rotated_head_view_.m[i][j]);
//        }
//    }
//    for (int i = 0; i < 4; ++i) {
//        for (int j = 0; j < 4; ++j) {
//            res.push_back(head_view_.m[i][j]);
//        }
//    }
    return res;
}

bool HelloCardboardApp::judgeIfTurningBack() {
    if(direction == 0) isTurningBack = false;
    else if (direction > 0) {
        if (angle[1] < 0) isTurningBack = false;
        else isTurningBack = true;
    }
    else if (direction < 0) {
        if (angle[1] > 0) isTurningBack = false;
        else isTurningBack = true;
    }
    return isTurningBack;
}

void HelloCardboardApp::OnDrawFrame(float _amp) {
  if (!UpdateDeviceParams()) {
    return;
  }

  bqueue_.insert(angle[1]);
  lAngle = angle[1];
  // Update Head Pose.
  head_view_ = GetPose();
  // We have to get angles first
  float* eulerAngle = GetEulerAngle(head_view_);
  // if iniAngle is not initialized, press button to initialize it
  if (amp != _amp && PLAN == 3) {
    firstIn = true;
    for (int i = 0; i < 3; ++i) {
      abAngle[i] = *(eulerAngle + i);
    }
  }
  for (int i = 0; i < 3; ++i) {
    angle[i] = *(eulerAngle+i);

    if (PLAN == 1 || PLAN == 2 || PLAN == 4) angle[i] = (angle[i]-0) * 180 / PI;
    else if (PLAN == 3 || PLAN == 0) angle[i] = (angle[i]-abAngle[i]) * 180 / PI;
  }
  amp = _amp;

  // Incorporate the floor height into the head_view
  head_view_ =
      head_view_ * GetTranslationMatrix({0.0f, kDefaultFloorHeight, 0.0f});

  float mainAngle = angle[1];
  // direction > 0 means speed to right; else speed to left
//  direction = lAngle - angle[1] > 0 ? 1 : -1;
  direction = bqueue_.direction_();

  angleDiff = std::abs(lAngle-angle[1]) * 60; if (isnan(angleDiff)) angleDiff = 0;
  bool tmpbool = isTurningBack;
  if (tmpbool != judgeIfTurningBack()) {startTurningBack = true; tamp = viewAngle / angle[1]; }







  maxAngle = std::max(abs(maxAngle), abs(mainAngle));
  //Plan A: amplified head rotation, baseline
  // we need to setup parameters including the amplified function
  //
  //Plan B: use head rotation angle to control whether to rotate the scene
  // we need to setup parameters including alpha, beta, aSpeed, bSpeed
  // maybe we can change aSpeed and/or bSpeed into a function()
  // write rotate_head_view_ by using head_view_
  if (PLAN == 0) {realization0(abAngle[1]); }
  else if (PLAN == 1) {realizationA(mainAngle); }
  else if (PLAN == 2) {realizationB(mainAngle); }
  else if (PLAN == 3) {
    //if (abAngle[1] * lAngleUse < 0) lAngleUse = abAngle[1]; // we must do this, or there will be violent shift in scene
    realizationC(angle[1]); // we only need the real head angles (and last angle) to do this
  }
  else if (PLAN == 4) {
      amp4 = realizationD();
  }
  viewAngle = *(GetEulerAngle(rotated_head_view_) + 1) * 180 / PI;
  // Bind buffer
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glDisable(GL_SCISSOR_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Draw eyes views
  for (int eye = 0; eye < 2; ++eye) {
    glViewport(eye == kLeft ? 0 : screen_width_ / 2, 0, screen_width_ / 2,
               screen_height_);

    Matrix4x4 eye_matrix = GetMatrixFromGlArray(eye_matrices_[eye]);
    //change head_view_ into tmpM
    Matrix4x4 eye_view = eye_matrix * rotated_head_view_;

    Matrix4x4 projection_matrix =
        GetMatrixFromGlArray(projection_matrices_[eye]);
    Matrix4x4 modelview_target = eye_view * model_target_;
    modelview_projection_target_ = projection_matrix * modelview_target;
    modelview_projection_room_ = projection_matrix * eye_view;

    mvp_target_vec_.reserve(NUM_TARGETS);
    for(int i = 0; i < NUM_TARGETS; i++) {
      Matrix4x4 model_view_target = eye_view * model_target_vec_[i];
      mvp_target_vec_[i] = projection_matrix * model_view_target;
    }

    mvp_obstacle_vec_.reserve(OBSTACLE_CNT);
    for(int i = 0; i < OBSTACLE_CNT; i++) {
      Matrix4x4 model_view_obstacle = eye_view * model_obstacle_vec_[i];
      mvp_obstacle_vec_[i] = projection_matrix * model_view_obstacle;
    }

    // Draw room and target
    DrawWorld();
  }

  // Render
  CardboardDistortionRenderer_renderEyeToDisplay(
      distortion_renderer_, /* target_display = */ 0, /* x = */ 0, /* y = */ 0,
      screen_width_, screen_height_, &left_eye_texture_description_,
      &right_eye_texture_description_);

  CHECKGLERROR("onDrawFrame");
  //return maxAngle;
  //return theta;

  return;
  //return amp;
}


// add new feature in this function
void HelloCardboardApp::OnTriggerEvent() {
  for(int i = 0; i < NUM_TARGETS; i++) {
    if (IsPointingAtTarget(i)) {
      HideTarget();
    }
  }
//  else {
//    float* tmp = GetEulerAngle();
//    for (int i = 0; i < 3; ++i) {
//        iniAngle[i] = *(tmp + i);
//
//    }
//    maxAngle = 0.0f;
//  }
}

void HelloCardboardApp::OnPause() { CardboardHeadTracker_pause(head_tracker_); }

void HelloCardboardApp::OnResume() {
  CardboardHeadTracker_resume(head_tracker_);

  // Parameters may have changed.
  device_params_changed_ = true;

  // Check for device parameters existence in external storage. If they're
  // missing, we must scan a Cardboard QR code and save the obtained parameters.
  uint8_t* buffer;
  int size;
  CardboardQrCode_getSavedDeviceParams(&buffer, &size);
  if (size == 0) {
    SwitchViewer();
  }
  CardboardQrCode_destroy(buffer);
}

void HelloCardboardApp::SwitchViewer() {
  CardboardQrCode_scanQrCodeAndSaveDeviceParams();
}

void HelloCardboardApp::SetParameter(float aSp, float bSp, float aAn, float bAn) {
  aSpeed = aSp; bSpeed = bSp; aAngle = aAn; bAngle = bAn;
}

void HelloCardboardApp::SwitchPlan(int flg) {
  PLAN = flg;
}

void HelloCardboardApp::SwitchSpeed(int idx) {
  speed_idx_ = idx;
  float angle = -PI/2 + (speed[idx] / 180.0f * PI * MULTIPLER + OFFSET) * dir;
  float distance = (kMinTargetDistance + kMaxTargetDistance) / 2;
  float height = 1.5f;
  std::array<float, 3> target_position = {std::cos(angle) * distance, height,
                                          std::sin(angle) * distance};
  model_target_ = GetTranslationMatrix(target_position);
}

void HelloCardboardApp::SwitchDir(int d) {
  dir = d;
  float angle = -PI/2 + (speed[speed_idx_] / 180.0f * PI * MULTIPLER + OFFSET) * dir;
  float distance = (kMinTargetDistance + kMaxTargetDistance) / 2;
  float height = 1.5f;
  std::array<float, 3> target_position = {std::cos(angle) * distance, height,
                                          std::sin(angle) * distance};
  model_target_ = GetTranslationMatrix(target_position);
}

bool HelloCardboardApp::UpdateDeviceParams() {
  // Checks if screen or device parameters changed
  if (!screen_params_changed_ && !device_params_changed_) {
    return true;
  }

  // Get saved device parameters
  uint8_t* buffer;
  int size;
  CardboardQrCode_getSavedDeviceParams(&buffer, &size);

  // If there are no parameters saved yet, returns false.
  if (size == 0) {
    return false;
  }

  CardboardLensDistortion_destroy(lens_distortion_);
  lens_distortion_ = CardboardLensDistortion_create(
      buffer, size, screen_width_, screen_height_);

  CardboardQrCode_destroy(buffer);

  GlSetup();

  CardboardDistortionRenderer_destroy(distortion_renderer_);
  distortion_renderer_ = CardboardDistortionRenderer_create();

  CardboardMesh left_mesh;
  CardboardMesh right_mesh;
  CardboardLensDistortion_getDistortionMesh(lens_distortion_, kLeft,
                                            &left_mesh);
  CardboardLensDistortion_getDistortionMesh(lens_distortion_, kRight,
                                            &right_mesh);

  CardboardDistortionRenderer_setMesh(distortion_renderer_, &left_mesh, kLeft);
  CardboardDistortionRenderer_setMesh(distortion_renderer_, &right_mesh,
                                      kRight);

  // Get eye matrices
  CardboardLensDistortion_getEyeFromHeadMatrix(
      lens_distortion_, kLeft, eye_matrices_[0]);
  CardboardLensDistortion_getEyeFromHeadMatrix(
      lens_distortion_, kRight, eye_matrices_[1]);
  CardboardLensDistortion_getProjectionMatrix(
      lens_distortion_, kLeft, kZNear, kZFar, projection_matrices_[0]);
  CardboardLensDistortion_getProjectionMatrix(
      lens_distortion_, kRight, kZNear, kZFar, projection_matrices_[1]);

  screen_params_changed_ = false;
  device_params_changed_ = false;

  CHECKGLERROR("UpdateDeviceParams");

  return true;
}

void HelloCardboardApp::GlSetup() {
  LOGD("GL SETUP");

  if (framebuffer_ != 0) {
    GlTeardown();
  }

  // Create render texture.
  glGenTextures(1, &texture_);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, screen_width_, screen_height_, 0,
               GL_RGB, GL_UNSIGNED_BYTE, 0);

  left_eye_texture_description_.texture = texture_;
  left_eye_texture_description_.left_u = 0;
  left_eye_texture_description_.right_u = 0.5;
  left_eye_texture_description_.top_v = 1;
  left_eye_texture_description_.bottom_v = 0;

  right_eye_texture_description_.texture = texture_;
  right_eye_texture_description_.left_u = 0.5;
  right_eye_texture_description_.right_u = 1;
  right_eye_texture_description_.top_v = 1;
  right_eye_texture_description_.bottom_v = 0;

  // Generate depth buffer to perform depth test.
  glGenRenderbuffers(1, &depthRenderBuffer_);
  glBindRenderbuffer(GL_RENDERBUFFER, depthRenderBuffer_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, screen_width_,
                        screen_height_);
  CHECKGLERROR("Create Render buffer");

  // Create render target.
  glGenFramebuffers(1, &framebuffer_);
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         texture_, 0);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER, depthRenderBuffer_);

  CHECKGLERROR("GlSetup");
}

void HelloCardboardApp::GlTeardown() {
  if (framebuffer_ == 0) {
    return;
  }
  glDeleteRenderbuffers(1, &depthRenderBuffer_);
  depthRenderBuffer_ = 0;
  glDeleteFramebuffers(1, &framebuffer_);
  framebuffer_ = 0;
  glDeleteTextures(1, &texture_);
  texture_ = 0;

  CHECKGLERROR("GlTeardown");
}

Matrix4x4 HelloCardboardApp::GetPose() {
  std::array<float, 4> out_orientation;
  std::array<float, 3> out_position;
  long monotonic_time_nano = GetMonotonicTimeNano();
  monotonic_time_nano += kPredictionTimeWithoutVsyncNanos;
  CardboardHeadTracker_getPose(head_tracker_, monotonic_time_nano,
                               &out_position[0], &out_orientation[0]);
  return GetTranslationMatrix(out_position) *
         Quatf::FromXYZW(&out_orientation[0]).ToMatrix();
}

float *HelloCardboardApp::GetEulerAngle(Matrix4x4 mat) {
  static float var[3];
  var[0] = -(float)asin((double)mat.m[1][2]);
  if (sqrt((double)(1.0F - mat.m[1][2] * mat.m[1][2])) >= 0.009999999776482582) {
    var[1] = -(float)atan2((double)(-mat.m[0][2]), (double)mat.m[2][2]);
    var[2] = -(float)atan2((double)(-mat.m[1][0]), (double)mat.m[1][1]);
  }
  else {
    var[1] = 0.0F;
    var[2] = -atan2(mat.m[0][1], mat.m[0][0]);

  }
  return var;
}

double *HelloCardboardApp::GetEulerAngleDouble(Matrix4x4 mat) {
    double var[3];
    var[0] = -asin((double)mat.m[1][2]);
    if (sqrt((double)(1.0F - mat.m[1][2] * mat.m[1][2])) >= 0.009999999776482582) {
        var[1] = -atan2((double)(-mat.m[0][2]), (double)mat.m[2][2]);
        var[2] = -atan2((double)(-mat.m[1][0]), (double)mat.m[1][1]);
    }
    else {
        var[1] = 0.0F;
        var[2] = -atan2(double(mat.m[0][1]), double(mat.m[0][0]));

    }
    return var;
}

void HelloCardboardApp::rotateM(ndk_hello_cardboard::Matrix4x4 &ans, float angle,
        ndk_hello_cardboard::Matrix4x4 src, float x, float y, float z) {
    Matrix4x4 sTemp;
    sTemp.m[0][3] = 0; sTemp.m[1][3] = 0; sTemp.m[2][3] = 0;
    sTemp.m[3][0] = 0; sTemp.m[3][1] = 0; sTemp.m[3][2] = 0; sTemp.m[3][3] = 1;
    angle = angle * PI / 180.0f;
    float s = (float)sin(angle), c = (float)cos(angle);
    if (x == 1.0f && y == 0.0f && z == 0.0f) {
        sTemp.m[0][0] = 1; sTemp.m[0][1] = 0; sTemp.m[0][2] = 0;
        sTemp.m[1][0] = 0; sTemp.m[1][1] = c; sTemp.m[1][2] = s;
        sTemp.m[2][0] = 0; sTemp.m[2][1] = -s; sTemp.m[2][2] = c;
    }
    else if (x == 0.0f && y == 1.0f && z == 0.0f) {
        sTemp.m[0][0] = c; sTemp.m[0][1] = 0; sTemp.m[0][2] = -s;
        sTemp.m[1][0] = 0; sTemp.m[1][1] = 1; sTemp.m[1][2] = 0;
        sTemp.m[2][0] = s; sTemp.m[2][1] = 0; sTemp.m[2][2] = c;
    }
    else if (x == 0.0f && y == 0.0f && z == 0.0f) {
        sTemp.m[0][0] = c; sTemp.m[0][1] = s; sTemp.m[0][2] = 0;
        sTemp.m[1][0] = -s; sTemp.m[1][1] = c; sTemp.m[1][2] = 0;
        sTemp.m[2][0] = 0; sTemp.m[2][1] = 0; sTemp.m[2][2] = 1;
    }
    else {
        float len = x*x + y*y + z*z;
        if (1.0f != len) {
            float recipLen = 1.0f / len;
            x *= recipLen; y *= recipLen; z *= recipLen;
        }
        float nc = 1.0f - c;
        float xy = x * y;
        float yz = y * z;
        float zx = z * x;
        float xs = x * s;
        float ys = y * s;
        float zs = z * s;
        sTemp.m[0][0] = x*x*nc + c; sTemp.m[0][1] = xy*nc + zs; sTemp.m[0][2] = zx*nc - ys;
        sTemp.m[1][0] = xy*nc - zs; sTemp.m[1][1] = y*y*nc + c; sTemp.m[1][2] = yz*nc + xs;
        sTemp.m[2][0] = zx*nc + ys; sTemp.m[2][1] = yz*nc - xs; sTemp.m[2][2] = z*z*nc + c;
    }

    ans = src * sTemp;
}

void HelloCardboardApp::DrawWorld() {
  DrawRoom();
  DrawTarget();
}

void HelloCardboardApp::DrawTarget() {
  glUseProgram(obj_program_);

//  std::array<float, 16> target_array = modelview_projection_target_.ToGlArray();
//  glUniformMatrix4fv(obj_modelview_projection_param_, 1, GL_FALSE,
//                     target_array.data());

//  if (IsPointingAtTarget()) {
//    target_object_selected_textures_[cur_target_object_].Bind();
//  } else {
//    target_object_not_selected_textures_[cur_target_object_].Bind();
//  }
//  target_object_meshes_[cur_target_object_].Draw();

  for(int i = 0; i < OBSTACLE_CNT; i++) {
    std::array<float, 16> target_array = mvp_obstacle_vec_[i].ToGlArray();
    glUniformMatrix4fv(obj_modelview_projection_param_, 1, GL_FALSE,
                       target_array.data());
    int type = 0;
    if(i < OBSTACLE_CNT / 2) type = 2;
    target_object_not_selected_textures_[type].Bind();
    target_object_meshes_[type].Draw();
  }

  int cur_selected = -1;
  for(int i = 0; i < NUM_TARGETS; i++) {
    if(is_present_[i] == false) continue;
    std::array<float, 16> target_array = mvp_target_vec_[i].ToGlArray();
    glUniformMatrix4fv(obj_modelview_projection_param_, 1, GL_FALSE,
                       target_array.data());
    if(IsPointingAtTarget(i)) {
      cur_selected = i;
      target_object_selected_textures_[cur_target_object_].Bind();
    } else {
      target_object_not_selected_textures_[cur_target_object_].Bind();
    }
    target_object_meshes_[cur_target_object_].Draw();
  }

  if(cur_selected >= 0) {
    if(cur_selected == last_ball_selected_) {
      selected_frames_++;
      if(selected_frames_ > ACTIVATED_FRAMES) {
        //delete the ball
        is_present_[cur_selected] = false;
        deleted_ball_ = cur_selected;
        selected_frames_ = 0;
        last_ball_selected_ = -1;
      }
    } else {
      last_ball_selected_ = cur_selected;
      selected_frames_ = 1;
    }
  }

  CHECKGLERROR("DrawTarget");
}

void HelloCardboardApp::DrawRoom() {
  glUseProgram(obj_program_);

  std::array<float, 16> room_array = modelview_projection_room_.ToGlArray();
  glUniformMatrix4fv(obj_modelview_projection_param_, 1, GL_FALSE,
                     room_array.data());

  room_tex_.Bind();
  room_.Draw();

  CHECKGLERROR("DrawRoom");
}

void HelloCardboardApp::HideTarget() {
  cur_target_object_ = RandomUniformInt(kTargetMeshCount);

  float angle = RandomUniformFloat(-M_PI, M_PI);
  float distance = RandomUniformFloat(kMinTargetDistance, kMaxTargetDistance);
  float height = RandomUniformFloat(kMinTargetHeight, kMaxTargetHeight);
  std::array<float, 3> target_position = {std::cos(angle) * distance, height,
                                          std::sin(angle) * distance};

  model_target_ = GetTranslationMatrix(target_position);
}

bool HelloCardboardApp::IsPointingAtTarget(int i) {
  // Compute vectors pointing towards the reticle and towards the target object
  // in head space.
  if(i < 0 || i >= NUM_TARGETS) return false;
  Matrix4x4 head_from_target = rotated_head_view_ * model_target_vec_[i];

  const std::array<float, 4> unit_quaternion = {0.f, 0.f, 0.f, 1.f};
  const std::array<float, 4> point_vector = {0.f, 0.f, -1.f, 0.f};
  const std::array<float, 4> target_vector = head_from_target * unit_quaternion;

  float angle = AngleBetweenVectors(point_vector, target_vector);
  return angle < kAngleLimit;
}



}  // namespace ndk_hello_cardboard
