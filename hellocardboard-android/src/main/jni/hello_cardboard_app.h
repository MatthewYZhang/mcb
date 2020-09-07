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

#ifndef HELLO_CARDBOARD_ANDROID_SRC_MAIN_JNI_HELLO_CARDBOARD_APP_H_
#define HELLO_CARDBOARD_ANDROID_SRC_MAIN_JNI_HELLO_CARDBOARD_APP_H_

#include <android/asset_manager.h>
#include <jni.h>

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <GLES2/gl2.h>
#include "cardboard.h"
#include "util.h"

#define PI 3.14159265758

namespace ndk_hello_cardboard {

/**
 * This is a sample app for the Cardboard SDK. It loads a simple environment and
 * objects that you can click on.
 */


class HelloCardboardApp {
 public:
  /**
   * Creates a HelloCardboardApp.
   *
   * @param vm JavaVM pointer.
   * @param obj Android activity object.
   * @param asset_mgr_obj The asset manager object.
   */
  HelloCardboardApp(JavaVM* vm, jobject obj, jobject asset_mgr_obj);

  ~HelloCardboardApp();

  /**
   * Initializes any GL-related objects. This should be called on the rendering
   * thread with a valid GL context.
   *
   * @param env The JNI environment.
   */
  void OnSurfaceCreated(JNIEnv* env);

  /**
   * Sets screen parameters.
   *
   * @param width Screen width
   * @param height Screen height
   */
  void SetScreenParams(int width, int height);

  /**
   * Draws the scene. This should be called on the rendering thread.
   */
  void OnDrawFrame(float amp);

  /**
   * Hides the target object if it's being targeted.
   */
  void OnTriggerEvent();

  /**
   * Pauses head tracking.
   */
  void OnPause();

  /**
   * Resumes head tracking.
   */
  void OnResume();

  /**
   * Allows user to switch viewer.
   */
  void SwitchViewer();

  /**
   * Allows user to set parameters including aSpeed, bSpeed, aAngle, bAngle
   */
  void SetParameter(float aSp, float bSp, float cSp, float dSp);

  /**
   * Allows user to set parameters including aSpeed, bSpeed, aAngle, bAngle
   */
  void SwitchPlan(int flg);

  void SwitchSpeed(int idx);

  void SwitchDir(int d);

  /**
   * Method for testing return a vector to jni
   * @return
   */
  std::vector<float> ReturnVector();

 private:
  /**
   * Default near clip plane z-axis coordinate.
   */
  static constexpr float kZNear = 0.1f;

  /**
   * Default far clip plane z-axis coordinate.
   */
  static constexpr float kZFar = 100.f;

  /**
   * Updates device parameters, if necessary.
   *
   * @return true if device parameters were successfully updated.
   */
  bool UpdateDeviceParams();

  /**
   * Initializes GL environment.
   */
  void GlSetup();

  /**
   * Deletes GL environment.
   */
  void GlTeardown();

  /**
   * Gets head's pose as a 4x4 matrix.
   *
   * @return matrix containing head's pose.
   */
  Matrix4x4 GetPose();

  /**
   * Gets head's euler angles as a float[3]
   *
   * @return a float*
   */
  float* GetEulerAngle();

  /**
    * rotate src matrix around axis(x,y,z) by angle(in degrees)
    *
    * @return void, with answer wrote in ans
   */
  void rotateM(Matrix4x4& ans, float angle, Matrix4x4 src, float x, float y, float z);


    /**
     * realization NO.0
     * using head rotation velocity to give amplification
     */
    void realization0(float mainAngle);

  /**
   * realization NO.1
   * amplifies head rotation degrees using a non-linear function
   */
  void realizationA(float mainAngle);

  /**
   * realization NO.2
   * modifies rotated_head_view_ by rotate head_view_ by certain degrees using rotateM
   */
  void realizationB(float mainAngle);

  /**
   * realization NO.3
   * amplifies head rotation degrees using a non-linear function
   */
    void realizationC(float mainAngle);
    /**
     *
     *
     *
     */
    float realizationD();
    /**
     *
     */
    void judgeIfTurningBack();

  /**
   * Draws all world-space objects for the given eye.
   */
  void DrawWorld();

  /**
   * Draws the target object.
   */
  void DrawTarget();

  /**
   * Draws the room.
   */
  void DrawRoom();

  /**
   * Finds a new random position for the target object.
   */
  void HideTarget();

  /**
   * Checks if user is pointing or looking at the target object by calculating
   * whether the angle between the user's gaze and the vector pointing towards
   * the object is lower than some threshold.
   *
   * @return true if the user is pointing at the target object.
   */
  bool IsPointingAtTarget();

  jobject java_asset_mgr_;
  AAssetManager* asset_mgr_;

  CardboardHeadTracker* head_tracker_;
  CardboardLensDistortion* lens_distortion_;
  CardboardDistortionRenderer* distortion_renderer_;

  CardboardEyeTextureDescription left_eye_texture_description_;
  CardboardEyeTextureDescription right_eye_texture_description_;

  bool screen_params_changed_;
  bool device_params_changed_;
  int screen_width_;
  int screen_height_;

  float projection_matrices_[2][16];
  float eye_matrices_[2][16];
  //iniAngle array is used in Plan 1 and Plan 2
  float iniAngle[3];
  //angle array is used in Plan 1 and Plan 2
  float angle[3];
  //abAngle array is used in Plan 3, records the real head_view
  float abAngle[3];

  float rAngle[3];
  int PLAN = 3;
  // the following members are used by Plan 2
  float flag = 0.0f;
  float theta = 0.0f;
  float maxAngle = 0.0f;
  float aStep = 0.04f;
  float bStep = 0.01f;
  float aSpeed = 0.5f;
  float bSpeed = 0.3f;
  float aAngle = 30.f;
  float bAngle = 30.f;

  //the following members are used by Plan 3
  //last angle to store
  float lAngle = 0.0f;
  //last angle we use
  float lAngleUse = 0.0f;
  //if the app first enter the scene
  bool firstIn = true;
  float offset = 0.0f;
  //initial amplification parameter
  float amp = 1.0f;
  float angleDiff = 0.0f;
  // if direction < 0: acceleration to left; else acceleration to right
  float direction = 0.0f;

  //
//  float lowSpeed = 0.3f;
//  float midSpeed = 0.8f;
//  float highSpeed = 1.5f;

  std::vector<float> speed = {22.0f, 45.0f, 70.0f};
  const float MULTIPLER = 2.8f;
  const float OFFSET = 0.3*PI;
  float dir = 1.0f; //1 right, -1 left
  int speed_idx_ = 0;

  bool isTurningBack = false;
  float viewAngle = 0.0f;
  float amp4 = 1.0f;
  float lastKeyAngles[3];

  GLuint depthRenderBuffer_;  // depth buffer
  GLuint framebuffer_;        // framebuffer object
  GLuint texture_;            // distortion texture

  GLuint obj_program_;
  GLuint obj_position_param_;
  GLuint obj_uv_param_;
  GLuint obj_modelview_projection_param_;

  Matrix4x4 head_view_;
  Matrix4x4 rotated_head_view_;
  Matrix4x4 last_key_head_view_;
  Matrix4x4 model_target_;
  Matrix4x4 model_target1_;

  Matrix4x4 modelview_projection_target_;
  Matrix4x4 mdoelview_projection_target1_;
  Matrix4x4 modelview_projection_room_;

  TexturedMesh room_;
  Texture room_tex_;

  std::vector<TexturedMesh> target_object_meshes_;
  std::vector<Texture> target_object_not_selected_textures_;
  std::vector<Texture> target_object_selected_textures_;
  int cur_target_object_;

        float GetAmp(float speed);
    };

}  // namespace ndk_hello_cardboard

#endif  // HELLO_CARDBOARD_ANDROID_SRC_MAIN_JNI_HELLO_CARDBOARD_APP_H_
