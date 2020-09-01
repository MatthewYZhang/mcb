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
package com.google.cardboard;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.net.Uri;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.Environment;
import android.provider.Settings;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.PopupMenu;
import android.widget.ProgressBar;
import android.widget.Toast;
import android.widget.EditText;
import android.widget.TextView;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import java.io.File;
import java.util.Date;
import java.io.FileOutputStream;


/**
 * A Google Cardboard VR NDK sample application.
 *
 * <p>This is the main Activity for the sample application. It initializes a GLSurfaceView to allow
 * rendering.
 */
public class VrActivity extends AppCompatActivity implements PopupMenu.OnMenuItemClickListener {
  static {
    System.loadLibrary("cardboard_jni");
  }

  private static final String TAG = VrActivity.class.getSimpleName();

  // Permission request codes
  private static final int PERMISSIONS_REQUEST_CODE = 2;
  float aSpeed = 0.0f;
  float bSpeed = 0.0f;
  float aAngle = 0.0f;
  float bAngle = 0.0f;
  float maxAngle = 0.0f;
  float angleDiff = 0.0f;
  float targetAngle = 60.0f;
  //Date date;
  int counter = 0;
  int refreshRate = 10;
  float amp = 1.0f;


  // Opaque native pointer to the native CardboardApp instance.
  // This object is owned by the VrActivity instance and passed to the native methods.
  private long nativeApp;

  private GLSurfaceView glView;

  @SuppressLint("ClickableViewAccessibility")
  @Override
  public void onCreate(Bundle savedInstance) {
    super.onCreate(savedInstance);
    //date = new Date();
    nativeApp = nativeOnCreate(getAssets());

    setContentView(R.layout.activity_vr);
    glView = findViewById(R.id.surface_view);
    glView.setEGLContextClientVersion(2);
    Renderer renderer = new Renderer();
    glView.setRenderer(renderer);
    glView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
    glView.setOnTouchListener(
        (v, event) -> {
          if (event.getAction() == MotionEvent.ACTION_DOWN) {
            // Signal a trigger event.
            glView.queueEvent(
                () -> {
                  nativeOnTriggerEvent(nativeApp);
                  maxAngle = 0.0f;
                  TextView t1 = (TextView) findViewById(R.id.txtOne);
                  t1.setText(maxAngle+"");
                });
            return true;
          }
          return false;
        });

    // TODO(b/139010241): Avoid that action and status bar are displayed when pressing settings
    // button.
    setImmersiveSticky();
    View decorView = getWindow().getDecorView();
    decorView.setOnSystemUiVisibilityChangeListener(
        (visibility) -> {
          if ((visibility & View.SYSTEM_UI_FLAG_FULLSCREEN) == 0) {
            setImmersiveSticky();
          }
        });

    // Forces screen to max brightness.
    WindowManager.LayoutParams layout = getWindow().getAttributes();
    layout.screenBrightness = 1.f;
    getWindow().setAttributes(layout);

    // Prevents screen from dimming/locking.
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
  }

  //TODO: Log the rotation speed, angle etc.
  private void createLogFile() {
    String path = Environment.getExternalStorageDirectory() + "/Cardboard";
    try {
      FileOutputStream fos = new FileOutputStream(path+"/log.txt");
      String info = "hello world";
      fos.write(info.getBytes());
      fos.flush();
      fos.close();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  protected void onPause() {
    super.onPause();
    nativeOnPause(nativeApp);
    glView.onPause();
  }

  @Override
  protected void onResume() {
    super.onResume();

    // Checks for activity permissions, if not granted, requests them.
    if (!arePermissionsEnabled()) {
      requestPermissions();
      return;
    }

    glView.onResume();
    nativeOnResume(nativeApp);
  }

  @Override
  protected void onDestroy() {
    super.onDestroy();
    nativeOnDestroy(nativeApp);
    nativeApp = 0;
  }

  @Override
  public void onWindowFocusChanged(boolean hasFocus) {
    super.onWindowFocusChanged(hasFocus);
    if (hasFocus) {
      setImmersiveSticky();
    }
  }

  private class Renderer implements GLSurfaceView.Renderer {
    @Override
    public void onSurfaceCreated(GL10 gl10, EGLConfig eglConfig) {
      nativeOnSurfaceCreated(nativeApp);
    }

    @Override
    public void onSurfaceChanged(GL10 gl10, int width, int height) {
      nativeSetScreenParams(nativeApp, width, height);
    }

    @Override
    public void onDrawFrame(GL10 gl10) {
      TextView t1 = (TextView) findViewById(R.id.txtOne);
      TextView t2 = (TextView) findViewById(R.id.txtTwo);
      ProgressBar p1 = (ProgressBar) findViewById(R.id.progress_bar1);
      ProgressBar p2 = (ProgressBar) findViewById(R.id.progress_bar2);
      angleDiff = nativeOnDrawFrame(nativeApp, amp);
      Log.e("time:", System.currentTimeMillis() + "");

      if (counter == 0) {
        t1.setText(angleDiff+"");
        Log.e("angleDiff", angleDiff+"");
        p1.setProgress((int)(angleDiff/(2*targetAngle)*100));
        p2.setProgress((int)(angleDiff/(2*targetAngle)*100));
        counter++;
      }
      else {
        counter++; if (counter == refreshRate) counter = 0;
      }
      t2.setText(amp+"");
    }
  }

  /** Callback for when close button is pressed. */
  public void closeSample(View view) {
    Log.d(TAG, "Leaving VR sample");
    finish();
  }

  public void raiseAmp(View view) {
    amp += 0.05f;
  }

  public void lowerAmp(View view) {
    amp -= 0.05f; if (amp < 1) amp = 1;
  }

  /** Callback for when settings_menu button is pressed. */
  public void showSettings(View view) {
    PopupMenu popup = new PopupMenu(this, view);
    MenuInflater inflater = popup.getMenuInflater();
    inflater.inflate(R.menu.settings_menu, popup.getMenu());
    popup.setOnMenuItemClickListener(this);
    popup.show();
  }

  @Override
  public boolean onMenuItemClick(MenuItem item) {
    if (item.getItemId() == R.id.switch_viewer) {
      nativeSwitchViewer(nativeApp);
      return true;
    }
    else if (item.getItemId() == R.id.change_parameters) {
      //write new function to set up parameters
      LayoutInflater factory = LayoutInflater.from(this);
      final View textEntryView = factory.inflate(R.layout.set_parameter, null);
      final EditText edit_aSpeed = (EditText) textEntryView.findViewById(R.id.edit_aSpeed);
      final EditText edit_bSpeed = (EditText) textEntryView.findViewById(R.id.edit_bSpeed);
      final EditText edit_aAngle = (EditText) textEntryView.findViewById(R.id.edit_aAngle);
      final EditText edit_bAngle = (EditText) textEntryView.findViewById(R.id.edit_bAngle);
      AlertDialog.Builder builder = new AlertDialog.Builder(this);
      builder.setTitle("Set Parameters");
      builder.setIcon(android.R.drawable.ic_dialog_info);
      builder.setView(textEntryView);
      builder.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
        @Override
        public void onClick(DialogInterface dialogInterface, int i) {
          String str_aSpeed = edit_aSpeed.getText().toString();
          String str_bSpeed = edit_bSpeed.getText().toString();
          String str_aAngle = edit_aAngle.getText().toString();
          String str_bAngle = edit_bAngle.getText().toString();
          aSpeed = Float.parseFloat(str_aSpeed); bSpeed = Float.parseFloat(str_bSpeed);
          aAngle = Float.parseFloat(str_aAngle); bAngle = Float.parseFloat(str_bAngle);
          Log.e("log", aSpeed+","+bSpeed+","+aAngle+","+bAngle);
          nativeSetParameter(nativeApp, aSpeed, bSpeed, aAngle, bAngle);
        }
      });
      builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
        @Override
        public void onClick(DialogInterface dialogInterface, int i) {
        }
      });
      builder.show();
      return true;
    }
    else if (item.getItemId() == R.id.switch_plan) {
      final String[] items = {"Plan 0", "Plan 1", "Plan 2", "Plan 3"};
      AlertDialog.Builder builder = new AlertDialog.Builder(this);
      builder.setTitle("Switch Plans");
      builder.setIcon(android.R.drawable.ic_dialog_info);
      builder.setItems(items, new DialogInterface.OnClickListener() {
        @Override
        public void onClick(DialogInterface dialogInterface, int i) {
          Log.e("select", items[i]);
          nativeSwitchPlan(nativeApp, i);
        }
      });
      builder.show();
      return true;
    }
    return false;
  }

  /**
   * Checks for activity permissions.
   *
   * @return whether the permissions are already granted.
   */
  private boolean arePermissionsEnabled() {
    return ActivityCompat.checkSelfPermission(this, Manifest.permission.READ_EXTERNAL_STORAGE)
        == PackageManager.PERMISSION_GRANTED;
  }

  /** Handles the requests for activity permissions. */
  private void requestPermissions() {
    final String[] permissions = new String[] {Manifest.permission.READ_EXTERNAL_STORAGE};
    ActivityCompat.requestPermissions(this, permissions, PERMISSIONS_REQUEST_CODE);
  }

  /** Callback for the result from requesting permissions. */
  @Override
  public void onRequestPermissionsResult(
      int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
    super.onRequestPermissionsResult(requestCode, permissions, grantResults);
    if (!arePermissionsEnabled()) {
      Toast.makeText(this, R.string.no_permissions, Toast.LENGTH_LONG).show();
      if (!ActivityCompat.shouldShowRequestPermissionRationale(
          this, Manifest.permission.READ_EXTERNAL_STORAGE)) {
        // Permission denied with checking "Do not ask again".
        launchPermissionsSettings();
      }
      finish();
    }
  }

  private void launchPermissionsSettings() {
    Intent intent = new Intent();
    intent.setAction(Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
    intent.setData(Uri.fromParts("package", getPackageName(), null));
    startActivity(intent);
  }

  private void setImmersiveSticky() {
    getWindow()
        .getDecorView()
        .setSystemUiVisibility(
            View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_FULLSCREEN
                | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
  }

  private native long nativeOnCreate(AssetManager assetManager);

  private native void nativeOnDestroy(long nativeApp);

  private native void nativeOnSurfaceCreated(long nativeApp);

  private native float nativeOnDrawFrame(long nativeApp, float amp);

  private native void nativeOnTriggerEvent(long nativeApp);

  private native void nativeOnPause(long nativeApp);

  private native void nativeOnResume(long nativeApp);

  private native void nativeSetScreenParams(long nativeApp, int width, int height);

  private native void nativeSwitchViewer(long nativeApp);

  private native void nativeSetParameter(long nativeApp, float aSp, float bSp, float aAn, float bAn);

  private native void nativeSwitchPlan(long nativeApp, int p);
}
