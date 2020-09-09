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
import android.widget.Button;
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
    private static final float[] speed = {22.0f, 45.0f, 70.0f};
    float aSpeed = 0.0f;
    float bSpeed = 0.0f;
    float aAngle = 0.0f;
    float bAngle = 0.0f;
    float maxAngle = 0.0f;
    float angleDiff = 0.0f;
    float nowAngle = 0.0f;
    float targetSpeed = 45.0f;
    float direction = 0.0f;
    float viewAngle = 0.0f;
    float tamp = 1.0f;
    float[] rotated;
    float[] origin;

    //Date date;
    int counter = 0;
    int refreshRate = 4;
    float amp = 1.0f;

    //Control variables for logging
    private boolean start = false;
    private FileOutputStream logFile;
    private String path = Environment.getExternalStorageDirectory() + "/Cardboard";

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
        rotated = new float[16]; origin = new float[16];
        // Prevents screen from dimming/locking.
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }

    //TODO: Log the rotation speed, angle etc.
    private void createLogFile() {
        String path = Environment.getExternalStorageDirectory() + "/Cardboard";
        try {
            FileOutputStream fos = new FileOutputStream(path+"/log-" + System.currentTimeMillis() + ".txt");
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
            //ProgressBar p1 = (ProgressBar) findViewById(R.id.progress_bar1);
            nativeOnDrawFrame(nativeApp, amp);

            float[] tmp = nativeTestReturnVector(nativeApp);
            // use the following results
            angleDiff = tmp[1]; maxAngle = tmp[2]; nowAngle = tmp[3]; direction = tmp[4]; viewAngle = tmp[5]; tamp = tmp[6];
//            for (int i = 7; i < 23; ++i) rotated[i-7] = tmp[i];
//            for (int i = 23; i < 39; ++i) origin[i-23] = tmp[i];
            /*
            Log.e("rotated", rotated[0] + " " +rotated[1] + " " + rotated[2] + " " + rotated[3] + " " +
                            rotated[4] + " " + rotated[5] + " " + rotated[6] + " " + rotated[7] + " " +
                            rotated[8] + " " + rotated[9] + " " + rotated[10] + " " + rotated[11] + " " +
                            rotated[12] + " " + rotated[13] + " " + rotated[14] + " " + rotated[15] + " ");
            Log.e("origin", origin[0] + " " +origin[1] + " " + origin[2] + " " + origin[3] + " " +
                    origin[4] + " " + origin[5] + " " + origin[6] + " " + origin[7] + " " +
                    origin[8] + " " + origin[9] + " " + origin[10] + " " + origin[11] + " " +
                    origin[12] + " " + origin[13] + " " + origin[14] + " " + origin[15] + " ");

             */
                    // use System.currentTimeMillis() to read time
                    // direction > 0 means speed to right; else speed to left
            if (start) {
                try {
                    //log time, direction, cur angle
                    String data = System.currentTimeMillis() + "," + angleDiff + "," + nowAngle + "," + direction + "\n";
                    logFile.write(data.getBytes());
                } catch(Exception e) {
                    e.printStackTrace();
                }
            }
            // todo: record logic here, when left button is pressed(for the first time), start == true
//            Log.e("tamp", tamp+"");
            t1.setText(viewAngle+"");
            t2.setText(tamp+"");
            String s = "";
            for(int i = 8; i < tmp.length; i++) {
                s += " " + tmp[i];
            }
            float rotateAngle = tmp[8];
            Log.e("angleDiff", angleDiff+" "+direction+" "+nowAngle+" "+tamp+" "+tmp[7]+" "+viewAngle+" "+rotateAngle+" "+s);
            // show related numbers on screen
            if (counter == 0) {
//                t1.setText(angleDiff+"");

//                Log.e("Direction", direction+"");
//                Log.e("NowAngle", nowAngle+"");
//                //p1.setProgress((int)(angleDiff/(2*targetSpeed)*100));
//                t2.setText(tamp+"");
                counter++;
            }
            else {
                counter++; if (counter == refreshRate) counter = 0;
            }

        }
    }

    /** Callback for when close button is pressed. */
    public void closeSample(View view) {
        Log.d(TAG, "Leaving VR sample");
        finish();
    }

    public void changeStatus(View view) { //S denotes start, E denotes end
        Button statusButton = (Button)findViewById(R.id.changeStatus);
        if(start == true) {
            statusButton.setText("S");
            //switch to normal
            nativeSwitchPlan(nativeApp, 5);
            try {
                logFile.close();
            } catch(Exception e) {
                e.printStackTrace();
            }
        } else {
            statusButton.setText("E");
            //switch to auto-rotation
            nativeSwitchPlan(nativeApp, 2);
            try {
                logFile = new FileOutputStream(path+"/log-" + targetSpeed + "-" + System.currentTimeMillis() + ".txt");
                String info = Float.toString(targetSpeed) + "\n";
                Log.v("Create file", "Create file");
                logFile.write(info.getBytes());
                logFile.flush();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        start = !start;
    }

    public void raiseAmp(View view) {
        amp += 0.15f;
    }

    public void lowerAmp(View view) {
        amp -= 0.05f;
        //if (amp < 1) amp = 1;
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
//          Log.e("log", aSpeed+","+bSpeed+","+aAngle+","+bAngle);
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
            final String[] items = {"Plan 0", "Plan 1", "Plan 2", "Plan 3", "Plan 4"};
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
        else if (item.getItemId() == R.id.switch_speed) {
            final String[] items = {"Low", "Middle", "High"};
            AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setTitle("Switch Speed");
            builder.setIcon(android.R.drawable.ic_dialog_info);
            builder.setItems(items, new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialogInterface, int i) {
                    Log.e("select speed", items[i]);
                    targetSpeed = speed[i];
                    nativeSwitchSpeed(nativeApp, i);
                }
            });
            builder.show();
            return true;
        }
        else if (item.getItemId() == R.id.switch_dir) {
            final String[] items = {"Left", "Right"};
            AlertDialog.Builder builder = new AlertDialog.Builder(this);
            builder.setTitle("Switch Dir");
            builder.setIcon(android.R.drawable.ic_dialog_info);
            builder.setItems(items, new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialogInterface, int i) {
                    Log.e("select dir", items[i]);
                    float dir = (i == 0) ? -1.0f : 1.0f;
                    nativeSwitchDir(nativeApp, dir);
                }
            });
            builder.show();
            return true;
        }
//    else if (item.getItemId() == R.id.change_target_speed) {
//      LayoutInflater factory = LayoutInflater.from(this);
//      final View textEntryView = factory.inflate(R.layout.change_speed, null);
//      final EditText edit_target_speed = (EditText) textEntryView.findViewById(R.id.edit_target_speed);
//
//      AlertDialog.Builder builder = new AlertDialog.Builder(this);
//      builder.setTitle("Change Target Speed");
//      builder.setIcon(android.R.drawable.ic_dialog_info);
//      builder.setView(textEntryView);
//      builder.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
//        @Override
//        public void onClick(DialogInterface dialogInterface, int i) {
//          String str_targetSpeed = edit_target_speed.getText().toString();
//          targetSpeed = Float.parseFloat(str_targetSpeed);
//
//        }
//      });
//      builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
//        @Override
//        public void onClick(DialogInterface dialogInterface, int i) {
//        }
//      });
//      builder.show();
//      return true;
//    }
        return false;
    }

    /**
     * Checks for activity permissions.
     *
     * @return whether the permissions are already granted.
     */
    private boolean arePermissionsEnabled() {
        boolean writePermission =
                ActivityCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE)
                        == PackageManager.PERMISSION_GRANTED;
        boolean readPermission = ActivityCompat.checkSelfPermission(this, Manifest.permission.READ_EXTERNAL_STORAGE)
                == PackageManager.PERMISSION_GRANTED;
        return (writePermission && readPermission);
    }

    /** Handles the requests for activity permissions. */
    private void requestPermissions() {
        final String[] permissions = new String[] {Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.WRITE_EXTERNAL_STORAGE};
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

    private native void nativeSwitchSpeed(long nativeApp, int p);

    private native void nativeSwitchDir(long nativeApp, float d);

    private native float[] nativeTestReturnVector(long nativeApp);
}
