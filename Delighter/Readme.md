# ORB-SLAM3 Monocular Phone-Stream Setup (WSL2 / Ubuntu) — Final

Complete, verified end-to-end guide: dependency build, code fixes, camera calibration, WSL2 networking, live execution, clean shutdown, and map save/load. Repo root used throughout: `/home/msr/dev/ORB_SLAM3`.

---

## 1. Directory Layout Assumed

```
/home/msr/dev/
├── opencv-4.4.0/          # OpenCV source, built and installed to /usr/local
├── Pangolin/              # Pangolin source
└── ORB_SLAM3/             # this repo
    ├── build/
    ├── lib/libORB_SLAM3.so
    ├── Thirdparty/DBoW2/lib/libDBoW2.so
    ├── Thirdparty/g2o/lib/libg2o.so
    ├── Vocabulary/ORBvoc.txt
    ├── Examples/Monocular/mono_stream.cc
    ├── src/Frame.cc
    ├── phone_camera_generic.yaml
    └── CMakeLists.txt
```

All commands below assume `cd /home/msr/dev/ORB_SLAM3` first unless stated otherwise.

---

## 2. Install Dependencies

```bash
sudo apt update
sudo apt install -y build-essential cmake git libeigen3-dev libglew-dev
dpkg -l | grep -i glew   # confirm libglew-dev, libglew2.1 installed
```

### 2.1 Build OpenCV 4.4.0 to /usr/local

```bash
cd /home/msr/dev
git clone https://github.com/opencv/opencv.git opencv-4.4.0
cd opencv-4.4.0
git checkout 4.4.0
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j2
sudo make install
sudo ldconfig
pkg-config --modversion opencv4   # expect: 4.4.0
```

### 2.2 Build Pangolin

```bash
cd /home/msr/dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j2
sudo make install
sudo ldconfig
```

---

## 3. CMakeLists.txt Fixes (Apply Before First Build)

### 3.1 Remove the ROS/catkin block

**Why:** ROS Noetic's `cv_bridge` links against Ubuntu's system OpenCV (4.2, at `/usr/lib/x86_64-linux-gnu/`). Linking it alongside a separately-built OpenCV 4.4 (`/usr/local/lib/`) causes the linker to merge both dependency sets into `libORB_SLAM3.so`, producing duplicate library versions in `ldd`, undefined behavior, and viewer crashes.

```bash
cd /home/msr/dev/ORB_SLAM3
cp CMakeLists.txt CMakeLists.txt.bak
grep -n -i "catkin" CMakeLists.txt      # confirm line numbers first
sed -i '/find_package(catkin/,/^)/d' CMakeLists.txt
sed -i '/${catkin_INCLUDE_DIRS}/d' CMakeLists.txt
sed -i '/${catkin_LIBRARIES}/d' CMakeLists.txt
grep -n -i "catkin" CMakeLists.txt      # must return nothing
```

### 3.2 Add GLEW to the link line

**Why:** Pangolin's `GlBufferData` destructor calls GLEW functions (`__glewDeleteBuffers`) not always pulled in transitively via `Pangolin_LIBRARIES`. Without this, linking fails with `undefined reference to '__glewDeleteBuffers'` (observed on `rgbd_tum`, `stereo_inertial_tum_vi`).

```bash
sed -i 's/\${Pangolin_LIBRARIES}/\${Pangolin_LIBRARIES}\n-lGLEW/' CMakeLists.txt
grep -n -A1 "Pangolin_LIBRARIES" CMakeLists.txt
```

---

## 4. Source Code Fixes

### 4.1 Frame.cc — remove leftover debug block

`src/Frame.cc`'s `ComputeBoW()` function contained a leftover debug hack from a prior developer, hardcoded to a nonexistent path (`/home/aazam/abc.txt`). It fires on **every** BoW computation (a normal, frequent operation for keyframes/relocalization) — not on actual tracking loss — printing misleading `Agent is stopped due to track loss` / `Unable to open file` messages and writing unnecessary blank images to disk on every call.

```bash
cd /home/msr/dev/ORB_SLAM3
cp src/Frame.cc src/Frame.cc.bak
sed -i '/\/\/ To write image when track is lost/,/cv::imwrite(ss1.str(), im);/d' src/Frame.cc
sed -i '/\/\/     \/\/cout<<"\\nAgent is stopped/d; /\/\/     cout<<"\\nAgent is stopped/d' src/Frame.cc
grep -n "Agent is stopped\|Unable to open file" src/Frame.cc   # must return nothing
```

### 4.2 mono_stream.cc — frame validation and timestamp monotonicity

The original loop passed every grabbed frame straight to `TrackMonocular()` with no validity check, and computed `tframe` fresh from `system_clock::now()` with no guarantee it was strictly increasing. Two real problems resulted:

- **Corrupted frames tracked as-is.** Network instability produces `[mjpeg @ ...] overread` decoder warnings; OpenCV can still return `true` from `cap.read()` with a garbage/partial `cv::Mat`, feeding bad data straight into the tracker (`TRACK_REF_KF: Less than 15 matches!!`).
- **Non-monotonic timestamps.** WSL2's guest clock can drift and resync backward relative to the host (a known WSL2 behavior, especially after idle/sleep). ORB-SLAM3 rejects any frame timestamped ≤ the previous one, forcing repeated Atlas map resets (`ERROR: Frame with a timestamp older than previous frame detected!`).

```bash
cd /home/msr/dev/ORB_SLAM3
cp Examples/Monocular/mono_stream.cc Examples/Monocular/mono_stream.cc.bak

cat > Examples/Monocular/mono_stream.cc << 'EOF'
#include <System.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>

int main(int argc, char **argv) {
    if (argc < 4) {
        std::cerr << "Usage: ./mono_stream path_to_vocabulary path_to_settings stream_url\n";
        return 1;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    cv::VideoCapture cap(argv[3]);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open stream: " << argv[3] << "\n";
        return -1;
    }

    cv::Mat frame;
    double last_tframe = 0.0;
    while (cap.read(frame)) {
        if (frame.empty()) {
            std::cerr << "Skipping empty/corrupted frame\n";
            continue;
        }

        double tframe = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        if (tframe <= last_tframe) {
            tframe = last_tframe + 1e-6;
        }
        last_tframe = tframe;

        SLAM.TrackMonocular(frame, tframe);
    }

    SLAM.Shutdown();
    return 0;
}
EOF

diff Examples/Monocular/mono_stream.cc.bak Examples/Monocular/mono_stream.cc
```

---

## 5. Full Clean Build

```bash
cd /home/msr/dev/ORB_SLAM3
rm -rf build
find . -name "CMakeCache.txt" -delete
find . -name "CMakeFiles" -type d -exec rm -rf {} +
mkdir build && cd build
cmake -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 -DCMAKE_BUILD_TYPE=Release ..
grep -i catkin CMakeCache.txt                                        # must return nothing
grep -i "FIND_PACKAGE_MESSAGE_DETAILS_OpenCV" CMakeCache.txt         # expect v4.4.0(4.4)
make -j2
```

Use `-j2`, not `-j$(nproc)`, on memory-constrained machines to avoid OOM-related build inconsistencies. If a target fails and halts the rest of the build, use `make -k -j2` to keep building remaining targets.

After any later edit to `Frame.cc` (part of the core library) or `mono_stream.cc` (an example), rerun `make -j2` from `build/` to relink.

---

## 6. Verify the Build Is Clean

```bash
cd /home/msr/dev/ORB_SLAM3
ldd Examples/Monocular/mono_stream | grep opencv
```

Expected: every module exactly once, all `4.4`, all `/usr/local/lib`:

```
libopencv_videoio.so.4.4    => /usr/local/lib/libopencv_videoio.so.4.4
libopencv_core.so.4.4       => /usr/local/lib/libopencv_core.so.4.4
libopencv_highgui.so.4.4    => /usr/local/lib/libopencv_highgui.so.4.4
libopencv_imgcodecs.so.4.4  => /usr/local/lib/libopencv_imgcodecs.so.4.4
libopencv_calib3d.so.4.4    => /usr/local/lib/libopencv_calib3d.so.4.4
libopencv_features2d.so.4.4 => /usr/local/lib/libopencv_features2d.so.4.4
libopencv_imgproc.so.4.4    => /usr/local/lib/libopencv_imgproc.so.4.4
libopencv_flann.so.4.4      => /usr/local/lib/libopencv_flann.so.4.4
```

Any `.so.4.2` entry, or any module listed twice, means Section 3.1 wasn't applied correctly — re-check and redo the clean build.

**Note:** `mono_stream` only exists inside `/home/msr/dev/ORB_SLAM3`. Running `ldd` from an unrelated directory correctly reports "No such file or directory" — that's a working-directory mistake, not a build failure.

---

## 7. Camera Calibration

The zero-distortion placeholder YAML works for verifying the pipeline runs, but causes drift and track loss. Do a real calibration for actual use.

1. Print a checkerboard (9x6 or 8x6 internal corners).
2. Start IP Webcam on the phone, note the stream URL from the app's on-screen display.
3. Capture 20-30 stills of the checkerboard from varied angles/distances, including near frame edges.
4. Run OpenCV's calibration routine (`cv2.calibrateCamera` or `opencv_interactive-calibration`) to get `fx, fy, cx, cy, k1, k2, p1, p2`.
5. Write into `phone_camera_generic.yaml`:

```yaml
%YAML:1.0

Camera.type: "PinHole"
Camera.fx: 948.839
Camera.fy: 948.839
Camera.cx: 640
Camera.cy: 360
Camera.k1: 0
Camera.k2: 0
Camera.p1: 0
Camera.p2: 0
Camera.k3: 0

Camera.width: 1280
Camera.height: 720
Camera.fps: 30.0
Camera.RGB: 1

ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
```

(Values shown are the actual ones loaded in the verified working session — replace `fx/fy/cx/cy`/distortion with your own measured calibration.)

---

## 8. Network Setup (WSL2-Specific — Required Before Running)

### 8.1 Diagnose

```bash
ip addr show | grep "inet "
nmcli device status
ping -c 4 <phone-ip>
cmd.exe /c ipconfig       # inspect what Windows itself sees
```

In the Windows output, distinguish:
- The **real WiFi adapter** (e.g. `Wireless LAN adapter Wi-Fi`) — mirrors correctly into WSL2.
- The **Mobile Hotspot virtual adapter** (e.g. `Local Area Connection* 4`, typically `192.168.137.1`) — does **not** mirror into WSL2, even with mirrored networking enabled.

### 8.2 Enable WSL2 Mirrored Networking

Requires Windows 11 22H2+ and a current WSL version (`wsl.exe --update` if unsure).

```bash
ls /mnt/c/Users/                      # find your Windows username
cat > /mnt/c/Users/<WindowsUsername>/.wslconfig << 'EOF'
[wsl2]
networkingMode=mirrored
dnsTunneling=true
autoProxy=true
firewall=true
EOF
wsl.exe --shutdown
```

Reopen the WSL terminal after shutdown finishes (a full VM restart, not just closing the window). Verify:

```bash
ip addr show | grep "inet "
```

WSL2's main interface should now carry the **same IP** as Windows' real WiFi adapter.

### 8.3 The Fix That Actually Works

Mirrored networking mirrors physical adapters only, not the Mobile Hotspot virtual adapter. So:

- **Do not** connect the phone to the laptop's Mobile Hotspot.
- **Do** connect the phone to the exact same real WiFi network the laptop's physical WiFi adapter uses.

```bash
ping -c 4 <phone-ip-on-shared-wifi>
curl -I http://<phone-ip-on-shared-wifi>:8080/video
```

Both must succeed before running `mono_stream`.

### 8.4 Fallback: AP Client Isolation

If `ping` still fails on shared WiFi (common on institutional networks that block device-to-device traffic), use USB tethering instead:

1. Connect phone to laptop via USB cable.
2. Enable **USB tethering** on the phone (not WiFi hotspot).
3. Check `cmd.exe /c ipconfig` for a new RNDIS-based adapter — being a real NIC, it mirrors into WSL2 correctly.

---

## 9. Run mono_stream

```bash
cd /home/msr/dev/ORB_SLAM3
LD_LIBRARY_PATH=/usr/local/lib ./Examples/Monocular/mono_stream Vocabulary/ORBvoc.txt phone_camera_generic.yaml http://<phone-ip>:8080/video
```

Critical details:
- Use `http://`, never `https://` — IP Webcam serves plain HTTP.
- Paste the **plain URL only**. Never pass literal Markdown link syntax (`[text](url)`) as the argument — it breaks OpenCV's `VideoCapture` parsing and can cause a segfault via the `CAP_IMAGES`/`icvExtractPattern` fallback error.

Success looks like:

```
Loading ORB Vocabulary. This could take a while...
Vocabulary loaded!
Initialization of Atlas from scratch
Creation of new map with id: 0
Starting the Viewer
```

followed by the Pangolin window opening with the live feed and growing point map. Move the phone with clear **translational** motion (side-to-side or forward, not pure rotation) at startup — the monocular initializer needs parallax to triangulate the first map points.

### 9.1 If tracking is still unstable after all fixes

`TRACK_REF_KF: Less than 15 matches!!` persisting, or `[mjpeg @ ...] overread` warnings in the log, point to genuine network-level stream corruption, not a code issue. In the IP Webcam app:
- Lower video resolution (e.g. 640x480 instead of 1280x720)
- Lower JPEG quality percentage
- Cap frame rate lower (e.g. 15 fps)
- Check WiFi signal strength on the phone

---

## 10. Closing mono_stream Cleanly

`Ctrl+C` can fail to terminate the process if it's blocked inside Pangolin's GUI event loop or a blocking network read — common with a live streaming source. Use these in order:

### 10.1 Preferred: find and kill the process

In a **new** terminal (leave the stuck one open):

```bash
ps aux | grep mono_stream
kill -9 <PID>
```

`-9` (SIGKILL) forces immediate termination, bypassing any hung cleanup code.

### 10.2 If it's a suspended job (from an earlier accidental Ctrl+Z)

```bash
jobs
kill -9 %1        # replace 1 with the actual job number shown
```

### 10.3 If the Pangolin window itself is frozen (WSLg)

```bash
sudo apt install wmctrl   # if not already installed
wmctrl -l                 # list windows and their IDs
wmctrl -ic <window_id>    # close a specific window
```

### 10.4 Last resort: full WSL2 restart

```bash
exit
```

Then in PowerShell:

```powershell
wsl --shutdown
```

Reopen the WSL terminal afterward. Guaranteed to clear anything stuck, but heavier-handed than the options above — use only if 10.1-10.3 don't work.

**Note:** `SLAM.Shutdown()` (and any `System.SaveAtlasToFile` write) only runs on a **clean** exit. A `kill -9` skips this — if you need the map saved, prefer a plain `Ctrl+C` or closing the Pangolin window normally first, and only escalate to `kill -9` if the process is genuinely unresponsive.

---

## 11. Saving and Loading the Map

Add to `phone_camera_generic.yaml` to save on clean shutdown:

```yaml
System.SaveAtlasToFile: "phone_map"
```

(`.osa` extension is appended automatically.) Run normally, then close the viewer or `Ctrl+C` for a clean exit (see Section 10 note above — forced kills skip the save). Confirm:

```bash
ls -la phone_map.osa
```

To reload instead of remapping, comment out `SaveAtlasToFile` and use:

```yaml
System.LoadAtlasFromFile: "phone_map"
```

Do not set both simultaneously — behavior is inconsistent across ORB-SLAM3 versions. Always load with the exact same `Vocabulary/ORBvoc.txt` used at save time; the file is checksum-validated and will be rejected otherwise.

---

## 12. Porting to SparkFun JetBot V2.1 (Jetson Nano)

- `phone_map.osa`, `Vocabulary/ORBvoc.txt`, and the general build steps carry over — the **camera calibration YAML does not**. The JetBot's onboard 136° FOV camera needs its own calibration (repeat Section 7).
- ORB-SLAM3 (and OpenCV, Pangolin) must be rebuilt natively on the Jetson Nano's ARM64 architecture — an x86_64 binary from this WSL2 build will not run there.
- Camera input differs: use a GStreamer pipeline via `nvarguscamerasrc` (CSI camera) or `/dev/video0` via V4L2 in a modified example source file, instead of an HTTP MJPEG URL.
- Prefer remapping the target space from scratch with the JetBot's own camera rather than reusing `phone_map.osa` for anything beyond a pipeline smoke-test, given the calibration mismatch.

---

## 13. Troubleshooting Reference

| Symptom | Cause | Fix |
|---|---|---|
| `ldd` shows both `.so.4.2` and `.so.4.4` for the same OpenCV module | ROS `cv_bridge` linked via leftover `find_package(catkin ...)` | Section 3.1 |
| `undefined reference to __glewDeleteBuffers` at link time | GLEW not explicitly linked | Section 3.2 |
| GTK crash / `Trace/breakpoint trap` on viewer launch | Mixed OpenCV/GL library versions | Full clean rebuild after 3.1/3.2 |
| `ldd: ... No such file or directory` | Wrong working directory | `cd /home/msr/dev/ORB_SLAM3` first |
| `Agent is stopped due to track loss` / `Unable to open file` spam | Leftover debug hack in `Frame::ComputeBoW()`, unrelated to real tracking state | Section 4.1 |
| `ERROR: Frame with a timestamp older than previous frame detected!` / repeated map resets | Non-monotonic `tframe` from WSL2 clock drift | Section 4.2 |
| `TRACK_REF_KF: Less than 15 matches!!` + `[mjpeg @ ...] overread` | Corrupted frames from unstable network stream | Section 9.1 — lower stream resolution/quality/FPS |
| `Connection timed out` / 100% ping loss to phone | WSL2 NAT isolation, or phone on laptop's Mobile Hotspot (unmirrorable) | Section 8 |
| `CAP_IMAGES` / `icvExtractPattern` error + segfault | Malformed stream URL (pasted Markdown syntax, or `https://` instead of `http://`) | Section 9 |
| `Ctrl+C` doesn't close the program/viewer | Process blocked in GUI/network event loop | Section 10 |
| Map fails to load / rejected | Vocabulary file mismatch | Load with the exact `ORBvoc.txt` used at save time |
| Track loss / drift during tracking | Inaccurate or generic calibration | Section 7 |
