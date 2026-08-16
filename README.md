# NavSight

**Beyond GPS: precision navigation in GPS-denied environments.**

NavSight keeps a rider's position on the map without depending on satellites. It is an
Android application built around a C++ visual-inertial core: the phone's camera and IMU
estimate motion, a geometric model of the road plane turns that motion into metres per
second, and a hidden-Markov map matcher constrains the displayed position to an
OpenStreetMap road graph held on the device. Positioning never uses a satellite signal or
a network round-trip.

The project was developed and validated in Haifa during a period of persistent, wide-area
GNSS jamming, where the phone's own GPS receiver froze for 61 seconds while moving and
inflated a measured 1,280 m route by 33%. That environment is the reason the system exists.

B.Sc. final project, Software Engineering, Shenkar College of Engineering.

---

## Results

Measured on a Samsung Galaxy S21 Ultra over real urban rides under active jamming, against
two independent references (a map-measured route, and GPS screened for per-ride health):

| | |
|---|---|
| Travelled-distance accuracy | **91% to 93%** |
| Median moving speed vs reference | within **3.5%** |
| Standstill reading | exactly **0 km/h** |
| Per-frame tracking cost | **15.4 ms** median, 13x under the 200 ms budget |
| System test cases | **13 / 13 pass** |

On the route where jammed GPS overstated the distance by 33%, NavSight stayed within 7% of
the map-measured truth. Full method and evidence: `docs/project_book/`.

## How it works

| Stage | What happens |
|---|---|
| **Capture** | Rear camera at 640x480 plus accelerometer and gyroscope, into the native core over JNI |
| **Track** | Shi-Tomasi corners under pyramidal KLT optical flow; IMU preintegrated between frames |
| **Fuse** | Error-state EKF (15-DOF core plus camera-IMU time offset and mount rotation) with MSCKF updates |
| **Heading** | Gyro-primary Madgwick AHRS; the compass only corrects through a weak, disturbance-gated term |
| **Speed** | The road is a plane at a calibrated camera height, so optical flow over it yields metres per second directly. A two-sided vote/zero-witness scheme reads a true standstill as exactly zero; a bounded inertial bridge carries speed through visual dropouts |
| **Constrain** | Newson-Krumm HMM matcher, Viterbi-decoded, advances a graph-constrained "rail ball" along the road network |

The engine is deliberately GNSS-free. GPS, when available, supplies a single bootstrap fix
to georeference the session and is otherwise used only as a validation reference.

## Layout

| Path | Contents |
|---|---|
| `app/src/main/cpp/` | C++ VIO core: `Tracker`, `EKFState`, `IMUPreintegrator`, MSCKF updater |
| `app/src/main/java/com/example/navsight1/` | Kotlin app: ViewModel, sensor pipeline, JNI bridge, map-matching stack, Compose UI |
| `docs/project_book/` | The project book (sources, figures, build pipeline, PDF and DOCX) |
| `docs/adr/`, `docs/study/` | Architecture decisions and research notes |
| `presentation/` | Defense deck, presenter guides and diagram generators |
| `tests/cpp/` | Desktop replay harness (same engine, MSYS2/MinGW build) |
| `tests/sims/` | Recorded ride telemetry used by the harness and the validation figures |
| `scripts/` | Asset pipeline, telemetry analysis, validation-figure generation |
| `Final-Project/` | Signed SRS and SDD, technical documents, bug reports |
| `OpenCV-android-sdk/` | OpenCV 4.5.3, vendored so every checkout builds identically |

## Building

Requirements: Android Studio with the NDK and CMake, JDK 17, and an Android 7.0+ device
(minSdk 24, compileSdk 34). The OpenCV SDK is vendored, so no separate download is needed.

```bash
git clone https://github.com/morad-z/NavSight1.git
cd NavSight1
./gradlew assembleDebug          # produces the v1.0-osm debug build
./gradlew testDebugUnitTest      # Kotlin unit tests
adb install app/build/outputs/apk/debug/app-debug.apk
```

Grant camera and location permissions on first launch, mount the phone, set the camera
height in the calibration sheet, and hold still while calibration converges.

To rebuild the offline search index from fresh OpenStreetMap data:

```bash
python scripts/build_haifa_assets.py
```

## Replay harness

Every behaviour-affecting change is validated against recorded rides before it reaches a
device. The same native engine is compiled for the desktop under MSYS2/MinGW
(`tests/cpp/`) and re-runs a recorded session deterministically, so two engine versions can
be compared on byte-identical input. CI re-scores recorded fixtures on every push
(`.github/workflows/replay.yml`).

## Team

| | |
|---|---|
| Roey Ben Harush | Application and UI: Compose interface, camera pipeline, UX |
| Tamir Sobuh | Map matching: OSM data layer, matcher, routing |
| Morad Zubidat | Engine: VIO core, EKF and sensor fusion, calibration, integration testing |

Supervisors: Mr. Amit Dunsky and Dr. Dvir Ross, Shenkar College of Engineering.

## Scope

Out of scope by design: SLAM mapping and 3D reconstruction, iOS, any cloud backend, and
voice-guided turn-by-turn navigation. SLAM landmarks, loop closure and windowed bundle
adjustment were implemented and measured, then disabled once map matching proved to supply
the same drift correction at a fraction of the memory and compute. That history is in
chapter 9 of the project book.
