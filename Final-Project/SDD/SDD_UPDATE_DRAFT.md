# NavSight SDD - Section 4: Design

## 4. Design

### a. Data Design

#### i. Database Description
The NavSight system operates as a real-time, offline processing engine and does not utilize a traditional persistent relational database (SQL) for its core navigation loop.
*   **Transient Sensor Storage:** High-frequency data from the camera (30 FPS) and IMU (200Hz) is buffered in circular buffers within the Native Memory (RAM) to minimize latency. This data is discarded once processed.
*   **Session Persistence:** If required for post-session analysis, trajectory data (waypoints) can be serialized to local JSON/CSV files, but this is optional and not required for real-time operation.
*   **Configuration Management:** Application settings (e.g., permissions, user preferences) are managed via Android `SharedPreferences`.

#### ii. Global Data Structures
The system relies on optimized data structures defined in the Native C++ layer to handle the intense data throughput required for VIO.

**1. Sensor Frame Packet (`CamFrame`)**
Represents the fundamental unit of visual input.
*   `jlong timestamp`: Nanosecond-precision timestamp synchronized with the Android `SystemClock.elapsedRealtimeNanos()`.
*   `cv::Mat yuvMat`: Raw image data in YUV_420_888 format. This is efficiently converted to Grayscale for computer vision processing.
*   `int width, height`: Dimensions of the video feed.

**2. Inertial Data Packets (`AccelData`, `GyroData`)**
Structures for buffering high-frequency inertial measurements.
*   `long timestamp_ns`: Timestamp of the sensor event.
*   `float x, y, z`: 3-axis acceleration ($m/s^2$) or angular velocity ($rad/s$).

**3. Vision Output Packet (`VisionOutput`)**
The result of the Visual Odometry pipeline for a single frame.
*   `cv::Mat rotation`: A 3x3 rotation matrix representing the device's relative rotation since the previous frame.
*   `cv::Mat translation`: A 3x1 translation vector representing relative movement.
*   `float tracking_quality`: A confidence score (0.0 - 1.0) derived from the ratio of feature inliers.
*   `bool is_valid`: A flag indicating if the visual tracking was successful. `False` triggers Dead Reckoning fallback.

**4. VIO Data Transfer Object (`VioData`)**
The bridge object used to transport the final fused pose from the C++ engine to the Kotlin UI layer.
*   `double x, y, z`: Global position in meters (relative to start).
*   `double roll, pitch, yaw`: Global device orientation.
*   `float[] trackedPoints`: Array of 2D coordinates for visualizing tracked features on the UI.
*   `double estimatedScale`: The real-time computed scale factor, allowing conversion from visual units to meters.

### b. Component Design

The system architecture is centered around the **VIO Engine Layer**, implemented in C++ for performance.

#### i. Vision Processing Module (Visual Odometry)
This module is responsible for estimating device motion from camera images. It utilizes a feature-based approach:
1.  **Feature Detection:** Uses the **Shi-Tomasi** corner detector (`cv::goodFeaturesToTrack`) to identify up to 200 distinct, trackable points in the image.
2.  **Optical Flow:** Implements the **Lucas-Kanade (LK)** Pyramidal method (`cv::calcOpticalFlowPyrLK`) to track these features from the previous frame to the current frame.
3.  **Outlier Rejection:** Applies the **RANSAC** algorithm with an Essential Matrix model (`cv::findEssentialMat`).
    *   **Configuration:** High confidence (99.99%) and a strict threshold (0.5 pixels) are used to aggressively filter out moving objects (e.g., cars, pedestrians) and sensor noise.
4.  **Pose Recovery:** Decomposes the Essential Matrix into rotation ($R$) and translation ($t$) components.

#### ii. Automatic Scale Estimation & Gravity Alignment
A critical innovation in NavSight is solving the "Monocular Scale Ambiguity" problem (where a single camera cannot tell the difference between 1 meter and 10 meters).
*   **Gravity Initialization:** At startup, the system collects 20+ accelerometer samples to determine the gravity vector ($g$). This establishes the "Down" direction and aligns the coordinate system with the real world.
*   **Scale Fusion:** The system continuously compares the "visual distance" (in arbitrary units) with the "inertial distance" (in meters) derived from double-integrating accelerometer data.
    $$ Scale = \frac{\iint a(t) \, dt}{||t_{vision}||} $$
    This allows the system to output real-world coordinates without requiring pre-calibration or stereo cameras.

#### iii. Sensor Fusion & Dead Reckoning (DR)
The Fusion Engine combines the outputs of the Visual pipeline and the IMU:
*   **VIO Mode (Primary):** When visual quality is high, the system fuses the precise relative motion from the camera with the rotation from the gyroscope. The accelerometer is used to correct scale.
*   **Dead Reckoning Mode (Fallback):** If visual tracking fails (e.g., `VisionOutput.is_valid == false` due to low light or untextured walls), the system automatically degrades to DR mode. In this state, position is propagated solely by integrating IMU acceleration and velocity. This ensures navigation continuity until visual features are recovered.

### c. Interface Design (JNI Bridge)

The Native Interface Layer facilitates high-speed communication between the Android Runtime (ART) and the Native code. It exposes the following API:

*   `startVIO()` / `stopVIO()`: Lifecycle management for the background processing thread.
*   `processCameraFrame(byte[] data, ...)`: Asynchronous entry point for video frames. Returns the *latest available* global pose immediately to ensure the UI remains responsive (60 FPS) even if processing takes longer.
*   `processAccelerometer(x,y,z)` / `processGyroscope(x,y,z)`: High-frequency hooks for feeding the inertial buffers.
*   `resetVIO()`: Zeros the global pose and re-initializes the gravity alignment vector.

### d. Detailed Design Diagrams

*(Refer to the Appendices for the complete visual diagrams)*
*   **Class Diagram:** Illustrates the relationship between the Android Activity, the JNI Bridge, and the C++ `VisionModule`.
*   **Sequence Diagram:** Details the parallel execution of the UI thread (sensor input) and the VIO Background thread (processing).
*   **State Diagram:** Defines the system lifecycle, including the transitions between "Tracking VIO" and "Degraded DR" states.
*   **Activity Diagram:** Maps the logic flow of the VIO thread, including the decision logic for outlier rejection and fallback triggering.