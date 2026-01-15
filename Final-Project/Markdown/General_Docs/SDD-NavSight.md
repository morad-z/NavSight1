
# NavSight - Software Design Document

**Authors:**
* Roey Ben Harush, [ID REDACTED]
* Tamir Sobuh, [ID REDACTED]
* Morad Zubidat, [ID REDACTED]

**Supervisor:**
* Amit Dunsky

**Date:** 07/01/2026

---

## 1. Introduction

### a. System Overview
NavSight is a self-contained, real-time mobile navigation system designed to operate effectively in environments where Global Navigation Satellite Systems (GNSS) are unavailable or unreliable. The system leverages the built-in camera and inertial sensors of a standard smartphone to provide continuous position and orientation tracking. By employing a hybrid approach that fuses Visual-Inertial Odometry (VIO) with Dead Reckoning (DR), NavSight delivers a robust localization solution tailored for urban micromobility, such as electric scooters and pedestrians. All processing is performed locally on the device, ensuring full functionality without requiring an internet connection or external servers.

### b. Purpose
The primary purpose of NavSight is to overcome the inherent limitations of GNSS-based navigation in challenging environments like urban canyons, tunnels, and dense indoor spaces. The system provides a reliable, continuous, and accurate method for tracking a user's trajectory by analyzing visual information from the environment and motion data from the device's inertial sensors. This establishes a foundation for GNSS-independent navigation, enhancing safety and utility for a variety of mobile applications.

### c. Scope
The scope of this project includes the design, implementation, and testing of a hybrid visual-inertial navigation system on the Android platform.

**Key In-Scope Elements:**
*   **Visual Odometry (VO):** Real-time motion estimation derived from analyzing sequential camera frames.
*   **Dead Reckoning (DR):** Position estimation based on data from the smartphone's Inertial Measurement Unit (IMU), including the accelerometer and gyroscope.
*   **Sensor Fusion Logic:** A core component responsible for combining VO and DR data to produce a single, robust tracking estimate and manage transitions between tracking modes.
*   **Native Android Application:** A mobile application that provides a user interface for controlling tracking sessions and visualizing the computed path in real-time.
*   **Offline Operation:** The entire system is designed to run locally on the device without any dependency on network connectivity.

**Key Out-of-Scope Elements:**
*   3D environment reconstruction and Simultaneous Localization and Mapping (SLAM).
*   Cloud-based data storage, remote processing, or integration with external hardware sensors.
*   Integration with third-party mapping APIs for turn-by-turn navigation.
*   Voice-guided or audio-based navigation cues.

### d. Definitions and Acronyms
*   **VIO (Visual-Inertial Odometry):** A technique that fuses data from a camera and an IMU to estimate an object's motion.
*   **VO (Visual Odometry):** A technique for estimating camera movement by analyzing sequential images.
*   **DR (Dead Reckoning):** A process of calculating one's current position by using a previously determined position and advancing that position based on known or estimated speeds over elapsed time.
*   **IMU (Inertial Measurement Unit):** A sensor that combines an accelerometer and a gyroscope to measure motion, orientation, and force.
*   **GNSS (Global Navigation Satellite System):** A general term for satellite navigation systems that provide autonomous geo-spatial positioning (e.g., GPS, Galileo).
*   **JNI (Java Native Interface):** A programming framework that enables Java code running in a Java Virtual Machine (JVM) to call and be called by native applications and libraries written in other languages such as C++.

### e. Constraints
*   **Hardware Dependency:** System performance is directly dependent on the quality and precision of the smartphone's camera and IMU sensors, as well as its processing power.
*   **Environmental Limitations:** The reliability of visual tracking is constrained by environmental factors such as poor lighting, rapid changes in brightness, and motion blur.
*   **Real-Time Processing:** All computations must be performed with minimal latency (under 200ms per frame) to ensure a smooth real-time user experience.
*   **Power Consumption:** Continuous use of the camera and processor-intensive computations place a significant demand on the device's battery.
*   **Platform:** The system is developed exclusively for the Android mobile operating system (Android 10+).

---

## 2. System Architecture

### a. Architectural Description and Design: Roles, Activities and Data
The NavSight system is designed using a multi-layered architecture that separates concerns between user interface, application logic, and low-level sensor processing. This design ensures modularity, facilitates concurrent processing, and isolates performance-critical native code from the high-level application framework.

The primary architectural layers are:

*   **1. Presentation Layer (Android UI):**
    *   **Description:** This is the top-level layer that the user interacts with. It is a native Android application implemented in Kotlin.
    *   **Responsibilities:**
        *   Rendering the live camera feed and overlaying the generated navigation path.
        *   Displaying system status, performance metrics, and user feedback.
        *   Accepting user commands (e.g., start, stop, pause tracking).
        *   Managing Android-specific lifecycle events and user permissions for camera and sensor access.

*   **2. Application Logic Layer:**
    *   **Description:** This layer acts as an orchestrator, mediating between the Presentation Layer and the Native Processing Core.
    *   **Responsibilities:**
        *   Managing the overall system state (e.g., Idle, Calibrating, Tracking, Paused).
        *   Receiving sensor data from the Android framework (camera frames, IMU samples).
        *   Forwarding sensor data to the native layer for processing via the native interface.
        *   Receiving processed motion data from the native layer and providing it to the Presentation Layer for visualization.

*   **3. Native Interface Layer (JNI Bridge):**
    *   **Description:** A thin, highly-optimized layer that facilitates bidirectional communication between the Java/Kotlin environment and the C++ Native Processing Core.
    *   **Responsibilities:**
        *   Exposing native functions to the Application Logic Layer.
        *   Marshalling data between Java/Kotlin data types and C++ data structures (e.g., converting bitmap frames to processable matrices).
        *   Managing the lifecycle of native objects and ensuring thread-safe data exchange between the two environments.

*   **4. Native Processing Core (C++):**
    *   **Description:** The heart of the system, where all computationally intensive VIO algorithms are executed. This layer is implemented in C++ for maximum performance and direct memory management. It operates concurrently in a dedicated background thread to avoid blocking the UI.
    *   **Sub-components:**
        *   **Vision Processing Module:** Analyzes incoming camera frames to detect and track visual features. It computes the device's relative translation and rotation between frames (Visual Odometry).
        *   **Inertial Pre-integration Module:** Accumulates high-frequency IMU sensor readings (accelerometer and gyroscope) over time to produce integrated motion estimates (Dead Reckoning).
        *   **Sensor Fusion Module:** The core logic engine that intelligently fuses the output from the Vision Processing Module and the Inertial Pre-integration Module. It uses filtering techniques to minimize drift, handle sensor inaccuracies, and produce a unified, robust motion estimate. It is also responsible for detecting failures in the visual data (e.g., due to low light) and dynamically switching to an IMU-only DR mode.
        *   **Concurrency Control Module:** A set of synchronization primitives (mutexes, atomic variables, and condition variables) that ensures thread-safe access to shared data structures (e.g., sensor data queues, shared module pointers) across different processing threads.

### b. The Life Cycle of the System
The system transitions through a well-defined set of states, managed by the Application Logic Layer:

1.  **Initialization:** Upon application launch, the system requests necessary permissions (camera, motion sensors). Once granted, it initializes the native processing components and enters an idle state.
2.  **Calibration:** Before tracking begins, the user is prompted to hold the device steady. During this phase, the system establishes a stable baseline for the inertial sensors and an initial reference frame.
3.  **Active Tracking (VIO Fused):** After calibration, the system enters its primary operational mode. The camera and IMU data streams are continuously processed by the native core, which fuses them to produce a high-accuracy motion track.
4.  **Degraded Tracking (DR Mode):** If the visual data becomes unreliable (e.g., in low light or due to fast motion), the Sensor Fusion Module automatically transitions to a Dead Reckoning mode, relying solely on IMU data to continue tracking. The system continuously attempts to recover the visual feed.
5.  **Recovery:** When visual quality improves, the system re-establishes visual tracking and seamlessly transitions back to the fully fused VIO mode.
6.  **Paused:** The user can temporarily suspend the tracking session. In this state, sensor processing is halted to conserve battery, and the session timer is paused. The system can resume from this state.
7.  **Termination:** When the user stops the tracking session, the native processing thread is terminated, all resources are released, and a final summary of the session is displayed.

---

## 3. Literature Survey
A comprehensive literature survey was conducted to inform the system's design, focusing on established and state-of-the-art techniques in visual-inertial odometry. Key research areas included feature-based Monocular SLAM (e.g., ORB-SLAM), direct methods (e.g., LSD-SLAM), and foundational principles of visual odometry and sensor fusion. The articles referenced in the project's Software Requirements Specification (SRS), such as "Visual Odometry: Part I" by Scaramuzza & Fraundorfer and works from the RPG Zurich lab, served as primary sources for understanding the core algorithms and mathematical models underpinning the VIO and DR components implemented in this system.

---

## 4. Design

### a. Data Design

*   **i. Database Description:**
    The system is designed for complete offline operation and does not utilize a traditional persistent database (e.g., SQL). All session data is stored ephemerally in the device's memory during an active tracking session. Processed metadata, such as path summaries and performance logs, may be stored temporarily in local application files for evaluation but are not part of a structured database schema.

*   **ii. Global Data Structures Design:**
    The system relies on several key conceptual data structures that flow between its layers.
    *   **Sensor Frame Packet:** A composite data object containing a single video frame, its corresponding timestamp, and a collection of all IMU (accelerometer and gyroscope) samples that were recorded since the previous video frame. This structure serves as the fundamental unit of input for the Native Processing Core.
    *   **Motion State Vector:** A structure representing the estimated state of the device at a specific point in time. Its attributes include:
        *   3D Position (x, y, z)
        *   3D Orientation (quaternion or rotation matrix)
        *   3D Velocity
        *   Timestamp
    *   **Tracked Feature Set:** A collection of 2D points representing the visual features currently being tracked across frames. This data is primarily internal to the Vision Processing Module but is used to provide visual feedback on the UI.
    *   **Path Trajectory:** An ordered list of historical Motion State Vectors. This data structure represents the user's complete path and is used by the Presentation Layer to render the trajectory on the screen.

### b. Structural Design

*   **i. Class Diagram:**
    The following diagram illustrates the high-level structural components of the system and their relationships.

    ```mermaid
    classDiagram
      class PresentationLayer {
        +startTracking()
        +stopTracking()
        +renderPath(PathTrajectory)
        +displayStatus(SystemState)
      }
      class ApplicationLogicLayer {
        -currentState: SystemState
        +onCameraFrame(Frame)
        +onImuSample(Sample)
        +handleUserCommand()
      }
      class NativeInterfaceBridge {
        +processSensorFrame(SensorFramePacket) : MotionStateVector
        +startNativeCore()
        +stopNativeCore()
      }
      class NativeProcessingCore {
        -isRunning: atomic~bool~
        +runProcessingLoop()
      }
      class VisionProcessingModule {
        +calculatePose(Frame) : Pose
      }
      class InertialPreintegrationModule {
        +integrateReadings(IMUSamples) : Pose
      }
      class SensorFusionModule {
        +fuse(VisionPose, InertialPose) : MotionStateVector
      }
      class ConcurrencyControlModule {
         -vio_mutex: mutex
         -frame_queue_mutex: mutex
      }

      PresentationLayer --> ApplicationLogicLayer : issues commands
      ApplicationLogicLayer --> NativeInterfaceBridge : invokes native methods
      NativeInterfaceBridge --> NativeProcessingCore : manages lifecycle
      ApplicationLogicLayer ..> PresentationLayer : updates UI
      NativeProcessingCore o-- VisionProcessingModule
      NativeProcessingCore o-- InertialPreintegrationModule
      NativeProcessingCore o-- SensorFusionModule
      NativeProcessingCore o-- ConcurrencyControlModule
    ```

### c. Interactions Design

*   **i. Use Cases:**
    *   **UC-1: Track a Navigation Path:** The user initiates the application, calibrates the device, and starts a tracking session. The system records their movement via VIO and displays the path on the screen. The user successfully navigates their route and stops the session, after which a summary is displayed.
    *   **UC-2: Handle Degraded Visuals:** During a tracking session, the user enters an area with poor lighting. The system detects the loss of visual features, automatically switches to DR mode using only inertial data, and notifies the user of the degraded tracking status. When lighting conditions improve, the system automatically resumes full VIO tracking.
    *   **UC-3: Pause and Resume a Session:** In the middle of a route, the user pauses the tracking session. The system suspends all sensor processing. The user later resumes the session, and the system continues tracking from the last known position.

*   **ii. Sequence Diagram:**
    This diagram shows the sequence of interactions for the primary use case: processing a single sensor frame packet.

    ```mermaid
    sequenceDiagram
      participant AppUI as PresentationLayer
      participant AppLogic as ApplicationLogicLayer
      participant JNI as NativeInterfaceBridge
      participant NativeCore as NativeProcessingCore

      AppLogic->>AppLogic: onSensorDataReceived(data)
      AppLogic->>JNI: processSensorFrame(sensorPacket)
      activate JNI

      JNI->>NativeCore: pushToQueue(sensorPacket)
      activate NativeCore
      Note right of NativeCore: Native thread processes the queue
      NativeCore-->>JNI: return immediateAcks
      deactivate NativeCore
      JNI-->>AppLogic: return
      deactivate JNI

      loop Asynchronous Processing
          NativeCore->>NativeCore: processFrame(packet)
          NativeCore-->>JNI: postResult(motionStateVector)
      end

      JNI-->>AppLogic: onNativeResult(motionStateVector)
      activate AppLogic
      AppLogic->>AppUI: renderPath(newPoint)
      deactivate AppLogic
    ```

*   **iii. Activity Diagram:**
    This diagram models the system's operational states, reflecting the lifecycle described in Section 2b.

    ```mermaid
    graph TD
        A[App Launched] --> B{Permissions?};
        B -- Granted --> C[Idle];
        B -- Denied --> B;
        C -- User presses Start --> D[Calibrating];
        D -- Calibration OK --> E[Tracking];
        E -- Visuals OK --> F[Fused VIO Mode];
        F -- Low Light/Blur --> G[Degraded DR Mode];
        G -- Visuals Recover --> F;
        E -- User presses Pause --> H[Paused];
        H -- User presses Resume --> E;
        E -- User presses Stop --> I[Session Ended];
        I --> J[Show Summary];
        J -- Dismiss --> C;
    ```

### d. Software Architecture Pattern

*   **i. Three-tier / Layered Architecture:**
    The system's design most closely follows a **Layered Architecture**.
    *   **Presentation Tier:** The Android UI components (`MainActivity.kt`).
    *   **Logic Tier:** A combination of the `ApplicationLogicLayer` in Kotlin and the C++ `NativeProcessingCore`. This tier contains all the business logic and core algorithms.
    *   **Data Tier:** This tier is abstract and represented by the transient flow of sensor data from the hardware and the in-memory data structures (`MotionStateVector`, `PathTrajectory`), rather than a persistent data store.

*   **ii. MVC / MVVM Structure:**
    Within the **Presentation Layer**, the application employs patterns similar to **Model-View-ViewModel (MVVM)**, a common standard in modern Android development.
    *   **Model:** The `VioData` data class and other state-holding structures act as the model.
    *   **View:** The Jetpack Compose UI elements that render the camera feed, path, and debug information.
    *   **ViewModel:** The `MainActivity` and associated state management logic serve a ViewModel-like role, holding UI state and exposing data from the lower layers to the view.

    This MVVM pattern is contained within the top layer, while the overall system adheres to the broader layered architecture described above.

### e. Testing Platform
The testing strategy is multi-faceted, encompassing different levels of the architecture:

*   **Unit Testing:** Native C++ modules for vision processing and inertial pre-integration are tested in isolation using a testing framework like GoogleTest. This validates the correctness of the core algorithms.
*   **Integration Testing:** The JNI boundary is tested to ensure correct data marshalling, thread safety, and lifecycle management of native objects.
*   **Application-level Testing:** The Android application is tested using tools like Espresso (for UI interaction) and JUnit (for application logic).
*   **Field Testing:** End-to-end system validation is performed through real-world field tests. This involves using the application on physical mid-range Android devices while navigating diverse urban environments (e.g., bright sunlight, shadows, tunnels) to assess performance, accuracy, and reliability under operational conditions.

---

## 5-8. Appendices
Appendices for POC discussion, Team Roles, Schedule, and System Screens are to be attached separately as per the project requirements. They will detail the proof-of-concept validation, assignment of responsibilities within the project team, a Gantt chart of the development timeline, and visual mockups or screenshots of the application's user interface.
