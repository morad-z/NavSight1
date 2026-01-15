# SDD Refinement & VIO Architecture Summary
**Date:** 2026-01-15
**Project:** NavSight - Visual-Inertial Odometry System
**Role:** Documentation Specialist

---

## 1. Executive Summary
Today's session focused on elevating the **Software Design Document (SDD)** from a high-level overview to a comprehensive technical specification suitable for engineering implementation. We also finalized the visual assets (UML diagrams) to meet professional standards.

## 2. Technical Enhancements

### A. Algorithm Documentation
We explicitly documented the core mathematical models driving the VIO engine. This demystifies the "black box" nature of the native library.

*   **Feature Detection (Shi-Tomasi):**
    *   **Logic:** Uses eigenvalue analysis of the structure tensor to identify corner points invariant to rotation.
    *   **Constraint:** Limits tracking to 200 features with minimum Euclidean distance separation to ensure uniform image coverage.

*   **Optical Flow (Lucas-Kanade):**
    *   **Mechanism:** Solves $I(x,y,t) = I(x+dx, y+dy, t+dt)$ using a pyramidal approach (coarse-to-fine) to handle large inter-frame displacements.
    *   **Output:** Vector field representing pixel motion, which correlates to camera translation and rotation.

*   **Manifold Preintegration (Forster et al.):**
    *   **Problem:** Integrating IMU data (200Hz) in the global frame requires re-computation every time the global pose changes.
    *   **Solution:** We integrate motion *relative* to the previous keyframe.
    *   **Rotation:** Accumulated on the SO(3) manifold: $\Delta R_{ij} = \prod Exp(\omega_k \Delta t)$.
    *   **Impact:** Reduces computational load by ~10x compared to naive integration.

*   **Automatic Scale Estimation:**
    *   **Challenge:** Monocular vision is scale-ambiguous (1m vs 10m movement looks identical).
    *   **Solution:** We fuse the double-integrated accelerometer displacement ($d_{imu} \approx 0.5 a t^2$) with the unitless visual displacement to derive a metric scale factor ($s = d_{imu} / d_{vis}$).

### B. Architecture Refinement
We clarified the boundaries between the **Managed (Kotlin)** and **Unmanaged (C++)** layers.

*   **Strict Three-Tier Model:**
    1.  **Presentation (UI):** Jetpack Compose, strictly for rendering.
    2.  **Logic (Orchestrator):** Sensor lifecycle, permissions, and JNI bridging.
    3.  **Data/Compute (Native):** The `VisionModule` and `IMUPreintegrator` handle all heavy math.

## 3. Visual Assets (UML)
We replaced generic diagrams with strict **UML 2.5.1** compliant diagrams using a professional "Enterprise" style (Black & White).

*   **Architecture Diagram:** Component-style, clearly showing the JNI boundary.
*   **Class Diagram:** Includes visibility modifiers (`+`, `-`), data types (`cv::Mat`, `Vec3d`), and relationships (`owns`, `delegates`).
*   **Sequence Diagram:** Models the main processing loop, showing the asynchronous nature of the 200Hz IMU stream vs. the 30Hz Camera stream.
*   **Activity Diagram:** A vertical state machine modeling the system lifecycle: `Idle -> Initialization (Gravity Align) -> Active Tracking (VIO/DR) -> Stop`.

## 4. Deliverables
The following artifacts were generated and are ready for submission:

1.  **`NavSight_SDD_Final_v4.docx`**: The complete, formatted design document.
2.  **`assets/*.png`**: High-resolution UML diagrams.
3.  **`generate_diagrams.py`**: A reproducible script for regenerating diagrams if the design changes.

---

**Next Steps:**
- [ ] Review the DOCX file for final formatting checks.
- [ ] Validate that the referenced "Schedule" (Appendix C) matches the project's actual Gantt chart.
- [ ] Ensure the "System Screens" (Appendix D) are populated with the latest screenshots from the `MainActivity` UI.
