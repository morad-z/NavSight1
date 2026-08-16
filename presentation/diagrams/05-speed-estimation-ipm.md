# NavSight — Speed Estimation via Inverse Perspective Mapping (IPM)

>_SUPERSEDED: the deck uses the single full-system diagram diagrams/pro/png/00-system-architecture-full.png; this sub-diagram is reference-only._

```mermaid
flowchart TB
    RAY["Camera ray r_i per road pixel<br/>(640x480, fx ~ 451)"]
    PLANE["Ground plane at mount height<br/>h = 1.05 m below camera"]
    Z["Depth Z_i = -h / (n_hat . r_i)"]
    A["Flow-per-speed a_i = (u_fwd - r_i*u_fwd_z)/Z_i"]
    VI["Per-point speed<br/>v_i = -(f_i . a_i)/(a_i . a_i) * (1/dt)"]
    FLOOR["Noise floor sigma_v_i = (sigma_px/fx)/(|a_i|*dt)<br/>sigma_px = 0.5 px"]

    subgraph TAX["Two-sided vote taxonomy"]
        VOTE["VOTE: v_i > 3*sigma AND forward-coherent<br/>(cos theta < -1/sqrt2, 45 deg cone)"]
        ZERO["ZERO-WITNESS: |v_i| < 3*sigma<br/>AND floor <= 1 m/s"]
    end

    DEC{"Resolve"}
    MED["median speed<br/>(>= 5 votes)"]
    LOCK["EXACT 0 km/h<br/>(>= 5 zero-witnesses)"]
    BRIDGE["Inertial/accel bridge<br/>predict v += a_fwd*dt each frame;<br/>EMA-correct toward vote-median;<br/>trusted ~6 s, then decay (alpha = 0.15)"]

    RAY --> PLANE --> Z --> A --> VI
    FLOOR --> VOTE
    FLOOR --> ZERO
    VI --> VOTE
    VI --> ZERO
    VOTE --> DEC
    ZERO --> DEC
    DEC -->|>=5 votes| MED
    DEC -->|>=5 zero-witnesses| LOCK
    DEC -->|else| BRIDGE
```

**Presenter caption:** The road is modelled as a plane at calibrated height h = 1.05 m, so each road pixel's ray gives a metric depth and a least-squares per-point speed from measured optical flow. A two-sided taxonomy classifies each point: a forward-coherent point above 3-sigma VOTES, a point at the noise floor is a ZERO-WITNESS. Five or more votes give the median speed; five or more zero-witnesses force an exact 0 km/h standstill lock; otherwise the inertial/accel bridge predicts speed by integrating forward acceleration each frame and EMA-corrects toward the vote-median, carrying speed for up to ~6 s before decaying (alpha = 0.15 is the post-budget decay rate, not the correction gain). The zero-witness branch is why standstill reads exactly 0 instead of rectifying noise into phantom speed.
