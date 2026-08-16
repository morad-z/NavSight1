>_SUPERSEDED: the deck uses the single full-system diagram diagrams/pro/png/00-system-architecture-full.png; this sub-diagram is reference-only._

# NavSight — HMM Map Matching (Newson & Krumm, Viterbi)

```mermaid
flowchart TB
    POS["Estimated position + heading<br/>(from EKF)"]

    subgraph STATES["Candidate road segments = HMM states (within 30 m)"]
        S1["segment A"]
        S2["segment B"]
        S3["segment C"]
    end

    EMIT["Emission<br/>log p_e = -d_perp^2 / (2 * sigma_z^2)<br/>sigma_z = 20 m"]
    TRANS["Transition<br/>log p_t = -|d_gc - d_route|/beta + b_way + b_rail<br/>beta = 6 · b_way = 0.7 · b_rail = 1.2"]
    VIT["Viterbi decode<br/>argmax sum (log p_e + log p_t)"]
    STEER["Junction steering<br/>branch by gyro-relative heading offset"]
    BALL["Graph-rail ball<br/>recovery bounded 25-60 m @ conf >= 0.55"]

    POS --> STATES
    STATES --> EMIT
    STATES --> TRANS
    EMIT --> VIT
    TRANS --> VIT
    VIT --> STEER
    STEER --> BALL
```

**Presenter caption:** Candidate road segments within 30 m are HMM states. Emission scores perpendicular distance (Gaussian, sigma_z = 20 m); transition rewards travel-distance consistency plus same-way (0.7) and rail (1.2) bonuses. Viterbi decodes the most likely road path, and at junctions the gyro-relative heading offset picks the branch. Wrong-fork recovery is bounded to 25-60 m at confidence >= 0.55, so the ball can never teleport across town.
