# Slide 22: Future Work

**Section:** Impact & Closing · **Slide:** 22 of 23 · **Estimated Time:** 1 minute

## On-Slide Content
- **Camera exposure cap** for low-light speed — bound the next lever after the IPM under-read/standstill fix.
- **Learned-inertial velocity** — a RNIN-VIO TFLite path (reusing our existing on-device inference seam) to strengthen the inertial/accel bridge.
- **Broaden OSM regions** — bundle more areas beyond the Haifa assets.
- **Per-mode scale (K) for more vehicles** — add calibrated K slots beyond walk/run/vehicle (e.g. measured scooter K).
- All grounded in the current architecture — each item adds a model on an existing path or extends an existing asset.

## Talking Points (what the presenter SAYS)
- Our future work is deliberately incremental — none of it requires re-architecting, because the seams are already in place.
- The most immediate lever is the camera exposure cap: our IPM root-cause analysis showed motion blur and starved road features hurt the near band in low light, and we already identified bounding exposure as the next improvement after the standstill and under-read fixes.
- The biggest capability bet is a learned-inertial velocity model — RNIN-VIO style — delivered as a TFLite path that reuses our existing on-device inference seam, which would make the inertial/accel bridge stronger when vision is degraded.
- Broadening map coverage is bundling more OSM road-matching assets, and supporting more vehicles is adding calibrated per-mode K slots — both extend existing mechanisms, not a rewrite.

## Potential Questions (Defense)
**Q:** Is learned-inertial velocity realistic on this hardware?
**A:** Yes — we treat it as a TFLite path that reuses an existing, proven on-device inference seam rather than introducing a new runtime, so the inertial/accel bridge gets stronger when vision is degraded. We have also benchmarked heavier on-device models against the Mali-G78 budget (a DA3/V2 INT8 bench cost 722 ms), so we know how to keep model choices honest to the hardware.

**Q:** How hard is it to add a new region or a new vehicle?
**A:** A new region is additional bundled OSM road-matching assets, the same form as the Haifa pack. A new vehicle is a new calibrated per-mode K slot alongside the existing walk/run/vehicle slots — for example a measured scooter K from a stop/brake calibration. Both extend existing mechanisms rather than changing the pipeline.

## Speaker Notes
- Frame every item as "the architecture already supports this" — the credibility of this slide is that nothing here is speculative re-design.
- Tie exposure cap back to the IPM story from the challenges slide — it is explicitly named as "the next lever."
- Note the DA3/V2 INT8 datapoint (722 ms vs budget) to show the team makes hardware-grounded model decisions, which de-risks the learned-inertial proposal.
