package com.example.navsight1

/**
 * An immutable, fully-built road region loaded on demand around the user's
 * location (RoadRegionManager). tileLat/tileLng identify the quantized tile this
 * region was fetched for (drives the trigger comparison). Published via a single
 * @Volatile write after both stores are fully built, so readers never see a
 * half-built region.
 */
internal class DynamicRoadRegion(
    val segments: SegmentStore,
    val graph: GraphStore,
    val bbox: BBoxDeg,
    val tileLat: Int,
    val tileLng: Int,
    val builtAtMs: Long,
    // Heading-aware matcher (§0.7): roundabouts in this region. Empty when none.
    val roundabouts: RoundaboutIndex = RoundaboutIndex(emptyList())
)
