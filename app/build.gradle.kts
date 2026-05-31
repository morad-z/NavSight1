import java.util.Properties

plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.kotlin.compose)
}

val localProps = Properties()
val localPropsFile = rootProject.file("local.properties")
if (localPropsFile.exists()) localProps.load(localPropsFile.inputStream())

android {
    namespace = "com.example.navsight1"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.example.navsight1"
        minSdk = 24
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"

        // Load API keys from local.properties only (never hardcode in source)
        val apiKey = localProps.getProperty("GOOGLE_MAPS_API_KEY") ?: ""
        
        manifestPlaceholders["GOOGLE_MAPS_API_KEY"] = apiKey
        buildConfigField("String", "GOOGLE_MAPS_API_KEY", "\"$apiKey\"")

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"

        externalNativeBuild {
            cmake {
                cppFlags.add("-std=c++17")
                arguments.add("-DANDROID_STL=c++_shared")
            }
        }
    }

    // Don't compress TFLite models — they need memory-mapping for performance.
    // NOTE: the Haifa OSM blobs (assets/osm/haifa/*.bin) are intentionally LEFT
    // COMPRESSED — OsmDataLayer reads them fully into memory via assets.open()
    // (transparent inflate, a few ms at startup), so AAPT deflate saves ~6 MB of
    // APK with no runtime cost. (Add "bin" to noCompress only if we switch to mmap.)
    androidResources {
        noCompress += "tflite"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
        debug {
            // OSM-migration coexistence: install this branch's debug build ALONGSIDE
            // the speed-branch app (com.example.navsight1) instead of overwriting it.
            // The suffix gives this build its own package (com.example.navsight1.osm),
            // its own data/prefs, and its own launcher entry. The debug source set
            // (app/src/debug/res) renames it to "NavSight OSM" so the two are
            // distinguishable in the launcher. Remove this block before merging to
            // the speed branch (it's debug-only and harmless, but the merged trunk
            // should ship a single applicationId).
            applicationIdSuffix = ".osm"
            versionNameSuffix = "-osm"
        }
    }
    
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
    
    kotlinOptions {
        jvmTarget = "11"
    }
    
    buildFeatures {
        compose = true
        buildConfig = true
        prefab = true
        mlModelBinding = false
    }

    externalNativeBuild {
        cmake {
            path = file("CMakeLists.txt")
        }
    }
    
    // Disable lint entirely (has unresolved Compose bugs)
    lint {
        disable += listOf(
            "MutableCollectionMutableState",
            "AutoboxingStateCreation"
        )
    }
    
}

dependencies {
    implementation(project(":sdk"))

    // Core dependencies
    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.activity.ktx)
    implementation(libs.androidx.lifecycle.runtime.ktx)
    implementation(libs.androidx.lifecycle.viewmodel.compose)
    implementation(libs.androidx.activity.compose)
    implementation("androidx.annotation:annotation:1.7.1")

    // Jetpack Compose
    implementation(platform(libs.androidx.compose.bom))
    implementation(libs.androidx.compose.ui)
    implementation(libs.androidx.compose.ui.graphics)
    implementation(libs.androidx.compose.ui.tooling.preview)
    implementation(libs.androidx.compose.material3)

    // OSM migration 2026-05-30 (§8M Build changes): removed paid Google client
    // libs — google-maps-services (Roads), places-client (Places), slf4j-simple
    // (Roads logging). Routing/snapping/search now run offline against the local
    // Haifa OSM data layer. The Google DISPLAY SDK is KEPT (hybrid v1).

    // Maps Compose (display)
    implementation(libs.maps.compose)

    // Google Maps display SDK (play-services-maps) — the GoogleMap composable
    implementation(libs.maps.services.client)

    // Location services (one bootstrap GPS fix → setSessionAnchor, ADR-004)
    implementation(libs.google.location.services)
    implementation(libs.google.tasks)

    // Permissions
    implementation(libs.accompanist.permissions)

    // CameraX (replaces CameraView — zero-copy frame delivery via ImageAnalysis)
    val cameraxVersion = "1.3.4"
    implementation("androidx.camera:camera-core:$cameraxVersion")
    implementation("androidx.camera:camera-camera2:$cameraxVersion")
    implementation("androidx.camera:camera-lifecycle:$cameraxVersion")
    implementation("androidx.camera:camera-view:$cameraxVersion")

    // TensorFlow Lite
    implementation("org.tensorflow:tensorflow-lite:2.14.0")
    implementation("org.tensorflow:tensorflow-lite-gpu:2.14.0")
    implementation("org.tensorflow:tensorflow-lite-gpu-api:2.14.0")
    implementation("org.tensorflow:tensorflow-lite-support:0.4.4")

    // Testing
    testImplementation(libs.junit)
    // Real org.json for JVM unit tests (Android's android.jar ships only stubs that
    // throw "not mocked"). Lets DynamicRoadsParseTest exercise RoadsJsonParser.
    testImplementation("org.json:json:20240303")
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)
    androidTestImplementation(platform(libs.androidx.compose.bom))
    androidTestImplementation(libs.androidx.compose.ui.test.junit4)
    debugImplementation(libs.androidx.compose.ui.tooling)
    debugImplementation(libs.androidx.compose.ui.test.manifest)

    implementation("androidx.compose.material:material-icons-extended")
}
