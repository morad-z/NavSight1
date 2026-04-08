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
                arguments.add("-DANDROID_STL=c++_static")
            }
        }
    }

    // Don't compress TFLite models — they need memory-mapping for performance
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

    // Google Maps
    implementation(libs.google.maps.services)

    // Maps Compose
    implementation(libs.maps.compose)

    // Roads API client library
    implementation(libs.maps.services.client)

    // Places API for destination search autocomplete
    implementation(libs.places.client)

    // SLF4J for Roads API logging
    implementation(libs.slf4j.simple)

    // Location services
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
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)
    androidTestImplementation(platform(libs.androidx.compose.bom))
    androidTestImplementation(libs.androidx.compose.ui.test.junit4)
    debugImplementation(libs.androidx.compose.ui.tooling)
    debugImplementation(libs.androidx.compose.ui.test.manifest)
}
