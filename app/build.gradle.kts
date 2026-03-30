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

        val apiKey = localProps.getProperty("GOOGLE_MAPS_API_KEY")
            ?: project.findProperty("GOOGLE_MAPS_API_KEY") as String? ?: ""

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
    }

    externalNativeBuild {
        cmake {
            path = file("CMakeLists.txt")
        }
    }
    
}

dependencies {
    implementation(project(":sdk"))

    // Core dependencies
    implementation("androidx.core:core-ktx:1.12.0")
    implementation("androidx.activity:activity-ktx:1.8.2")
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.6.2")
    implementation("androidx.lifecycle:lifecycle-viewmodel-compose:2.6.2")
    implementation("androidx.activity:activity-compose:1.8.2")

    // Jetpack Compose
    implementation(platform("androidx.compose:compose-bom:2023.08.00"))
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.ui:ui-graphics")
    implementation("androidx.compose.ui:ui-tooling-preview")
    implementation("androidx.compose.material3:material3")

    // Google Maps
    implementation("com.google.android.gms:play-services-maps:18.2.0")

    // Maps Compose
    implementation("com.google.maps.android:maps-compose:2.11.4")

    // Roads API client library
    implementation("com.google.maps:google-maps-services:2.2.0")

    // Places API for destination search autocomplete
    implementation("com.google.android.libraries.places:places:3.3.0")

    // SLF4J for Roads API logging
    implementation("org.slf4j:slf4j-simple:1.7.36")

    // Location services (kept)
    implementation("com.google.android.gms:play-services-location:21.0.1")
    implementation("com.google.android.gms:play-services-tasks:18.1.0")

    // Permissions
    implementation("com.google.accompanist:accompanist-permissions:0.32.0")

    // Camera
    implementation("com.otaliastudios:cameraview:2.7.2")

    // Testing
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
    androidTestImplementation(platform("androidx.compose:compose-bom:2023.08.00"))
    androidTestImplementation("androidx.compose.ui:ui-test-junit4")
    debugImplementation("androidx.compose.ui:ui-tooling")
    debugImplementation("androidx.compose.ui:ui-test-manifest")
}
