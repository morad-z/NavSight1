pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
        // Add JitPack as a fallback repository
        maven { url = uri("https://jitpack.io") }
    }
}

rootProject.name = "NavSight1"
include(":app")
include(":sdk")
project(":sdk").projectDir = File(rootDir, "OpenCV-android-sdk/sdk/")