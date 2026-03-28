cmake_minimum_required(VERSION 3.22.1)

project("navsight")

# נתיב יחסי ל-OpenCV SDK
set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../OpenCV-android-sdk/sdk/native/jni")

# מחקנו את set(OpenCV_SHARED ON) - אנחנו נותנים לו להשתמש בסטטי
# כי ה-Gradle עכשיו מוגדר ל-c++_static, אז זה יעבוד חלק.

find_package(OpenCV REQUIRED)

add_library(
        navsight
        SHARED
        src/main/cpp/native-lib.cpp
        src/main/cpp/VisionModule.cpp
        src/main/cpp/IMUPreintegrator.cpp
)

find_library(
        log-lib
        log
)

# שים לב לסדר הקישור: לפעמים סדר הלינקינג משנה בסטטי.
# שמנו את OpenCV בסוף כדי שהוא ימצא את הסמלים שהוא צריך ב-android/log.
target_link_libraries(
        navsight
        android
        ${log-lib}
        ${OpenCV_LIBS}
)