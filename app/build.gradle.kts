plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
}

android {
    namespace = "com.android.msckfs"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.android.msckfs"
        minSdk = 33
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
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
        viewBinding = true
    }

}

dependencies {

    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.appcompat)
    implementation(libs.material)
    implementation(libs.androidx.activity)
    implementation(libs.androidx.constraintlayout)
    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)

    // https://mvnrepository.com/artifact/org.boofcv/boofcv-core
    implementation(libs.boofcv.core)
    // https://mvnrepository.com/artifact/org.boofcv/boofcv-android
    implementation(libs.boofcv.boofcv.android)


    // CameraX core library using the camera2 implementation
    val camerax_version = "1.5.0-beta01"
    // The following line is optional, as the core library is included indirectly by camera-camera2
    implementation(libs.androidx.camera.core)
    implementation(libs.androidx.camera.camera2)
    // If you want to additionally use the CameraX Lifecycle library
    implementation(libs.androidx.camera.lifecycle)
    // If you want to additionally use the CameraX VideoCapture library
    implementation(libs.androidx.camera.video)
    // If you want to additionally use the CameraX View class
    implementation(libs.androidx.camera.view)
    // If you want to additionally add CameraX ML Kit Vision Integration
    implementation(libs.androidx.camera.mlkit.vision)
    // If you want to additionally use the CameraX Extensions library
    implementation(libs.androidx.camera.extensions)
    implementation(libs.androidx.camera.effects)

    // https://mvnrepository.com/artifact/org.ojalgo/ojalgo
    // implementation(libs.ojalgo)
    // https://mvnrepository.com/artifact/org.ejml/ejml-core
    implementation("org.ejml:ejml-core:0.44.0")

    // implementation(libs.commons.geometry.euclidean) // Quaternions
    // https://mvnrepository.com/artifact/org.apache.commons/commons-collections4
    implementation(libs.commons.collections4)
}