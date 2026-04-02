plugins {
    id("com.android.library")
}

repositories {
    mavenCentral()
}

android {
    namespace = "com.pedropathing.revhub"
    compileSdk = 36
    defaultConfig {
        minSdk = 24
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
}

dependencies {
    implementation(project(":core"))
}