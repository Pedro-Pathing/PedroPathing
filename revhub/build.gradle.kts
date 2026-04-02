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
    publishing {
        singleVariant("release") {
            withSourcesJar()
        }
    }
}

dependencies {
    compileOnly(libs.bundles.ftc)
    api(project(":core"))
}

