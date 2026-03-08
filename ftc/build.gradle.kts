plugins {
    id("com.android.library")
    id("org.jetbrains.dokka")
    id("maven-publish")
    kotlin("android")
}

android {
    namespace = "com.pedropathing.ftc"

    //noinspection GradleDependency
    compileSdk = 34 // this is 34 on purpose

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    publishing {
        singleVariant("release") {
            withSourcesJar()
        }
    }

    defaultConfig {
        minSdk = 21
    }
}

dependencies {
    compileOnly(libs.bundles.ftc)
    api(project(":core"))
    dokkaPlugin(libs.dokka.java.plugin)
}

val dokkaJar = tasks.register<Jar>("dokkaJar") {
    dependsOn(tasks.named("dokkaGenerate"))
    from(dokka.basePublicationsDirectory.dir("html"))
    archiveClassifier = "html-docs"
}


afterEvaluate {
    publishing {
        publications {
            create<MavenPublication>("mavenRelease") {
                from(components["release"])
                artifact(dokkaJar)

                groupId = "com.millburnx"
                artifactId = "pedropathing-ftc"
                version = "0.1.2"

                pom {
                    name.set("Pedro Pathing FTC")
                    description.set(
                        "A path follower designed to revolutionize autonomous pathing in robotics"
                    )
                    url.set("https://github.com/Pedro-Pathing/PedroPathing")
                }
            }
        }
    }
}