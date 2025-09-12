import org.gradle.api.tasks.javadoc.Javadoc
import org.gradle.jvm.tasks.Jar
import org.gradle.external.javadoc.JavadocMemberLevel

plugins {
    id("com.android.library")
    id("maven-publish")
    id("org.jetbrains.dokka")
}

android {
    namespace = "com.pedropathing.ftc"
    compileSdk = 30

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    publishing {
        singleVariant("release") {
            withSourcesJar()
            withJavadocJar()
        }
    }

    sourceSets {
        getByName("main") {
            java.srcDirs("src/main/kotlin")
        }
    }

    defaultConfig {
        minSdk = 21
    }
}

android.libraryVariants.configureEach {
    val newName = "generate${name.replaceFirstChar { it.uppercase() }}Javadoc"
    val newJavadocTask = tasks.register<Javadoc>(newName) {
        group = "Documentation"
        description = "Generates Javadoc for $name"
        source = javaCompileProvider.get().source
        val androidJar =
            "${android.sdkDirectory}/platforms/${android.compileSdkVersion}/android.jar"
        classpath = files(getCompileClasspath(null)) + files(androidJar)
        with(options as StandardJavadocDocletOptions) {
            memberLevel = JavadocMemberLevel.PROTECTED
            links("https://developer.android.com/reference")
            encoding = "UTF-8"
        }
    }

    if (name == "release") {
        tasks.named<Jar>("javadocJar") {
            dependsOn(newJavadocTask)
            from(newJavadocTask.map { it.destinationDir!! })
        }
    }
}

tasks.register<Jar>("javadocJar") {
    archiveClassifier = "javadoc"
}

tasks.register<Jar>("sourcesJar") {
    from(android.sourceSets["main"].java.srcDirs)
    archiveClassifier = "sources"
}

dependencies {
    compileOnly(libs.bundles.ftc)
    api(project(":core"))
}

publishing {
    publications {
        register<MavenPublication>("release") {
            groupId = "com.pedropathing"
            artifactId = "ftc"
            version = "2.0.1"

            afterEvaluate {
                from(components["release"])
            }
        }
    }

    repositories {
        maven {
            name = "publishing"
            url = uri("../../maven.pedropathing.com")
        }
    }
}