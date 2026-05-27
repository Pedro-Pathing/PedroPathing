plugins {
    id("java-library")
    id("io.deepmedia.tools.deployer")
    id("org.jetbrains.dokka")
    id("io.freefair.lombok")
    id("io.freefair.aspectj.post-compile-weaving")
    id("com.diffplug.spotless")
    id("net.ltgt.errorprone")
}

dependencies {
    dokkaPlugin(libs.dokka.java.plugin)
    implementation(libs.aspectj.rt)
    aspect(libs.aspectj.tools)
    errorprone(libs.error.prone.core)
    testImplementation("org.junit.jupiter:junit-jupiter-api:5.9.3")
    testRuntimeOnly("org.junit.jupiter:junit-jupiter-engine:5.9.3")
    testImplementation("org.junit.platform:junit-platform-launcher:1.9.3")
    testRuntimeOnly("org.junit.platform:junit-platform-engine:1.9.3")
    testImplementation("org.assertj:assertj-core:3.24.2")
}

java {
    sourceCompatibility = JavaVersion.VERSION_1_8
    targetCompatibility = JavaVersion.VERSION_1_8
}

tasks.test {
    useJUnitPlatform()
}

val dokkaJar =
    tasks.register<Jar>("dokkaJar") {
        description = "Generates a Dokka Jar"
        dependsOn(tasks.named("dokkaGenerate"))
        from(dokka.basePublicationsDirectory.dir("html"))
        archiveClassifier = "html-docs"
    }

deployer {
    projectInfo {
        name = "Pedro Pathing Core"
        description = "A path follower designed to revolutionize autonomous pathing in robotics"
        url = "https://pedropathing.com"
        scm {
            fromGithub("Pedro-Pathing", "PedroPathing")
        }
        license("BSD 3-Clause License", "https://opensource.org/licenses/BSD-3-Clause")

        developer("Baron Henderson", "baron@pedropathing.com")
        developer("Havish Sripada", "havish@pedropathing.com")
        developer("Davis Luxenberg", "davis@pedropathing.com")
    }

    content {
        component {
            fromJava()
            javaSources()
            docs(dokkaJar)
        }
    }

    if (System.getenv("PUBLISH_PEDRO") == "yes please") {
        signing {
            key = secret("MVN_GPG_KEY")
            password = secret("MVN_GPG_PASSWORD")
        }

        centralPortalSpec {
            auth {
                user = secret("SONATYPE_USERNAME")
                password = secret("SONATYPE_PASSWORD")
            }
            allowMavenCentralSync = false
        }

        nexusSpec("snapshot") {
            repositoryUrl = "https://central.sonatype.com/repository/maven-snapshots/"
            auth {
                user = secret("SONATYPE_USERNAME")
                password = secret("SONATYPE_PASSWORD")
            }
        }
    }

    localSpec()
}

spotless {
    java {
        target("src/**/*.java")

        palantirJavaFormat()
        removeUnusedImports()
        trimTrailingWhitespace()
        endWithNewline()

        licenseHeaderFile(rootProject.file("notice.txt"))
    }

    kotlinGradle {
        ktlint("1.2.1")
        target("*.gradle.kts")
    }

    format("misc") {
        target("*.md", "*.yaml", "*.yml", "*.json", ".gitignore")
        trimTrailingWhitespace()
        endWithNewline()
    }
}
