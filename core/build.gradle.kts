plugins {
    id("java-library")
    id("io.deepmedia.tools.deployer")
    id("org.jetbrains.dokka")
}

dependencies {
    compileOnly(libs.annotations)
    dokkaPlugin(libs.dokka.java.plugin)
}

java {
    sourceCompatibility = JavaVersion.VERSION_1_8
    targetCompatibility = JavaVersion.VERSION_1_8
}

val dokkaJar = tasks.register<Jar>("dokkaJar") {
    dependsOn(tasks.named("dokkaGenerate"))
    from(dokka.basePublicationsDirectory.dir("html"))
    archiveClassifier = "html-docs"
}

deployer {
    projectInfo {
        name = "Pedro Pathing Core"
        description = "A path follower designed to revolutionize autonomous pathing in robotics"
        url = "https://github.com/Pedro-Pathing/PedroPathing"
        scm {
            fromGithub("Pedro-Pathing", "PedroPathing")
        }
        license("BSD 3-Clause License", "https://opensource.org/licenses/BSD-3-Clause")

        developer("Baron Henderson", "baron@pedropathing.com")
        developer("Havish Sripada", "havish@pedropathing.com")
    }

    signing {
        key = secret("MVN_GPG_KEY")
        password = secret("MVN_GPG_PASSWORD")
    }

    content {
        component {
            fromJava()
            javaSources()
            docs(dokkaJar)
        }
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

    localSpec()
}