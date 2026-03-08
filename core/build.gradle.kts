plugins {
    id("java-library")
    id("maven-publish")
    id("org.jetbrains.dokka")
}

dependencies {
    compileOnly(libs.annotations)
    dokkaPlugin(libs.dokka.java.plugin)
}

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

val dokkaJar = tasks.register<Jar>("dokkaJar") {
    dependsOn(tasks.named("dokkaGenerate"))
    from(dokka.basePublicationsDirectory.dir("html"))
    archiveClassifier = "html-docs"
}

publishing {
    publications {
        create<MavenPublication>("mavenJava") {
            from(components["java"])
            artifact(dokkaJar)

            groupId = "com.millburnx"
            artifactId = "pedropathing-core"
            version = "0.1.2"

            pom {
                name.set("Pedro Pathing Core")
                description.set("A path follower designed to revolutionize autonomous pathing in robotics")
                url.set("https://github.com/Pedro-Pathing/PedroPathing")
            }
        }
    }
}