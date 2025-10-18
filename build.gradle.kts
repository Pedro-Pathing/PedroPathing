plugins {
    alias(libs.plugins.android.library) apply false
    alias(libs.plugins.dokka) apply false
    alias(libs.plugins.deployer) apply false
    alias(libs.plugins.kotlin.android) apply false
}

subprojects {
    group = "com.pedropathing"
    version = property("version") as String
}

tasks.register("deployCentralPortal") {
    group = "publishing"
    description = "Publishes all subprojects to Maven Central."
    dependsOn(subprojects.map { it.tasks.named("deployCentralPortal") })
}

tasks.register("deployNexusSnapshot") {
    group = "publishing"
    description = "Publishes all subprojects to Maven Central Snapshots."
    dependsOn(subprojects.map { it.tasks.named("deployNexusSnapshot") })
}

tasks.register("deployLocal") {
    group = "publishing"
    description = "Publishes all subprojects to Maven Local."
    dependsOn(subprojects.map { it.tasks.named("deployLocal") })
}