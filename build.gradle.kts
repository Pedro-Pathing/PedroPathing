import com.android.builder.symbols.exportToCompiledJava
import org.jetbrains.dokka.plugability.configuration

plugins {
	id("org.jetbrains.kotlin.android") version "1.9.20"
	id("org.jetbrains.kotlin.plugin.serialization") version "1.9.20"
	id("com.android.library") version "8.1.2"

	`maven-publish`

	id("org.jetbrains.dokka") version "1.9.20"
}

tasks.named<org.jetbrains.dokka.gradle.DokkaTask>("dokkaJavadoc") {
	outputDirectory.set(file("${buildDir}/dokka"))

	dokkaSourceSets {
		named("main") {
			sourceRoots.from(file("src/main/kotlin"), file("src/main/java"))
			reportUndocumented.set(true)
			skipDeprecated.set(false)
			jdkVersion.set(8) // or adjust based on your requirements
			perPackageOption {
				matchingRegex.set("com\\.acmerobotics\\.roadrunner\\.ftc.*")
				suppress.set(true) // Exclude this package
			}
		}
	}
}

val defaultMinSdkVersion by extra(29)
val defaultMinSdkVersion1 by extra(23)

repositories {
	mavenCentral()
	google()
	maven("https://maven.brott.dev/")
}

android {
	namespace = "com.pedropathing.pedropathing"
	compileSdk = 32

	compileOptions {
		sourceCompatibility = JavaVersion.VERSION_1_8
		targetCompatibility = JavaVersion.VERSION_1_8
	}
	kotlinOptions {
		jvmTarget = "1.8"
	}
	publishing {
		singleVariant("release")
	}
	sourceSets {
		getByName("main") {
			java.srcDirs("src/main/kotlin")
		}
	}
	defaultConfig {
		minSdk = 23
	}
}

dependencies {
	compileOnly("org.firstinspires.ftc:RobotCore:10.1.1")
	compileOnly("org.firstinspires.ftc:Hardware:10.1.1")
	compileOnly("org.firstinspires.ftc:FtcCommon:10.1.1")
	compileOnly("org.firstinspires.ftc:RobotServer:10.1.1")
	compileOnly("org.firstinspires.ftc:OnBotJava:10.1.1")

	implementation("com.acmerobotics.dashboard:dashboard:0.4.16") {
		exclude(group = "org.firstinspires.ftc")
	}

	implementation("org.apache.commons:commons-math3:3.6.1")
	dokkaGfmPlugin("org.jetbrains.dokka:kotlin-as-java-plugin:1.9.20")
	implementation("org.jetbrains.kotlin:kotlin-stdlib-jdk8:1.9.20")
}


// CONFIGURE PUBLICATION
publishing {
	publications {
		register<MavenPublication>("release") {
			groupId = "com.pedropathing"
			artifactId = "pedro"
			version = "1.0.2"

			afterEvaluate {
				from(components["release"])
			}
		}
	}

	repositories {
		maven {
			name = "publishing"
			url = uri("./maven.pedropathing.com")
		}
	}
}