import us.ihmc.jros2.generator.jros2GenTask

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.jros2.generator") version "1.3.0"
}

ihmc {
   group = "us.ihmc"
   version = "0.2.4"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc_hands_ros2"
   openSource = true

   configureDependencyResolution()
   configurePublications()

   resourceDirectory("main", "generated-idl")
   javaDirectory("main", "generated-java")

   resourceDirectory("main", "../../msg")
   resourceDirectory("main", "../../urdf")
   resourceDirectory("main", "../../meshes")
}

mainDependencies {
   api("us.ihmc:jros2:1.3.0")
   api("us.ihmc:ihmc-robotics-tools:0.15.8")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
   api(junit.jupiterApi())
   api("org.junit.jupiter:junit-jupiter-params:5.9.2")
}

tasks.register<jros2GenTask>("generateMessages") {
   description = "Generate IHMC hands ROS 2 interfaces using jros2"
   group = "build"

   packagePaths = listOf(
      projectDir.absolutePath,
   )

   outputDir = projectDir.resolve("src/main/generated-java").absolutePath
}

tasks.named("compileJava") {
   dependsOn("generateMessages")
}
