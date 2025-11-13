plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   group = "us.ihmc"
   version = "0.1.2"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc_hands_ros2"
   openSource = true

   configureDependencyResolution()
   configurePublications()

   // Generated directories
   resourceDirectory("main", "generated-idl")
   javaDirectory("main", "generated-java")

   // Other resources
   resourceDirectory("main", "../../msg")
   resourceDirectory("main", "../../urdf")
   resourceDirectory("main", "../../meshes")
}

mainDependencies {
   api("us.ihmc:ros2-library:1.2.4")
   api("us.ihmc:ihmc-robotics-tools:0.15.6")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
   api(junit.jupiterApi())
   // Must use JUnit version from IHMCCIPlugin.kt https://github.com/ihmcrobotics/ihmc-build/blob/develop/src/main/kotlin/us/ihmc/ci/IHMCCIPlugin.kt
   api("org.junit.jupiter:junit-jupiter-params:5.9.2")
}

val generator = us.ihmc.ros2.rosidl.ROS2InterfaceGenerator()
tasks.register("generateMessages") {
   doFirst {
      // Delete old generated files
      delete("src/main/generated-idl")
      delete("src/main/generated-ros1")
      delete("src/main/generated-java")

      // Generate the messages into build/tmp/generateMessages
      generator.addPackageRootToIDLGenerator(file("./").toPath())
      generator.generate(file("src/main/generated-idl").toPath(),
                         file("src/main/generated-ros1").toPath(),
                         file("src/main/generated-java").toPath())

      us.ihmc.ros2.rosidl.ROS2InterfaceGenerator.convertDirectoryToUnixEOL(file("src/main/generated-idl").toPath())
      us.ihmc.ros2.rosidl.ROS2InterfaceGenerator.convertDirectoryToUnixEOL(file("src/main/generated-ros1").toPath())
      us.ihmc.ros2.rosidl.ROS2InterfaceGenerator.convertDirectoryToUnixEOL(file("src/main/generated-java").toPath())
   }
}
