# Diff Details

Date : 2024-09-20 20:20:31

Directory c:\\Users\\eitan\\Documents\\Projects\\Robotics\\Robot-Code-2024-Off-Season

Total : 76 files, 5074 codes, 919 comments, 1083 blanks, all 7076 lines

[Summary](results.md) / [Details](details.md) / [Diff Summary](diff.md) / Diff Details

## Files

| filename                                                                                                                                          | language        | code | comment | blank | total |
|:--------------------------------------------------------------------------------------------------------------------------------------------------|:----------------|-----:|--------:|------:|------:|
| [.github/workflows/main.yml](/.github/workflows/main.yml)                                                                                         | YAML            |   18 |       0 |     8 |    26 |
| [.idea/vcs.xml](/.idea/vcs.xml)                                                                                                                   | XML             |    6 |       0 |     1 |     7 |
| [.run/Build & Deploy Robot for Debugging.run.xml](/.run/Build%20&%20Deploy%20Robot%20for%20Debugging.run.xml)                                     | XML             |   26 |       0 |     1 |    27 |
| [.run/Build & Deploy Robot.run.xml](/.run/Build%20&%20Deploy%20Robot.run.xml)                                                                     | XML             |   26 |       0 |     1 |    27 |
| [.run/Build Robot.run.xml](/.run/Build%20Robot.run.xml)                                                                                           | XML             |   25 |       0 |     1 |    26 |
| [.run/Clean Build & Deploy Robot for Debugging.run.xml](/.run/Clean%20Build%20&%20Deploy%20Robot%20for%20Debugging.run.xml)                       | XML             |   27 |       0 |     1 |    28 |
| [.run/Clean Build & Deploy Robot.run.xml](/.run/Clean%20Build%20&%20Deploy%20Robot.run.xml)                                                       | XML             |   27 |       0 |     1 |    28 |
| [.run/Clean Build Robot.run.xml](/.run/Clean%20Build%20Robot.run.xml)                                                                             | XML             |   26 |       0 |     1 |    27 |
| [.run/Clean.run.xml](/.run/Clean.run.xml)                                                                                                         | XML             |   25 |       0 |     1 |    26 |
| [.run/Debug Robot via IP.run.xml](/.run/Debug%20Robot%20via%20IP.run.xml)                                                                         | XML             |   13 |       0 |     1 |    14 |
| [.run/Debug Robot via USB.run.xml](/.run/Debug%20Robot%20via%20USB.run.xml)                                                                       | XML             |   13 |       0 |     1 |    14 |
| [.run/Simulate Robot Code.run.xml](/.run/Simulate%20Robot%20Code.run.xml)                                                                         | XML             |   24 |       0 |     0 |    24 |
| [.wpilib/wpilib_preferences.json](/.wpilib/wpilib_preferences.json)                                                                               | JSON            |    6 |       0 |     0 |     6 |
| [README.md](/README.md)                                                                                                                           | Markdown        |   27 |       0 |    20 |    47 |
| [WPILib-License.md](/WPILib-License.md)                                                                                                           | Markdown        |   22 |       0 |     3 |    25 |
| [build.gradle](/build.gradle)                                                                                                                     | Gradle          |   99 |      20 |    27 |   146 |
| [gradle/wrapper/gradle-wrapper.properties](/gradle/wrapper/gradle-wrapper.properties)                                                             | Java Properties |    7 |       0 |     1 |     8 |
| [gradlew.bat](/gradlew.bat)                                                                                                                       | Batch           |   41 |      30 |    22 |    93 |
| [networktables.json](/networktables.json)                                                                                                         | JSON            |    1 |       0 |     1 |     2 |
| [settings.gradle](/settings.gradle)                                                                                                               | Gradle          |   28 |       0 |     3 |    31 |
| [simgui.json](/simgui.json)                                                                                                                       | JSON            |   47 |       0 |     1 |    48 |
| [src/main/deploy/pathplanner/navgrid.json](/src/main/deploy/pathplanner/navgrid.json)                                                             | JSON            |    1 |       0 |     0 |     1 |
| [src/main/java/frc/robot/AbstractClasses/NinjasController.java](/src/main/java/frc/robot/NinjasLib/NinjasController.java)                   | Java            |  125 |      71 |    30 |   226 |
| [src/main/java/frc/robot/AbstractClasses/NinjasSimulatedController.java](/src/main/java/frc/robot/NinjasLib/NinjasSimulatedController.java) | Java            |  126 |       0 |    25 |   151 |
| [src/main/java/frc/robot/AbstractClasses/NinjasSparkMaxController.java](/src/main/java/frc/robot/NinjasLib/NinjasSparkMaxController.java)   | Java            |  125 |       0 |    34 |   159 |
| [src/main/java/frc/robot/AbstractClasses/NinjasSubsystem.java](/src/main/java/frc/robot/NinjasLib/NinjasSubsystem.java)                     | Java            |   59 |      67 |    22 |   148 |
| [src/main/java/frc/robot/AbstractClasses/NinjasTalonFXController.java](/src/main/java/frc/robot/NinjasLib/NinjasTalonFXController.java)     | Java            |  102 |       0 |    22 |   124 |
| [src/main/java/frc/robot/AbstractClasses/NinjasTalonSRXController.java](/src/main/java/frc/robot/NinjasLib/NinjasTalonSRXController.java)   | Java            |   84 |       0 |    25 |   109 |
| [src/main/java/frc/robot/AbstractClasses/NinjasVictorSPXController.java](/src/main/java/frc/robot/NinjasLib/NinjasVictorSPXController.java) | Java            |   54 |       0 |    17 |    71 |
| [src/main/java/frc/robot/AutoCommandBuilder.java](/src/main/java/frc/robot/AutoCommandBuilder.java)                                               | Java            |   29 |      16 |     7 |    52 |
| [src/main/java/frc/robot/Constants.java](/src/main/java/frc/robot/Constants.java)                                                                 | Java            |  371 |      52 |    96 |   519 |
| [src/main/java/frc/robot/DataClasses/ControllerConstants.java](/src/main/java/frc/robot/DataClasses/ControllerConstants.java)                     | Java            |    5 |       9 |     3 |    17 |
| [src/main/java/frc/robot/DataClasses/MainControllerConstants.java](/src/main/java/frc/robot/DataClasses/MainControllerConstants.java)             | Java            |   17 |      24 |    15 |    56 |
| [src/main/java/frc/robot/DataClasses/PIDFConstants.java](/src/main/java/frc/robot/DataClasses/PIDFConstants.java)                                 | Java            |   51 |      30 |    17 |    98 |
| [src/main/java/frc/robot/DataClasses/SimulatedControllerConstants.java](/src/main/java/frc/robot/DataClasses/SimulatedControllerConstants.java)   | Java            |   13 |       2 |     4 |    19 |
| [src/main/java/frc/robot/DataClasses/StateEndCondition.java](/src/main/java/frc/robot/DataClasses/StateEndCondition.java)                         | Java            |   11 |       0 |     4 |    15 |
| [src/main/java/frc/robot/DataClasses/SwerveModuleConstants.java](/src/main/java/frc/robot/DataClasses/SwerveModuleConstants.java)                 | Java            |   14 |       8 |     4 |    26 |
| [src/main/java/frc/robot/DataClasses/VisionEstimation.java](/src/main/java/frc/robot/DataClasses/VisionEstimation.java)                           | Java            |   12 |      10 |     6 |    28 |
| [src/main/java/frc/robot/DataClasses/VisionOutput.java](/src/main/java/frc/robot/DataClasses/VisionOutput.java)                                   | Java            |   15 |       9 |    11 |    35 |
| [src/main/java/frc/robot/Main.java](/src/main/java/frc/robot/Main.java)                                                                           | Java            |    8 |       3 |     5 |    16 |
| [src/main/java/frc/robot/Robot.java](/src/main/java/frc/robot/Robot.java)                                                                         | Java            |   52 |       0 |    19 |    71 |
| [src/main/java/frc/robot/RobotContainer.java](/src/main/java/frc/robot/RobotContainer.java)                                                       | Java            |   81 |       3 |    22 |   106 |
| [src/main/java/frc/robot/RobotState.java](/src/main/java/frc/robot/RobotState.java)                                                               | Java            |  126 |      53 |    21 |   200 |
| [src/main/java/frc/robot/StateMachine.java](/src/main/java/frc/robot/StateMachine.java)                                                           | Java            |  135 |      15 |    34 |   184 |
| [src/main/java/frc/robot/Subsystems/Elevator.java](/src/main/java/frc/robot/Subsystems/Elevator.java)                                             | Java            |   43 |       0 |    10 |    53 |
| [src/main/java/frc/robot/Subsystems/Leds.java](/src/main/java/frc/robot/Subsystems/Leds.java)                                                     | Java            |   47 |       6 |     7 |    60 |
| [src/main/java/frc/robot/Subsystems/Shooter.java](/src/main/java/frc/robot/Subsystems/Shooter.java)                                               | Java            |   27 |       0 |     9 |    36 |
| [src/main/java/frc/robot/Subsystems/ShooterAngle.java](/src/main/java/frc/robot/Subsystems/ShooterAngle.java)                                     | Java            |   38 |       0 |    10 |    48 |
| [src/main/java/frc/robot/Subsystems/ShooterFeeder.java](/src/main/java/frc/robot/Subsystems/ShooterFeeder.java)                                   | Java            |   41 |       0 |    10 |    51 |
| [src/main/java/frc/robot/Swerve/CANCoderUtil.java](/src/main/java/frc/robot/Swerve/CANCoderUtil.java)                                             | Java            |   12 |      24 |     4 |    40 |
| [src/main/java/frc/robot/Swerve/CANSparkMaxUtil.java](/src/main/java/frc/robot/Swerve/CANSparkMaxUtil.java)                                       | Java            |   38 |      30 |     6 |    74 |
| [src/main/java/frc/robot/Swerve/DriveAssist.java](/src/main/java/frc/robot/Swerve/DriveAssist.java)                                               | Java            |   97 |      22 |    25 |   144 |
| [src/main/java/frc/robot/Swerve/LocalADStarAK.java](/src/main/java/frc/robot/Swerve/LocalADStarAK.java)                                           | Java            |   57 |      32 |    14 |   103 |
| [src/main/java/frc/robot/Swerve/OnboardModuleState.java](/src/main/java/frc/robot/Swerve/OnboardModuleState.java)                                 | Java            |   39 |      14 |     5 |    58 |
| [src/main/java/frc/robot/Swerve/Swerve.java](/src/main/java/frc/robot/Swerve/Swerve.java)                                                         | Java            |   82 |      25 |    17 |   124 |
| [src/main/java/frc/robot/Swerve/SwerveDemand.java](/src/main/java/frc/robot/Swerve/SwerveDemand.java)                                             | Java            |   24 |       0 |     4 |    28 |
| [src/main/java/frc/robot/Swerve/SwerveIO.java](/src/main/java/frc/robot/Swerve/SwerveIO.java)                                                     | Java            |  224 |     116 |    57 |   397 |
| [src/main/java/frc/robot/Swerve/SwerveModule.java](/src/main/java/frc/robot/Swerve/SwerveModule.java)                                             | Java            |  145 |      25 |    34 |   204 |
| [src/main/java/frc/robot/Swerve/SwerveSimulated.java](/src/main/java/frc/robot/Swerve/SwerveSimulated.java)                                       | Java            |   32 |       0 |     6 |    38 |
| [src/main/java/frc/robot/TeleopCommandBuilder.java](/src/main/java/frc/robot/TeleopCommandBuilder.java)                                           | Java            |   40 |       0 |     6 |    46 |
| [src/main/java/frc/robot/Vision/LimelightHelpers.java](/src/main/java/frc/robot/Vision/LimelightHelpers.java)                                     | Java            |  918 |     102 |   234 | 1,254 |
| [src/main/java/frc/robot/Vision/NoteDetection.java](/src/main/java/frc/robot/Vision/NoteDetection.java)                                           | Java            |   40 |       7 |    12 |    59 |
| [src/main/java/frc/robot/Vision/Vision.java](/src/main/java/frc/robot/Vision/Vision.java)                                                         | Java            |    2 |       0 |     2 |     4 |
| [src/main/java/frc/robot/Vision/VisionCamera.java](/src/main/java/frc/robot/Vision/VisionCamera.java)                                             | Java            |   80 |      15 |    20 |   115 |
| [src/main/java/frc/robot/Vision/VisionIO.java](/src/main/java/frc/robot/Vision/VisionIO.java)                                                     | Java            |   91 |      41 |    23 |   155 |
| [src/main/java/frc/robot/Vision/VisionSimulated.java](/src/main/java/frc/robot/Vision/VisionSimulated.java)                                       | Java            |   38 |       0 |    10 |    48 |
| [src/test/java/frc/robot/IntakeTest.java](/src/test/java/frc/robot/IntakeTest.java)                                                               | Java            |    3 |      11 |     2 |    16 |
| [src/test/java/frc/robot/Swerve/SwerveTest.java](/src/test/java/frc/robot/Swerve/SwerveTest.java)                                                 | Java            |   10 |      17 |     5 |    32 |
| [update_ninjas_subsystem.py](/update_ninjas_subsystem.py)                                                                                         | Python          |   19 |      10 |    11 |    40 |
| [vendordeps/NavX.json](/vendordeps/NavX.json)                                                                                                     | JSON            |   40 |       0 |     0 |    40 |
| [vendordeps/PathplannerLib.json](/vendordeps/PathplannerLib.json)                                                                                 | JSON            |   38 |       0 |     0 |    38 |
| [vendordeps/Phoenix5.json](/vendordeps/Phoenix5.json)                                                                                             | JSON            |  151 |       0 |     0 |   151 |
| [vendordeps/Phoenix6.json](/vendordeps/Phoenix6.json)                                                                                             | JSON            |  339 |       0 |     0 |   339 |
| [vendordeps/REVLib.json](/vendordeps/REVLib.json)                                                                                                 | JSON            |   74 |       0 |     0 |    74 |
| [vendordeps/WPILibNewCommands.json](/vendordeps/WPILibNewCommands.json)                                                                           | JSON            |   38 |       0 |     1 |    39 |
| [vendordeps/photonlib.json](/vendordeps/photonlib.json)                                                                                           | JSON            |   57 |       0 |     0 |    57 |

[Summary](results.md) / [Details](details.md) / [Diff Summary](diff.md) / Diff Details