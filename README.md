# Ninjas #4744 - 2024 Offseason FRC Robot Code

Welcome to the official repository for the 2024 offseason robot code for Team Ninjas #4744! This repository contains all the code, documentation, and resources related to our robotics development during the offseason in preparation for the 2025 FRC competition.

## 📚 About Us

**Team Name:** Ninjas  
**Team Number:** 4744  
**Season:** 2024 Offseason  
**Location:** Hadera/Israel

Team Ninjas #4744 is a dedicated and enthusiastic group of high school students committed to the exploration and development of robotics and engineering. Each year, we participate in the FIRST Robotics Competition (FRC), where we design, build, and program a robot to compete in a game challenge announced at the beginning of the season. During the offseason, we work on refining our skills, experimenting with new ideas, and improving our robot's capabilities.

## 💻 Repository Overview

This repository contains the following:

- **Swerve Code**: Our own code for controlling swerve including many parameters and many useful methods.

- **Vision Code**: With Photonvision and our 3 cameras we have some extremely precise vision estimations that connect with the swerve's odometry with Kalman filter.

- **Abstractions For Controllers And Subsystems**: We have many abstraction classes like NinjasController so we can control any controller with ease without the need to write different code for TalonFX, SparkMax and more...

- **Robot's Subsystems**: All the subsystems on our robot including Shooter, Indexer, FloorIntake and more...

- **Drive Assist**: Our own code for calculating drive assist and helping the driver.

- **Autonomy**: Autonomy with pathplanner already made files in the GUI and some on the fly autonomy like auto driving.

## 🛠️ Technologies Used

- **Programming Language**: Java
- **FRC Library**: WPILib
- **Build System**: GradleRIO
- **Version Control**: Git

## 🚀 Getting Started

### Prerequisites

To work with this code, you'll need the following installed on your development machine:

- [WPIlib + Visual Studio Code](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
