package frc.robot.NinjasLib.Vision;

import frc.robot.NinjasLib.DataClasses.VisionEstimation;

@FunctionalInterface
public interface FOMCalculator {
    double calculateFOM(VisionEstimation estimation);
}
