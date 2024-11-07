package frc.robot.Constants;

import frc.robot.NinjasLib.DataClasses.ControllerConstants;
import frc.robot.NinjasLib.DataClasses.MainControllerConstants;
import frc.robot.NinjasLib.DataClasses.PIDFConstants;
import frc.robot.NinjasLib.DataClasses.SimulatedControllerConstants;

public class ShooterConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants =
        new SimulatedControllerConstants();

    static {
        kControllerConstants.main.id = 30;
        kControllerConstants.main.inverted = true;
        kControllerConstants.currentLimit = 40;
        kControllerConstants.subsystemName = "Shooter";

        kControllerConstants.PIDFConstants = new PIDFConstants();
        kControllerConstants.PIDFConstants.kS = 0.185;
        kControllerConstants.PIDFConstants.kV = 0.117;
        kControllerConstants.PIDFConstants.kCruiseVelocity = 3000;
        kControllerConstants.PIDFConstants.kAcceleration = 6000;
        kControllerConstants.velocityGoalTolerance = 600;
        kControllerConstants.encoderConversionFactor = 60;

        kControllerConstants.followers = new ControllerConstants[] {new ControllerConstants()};
        kControllerConstants.followers[0].id = 31;
        kControllerConstants.followers[0].inverted = true;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorTorque = 1;
    }

    public static final double kMinimumShootTolerance = 200;

    public enum States {
        kSpeaker(5580),
        kAmp(1625),
        kOuttake(1200),
        kDelivery(4600);

        private final int value;

        States(int value) {
            this.value = value;
        }

        public int get(){
            return value;
        }
    }
}