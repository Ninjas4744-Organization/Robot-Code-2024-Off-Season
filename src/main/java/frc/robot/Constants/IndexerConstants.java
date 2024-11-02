package frc.robot.Constants;

import frc.robot.NinjasLib.DataClasses.MainControllerConstants;
import frc.robot.NinjasLib.DataClasses.SimulatedControllerConstants;

public class IndexerConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants =
        new SimulatedControllerConstants();

    static {
        kControllerConstants.main.id = 20;
        kControllerConstants.main.inverted = false;
        kControllerConstants.currentLimit = 50;
        kControllerConstants.subsystemName = "Indexer";

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorTorque = 1;
    }

    public enum States {
        kIntake(1),
        kIndex(1),
        kIndexBack(-1),
        kShoot(1),
        kOuttake(1);

        private final int value;

        States(int value) {
            this.value = value;
        }

        public int get(){
            return value;
        }
    }
}