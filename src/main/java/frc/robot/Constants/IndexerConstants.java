package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

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
        kIndex(0.5),
        kIndexBack(-1),
        kShoot(1),
        kOuttake(1);

        private final double value;

        States(double value) {
            this.value = value;
        }

        public double get() {
            return value;
        }
    }
}