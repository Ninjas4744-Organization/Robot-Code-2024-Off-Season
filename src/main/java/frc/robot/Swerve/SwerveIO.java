package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public abstract class SwerveIO extends SubsystemBase {

	static SwerveIO _instance;

	public static SwerveIO getInstance() {
		if (_instance == null) {
			if (Robot.isReal()) _instance = new Swerve();
			else _instance = new SimSwerve();
		}
		return _instance;
	}

	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {}

	public void drive(ChassisSpeeds speeds) {}

	public void resetOdometry(Pose2d pose) {}

	public void resetModulesToAbsolute() {}

	public void setBaybladeMode(boolean isBayblade) {}

	public void setIsDriveAssist(boolean isDriveAssist) {}

	public void lookAt(Translation2d angle, double roundToAngle) {}

	public abstract SwerveModulePosition[] getModulePositions();

	public abstract ChassisSpeeds getChassisSpeeds();
}
