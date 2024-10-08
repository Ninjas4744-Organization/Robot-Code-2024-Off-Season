package frc.robot.NinjasLib.Swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.NinjasLib.DataClasses.SwerveModuleConstants;
import frc.robot.NinjasLib.Swerve.Extras.CANCoderUtil;
import frc.robot.NinjasLib.Swerve.Extras.CANCoderUtil.CCUsage;
import frc.robot.NinjasLib.Swerve.Extras.CANSparkMaxUtil;
import frc.robot.NinjasLib.Swerve.Extras.CANSparkMaxUtil.Usage;
import frc.robot.NinjasLib.Swerve.Extras.OnboardModuleState;
import frc.robot.RobotState;

public class SwerveModule {
	public int moduleNumber;
	public double m_angleKP;
	public double m_angleKI;
	public double m_angleKD;
	public double m_angleKFF;
	private Rotation2d lastAngle;
	private Rotation2d angleOffset;

	private CANSparkMax angleMotor;
	private CANSparkMax driveMotor;

	private RelativeEncoder driveEncoder;
	private RelativeEncoder integratedAngleEncoder;

	private CANcoder angleEncoder;

	private SparkPIDController driveController;
	private SparkPIDController angleController;

	private final SimpleMotorFeedforward feedforward =
			new SimpleMotorFeedforward(SwerveConstants.kDriveS, SwerveConstants.kDriveV, SwerveConstants.kDriveA);

	public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
		this.moduleNumber = moduleNumber;
		this.m_angleKP = SwerveConstants.kAngleP;
		this.m_angleKI = SwerveConstants.kAngleI;
		this.m_angleKD = SwerveConstants.kAngleD;
		this.m_angleKFF = SwerveConstants.kAngleFF;
		angleOffset = moduleConstants.angleOffset;

		if (!RobotState.isSimulated()) {
			/* Angle Encoder Config */
			angleEncoder = new CANcoder(moduleConstants.cancoderID);
			configAngleEncoder();

			/* Angle Motor Config */
			angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
			integratedAngleEncoder = angleMotor.getEncoder();
			angleController = angleMotor.getPIDController();
			configAngleMotor();

			/* Drive Motor Config */
			driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
			driveEncoder = driveMotor.getEncoder();
			driveController = driveMotor.getPIDController();
			configDriveMotor();
		}

		lastAngle = getState().angle;
	}

	public SwerveModuleState getState() {
		if (RobotState.isSimulated()) return new SwerveModuleState(0, getAngle());
		else return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
	}

	public SwerveModulePosition getPosition() {
		if (RobotState.isSimulated()) return new SwerveModulePosition(0, getAngle());
		else return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
		desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

		setAngle(desiredState);
		setSpeed(desiredState, isOpenLoop);
	}

	private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
		if (RobotState.isSimulated()) return;

		if (isOpenLoop) {
			double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
			SmartDashboard.putNumber("driveWheels", percentOutput);
			driveMotor.set(percentOutput);
		} else {
			driveController.setReference(
					desiredState.speedMetersPerSecond,
					ControlType.kVelocity,
					0,
					feedforward.calculate(desiredState.speedMetersPerSecond));
		}
	}

	private void setAngle(SwerveModuleState desiredState) {
		// Prevent rotating module if speed is less then 1%. Prevents Jittering.
		Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
				? lastAngle
				: desiredState.angle;

		angleController.setReference(angle.getDegrees(), ControlType.kPosition);
		lastAngle = angle;
	}

	public void resetToAbsolute() {
		if (RobotState.isSimulated()) return;

		double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
		// integratedAngleEncoder.setPosition(absolutePosition);

		double currentAngle = integratedAngleEncoder.getPosition();
		double angleDiff = (absolutePosition - currentAngle) % 360;

		double targetAngle = currentAngle + angleDiff;
		if (angleDiff <= -180) targetAngle += 360;

		if (angleDiff >= 180) targetAngle -= 360;

		if (Math.abs(targetAngle - currentAngle) > 2) integratedAngleEncoder.setPosition(targetAngle);

		System.out.println("Encoder: " + integratedAngleEncoder.getPosition() + "  ->  Absolute: " + targetAngle);
	}

	public Rotation2d getCanCoder() {
		angleEncoder.getAbsolutePosition().refresh();
		return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue() * 360);
	}

	private void configAngleEncoder() {
		CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kSensorDataOnly);
	}

	private void configAngleMotor() {
		// resets angle motor
		angleMotor.restoreFactoryDefaults();
		// limits can bus usage
		CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
		// sets current limit
		angleMotor.setSmartCurrentLimit(SwerveConstants.kAngleContinuousCurrentLimit);
		// sets inversion
		angleMotor.setInverted(SwerveConstants.angleInvert);
		// sets brake mode or not
		angleMotor.setIdleMode(SwerveConstants.angleNeutralMode);
		// sets a conversion factor for the encoder so it output correlates with the
		// rotation of the module
		integratedAngleEncoder.setPositionConversionFactor(SwerveConstants.angleConversionFactor);
		// oops pid loop time sets the pid
		angleController.setP(m_angleKP);
		angleController.setI(m_angleKI);
		angleController.setD(m_angleKD);
		angleController.setFF(m_angleKFF);
		angleMotor.enableVoltageCompensation(SwerveConstants.kVoltageComp);
		// burns spark max
		angleMotor.burnFlash();

		Timer.delay(1.0);
		// resets to the cancoder
		resetToAbsolute();
	}

	private void configDriveMotor() {
		// factory resets the spark max
		driveMotor.restoreFactoryDefaults();
		// full utilisation on the can loop hell yea
		CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
		// sets current limit
		driveMotor.setSmartCurrentLimit(SwerveConstants.kDriveContinuousCurrentLimit);
		// sets inverted or not
		driveMotor.setInverted(SwerveConstants.driveInvert);
		// sets brake mode or not
		driveMotor.setIdleMode(SwerveConstants.driveNeutralMode);
		// sets encoder to read velocities as meters per second
		driveEncoder.setVelocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);
		// sets encoder to read positions as meters traveled
		driveEncoder.setPositionConversionFactor(SwerveConstants.driveConversionPositionFactor);
		// pid setting fun
		driveController.setP(SwerveConstants.kDriveP);
		driveController.setI(SwerveConstants.kDriveI);
		driveController.setD(SwerveConstants.kDriveD);
		driveController.setFF(SwerveConstants.kDriveFF);
		driveMotor.enableVoltageCompensation(SwerveConstants.kVoltageComp);
		// burns to spark max
		driveMotor.burnFlash();
		// resets encoder position to 0
		driveEncoder.setPosition(0.0);
	}

	private Rotation2d getAngle() {
		if (RobotState.isSimulated()) return new Rotation2d();
		else return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
	}
}
