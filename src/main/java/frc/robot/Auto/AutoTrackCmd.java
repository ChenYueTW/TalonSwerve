package frc.robot.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoTrackCmd extends CommandBase {
	private final SwerveSubsystem swerveSubsystem;
	private final Limelight limelight;
	private final XboxController controller;
	private final PIDController flotPid;
	private final PIDController rotationPid;

	public AutoTrackCmd(SwerveSubsystem swerveSubsystem, Limelight limelight, XboxController controller) {
		this.swerveSubsystem = swerveSubsystem;
		this.limelight = limelight;
		this.controller = controller;
		this.flotPid = new PIDController(0.45, 0, 0);
		this.rotationPid = new PIDController(0.49, 0, 0);

		addRequirements(this.swerveSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double apriltagId = this.limelight.getAprilTagId();
		double distanceToGoalVerticalMeters = this.limelight.getDistanceToGoalVerticalMeters();
		double distanceToGoalHorizontalMeters = MathUtil.applyDeadband(this.limelight.getDistanceToGoalHorizontalMeters(distanceToGoalVerticalMeters), -1);

		double limelightDistance = 1.45226679422054; // distance to aprilTag 170cm
		double limelightHorizontal = -0.041075960895223;
		double verticalSpeed = MathUtil.applyDeadband(
			this.flotPid.calculate(distanceToGoalVerticalMeters, limelightDistance), -0.05) * LimelightConstants.VERTICAL_MAX_SPEED;
		double rotationSpeed = MathUtil.applyDeadband(
			this.rotationPid.calculate(distanceToGoalHorizontalMeters, limelightHorizontal), 0.25) * LimelightConstants.HORIZONTAL_MAX_SPEED;
		SmartDashboard.putNumber("Calculate Vertical", verticalSpeed);
		SmartDashboard.putNumber("Calculate Rotation", rotationSpeed);

		if (verticalSpeed < 0 && apriltagId != -1) {
			this.swerveSubsystem.driveSwerve(-verticalSpeed, 0.0, rotationSpeed, LimelightConstants.gyroField);
		} else {
			this.swerveSubsystem.driveSwerve(0.0, 0.0, rotationSpeed, LimelightConstants.gyroField);
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.swerveSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
