package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCmd;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Auto.AutoPath;
import frc.robot.Auto.AutoTrackCmd;

public class RobotContainer {
	private final GamepadJoystick joystick = new GamepadJoystick(GamepadJoystick.CONTROLLER_PORT);
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	private final SwerveDriveCmd swerveDriveCmd = new SwerveDriveCmd(swerveSubsystem, joystick);
	private final AutoPath autoPath = new AutoPath(swerveSubsystem);
	private final Limelight limelight = new Limelight();

	public RobotContainer() {
		this.swerveSubsystem.setDefaultCommand(this.swerveDriveCmd);
	}

	public Command getAutonomousCommand() {
		return new AutoTrackCmd(this.swerveSubsystem, this.limelight, this.joystick);
	}
}