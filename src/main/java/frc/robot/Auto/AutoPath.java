package frc.robot.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoPath {
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController xPid;
    private final PIDController yPid;
    private final PIDController rotationPid;
    private final PathPlannerTrajectory trajectory;
    private final PPSwerveControllerCommand autoPath;

    public AutoPath(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.xPid = new PIDController(0.1, 0, 0);
        this.yPid = new PIDController(0.1, 0, 0);
        this.rotationPid = new PIDController(0.1, 0, 0);
        this.trajectory =  PathPlanner.loadPath(
            "SwervePath",
            AutoConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND,
            AutoConstants.PHYSICAL_MAX_ACCELERATION_METERS_PER_SECONE
        );
        this.autoPath = new PPSwerveControllerCommand(
            this.trajectory,
            this.swerveSubsystem::getPose,
            this.xPid,
            this.yPid,
            this.rotationPid,
            this.swerveSubsystem::setAutoModuleState,
            this.swerveSubsystem
        );
    }

    public Command autoPath() {
        return new SequentialCommandGroup(
            this.autoPath,
            Commands.run(this.swerveSubsystem::stopModules, this.swerveSubsystem)
        );
    }
}
