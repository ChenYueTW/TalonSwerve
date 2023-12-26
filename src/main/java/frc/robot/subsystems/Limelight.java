package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightCamera;
import frc.robot.Constants.LimelightConstants;
import frc.robot.lib.IDashboardProvider;

public class Limelight extends SubsystemBase implements IDashboardProvider {
    private final NetworkTable table;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry tid;
    private double distanceToGoalVerticalMeters;
    private double distanceToGoalHorizontalMeters;

    public Limelight() {
        this.registerDashboard();
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        this.tx = this.table.getEntry("tx");
        this.ty = this.table.getEntry("ty");
        this.tid = this.table.getEntry("tid");
        this.table.getEntry("ledMode").setNumber(LimelightCamera.ledMode);
        this.table.getEntry("camMode").setNumber(LimelightCamera.camMode);
        this.table.getEntry("crop").setDoubleArray(LimelightCamera.cameraPose);
    }

    public double getDistanceToGoalVerticalMeters() {
        double verticalOffset = this.ty.getDouble(0.0);

        double mountAngleDeg = LimelightConstants.MOUNT_ANGLE_DEG;
        double lensHeightMeters = LimelightConstants.LENS_HEIGHT_METERS;
        double goalHeightMeters = LimelightConstants.GOAL_HEIGHT_METERS;

        double angleToGoalDeg = mountAngleDeg + verticalOffset;
        double angleToGoalRad = angleToGoalDeg * (Math.PI / 180.0);
        this.distanceToGoalVerticalMeters = Math.abs((goalHeightMeters - lensHeightMeters) / Math.tan(angleToGoalRad));

        return this.distanceToGoalVerticalMeters;
    }

    public double getDistanceToGoalHorizontalMeters(double distanceToGoalVerticalMeters) {
        if (distanceToGoalVerticalMeters == -1) {
            distanceToGoalVerticalMeters = this.getDistanceToGoalVerticalMeters();
        }
        double horizontalOffset = this.tx.getDouble(0.0);

        double horizontalOffsetRad = horizontalOffset * (Math.PI / 180.0);

        this.distanceToGoalHorizontalMeters = (Math.tan(horizontalOffsetRad) * distanceToGoalVerticalMeters) - LimelightConstants.HORIZONTAL_OFFSET_METERS;
        return this.distanceToGoalHorizontalMeters;
    }

    public double getAprilTagId() {
        return this.tid.getDouble(0.0);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Vertical", this.distanceToGoalVerticalMeters);
        SmartDashboard.putNumber("Horizontal", this.distanceToGoalHorizontalMeters);
        SmartDashboard.putNumber("AprilTag", this.getAprilTagId());
    }
}
