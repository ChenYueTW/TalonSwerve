package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.IDashboardProvider;
import frc.robot.lib.LazyTalon;

public class SwerveModule implements IDashboardProvider{
    private final LazyTalon driveMotor;
    private final LazyTalon turnMotor;

    private final CANCoder turnEncoder;

    private final PIDController turnPidController;

    private final String motorName;
    private double driveOutput;
    private double turnOutput;

    private final double turningEncoderOffset;

    public SwerveModule(
        int driveMotorPort, int turnMotorPort, int turnEncoderPort,
        boolean driveMotorReverse, boolean turnMotorReverse,
        double turnEncoderOffset, String motorName
    ){
        this.registerDashboard();

        this.driveMotor = new LazyTalon(driveMotorPort, driveMotorReverse, SwerveConstants.DRIVE_GEAR_RATIO);
        this.turnMotor = new LazyTalon(turnMotorPort, turnMotorReverse, SwerveConstants.TURN_GEAR_RATIO);

        this.turnEncoder = new CANCoder(turnEncoderPort);

        this.turnEncoder.configFactoryDefault();

        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        this.turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        this.turnEncoder.configSensorDirection(false);
        this.turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        this.turnPidController = new PIDController(0.009, 0, 0);
        this.turnPidController.enableContinuousInput(-180, 180);

        this.motorName = motorName;
        this.turningEncoderOffset = turnEncoderOffset;
        this.resetEncoders();
    }

    public void resetEncoders() {
        this.driveMotor.setPosition(0);
        this.turnMotor.setPosition(this.getTurnPosition());
    }

    public double getTurnPosition() {
        return this.turnEncoder.getAbsolutePosition() - this.turningEncoderOffset;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.driveMotor.getVelocity(),
            Rotation2d.fromDegrees(this.getTurnPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveMotor.getPosition(),
            Rotation2d.fromDegrees(this.getTurnPosition())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getState().angle);

        this.driveOutput = state.speedMetersPerSecond / SwerveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        this.turnOutput = this.turnPidController.calculate(
            this.getTurnPosition(), state.angle.getDegrees()
        );

        this.driveMotor.setSpeed(this.driveOutput);
        this.turnMotor.setSpeed(this.turnOutput);
    }

    public void setAutoDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getState().angle);

        this.driveOutput = state.speedMetersPerSecond;
        this.turnOutput = this.turnPidController.calculate(this.turnMotor.getPosition(), state.angle.getDegrees());

        this.driveMotor.setSpeed(this.driveOutput);
        this.turnMotor.setSpeed(this.turnOutput);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber(this.motorName + " DrivePosition", this.driveMotor.getPosition());
        SmartDashboard.putNumber(this.motorName + " DriveVelocity", this.driveMotor.getVelocity());
        SmartDashboard.putNumber(this.motorName + " TurnPosition", this.getTurnPosition());
        SmartDashboard.putNumber(this.motorName + " TurnVelocity", this.turnEncoder.getVelocity());
        SmartDashboard.putNumber(this.motorName + " DriveMotor", this.driveOutput);
        SmartDashboard.putNumber(this.motorName + " TurnMotor", this.turnOutput);
    }

    public void stop() {
        this.driveMotor.setSpeed(0);
        this.turnMotor.setSpeed(0);
    }
}
