package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveConstants;

public class LazyTalon extends TalonFX{
    double gearRatio;

    public LazyTalon(int motorPort, boolean reverse, double gearRatio) {
        super(motorPort);
        this.configFactoryDefault();
        this.setInverted(reverse);
        this.setNeutralMode(NeutralMode.Brake);
        this.configVoltageCompSaturation(SwerveConstants.MAX_VOLTAGE);
        this.enableVoltageCompensation(true);
        this.gearRatio = gearRatio;
    }

    public void setPosition(double getAbsolutePosition) {
        double rotaion = Units.degreesToRotations(getAbsolutePosition);
        this.setSelectedSensorPosition(rotaion * (2048.0 / this.gearRatio));
    }

    public double getVelocity() {
        return this.getSelectedSensorVelocity() * this.gearRatio;
    }
    
    public double getPosition() {
        return this.getSelectedSensorPosition() * this.gearRatio;
    }

    public double getTurnPosition() {
        double degrees = Units.rotationsToDegrees(this.getSelectedSensorPosition());
        return degrees / (2048.0 / this.gearRatio);
    }

    public double getAbsolutePosition(double absolutePosition) {
        double degrees = Units.rotationsToDegrees(absolutePosition);
        return degrees / (2048.0 / this.gearRatio);
    }

    public void setSpeed(double speed) {
        this.set(TalonFXControlMode.PercentOutput, speed);
    }
}
