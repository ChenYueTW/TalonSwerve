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

    public void setEncoder(double degrees) {
        double ratation = Units.degreesToRotations(degrees);
        this.setSelectedSensorPosition(ratation * (2048.0 / this.gearRatio));
    }

    public double getVelocity() {
        return this.getSelectedSensorVelocity() * this.gearRatio;
    }

    public double getPosition() {
        double degrees = Units.rotationsToDegrees(this.getSelectedSensorPosition());
        return degrees / (2048.0 / this.gearRatio);
    }

    public void setSpeed(double speed) {
        this.set(TalonFXControlMode.PercentOutput, speed);
    }
}
