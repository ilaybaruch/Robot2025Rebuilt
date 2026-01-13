package frc.robot.Subsystem.DifferentialShooting;

import org.littletonrobotics.junction.AutoLog;

public interface DifferentialShootingIO {

    @AutoLog

    public static class DifferentialShootingIOInputs {
        double backMotorVelocity;
        double frontMotorVelocity;
        double backMotorAppliedVolts;
        double frontMotorAppliedVolts;
    }

    public default void updateInputs(DifferentialShootingIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setSpeed(double speed) {
    }

    public default void stopMotor() {
    }

    public default void setDifferentialVoltage(double voltage, double diff) {
    }

}
