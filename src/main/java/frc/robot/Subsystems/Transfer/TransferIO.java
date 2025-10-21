package frc.robot.Subsystems.Transfer;

import org.littletonrobotics.junction.AutoLog;

public interface TransferIO {

    @AutoLog

    public static class TransferInputs {
        public double voltage;
        public double current;
        public double output;
        public boolean isCoralIn;
    }

    public default void updateInputs(TransferInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setSpeed(double speed) {
    }

    public default void stopMotor() {
    }

    public default boolean isCoralIn() {
        return false;
    }

}
