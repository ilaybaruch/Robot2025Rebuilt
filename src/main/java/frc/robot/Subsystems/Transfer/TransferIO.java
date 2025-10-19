package frc.robot.Subsystems.Transfer;

public interface TransferIO {

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
