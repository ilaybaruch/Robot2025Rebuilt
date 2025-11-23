package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog

    public static class ArmInputs {
        public double voltage;
        public double output;
        public double position;
        public double velocity;
        public boolean downLimitSwitchPressed;
        public boolean upLimitSwitchPressed;
        public boolean atGoal;
    }

    public default void updateInputs(ArmInputs inputs) {
    }

    public default void setSpeed(double speed) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void resistGravity() {
    }

    public default void setGoal(double goal) {
    }

    public default boolean upSwitchPressed() {
        return false;
    }

    public default boolean downSwitchPressed() {
        return false;
    }

    public default void resetIfPressed() {

    }

    public default boolean atGoal() {
        return false;
    }

    public default double getPos() {
        return 0;
    }

    public default void resetPID() {
    }

    public default void resetPID(double newGoal) {
    }

    public default void setPIDValues() {
    }

    public default void addKg(double KG) {
    }

    public default boolean isHigh() {
        return false;
    }
}
