package frc.robot.Subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog

    public static class ArmInputs {
        public double voltage;
        public double output;
        public double position;
        public boolean downLimitSwitchPressed;
        public boolean upLimitSwitchPressed;
        public boolean atGoal;
    }

    public default void setSpeed(double speed) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void resistGravity() {
    }

    public default void stopElevator() {
    }

    public default void brakeElevator() {
    }

    public default void goToPostion(double goal) {
    }

    public default boolean upSwitchPressed() {
        return false;
    }

    public default boolean downSwitchPressed() {
        return false;
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
}
