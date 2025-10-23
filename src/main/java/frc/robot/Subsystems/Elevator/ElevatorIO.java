package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

   @AutoLog

   public static class ElevatorInputs {
      public double voltage;
      public double output;
      public boolean isSwitchPressed;
      public boolean atGoal;
   }

   public default void updateInputs(ElevatorInputs inputs) {
   }

   public default boolean isPressed() {
      return false;
   }

   public default void setVolatge(double voltage) {
   }

   public default void setSpeed(double speed) {
   }

   public default void setFF() {
   }

   public default void setPIDWithFF(double goal) {
   }

   public default void resetPID() {
   }

   public default double getPos() {
      return 0;
   }

   public default void setPIDValues() {
   }
}
