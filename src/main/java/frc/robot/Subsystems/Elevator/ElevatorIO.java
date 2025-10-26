package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

   @AutoLog

   public static class ElevatorInputs {
      public double voltage;
      public double output;
      public double position;
      public boolean isSwitchPressed;
      public boolean atGoal;
   }

   public default void updateInputs(ElevatorInputs inputs) {
   }

   public default boolean isPressed() {
      return false;
   }

   public default void resetEncouderIfPressed() {
   }

   public default void setVolatge(double voltage) {
   }

   public default void setSpeed(double speed) {
   }

   public default void stopElevator(){
   }

   public default void resistGravity() {
   }

   public default void setGoal(double goal) {
   }

   public default void resetPID() {
   }

   public default void resetPID(double newGoal) {
   }

   public default double getPos() {
      return 0;
   }

   public default boolean atGoal(){
      return false;
   }

   public default void setPIDValues() {
   }
}
