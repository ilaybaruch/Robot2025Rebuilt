package frc.robot.Subsystem.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

   @AutoLog
   
   public static class ElevatorInputs{
    public double voltage;
    public double output;
    public boolean isSwitchPressed;
    public boolean atGoal;
   }

   public default void updateInputs(ElevatorInputs inputs){}

    
}
