package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Transfer.Transfer;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.Subsystems.Arm.ArmConstants.*;
import static frc.robot.Subsystems.Transfer.TransferConstants.*;


public class MultiSystemCommads extends Command {

    ElevatorCommands elevatorCommands = new ElevatorCommands();
    ArmCommands armCommands = new ArmCommands();
    TransferCommands  transferCommands = new TransferCommands();

    public Command intakeCoral(Elevator elevator,Arm arm, Transfer transfer){
        return new FunctionalCommand(() -> {elevator.getIO().stopElevator();
        elevator.getIO().resetPID(0);
        arm.getIO().resistGravity();
        arm.getIO().resetPID(0);},

         ()-> {elevator.getIO().setGoal(ELEVATOR_INTAKE_POS);
        arm.getIO().setGoal(ARM_CLOSE_POS);
        transfer.getIO().setVoltage(INTAKE_VOLTAGE);},

         interrupted ->{elevator.getIO().stopElevator();
        arm.getIO().resistGravity();
         transfer.getIO().stopMotor();},

        ()->transfer.getIO().isCoralIn(), elevator, arm, transfer);
    }
    
}
