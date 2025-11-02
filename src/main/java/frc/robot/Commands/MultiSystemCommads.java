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
        return Commands.sequence(
            Commands.parallel(
            elevatorCommands.goToPosition(2.26, elevator),
            transferCommands.intakeCoral(transfer),
            armCommands.setVoltage(-1, arm)
            ).unless(() -> transfer.getIO().isCoralIn())
            .until(() -> transfer.getIO().isCoralIn()),
            elevatorCommands.goToPosition(10, elevator)          
        );
    }

    public Command L1(Elevator elevator,Arm arm){
        return Commands.parallel(
        elevatorCommands.goToPosition(ELEVATOR_L1_POS, elevator)
        ,armCommands.setVoltage(ARM_L1_POS, arm));
    }

    public Command L2(Elevator elevator,Arm arm){
        return Commands.parallel(
        elevatorCommands.goToPosition(ELEVATOR_L2_POS, elevator)
        ,armCommands.setVoltage(ARM_L2_POS, arm));
    }

    public Command L3(Elevator elevator,Arm arm){
        return Commands.parallel(
        elevatorCommands.goToPosition(ELEVATOR_L3_POS, elevator)
        ,armCommands.setVoltage(ARM_L3_POS, arm));
    }

    public Command L4(Elevator elevator,Arm arm){
        return Commands.parallel(
        elevatorCommands.goToPosition(ELEVATOR_L4_POS, elevator)
        ,armCommands.setVoltage(ELEVATOR_L4_POS, arm));
    }

    public Command CloseAll(Elevator elevator,Arm arm){
        return Commands.parallel(
        elevatorCommands.closeElevator(elevator)
        ,armCommands.closeArm(arm));
    }
    
}
