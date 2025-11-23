package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Transfer.Transfer;

import static frc.robot.Subsystems.Arm.ArmConstants.ARM_L3_POS;
import static frc.robot.Subsystems.Arm.ArmConstants.ARM_L4_POS;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

import java.util.function.BooleanSupplier;

public class MultiSystemCommands extends SubsystemBase {

    ElevatorCommands elevatorCommands = new ElevatorCommands();
    ArmCommands armCommands = new ArmCommands();
    TransferCommands transferCommands = new TransferCommands();

    public Command IntakeFunctional(Transfer transfer, Elevator elevator, Arm arm) {
        return new FunctionalCommand(null, null, null, null, null);
    }

    public Command IntakeSequence(Transfer transfer, Elevator elevator, Arm arm) {
        return Commands.sequence(
                Commands.parallel(
                        elevatorCommands.goToPosition(2.26, elevator),
                        transferCommands.intakeCoral(transfer),
                        armCommands.setVoltage(-1, arm)).unless(() -> transfer.getIO().isCoralIn())
                        .until(() -> transfer.getIO().isCoralIn()),
                elevatorCommands.goToPosition(10, elevator));
    }

    public Command L3(Elevator elevator, Arm arm) {
        return Commands.parallel(
                elevatorCommands.goToPosition(ELEVATOR_L3_POS, elevator), armCommands.goToPositon(ARM_L3_POS, arm));
    }

    public Command L4(Elevator elevator, Arm arm, BooleanSupplier isButtonPressed) {
        return Commands.sequence(
                armCommands.goToPositon(ARM_L4_POS, arm),
                elevatorCommands.goToPosition(ELEVATOR_L4_POS, elevator).onlyIf(isButtonPressed));
    }

    public Command CloseAll(Elevator elevator, Arm arm) {
        return Commands.parallel(
                elevatorCommands.closeElevator(elevator), armCommands.closeArm(arm));
    }

}
