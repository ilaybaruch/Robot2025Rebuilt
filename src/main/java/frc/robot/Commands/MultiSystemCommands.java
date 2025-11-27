package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Transfer.Transfer;

import static frc.robot.Subsystems.Arm.ArmConstants.*;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.Subsystems.Transfer.TransferConstants.*;

import java.util.function.BooleanSupplier;

public class MultiSystemCommands extends SubsystemBase {

        ElevatorCommands elevatorCommands = new ElevatorCommands();
        ArmCommands armCommands = new ArmCommands();
        TransferCommands transferCommands = new TransferCommands();

        public Command IntakeSequence(Transfer transfer, Elevator elevator, Arm arm) {
                return Commands.sequence(
                                Commands.parallel(
                                                elevatorCommands.goToPosition(2.26, elevator),
                                                transferCommands.intakeCoral(transfer),
                                                armCommands.setVoltage(-1, arm))
                                                .unless(() -> transfer.getIO().isCoralIn())
                                                .until(() -> transfer.getIO().isCoralIn()),
                                elevatorCommands.goToPosition(10, elevator));
        }

        public Command L4(Elevator elevator, Arm arm, BooleanSupplier isButtonPressed) {
                return Commands.sequence(
                                armCommands.goToPositon(ARM_OPEN_POS, arm).alongWith(
                                                elevatorCommands.goToPosition(ELEVATOR_MAX_POS, elevator)),
                                armCommands.goToPositon(ARM_L4_POS, arm)
                                                .onlyIf(isButtonPressed));
        }

        public Command L3(Elevator elevator, Arm arm, BooleanSupplier isButtonPressed) {
                return Commands.sequence(
                                armCommands.goToPositon(ARM_OPEN_POS, arm).alongWith(
                                                elevatorCommands.goToPosition(0, elevator)),
                                armCommands.goToPositon(ARM_L3_POS, arm)
                                                .onlyIf(isButtonPressed));
        }

        public Command CloseAll(Elevator elevator, Arm arm) {
                return Commands.parallel(
                                elevatorCommands.closeElevator(elevator), armCommands.closeArm(arm));
        }

        public Command outtakeCoral(Arm arm, Transfer transfer) {
                return new ConditionalCommand(transferCommands.outtakeCoral(transfer, TRANSFER_OUTTAKE_SPEED),
                                transferCommands.outtakeCoral(transfer, -TRANSFER_OUTTAKE_SPEED),
                                () -> arm.getIO().isHigh());
        }

        public Command AlgaeIntake(Transfer transfer, Elevator elevator, Arm arm, BooleanSupplier isButtonPressed) {
                return Commands.sequence(
                                elevatorCommands.goToPosition(ELEVATOR_L2_POS, elevator)
                                                .alongWith(armCommands.goToPositon(-0.9, arm))
                                                .alongWith(transferCommands.outtakeCoral(transfer, 3))
                                                .until(isButtonPressed),
                                (armCommands.ArmGoToStart(arm)
                                                .alongWith(elevatorCommands.goToPosition(1, elevator))
                                                .alongWith(transferCommands.outtakeCoral(transfer, 0.5)))
                                                .onlyWhile(isButtonPressed)
                                                .andThen(transferCommands.outtakeCoral(transfer, -10)));
        }

}
