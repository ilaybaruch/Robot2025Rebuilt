package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Subsystems.Transfer.Transfer;
import static frc.robot.Subsystems.Transfer.TransferConstants.*;

public class TransferCommands {
    public Command intakeCoral(Transfer transfer) {
        return Commands
                .runEnd(() -> transfer.getIO().setVoltage(TRANSFER_INTAKE_VOLTAGE), () -> transfer.getIO().stopMotor(),
                        transfer)
                .until(() -> transfer.getIO().isCoralIn()); // shalom
    }

    public Command returnCoral(Transfer transfer) {
        return Commands.runEnd(() -> transfer.getIO().setVoltage(TRANSFER_RETURN_SPEED),
                () -> transfer.getIO().stopMotor(),
                transfer);
    }

    public Command outtakeCoral(Transfer transfer, double voltage) {
        return Commands.runEnd(() -> transfer.getIO().setVoltage(voltage), () -> transfer.getIO().stopMotor(),
                transfer);
    }

}
