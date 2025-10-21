package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Transfer.Transfer;
import static frc.robot.Subsystems.Transfer.TransferConstants.*;

public class TransferCommands {
    public static Command intakeCoral(Transfer transfer){
        return Commands.runEnd(()-> transfer.getIO().setVoltage(INTAKE_VOLTAGE), ()-> transfer.getIO().stopMotor(), transfer)
        .until(()->transfer.getIO().isCoralIn());
    }
    
}
