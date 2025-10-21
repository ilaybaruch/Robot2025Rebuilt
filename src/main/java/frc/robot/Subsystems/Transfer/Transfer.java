package frc.robot.Subsystems.Transfer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {
    TransferIO transferIO;
    TransferInputsAutoLogged transferInputs = new TransferInputsAutoLogged();

    public Transfer(TransferIO transferIO) {
        this.transferIO = transferIO;
    }

    public TransferIO getIO() {
        return transferIO;
    }
}
