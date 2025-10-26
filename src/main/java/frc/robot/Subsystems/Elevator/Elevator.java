package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    ElevatorIO elevatorIO;
    ElevatorInputsAutoLogged Inputs = new ElevatorInputsAutoLogged();

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public ElevatorIO getIO(){
        return elevatorIO;
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(Inputs);
        Logger.processInputs("elevator", Inputs);
        getIO().resetEncouderIfPressed();
        // need to add current command
    }

}
