package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.ElevatorCommands;

public class Elevator extends SubsystemBase {
    ElevatorIO elevatorIO;
    ElevatorInputsAutoLogged Inputs = new ElevatorInputsAutoLogged();
    ElevatorCommands elevatorCommands = new ElevatorCommands();

    public Elevator(ElevatorIO elevatorIO) {
        //// setDefaultCommand(elevatorCommands.RunElevatorFF(new Elevator(new ElevatorIOReal())));
        // setDefaultCommand(new RepeatCommand(
        //     new ConditionalCommand(
        //         this.runOnce(()-> elevatorIO.setVolatge(0)),
        //         this.runOnce(()->elevatorIO.resistGravity()), 
        //      () -> elevatorIO.isPressed())
        // )); 
        setDefaultCommand(new RepeatCommand(new ConditionalCommand(this.runOnce(() -> elevatorIO.setVolatge(0)),
                this.runOnce(elevatorIO::resistGravity), elevatorIO::isPressed))
                .beforeStarting(new PrintCommand("Elevator default command")));

        this.elevatorIO = elevatorIO;
    }

    public ElevatorIO getIO(){
        return elevatorIO;
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(Inputs);
        Logger.processInputs("elevator", Inputs);
    }

}
