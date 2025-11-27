package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Subsystems.Elevator.Elevator;

public class ElevatorCommands extends Command {

    public Command setVoltage(DoubleSupplier voltage, Elevator elevator) {
        return Commands.runEnd(() -> elevator.getIO().setVolatge(voltage.getAsDouble()),
                () -> elevator.getIO().stopElevator(), elevator).withName("elevator normal voltage");
    }

    public Command goToPosition(double goal, Elevator elevator) {
        return new FunctionalCommand(
                () -> {
                    elevator.getIO().stopElevator();
                    elevator.getIO().resetPID(goal);
                },
                () -> elevator.getIO().setGoal(goal),
                interrupted -> {
                    elevator.getIO().stopElevator();
                }, () -> elevator.getIO().atGoal(), elevator).withName("go to position:" + goal);
    }

    public Command ElevatorDown(Elevator elevator) {
        return new FunctionalCommand(() -> {
            elevator.getIO().stopElevator();
            elevator.getIO().resetPID(0);
        },
                () -> elevator.getIO().setGoal(0),
                interrupted -> {
                    elevator.getIO().stopElevator();
                },
                () -> elevator.getIO().atGoal(),
                elevator).withName("elevator down").andThen(() -> elevator.getIO().setSpeed(-0.1))
                .until(() -> elevator.getIO().atGoal());
    }

    public Command closeElevator(Elevator elevator) {
        return Commands.sequence(
                goToPosition(0, elevator),
                setVoltage(() -> -1, elevator).until(elevator.getIO()::isPressed));
    }

}
