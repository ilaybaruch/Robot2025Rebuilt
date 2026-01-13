package frc.robot.Subsystem.DifferentialShooting;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DifferentialShooting extends SubsystemBase {
    DifferentialShootingIOInputsAutoLogged differentialShootingIOInputsAutoLogged = new DifferentialShootingIOInputsAutoLogged();
    DifferentialShootingIO io;

    DifferentialShooting(DifferentialShootingIO io) {
        this.io = io;
    }

}
