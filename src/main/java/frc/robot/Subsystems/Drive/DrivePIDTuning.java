package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

public class DrivePIDTuning {
    // set turn Logged Network Number
    LoggedNetworkNumber turnKpTune = new LoggedNetworkNumber("turn KP", turnKp);
    LoggedNetworkNumber turnKdTune = new LoggedNetworkNumber("turn KD", turnKd);
    LoggedNetworkNumber turnKsTune = new LoggedNetworkNumber("turn KS", turnKs);
    LoggedNetworkNumber turnFFTune = new LoggedNetworkNumber("turn KS", turnFF);
    // set drive Logged Network Number
    LoggedNetworkNumber driveKpTune = new LoggedNetworkNumber("drive KP", driveKp);
    LoggedNetworkNumber driveKiTune = new LoggedNetworkNumber("drive KI", driveKi);
    LoggedNetworkNumber driveKsTune = new LoggedNetworkNumber("drive KS", driveKs);
    LoggedNetworkNumber driveKvTune = new LoggedNetworkNumber("drive KV", driveKv);

    // get turn values functions
    public double getTurnKp() {
        return turnKpTune.get();
    }

    public double getTurnKd() {
        return turnKdTune.get();
    }

    public double getTurnKs() {
        return turnKsTune.get();
    }

    public double getTurnFF() {
        return turnFFTune.get();
    }

    // get drive values functions
    public double getDriveKp() {
        return driveKpTune.get();
    }

    public double getDriveKi() {
        return driveKiTune.get();
    }

    public double getDriveKs() {
        return driveKsTune.get();
    }

    public double getDriveKv() {
        return driveKvTune.get();
    }

}