package frc.robot.Subsystems.Elevator;

import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorTuning {

    LoggedNetworkNumber kpTune = new LoggedNetworkNumber("kP", Kp);
    LoggedNetworkNumber kiTune = new LoggedNetworkNumber("ki", Ki);
    LoggedNetworkNumber kdTune = new LoggedNetworkNumber("kd", Kd);
    LoggedNetworkNumber kgTune = new LoggedNetworkNumber("kg", Kg);
    LoggedNetworkNumber kaTune = new LoggedNetworkNumber("ka", Ka);
    LoggedNetworkNumber ksTune = new LoggedNetworkNumber("ks", Ks);
    LoggedNetworkNumber kvTune = new LoggedNetworkNumber("kv", Kv);
    LoggedNetworkNumber maxAcceleration = new LoggedNetworkNumber("max acceleration", MAX_ACCELERATION);
    LoggedNetworkNumber maxVelocity = new LoggedNetworkNumber("max velocity", MAX_VELOCITY);

    public double getKp() {
        return kpTune.get();
    }

    public double getKi() {
        return kiTune.get();
    }

    public double getKd() {
        return kdTune.get();
    }

    public double getKv() {
        return kvTune.get();
    }

    public double getKa() {
        return kaTune.get();
    }

    public double getKg() {
        return kgTune.get();
    }

    public double getKs() {
        return ksTune.get();
    }

    public double getMaxAcceleration() {
        return maxAcceleration.get();
    }

    public double getMaxVelocity() {
        return maxVelocity.get();
    }
}