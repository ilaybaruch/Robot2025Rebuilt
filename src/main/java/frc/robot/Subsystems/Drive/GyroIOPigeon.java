package frc.robot.Subsystems.Drive;

import java.util.Queue;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class GyroIOPigeon implements GyroIO {

    private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(pigeonCanId);
    // private final Queue<Double> yawPositionQueue;
    // private final Queue<Double> yawTimestampQueue;

    public GyroIOPigeon() {
        pigeon.reset();

        // yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        // yawPositionQueue =
        // OdometryThread.getInstance().registerSignal(pigeon::getYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.getState() != PigeonState.NoComm;
        // inputs.yawPosition = Rotation2d.fromDegrees(pigeon.getYaw()).minus()
    }

}
