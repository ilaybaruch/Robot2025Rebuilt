package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {

    public static final int MOTOR_ID = 17;
    public static final int UP_LIMIT_SWITCH_CHANNEL = 1;
    public static final int DOWN_LIMIT_SWITCH_CHANNEL = 2;
    public static final boolean UP_SWITCH_NORMALLY_OPEN = true;// need to check
    public static final boolean DOWN_SWITCH_NORMALLY_OPEN = true;// need to check
    public static final double Kp = 3.3; // idk amen
    public static final double Ki = 0.0;
    public static final double Kd = 0.0;
    public static final double Ks = 0.0;
    public static final double Kg = 0.48;
    public static final double Kv = 0.0;
    public static final double Ka = 0.0;
    public static final double MOTOR_MAX_VOLTAGE = 12;
    public static final double MOTOR_MIN_VOLTAGE = 0;
    public static final int CURRENT_LIMIT = 40;
    public static final double VOLTAGE_COMPENSATION = 12;
    public static final double POSITION_CONVERSION_FACTOR = 1.0 / 40 /* gears */ * (2 * Math.PI)/* rads */;
    public static final boolean INVERTED = true; // need to check if true
    public static final double MAX_ACCELERATION = 16;
    public static final double MAX_VELOCITY = 3;
    public static final double TOLERANCE = Units.degreesToRadians(3.0);
    public static final double ARM_L1_POS = -1.24;
    public static final double ARM_L2_POS = -1.2;
    public static final double ARM_L3_POS = 1.1;
    public static final double ARM_L4_POS = 1.28;
    public static final double ARM_OPEN_POS = Math.PI / 2.0; // 2.13 / 2.09
    public static final double ARM_CLOSE_POS = Math.PI / -2;// -1.96
    public static final double CORAL_KG = 0;

}
