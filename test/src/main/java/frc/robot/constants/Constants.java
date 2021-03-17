package frc.robot.constants;

public class Constants {
    public static double WHEEL_RADIUS = 0.25;
    public static double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    public static double SIM_ENCODER_PULSES_PER_ROTATION = 4096;
    public static double REAL_ENCODER_PULSES_PER_ROTATION = 16.35;
    public static double SIM_DRIVE_KP = 0.0001;
    public static double SIM_DRIVE_KI = 1e-6;
    public static double SIM_DRIVE_KD = 0.0;
    public static double SIM_DRIVE_KF = 0;
    public static double REAL_DRIVE_KP = 0.0001;
    public static double REAL_DRIVE_KI = 0.00001;
    public static double REAL_DRIVE_KD = 0.000015;
    public static double REAL_DRIVE_KF = 0.0;
}

