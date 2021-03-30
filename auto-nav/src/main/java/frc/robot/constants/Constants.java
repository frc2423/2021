package frc.robot.constants;

public class Constants {
    public static double MAX_SPEED = 9.0; // feet per second
    public static double TRACK_WIDTH = 1.9375;
    // public static double TRACK_WIDTH = 1.875; // 22.5 in (measured)
    public static double WHEEL_RADIUS = 0.25;
    public static double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    public static double SIM_ENCODER_PULSES_PER_ROTATION = 4096;
    public static double REAL_ENCODER_PULSES_PER_ROTATION = 12.23;
    public static double REAL_DRIVE_KP = 0.07;
    public static double REAL_DRIVE_KI = 0.00001;
    public static double REAL_DRIVE_KD = 0.0;
    public static double REAL_DRIVE_KF = 0.08;
    
}

