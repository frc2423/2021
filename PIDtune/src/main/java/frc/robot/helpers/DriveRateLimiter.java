package frc.robot.helpers;

import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * A class that can be used to limit the acceleration of a drivetrain's forward speed
 * or turn rate. Deacceleration happens instantly.
 */
public class DriveRateLimiter {

    private final double accelLimit;
    private final double deaccelLimit;
    private double prevVal;
    private double prevTime;


    /**
     * Creates a new DriveRateLimiter with the given acceleration and deacceleration limits and initial value.
     * 
     * @param accelLimit The increasing rate-of-change limit, in units per second.
     * @param deaccelLimit The decreasing rate-of-change limit, in units per second.
     * @param initialValue The initial value of the input.
     */
    public DriveRateLimiter(double accelLimit, double deaccelLimit, double initialValue) {
        this.accelLimit = accelLimit;
        this.deaccelLimit = deaccelLimit;
        prevVal = initialValue;
        prevTime = Timer.getFPGATimestamp();
    }

    
    /**
     * Creates a new DriveRateLimiter with the given rate limit and an initial value of zero.
     * 
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public DriveRateLimiter(double accelLimit, double deaccelLimit) {
        this(accelLimit, deaccelLimit, 0);
    }

    /**
     * Filters the input to limit its drive rate.
     *
     * @param input The input value whose drive rate is to be limited.
     * @return The filtered value, which will not change faster than the drive rate.
     */
    public double calculate(double input) {
        boolean isSlower = Math.abs(input) < Math.abs(prevVal);
        double rateLimit = isSlower ? deaccelLimit : accelLimit;
        double currentTime = Timer.getFPGATimestamp();
        double elapsedTime = currentTime - prevTime;
        prevVal +=
            MathUtil.clamp(input - prevVal, -rateLimit * elapsedTime, rateLimit * elapsedTime);
        prevTime = currentTime;
        return prevVal;
    }

    /**
     * Resets the drive rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        prevVal = value;
        prevTime = Timer.getFPGATimestamp();
    }
}
