package frc.robot;

import edu.wpi.first.wpilibj.SlewRateLimiter;

/**
 * A class that can be used to limit the acceleration of a drivetrain's forward speed
 * or turn rate. Deacceleration happens instantly.
 */
public class DriveRateLimiter {

    private SlewRateLimiter slewRateLimiter;
    private double prevVal;

    /**
     * Creates a new DriveRateLimiter with the given rate limit and initial value.
     * 
     * @param rateLimit The rate-of-change limit, in units per second.
     * @param initialValue The initial value of the input.
     */
    public DriveRateLimiter(double rateLimit, double initialValue) {
        prevVal = initialValue;
        slewRateLimiter = new SlewRateLimiter(rateLimit, initialValue);
    }

    
    /**
     * Creates a new DriveRateLimiter with the given rate limit and an initial value of zero.
     * 
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public DriveRateLimiter(double rateLimit) {
        this(rateLimit, 0);
    }

    /**
     * Calculates the next input.
     *
     * @param input The input value whose rate is to be limited.
     * @return The filtered value, which will not change faster than the rate limit.
     */
    public double calculate(double input) {
        boolean isSlower = Math.abs(input) < Math.abs(prevVal);
        double val = 0; 
        if (isSlower) {
            reset(input);
            val = input;
        } else {
            val = slewRateLimiter.calculate(input);
        }
        prevVal = val;
        return val;
    }

    /**
   * Resets the drive rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
    public void reset(double value) {
        slewRateLimiter.reset(value);
    }
}
