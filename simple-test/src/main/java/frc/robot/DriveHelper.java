package frc.robot;

import edu.wpi.first.wpiutil.math.MathUtil;


public class DriveHelper {

    public static double[] getArcadeSpeeds(double xSpeedInput, double zRotationInput, boolean squareInputs) {
        double xSpeed = MathUtil.clamp(xSpeedInput, -1.0, 1.0);
        double zRotation = MathUtil.clamp(zRotationInput, -1.0, 1.0);
        if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }
        var maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
        double leftMotorOutput;
        double rightMotorOutput;
        if (xSpeed >= 0.0) {
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else if (zRotation >= 0.0) {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
        } else {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
        }
        var lm_speed = MathUtil.clamp(leftMotorOutput, -1.0, 1.0);
        var rm_speed = MathUtil.clamp(rightMotorOutput, -1.0, 1.0);

        double[] returnArray = { lm_speed, rm_speed };
        return returnArray;
    }

    public static double applyDeadband(double value) {
        return applyDeadband(value, .1);
    }

    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        // if we made it here, we're outside the deadband
        final double slope = 1.0 / (1 - deadband);
        final double xDist = (Math.abs(value) - deadband);
        final double yVal = xDist * slope;

        if (value < 0) {
            return -yVal;
        }
        return yVal;
    }

    public static double squareInputs(double initVal) {
        if (initVal < 0) {
            return (Math.pow(initVal, 2)) * -1;
        }
        return Math.pow(initVal, 2);
    }
}
