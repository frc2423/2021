// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot2 extends TimedRobot {
  private double[] getSpeeds(double xSpeedInput, double zRotationInput, boolean squareInputs) {
        var xSpeed = MathUtil.clamp(xSpeedInput, -1.0, 1.0);
        var zRotation = MathUtil.clamp(zRotationInput, -1.0, 1.0);
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
        var lm_speed = (MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * 1);
        var rm_speed = (MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * -1);
        
        double[] returnArray = { lm_speed, rm_speed };
        return returnArray;
    }
}
