// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.team2423.util.RateLimiter; // :)
import frc.team2423.util.DriveHelper;
import frc.team2423.util.NtHelper;
import frc.team2423.devices.NeoMotor;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private ColorSensor colorSensor = new ColorSensor();

  private XboxController joystick;
  private NeoMotor leftMotor;
  private NeoMotor rightMotor;

  private RateLimiter speedLimiter = new RateLimiter(0.7, 1.2);
  private RateLimiter turnLimiter = new RateLimiter(2, 3.5);
  
  @Override
  public void robotInit() {
    colorSensor.addColor("orange", .469, .428, .102);
    colorSensor.addColor("pink", .443, .360, .198);
    colorSensor.addColor("lime", .191, .621, .188);
    colorSensor.addColor("yellow", .365, .521, .114);
    colorSensor.addColor("blue", .137, .390, .473);
    colorSensor.addColor("other", .255, .473, .272);

    joystick = new XboxController(0);
    leftMotor = new NeoMotor(1, "tortellini");
    rightMotor = new NeoMotor(2, "princess peach");
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("teleopInit");
  }

  /** This func 
   * tion is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Color color = colorSensor.getRawColor();
    System.out.println(color);
    double[] a = new double[3];
    a[0] = color.red;
    a[1] = color.green;
    a[2] = color.blue;
    NtHelper.setDouble("/robot/red", color.red);
    NtHelper.setDouble("/robot/green", color.green);
    NtHelper.setDouble("/robot/blue", color.blue);

    if (colorSensor.isColor("blue")) {
      NtHelper.setString("/robot/color", "blue");
    } else  if (colorSensor.isColor("orange")) {
      NtHelper.setString("/robot/color", "orange");

    } else  if (colorSensor.isColor("lime")) {
      NtHelper.setString("/robot/color", "lime");

    } else  if (colorSensor.isColor("pink")) {
      NtHelper.setString("/robot/color", "pink");

    } else  if (colorSensor.isColor("yellow")) {
      NtHelper.setString("/robot/color", "yellow");

    } else  if (colorSensor.isColor("other")) {
      NtHelper.setString("/robot/color", "other");

    }

    double x = joystick.getX(Hand.kRight);
    double y = joystick.getY(Hand.kLeft);
    double turn = turnLimiter.calculate(DriveHelper.applyDeadband(x));
    double speed = speedLimiter.calculate(DriveHelper.applyDeadband(-y));
    arcade(speed, turn);

  }

  public void arcade(double speed, double turn) {
    double[] speeds = DriveHelper.getArcadeSpeeds(speed, turn, false);
    leftMotor.setPercent(speeds[0]);
    rightMotor.setPercent(speeds[1]);
  }
}
