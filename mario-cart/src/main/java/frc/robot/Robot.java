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


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private ColorSensor colorSensor = new ColorSensor();

  @Override
  public void robotInit() {

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
    NtHelper.setDoubleArray("robot/colorsensor", a);
  }
}
