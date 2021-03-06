// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;



import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private CANSparkMax lb_motor;
  private CANSparkMax lf_motor;
  private CANSparkMax rb_motor;
  private CANSparkMax rf_motor;

  private XboxController xboxController;

  private double defaultP = 0.0001;
  private double defaultI = 0.0;
  private double defaultD = 0.000015;
  private double defaultF = 0.0;

  private double countsPerRev = 16.35;
  private double ftPerRev = 1.57;
  private double maxSpeed = 9.0;  // feet per second
  private double conversionFactor = ftPerRev / countsPerRev;


  @Override
  public void robotInit() {
    xboxController = new XboxController(0);


    lf_motor = new CANSparkMax(1, MotorType.kBrushless);
    lb_motor = new CANSparkMax(4, MotorType.kBrushless);
    rf_motor = new CANSparkMax(6, MotorType.kBrushless);
    rb_motor = new CANSparkMax(5, MotorType.kBrushless);

    lf_motor.restoreFactoryDefaults();
    lb_motor.restoreFactoryDefaults();
    rf_motor.restoreFactoryDefaults();
    rb_motor.restoreFactoryDefaults();

    setConversionFactor(lf_motor, conversionFactor);
    setConversionFactor(lb_motor, conversionFactor);
    setConversionFactor(rf_motor, conversionFactor);
    setConversionFactor(rb_motor, conversionFactor);

    rf_motor.setInverted(true);
    rb_motor.setInverted(true);

    lf_motor.follow(lb_motor);
    rf_motor.follow(rb_motor);


    // setPids(lf_motor);
    setPids(lb_motor);
    // setPids(rf_motor);
    setPids(rb_motor);

    System.out.println("robotInit");

  }

  private void setPids(CANSparkMax motor) {
    CANPIDController pidController = motor.getPIDController();
    pidController.setP(defaultP);
    pidController.setI(defaultI);
    pidController.setD(defaultD);
    pidController.setFF(defaultF);
  }

  private void setConversionFactor(CANSparkMax motor, double factor){
    CANEncoder encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(factor);
    encoder.setVelocityConversionFactor(factor / 60);
}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("teleopInit");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double left = xboxController.getY(Hand.kLeft);
    double right = xboxController.getY(Hand.kRight);

    lb_motor.getPIDController().setReference(left * maxSpeed, ControlType.kVelocity);
    rb_motor.getPIDController().setReference(right * maxSpeed, ControlType.kVelocity);
  }
}
