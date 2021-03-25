// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import frc.robot.helpers.NtHelper;
import java.util.HashMap;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  private CANPIDController leadPidController;
  private CANEncoder leadEncoder;
  private HashMap<Integer, CANSparkMax> motorMap = new HashMap<Integer, CANSparkMax>();
 


  @Override
  public void robotInit() {
    NtHelper.listen("/pid/kP", (table) -> setPidsDashboard());
    NtHelper.listen("/pid/kI", (table) -> setPidsDashboard());
    NtHelper.listen("/pid/kD", (table) -> setPidsDashboard());
    NtHelper.listen("/pid/kF", (table) -> setPidsDashboard());
    NtHelper.listen("/pid/canId", (table) -> createMotorObject());
    NtHelper.listen("/pid/wheelRadius", (table) -> setConversionFactor());
    NtHelper.listen("/pid/encoderPulsesPerRotation", (table) -> setConversionFactor());
    createMotorObject();

  }

  private void zeroMotorTimeout() {
    for (CANSparkMax motor : motorMap.values()) {
      motor.setCANTimeout(0);
    }
  }

  private void createMotorObject()  {
    if (!motorMap.containsKey(getCanId())) {
      motorMap.put(getCanId(), new CANSparkMax(getCanId(), MotorType.kBrushless));
    }
    CANSparkMax leadMotor = motorMap.get(getCanId());
    leadMotor.setCANTimeout(1000);
    leadMotor.restoreFactoryDefaults();
    leadEncoder = leadMotor.getEncoder();
    leadPidController = leadMotor.getPIDController();
    setPids(leadPidController);
    setConversionFactor();
  }

  private int getCanId() {
    return (int)NtHelper.getDouble("/pid/canId", 0);
  }
  private double getWheelRadius() {
    return NtHelper.getDouble("/pid/wheelRadius", 1);
  }
  private double getEncoderPulsesPerRotation() {
    return NtHelper.getDouble("/pid/encoderPulsesPerRotation", 1);
  }

  private double getSetPoint() {
    return NtHelper.getDouble("/pid/setPoint", 0.0);
  }

  private double getP() {
    return NtHelper.getDouble("/pid/kP", 0);
  }

  private double getI() {
    return NtHelper.getDouble("/pid/kI", 0);
  }

  private double getD() {
    return NtHelper.getDouble("/pid/kD", 0);
  }

  private double getF() {
    return NtHelper.getDouble("/pid/kF", 0);
  }

  private void setPidsDashboard() {
    setPids(leadPidController);
  }

  private void setPids(CANPIDController pidController) {
    setPids(
      pidController,
      getP(),
      getI(),
      getD(),
      getF()
    );
  }

  private void setPids(CANPIDController pidController, double kP, double kI, double kD, double kF) {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setFF(kF);
  }

  private void setConversionFactor(){
    double circumference = getWheelRadius()*2*Math.PI;
    double factor = circumference/getEncoderPulsesPerRotation();

    NtHelper.setDouble("/pid/circumference", circumference);
    NtHelper.setDouble("/pid/factor", factor);
    leadEncoder.setPositionConversionFactor(factor);
    leadEncoder.setVelocityConversionFactor(factor / 60);
  }

  public double getEncoderCount() {
    return leadEncoder.getPosition() / leadEncoder.getPositionConversionFactor();
  }

  @Override
  public void teleopInit() {
    leadEncoder.setPosition(0);
  }

  @Override
  public void autonomousInit() {
    leadEncoder.setPosition(0);
  }

  @Override
  public void testInit() {
    leadEncoder.setPosition(0);
  }

  private boolean isDutyCycle() {
    return NtHelper.getBoolean("/pid/isDutyCycle", false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (isDutyCycle()) {
      leadPidController.setReference(0, ControlType.kDutyCycle);
    } else {
      System.out.println("setPoint: " +  getSetPoint());
      leadPidController.setReference(getSetPoint(), ControlType.kVelocity);
    }
    NtHelper.setDouble("/pid/encoderCount", getEncoderCount());
    NtHelper.setDouble("/pid/encoderConversionFactor", leadEncoder.getPositionConversionFactor());
    NtHelper.setDouble("/pid/velocity", leadEncoder.getVelocity());
  }
}