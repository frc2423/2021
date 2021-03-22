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
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  private CANPIDController leadPidController;
  private CANEncoder leadEncoder;
 


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

  private void createMotorObject()  {
      CANSparkMax leadMotor = new CANSparkMax(getCanId(), MotorType.kBrushless);
      leadMotor.restoreFactoryDefaults();
    leadEncoder = leadMotor.getEncoder();
    leadPidController = leadMotor.getPIDController();
    setPids(leadPidController);
    setConversionFactor();
  }

  private int getCanId() {
    return (int)NtHelper.getDouble("/pid/canId", 0.0);
  }
  private double getWheelRadius() {
    return NtHelper.getDouble("/pid/wheelRadius", 1);
  }
  private double getEncoderPulsesPerRotation() {
    return NtHelper.getDouble("/pid/encoderPulsesPerRotation", 1);
  }

  private double getSetPoint() {
    return NtHelper.getDouble("/drive/setPoint", 0.0);
  }

  private double getP() {
    return NtHelper.getDouble("/drive/kP", 0);
  }

  private double getI() {
    return NtHelper.getDouble("/drive/kI", 0);
  }

  private double getD() {
    return NtHelper.getDouble("/drive/kD", 0);
  }

  private double getF() {
    return NtHelper.getDouble("/drive/kF", 0);
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
    leadEncoder.setPositionConversionFactor(factor);
    leadEncoder.setVelocityConversionFactor(factor / 60);
  }


  /** This function is called periodically during operator control. */
  @Override
  public void robotPeriodic() {
    leadPidController.setReference(getSetPoint(), ControlType.kVelocity);
  }
}