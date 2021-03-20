// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SPI.Port;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;

import frc.robot.helpers.DriveHelper;
import frc.robot.helpers.NtHelper;

import com.kauailabs.navx.frc.AHRS;

import java.util.HashMap;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;


import frc.robot.helpers.TrajectoryHelper;
import frc.robot.helpers.OdometryHelper;
import frc.robot.constants.Constants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private CANPIDController leftFollowerPID;
  private CANPIDController rightFollowerPID;

  private CANPIDController leftPidController;
  private CANPIDController rightPidController;
  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;
  private Gyro gyro = new AHRS(Port.kMXP);
  private DoubleSolenoid gear_switcher;

  private XboxController xboxController;

  String trajectoryName = "Straight";

  private final Timer timer = new Timer();
  private Trajectory trajectory;
  private TrajectoryHelper trajectoryHelper = new TrajectoryHelper(Constants.TRACK_WIDTH);
  private OdometryHelper odometryHelper;

  private double leftSpeed = 0.0;
  private double rightSpeed = 0.0;

  @Override
  public void robotInit() {
    xboxController = new XboxController(0);

    gear_switcher = new DoubleSolenoid(0, 1);
    odometryHelper = new OdometryHelper(gyro.getAngle());

    CANSparkMax leftFollowerMotor = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax leftLeadMotor = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax rightFollowerMotor = new CANSparkMax(6, MotorType.kBrushless);
    CANSparkMax rightLeadMotor = new CANSparkMax(5, MotorType.kBrushless);

    leftPidController = leftLeadMotor.getPIDController();
    rightPidController = rightLeadMotor.getPIDController();
    leftEncoder = leftLeadMotor.getEncoder();
    rightEncoder = rightLeadMotor.getEncoder();
    leftFollowerPID = leftFollowerMotor.getPIDController();
    rightFollowerPID = rightFollowerMotor.getPIDController();
    
    leftFollowerMotor.restoreFactoryDefaults();
    leftLeadMotor.restoreFactoryDefaults();
    rightFollowerMotor.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();

    leftFollowerMotor.follow(leftLeadMotor);
    rightFollowerMotor.follow(rightLeadMotor);

    setConversionFactor(leftLeadMotor, Constants.WHEEL_CIRCUMFERENCE / Constants.REAL_ENCODER_PULSES_PER_ROTATION);
    setConversionFactor(rightLeadMotor, Constants.WHEEL_CIRCUMFERENCE / Constants.REAL_ENCODER_PULSES_PER_ROTATION);
    setConversionFactor(rightFollowerMotor, Constants.WHEEL_CIRCUMFERENCE / Constants.REAL_ENCODER_PULSES_PER_ROTATION);
    setConversionFactor(leftFollowerMotor, Constants.WHEEL_CIRCUMFERENCE / Constants.REAL_ENCODER_PULSES_PER_ROTATION);

    NtHelper.listen("/drive/kP", (table) -> setPidsDashboard());
    NtHelper.listen("/drive/kI", (table) -> setPidsDashboard());
    NtHelper.listen("/drive/kD", (table) -> setPidsDashboard());
    NtHelper.listen("/drive/kF", (table) -> setPidsDashboard());
    NtHelper.listen("/drive/setPoint", (table) -> setSetPoint());

    rightFollowerMotor.setInverted(true);
    rightLeadMotor.setInverted(true);

    setPids(leftPidController);
    setPids(rightPidController);
    setPids(leftFollowerPID);
    setPids(rightFollowerPID);

    trajectory = TrajectoryHelper.getTrajectory(trajectoryName);
  }

  private double getSetPoint() {
    return NtHelper.getDouble("/drive/setPoint", 0.0);
  }

  private void setSetPoint() {
    leftSpeed = getSetPoint();
    rightSpeed = getSetPoint();
  }

  private double getP() {
    return NtHelper.getDouble("/drive/kP", Constants.REAL_DRIVE_KP);
  }

  private double getI() {
    return NtHelper.getDouble("/drive/kI", Constants.REAL_DRIVE_KI);
  }

  private double getD() {
    return NtHelper.getDouble("/drive/kD", Constants.REAL_DRIVE_KD);
  }

  private double getF() {
    return NtHelper.getDouble("/drive/kF", Constants.REAL_DRIVE_KF);
  }

  private void setPidsDashboard() {
    leftPidController.setP(getP());
    leftPidController.setI(getI());
    leftPidController.setD(getD());
    leftPidController.setFF(getF());
    rightPidController.setP(getP());
    rightPidController.setI(getI());
    rightPidController.setD(getD());
    rightPidController.setFF(getF());
    leftFollowerPID.setP(getP());
    leftFollowerPID.setI(getI());
    leftFollowerPID.setD(getD());
    leftFollowerPID.setFF(getF());
    rightFollowerPID.setP(getP());
    rightFollowerPID.setI(getI());
    rightFollowerPID.setD(getD());
    rightFollowerPID.setFF(getF());
  }

  private void setPids(CANPIDController pidController) {
    pidController.setP(Constants.REAL_DRIVE_KP);
    pidController.setI(Constants.REAL_DRIVE_KI);
    pidController.setD(Constants.REAL_DRIVE_KD);
    pidController.setFF(Constants.REAL_DRIVE_KF);
  }

  private void setConversionFactor(CANSparkMax motor, double factor){
    CANEncoder encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(factor);
    encoder.setVelocityConversionFactor(factor / 60);
  }

  @Override
  public void autonomousInit() {
    resetDrive();
    timer.reset();
    timer.start();
    odometryHelper.resetOdometry(gyro.getAngle());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    odometryHelper.updateOdometry(gyro.getAngle(), leftEncoder.getPosition(), rightEncoder.getPosition());
    System.out.println(" Pos X "+ odometryHelper.getCurrentPose().getX()+ " Pos Y "+ odometryHelper.getCurrentPose().getY());
    double[] speeds = trajectoryHelper.getTrajectorySpeeds(trajectory, odometryHelper.getCurrentPose(), timer.get());
    tank(speeds[0], -speeds[1]);
    System.out.println(speeds[0] + "  " + speeds[1]);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    resetDrive();
  }

  public void tank(double leftFeetPerSecond, double rightFeetPerSecond) {
    leftPidController.setReference(leftFeetPerSecond, ControlType.kVelocity);
    rightPidController.setReference(rightFeetPerSecond, ControlType.kVelocity);
  }

  public void arcade(double speed, double turn) {
    double[] speeds = DriveHelper.getArcadeSpeeds(speed, turn, false);
    double leftSpeed = speeds[0] * Constants.MAX_SPEED;
    double rightSpeed = speeds[1] * Constants.MAX_SPEED;
    tank(leftSpeed, rightSpeed);
  }

  public void tankPercent(double left, double right) {
    leftPidController.setReference(left, ControlType.kDutyCycle);
    rightPidController.setReference(right, ControlType.kDutyCycle);
  }

  public void arcadePercent(double speed, double turn) {
    double[] speeds = DriveHelper.getArcadeSpeeds(speed, turn, false);
    tankPercent(speeds[0], speeds[1]);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double x = xboxController.getX(Hand.kRight);
    double y = xboxController.getY(Hand.kRight);
    double turn = DriveHelper.applyDeadband(x);
    double speed = DriveHelper.applyDeadband(-y);
    arcade(speed, turn);
  }

  @Override
  public void testInit() {
    resetDrive();
  }

  //for working on PID tunning
  @Override
  public void testPeriodic() {
    arcade(leftSpeed, 0);
  }

  public void resetDrive(Pose2d pose) {
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
    gyro.reset();
  }

  public void resetDrive() {
    resetDrive(new Pose2d());
  }
}
