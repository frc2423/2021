// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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
import java.util.List;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import edu.wpi.first.wpilibj.DoubleSolenoid;


import frc.robot.helpers.TrajectoryHelper;
import frc.robot.helpers.OdometryHelper;
import frc.robot.helpers.TrajectoryGeneration;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.helpers.TrajectoryGeneration;

import frc.robot.helpers.Pose;
import frc.robot.helpers.Rot;
import frc.robot.helpers.Translate;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class TrajectoryExample extends TimedRobot {


  private CANPIDController leftPidController;
  private CANPIDController rightPidController;
  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;
  private Gyro gyro = new AHRS(Port.kMXP);
  private DoubleSolenoid gear_switcher;

  private XboxController xboxController;

  String trajectoryName = "straight";

  private final Timer timer = new Timer();
  private Trajectory trajectory;
  private TrajectoryHelper trajectoryHelper = new TrajectoryHelper(Constants.TRACK_WIDTH);
  private OdometryHelper odometryHelper;
  private Trajectory exampleTrajectory;
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
    
    leftFollowerMotor.restoreFactoryDefaults();
    leftLeadMotor.restoreFactoryDefaults();
    rightFollowerMotor.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();

    leftFollowerMotor.follow(leftLeadMotor);
    rightFollowerMotor.follow(rightLeadMotor);

    setConversionFactor(leftLeadMotor, Constants.WHEEL_CIRCUMFERENCE / Constants.REAL_ENCODER_PULSES_PER_ROTATION);
    setConversionFactor(rightLeadMotor, Constants.WHEEL_CIRCUMFERENCE / Constants.REAL_ENCODER_PULSES_PER_ROTATION);

    rightFollowerMotor.setInverted(true);
    rightLeadMotor.setInverted(true);

    setPids(leftPidController);
    setPids(rightPidController);

    //trajectory = TrajectoryHelper.getTrajectory(trajectoryName);
    TrajectoryGeneration.setConfig(Constants.MAX_SPEED, Constants.MAX_ACCLERATION, trajectoryHelper);

    //giving stuff in ft
    exampleTrajectory = TrajectoryGeneration.Generate(
      new Pose(0,0, new Rot(0)), //start
      new Pose(3, 0, new Rot(0)),//end
      List.of( //waypoints
          new Translate(1, 0),
          new Translate(2, 0)
      )
    );

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
    odometryHelper.resetOdometry(exampleTrajectory.getInitialPose());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double[] speeds = trajectoryHelper.getTrajectorySpeeds(exampleTrajectory, odometryHelper.getCurrentPose(), timer.get());
    tank(speeds[0], speeds[1]);
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

  /** This function is called periodically during operator control. */
  public void resetDrive(Pose2d pose) {
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
    gyro.reset();
  }

  public void resetDrive() {
    resetDrive(new Pose2d());
  }
}
