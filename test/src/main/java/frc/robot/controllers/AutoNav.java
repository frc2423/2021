// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

//k
import edu.wpi.first.wpilibj.GenericHID.Hand; // W
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.XboxController; // A
import edu.wpi.first.wpilibj.RobotBase; // R
import edu.wpi.first.wpilibj.SlewRateLimiter;

import frc.robot.subsystems.Drive; // Q
import frc.robot.subsystems.Shooter;
import frc.robot.helpers.DriveHelper;
import frc.robot.Manager;
import frc.robot.TrajectoryFollower;
import frc.robot.devices.IBallTracker;
import frc.robot.helpers.DriveHelper;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class AutoNav extends Controller {

  private XboxController xboxController;
  private Drive driveBase;
  private Shooter shooter;
  private IBallTracker ballTracker;

  private double joystickDeadband = 0.17;

  private TrajectoryFollower follower;

  String trajectoryJSON = "Straight";

  private SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  private double prevSpeed = 0;
  private double prevRotation = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {

    xboxController = new XboxController(0);

    driveBase = Manager.getSubsystem("drive", Drive.class);
    shooter = Manager.getSubsystem("shooter", Shooter.class);

    ballTracker = Manager.getDevice("ballTracker", IBallTracker.class);

    follower = new TrajectoryFollower(driveBase);
    follower.addTrajectory(trajectoryJSON);

    ballTracker.addSimulatedBall(10, 0);
  }

  public String detectField() {
    double dist = ballTracker.getDistanceFromTarget();
    double angle = ballTracker.getAngleFromTarget();
    double threshold = Units.feetToMeters(10);
    double blueAngle = 10; // needs to be changed
    double redAngle = 10; // nneds to be changed

    if(dist > threshold){
      if(angle > blueAngle){
        return "GalacticSearchBlueA";
      } else {
        return "GalacticSearchBlueB";
      }
    } else {
      if(angle > redAngle){
        return "GalacticSearchRedA";
      } else {
        return "GalacticSearchRedB";
      }
    }
  }

  @Override
  public void autonomousInit() {
    follower.stopFollowing();
    follower.initFollowing(trajectoryJSON);
    driveBase.setDefaultPIDs();
    driveBase.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    follower.follow();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    follower.stopFollowing();
    driveBase.begin();
    driveBase.setDefaultPIDs();
    driveBase.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    double x = DriveHelper.applyDeadband(
      RobotBase.isReal() ? xboxController.getX(Hand.kRight) : xboxController.getRawAxis(0)
    );
    double y = -DriveHelper.applyDeadband(
      RobotBase.isReal() ? xboxController.getY(Hand.kRight) : xboxController.getRawAxis(1)
    );
    double turnRate = 0;
    double speed = 0;

    boolean isXSlower = Math.abs(x) < Math.abs(prevRotation);
    boolean isYSlower = Math.abs(y) < Math.abs(prevSpeed);
    if (isXSlower){
      rotLimiter.reset(x);
      turnRate = x;
    } else {
      turnRate = rotLimiter.calculate(x);
    }
    if (isYSlower){
      speedLimiter.reset(y);
      speed = y;
    } else {
      speed = speedLimiter.calculate(y);
    }

    driveBase.setArcadePercent(speed, turnRate); 
    prevRotation = turnRate;
    prevSpeed = speed;

   // driveBase.setArcadeSpeeds(2,0);
   //driveBase.setArcadeSpeeds(4, 0);

    if (xboxController.getBumperPressed(Hand.kLeft)) {
      driveBase.switchGears();
    }

    ballTracker.giveRobotPose(driveBase.getPose());
  }

  @Override
  public void testInit() {
    follower.stopFollowing();
    driveBase.reset();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //NtHelper.setDouble("/drive/velocity", driveBase.getLeftVelocity());

   // double y = NtHelper.getDouble("/drive/setPoint", 0);

    //driveBase.setTankSpeeds(y, y);


    if (xboxController.getBumperPressed(Hand.kLeft)) {
      driveBase.switchGears();
    }

    if (xboxController.getAButton()) {
    } 


  }
}