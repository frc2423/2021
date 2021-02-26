// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.wpilibj.TimedRobot; // K
import edu.wpi.first.wpilibj.GenericHID.Hand; // W
import edu.wpi.first.wpilibj.XboxController; // A
import edu.wpi.first.wpilibj.RobotBase; // R

import frc.robot.subsystems.Drive; // Q
import frc.robot.subsystems.SimDrive; // S
import frc.robot.subsystems.IDrive;
import frc.robot.subsystems.ISubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.helpers.DriveHelper;
import frc.robot.helpers.NtHelper;
import frc.robot.TrajectoryFollower;
import frc.robot.devices.IBallTracker;

import java.util.HashMap;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class GalacticSearch extends Controller {

  private XboxController xboxController;
  private IDrive driveBase;
  private Shooter shooter;
  private IBallTracker ballTracker;

  private double joystickDeadband = 0.17;

  private TrajectoryFollower follower;

  String trajectoryJSON = "Forward";

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit(HashMap<String, ISubsystem> subsystems, HashMap<String, Object> devices) {

    xboxController = new XboxController(0);

    driveBase = (IDrive)subsystems.get("drive");
    shooter = new Shooter();

    ballTracker = (IBallTracker)devices.get("ballTracker");

    follower = new TrajectoryFollower(driveBase);
    follower.addTrajectory(trajectoryJSON);

    ballTracker.addSimulatedBall(10, 0);
  }

  @Override
  public void autonomousInit() {
    follower.initFollowing(trajectoryJSON);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    follower.follow();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    driveBase.init();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double x = RobotBase.isReal() ? xboxController.getX(Hand.kRight) : xboxController.getRawAxis(0);
    double y = RobotBase.isReal() ? xboxController.getY(Hand.kRight) : xboxController.getRawAxis(1);

    driveBase.setArcadePercent(
      DriveHelper.applyDeadband(-y, joystickDeadband), 
      DriveHelper.applyDeadband(x, joystickDeadband)
    );

    if (xboxController.getBumperPressed(Hand.kLeft)) {
      driveBase.switchGears();
    }

    ballTracker.giveRobotPose(driveBase.getPose());

    System.out.println("Has targets:" + ballTracker.hasTargets() + ", Distance from target:" + ballTracker.getDistanceFromTarget() + ", Angle from target:" + ballTracker.getAngleFromTarget() + ", Pitch:" + ballTracker.getPitchFromTarget());
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
        shooter.execute();
    } 


  }
}
