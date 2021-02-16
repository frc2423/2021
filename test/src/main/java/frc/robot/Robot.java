// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot; // K
import edu.wpi.first.wpilibj.GenericHID.Hand; // W
import edu.wpi.first.wpilibj.XboxController; // A
import edu.wpi.first.wpilibj.RobotBase; // R

import frc.robot.subsystems.Drive; // Q
import frc.robot.subsystems.SimDrive; // S
import frc.robot.subsystems.IDrive;
import frc.robot.helpers.DriveHelper;
import frc.robot.helpers.NtHelper;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private XboxController xboxController;
  private IDrive driveBase;

  private double joystickDeadband = 0.17;

  private TrajectoryFollower follower;

  String trajectoryJSON = "Barrel";

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    xboxController = new XboxController(0);
    if(isReal()){
        driveBase = new Drive();
    } else {
        driveBase = new SimDrive();
    }

    follower = new TrajectoryFollower(driveBase);

    follower.addTrajectory(trajectoryJSON);
  }

  public void switchGears() {
    driveBase.switchGears();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    driveBase.execute();
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
      switchGears();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //NtHelper.setDouble("/drive/velocity", driveBase.getLeftVelocity());

    double x = NtHelper.getDouble("/drive/setPoint", 0);
    double y = NtHelper.getDouble("/drive/setPoint", 0);

    driveBase.setArcadePercent(
      DriveHelper.applyDeadband(-y, joystickDeadband), 
      DriveHelper.applyDeadband(x, joystickDeadband)
    );
    
    

    if (xboxController.getBumperPressed(Hand.kLeft)) {
      switchGears();
    }


  }
}
