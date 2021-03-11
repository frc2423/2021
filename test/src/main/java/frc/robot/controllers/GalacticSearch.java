// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID.Hand; // W
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.XboxController; // A
import edu.wpi.first.wpilibj.RobotBase; // R

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.helpers.DriveHelper;
import frc.robot.Manager;
import frc.robot.TrajectoryFollower;
import frc.robot.devices.IBallTracker;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class GalacticSearch extends Controller {

  private XboxController xboxController;
  private Drive driveBase;
  private Shooter shooter;
  private IBallTracker ballTracker;

  private double joystickDeadband = 0.17;

  private TrajectoryFollower follower;

  private Intake intake;

  String trajectoryJSON = "Forward";

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

    intake = Manager.getSubsystem("intake", Intake.class);

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
    String field = detectField();
    follower.addTrajectory(field);
    follower.initFollowing(field);
    intake.intakeDown();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    follower.follow();
    intake.intake();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    driveBase.begin();
    driveBase.setDefaultPIDs();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    System.out.println("RUNNING GLACTIC");

    double x = RobotBase.isReal() ? xboxController.getX(Hand.kRight) : xboxController.getRawAxis(0);
    double y = RobotBase.isReal() ? xboxController.getY(Hand.kRight) : xboxController.getRawAxis(1);


    System.out.println("Y:" + DriveHelper.squareInputs(DriveHelper.applyDeadband(-y, joystickDeadband)));

    driveBase.setArcadePercent(
      DriveHelper.squareInputs(DriveHelper.applyDeadband(-y, joystickDeadband)), 
      DriveHelper.squareInputs(DriveHelper.applyDeadband(x, joystickDeadband))
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
