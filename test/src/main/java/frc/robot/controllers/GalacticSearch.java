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
import frc.robot.devices.IMotor;
import frc.robot.constants.Constants;



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

  private void setDefaultPids() {
    double kP = RobotBase.isReal() ? Constants.REAL_DRIVE_KP : Constants.SIM_DRIVE_KP;
    double kI = RobotBase.isReal() ? Constants.REAL_DRIVE_KI : Constants.SIM_DRIVE_KI;
    double kD = RobotBase.isReal() ? Constants.REAL_DRIVE_KD : Constants.SIM_DRIVE_KD;
    double kF = RobotBase.isReal() ? Constants.REAL_DRIVE_KF : Constants.SIM_DRIVE_KF;
    Manager.getDevice("leftLeadMotor", IMotor.class).setPidf(kP, kI, kD, kF);
    Manager.getDevice("rightLeadMotor", IMotor.class).setPidf(kP, kI, kD, kF);
  }

  @Override
  public void autonomousInit() {
    String field = detectField();
    follower.addTrajectory(field);
    follower.initFollowing(field);
    intake.intakeDown();
    setDefaultPids();
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
    setDefaultPids();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    System.out.println("RUNNING GLACTIC");

    double x = RobotBase.isReal() ? xboxController.getX(Hand.kRight) : xboxController.getRawAxis(0);
    double y = RobotBase.isReal() ? xboxController.getY(Hand.kRight) : xboxController.getRawAxis(1);



    driveBase.setArcadePercent(
      DriveHelper.squareInputs(DriveHelper.applyDeadband(-y, joystickDeadband)), 
      DriveHelper.squareInputs(DriveHelper.applyDeadband(x, joystickDeadband))
    );

    ballTracker.giveRobotPose(driveBase.getPose());
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //NtHelper.setDouble("/drive/velocity", driveBase.getLeftVelocity());

   // double y = NtHelper.getDouble("/drive/setPoint", 0);

    //driveBase.setTankSpeeds(y, y);

    if (xboxController.getAButton()) {
        shooter.execute();
    } 


  }
}
