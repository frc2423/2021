// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import java.util.List;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SimDrive;
import frc.robot.subsystems.IDrive;
import frc.robot.helpers.DriveHelper;
import edu.wpi.first.wpilibj.util.Units;

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

  private final RamseteController ramsete = new RamseteController();
  private final Timer timer = new Timer();
  private Trajectory trajectory;


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

    trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 2, new Rotation2d()),
            List.of(),
            new Pose2d(6, 4, new Rotation2d()),
            new TrajectoryConfig(2, 2));

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
    timer.reset();
    timer.start();
    driveBase.reset(trajectory.getInitialPose());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double elapsed = timer.get();
    Trajectory.State reference = trajectory.sample(elapsed);
    ChassisSpeeds speeds = ramsete.calculate(driveBase.getPose(), reference);

    // set robot speed and rotation 
    driveBase.setArcadeSpeeds(
        Units.metersToFeet(speeds.vxMetersPerSecond),
        Units.radiansToDegrees(speeds.omegaRadiansPerSecond)
    );
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
  public void testPeriodic() {}
}
