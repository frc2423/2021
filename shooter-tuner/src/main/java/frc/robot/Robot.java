// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private NeoMotor intakeMotor;
  private NeoMotor greenWheel;
  private NeoMotor beltMotor;
  private NeoMotor shooterFeederMotor;
  private NeoMotor shooterBottomWheel;
  private NeoMotor shooterTopWheel;
  private DoubleSolenoid intakeValve;

  private NeoMotor flMotor;
  private NeoMotor frMotor;
  private NeoMotor blMotor;
  private NeoMotor brMotor;
  private XboxController xboxController;
  private DriveRateLimiter speedLimiter = new DriveRateLimiter(0.7, 1.2);
  private DriveRateLimiter turnLimiter = new DriveRateLimiter(2, 3.5);


  @Override
  public void robotInit() {
    System.out.println("robotInit");

    intakeValve = new DoubleSolenoid(2, 3);
    intakeMotor = new NeoMotor(7);
    greenWheel = new NeoMotor(2);
    beltMotor = new NeoMotor(3);
    shooterFeederMotor = new NeoMotor(8);
    shooterBottomWheel = new NeoMotor(10);
    shooterTopWheel = new NeoMotor(11);

    flMotor = new NeoMotor(1);
    frMotor = new NeoMotor(6);
    blMotor = new NeoMotor(4);
    brMotor = new NeoMotor(5);

    greenWheel.setInverted(true);
    shooterTopWheel.setInverted(true);
    shooterFeederMotor.setInverted(true);

    frMotor.setInverted(true);
    brMotor.setInverted(true);
    flMotor.follow(blMotor);
    frMotor.follow(brMotor);

    shooterFeederMotor.setConversionFactor(.16, .8);
    shooterFeederMotor.setPidf(.005, .00003, 0, 0);

    shooterBottomWheel.setConversionFactor(.16, 1);
    shooterBottomWheel.setPidf(.05, .0001, 0, 0);

    shooterTopWheel.setConversionFactor(.16, 1);
    shooterTopWheel.setPidf(.05, .0001, 0, 0);

  }

  public void arcade(double speed, double turn) {
    double[] speeds = DriveHelper.getArcadeSpeeds(speed, turn, false);
    double leftSpeed = speeds[0];
    double rightSpeed = speeds[1];
    blMotor.setPercent(leftSpeed);
    brMotor.setPercent(rightSpeed);
  }

  private boolean runIntake() {
    return NtHelper.getBoolean("/shooter/intake", false);
  }

  private boolean runBelt() {
    return NtHelper.getBoolean("/shooter/storageMotor", false);
  }

  private double getShooterFeederMotor() {
    return NtHelper.getDouble("/shooter/shooterFeederMotor", 0.0);
  }

  private double getShooterBottomMotor() {
    return NtHelper.getDouble("/shooter/shooterBottomMotor", 0.0);
  }

  private double getShooterTopMotor() {
    return NtHelper.getDouble("/shooter/shooterTopMotor", 0.0);
  }

  private boolean runShooter() {
    return NtHelper.getBoolean("/shooter/runShooter", false);
  }

  private boolean seesTarget() {
    return NtHelper.getDouble("/limelight/tv", 0.0) == 1;
  }

  private double getTargetOffset() {
    return NtHelper.getDouble("/limelight/tx", 0.0);
  }

  private boolean shooterAutoAim() {
    return NtHelper.getBoolean("/shooter/autoAim", false);
  }

  private void driveJoystick() {
    double x = xboxController.getX(Hand.kRight);
    double y = xboxController.getY(Hand.kLeft);
    double turn = turnLimiter.calculate(DriveHelper.applyDeadband(x));
    double speed = speedLimiter.calculate(DriveHelper.applyDeadband(-y));
    arcade(speed * 0.8, turn * 0.45);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("teleopInit");
  }

  public void robotPeriodic() {

    if (runIntake()) {
      intakeMotor.setPercent(.5);
      intakeValve.set(DoubleSolenoid.Value.kReverse);
    } else {
      intakeMotor.setPercent(0);
      intakeValve.set(DoubleSolenoid.Value.kForward);
    }

    if (runIntake() || runBelt()) {
      greenWheel.setPercent(0.5);
    } else {
      greenWheel.setPercent(0);
    }

    if (runBelt()) {
      beltMotor.setPercent(0.5);
    } else {
      beltMotor.setPercent(0);
    }

    if (runShooter()) {
      shooterFeederMotor.setSpeed(getShooterFeederMotor(), true);
      shooterBottomWheel.setSpeed(getShooterBottomMotor(), true);
      shooterTopWheel.setSpeed(getShooterTopMotor(), true);
    } else {
      shooterFeederMotor.setPercent(0);
      shooterBottomWheel.setPercent(0);
      shooterTopWheel.setPercent(0);
    }
  }
}
