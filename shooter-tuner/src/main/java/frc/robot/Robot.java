// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.AnalogInput;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.CameraServer;


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
  private XboxController xboxController = new XboxController(0);
  private DriveRateLimiter speedLimiter = new DriveRateLimiter(0.7, 1.2);
  private DriveRateLimiter turnLimiter = new DriveRateLimiter(2, 3.5);

  private int numBalls = 0;
  private ArrayList<Boolean> ballReadings;
  private AnalogInput ballSensor;
  public enum StorageStates {
    NOTHING, SEESBALL, FEEDBALL
  }
  private StorageStates storageState = StorageStates.NOTHING;

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

    ballReadings = new ArrayList<Boolean>();
    ballSensor = new AnalogInput(0);
    CameraServer.getInstance().startAutomaticCapture();
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

  private void driveAutoAim() {
    if (seesTarget()) {
      double targetOffset = getTargetOffset();
      if (Math.abs(targetOffset) < 2.5){
        arcade(0.0, 0.0);
      } else if (targetOffset < -0.5) {
        arcade(0.0, -0.08);
      } else if (targetOffset > 0.5) {
        arcade(0.0, 0.08);
      } else {
        arcade(0.0, 0.0);
      }
    } else {
      arcade(0.0, 0.0);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("teleopInit");
  }

  public void robotPeriodic() {

    if (runIntake()) {
      intakeStates();
      intakeValve.set(DoubleSolenoid.Value.kReverse);
    } else {
      intakeMotor.setPercent(0);
      greenWheel.setPercent(0);
      intakeValve.set(DoubleSolenoid.Value.kForward);
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

    if (shooterAutoAim()) {
      driveAutoAim();
    } else {
      driveJoystick();
    }
  }

  private boolean seesBall() {
    boolean ballReading = ballSensor.getValue() > 500;
    ballReadings.remove(0);
    ballReadings.add(ballReading);

    int trueReadings = 0;
    int falseReadings = 0;

    for (boolean reading : ballReadings) {
      if (reading) {
        trueReadings++;
      } else {
        falseReadings++;
      }
    }
    return trueReadings > falseReadings;
  }

  private void intakeStates() {
    if (storageState == StorageStates.NOTHING) {
      // do something
      beltMotor.setPercent(0.0);
      greenWheel.setPercent(0.0);

      // transitions
      else if (seesBall() && numBalls < 5) {
        storageState = StorageStates.SEESBALL;
      }
    
    } else if (storageState == StorageStates.SEESBALL) {
      // do something
      numBalls++;

      // transitions
      storageState = StorageStates.FEEDBALL;
    
    } else if (storageState == StorageStates.FEEDBALL) {
      // do something
      if (numBalls < 5) {
        beltMotor.setPercent(0.5);
        greenWheel.setPercent(0.5);
      } else {
        beltMotor.setPercent(0.0);
        greenWheel.setPercent(0.0);
      }

      // transitions
      if (!seesBall()) {
        storageState = StorageStates.NOTHING;
      }
    
    }
  }
}
