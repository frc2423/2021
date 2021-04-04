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

  private ArrayList<Boolean> ballReadings;
  private AnalogInput ballSensor;
  public enum StorageStates {
    NOTHING, SEESBALL, FEEDBALL
  }
  public enum ShooterStates {
    DRIVE, INTAKE, AIM, REVSHOOTER, SHOOT
  }
  private StorageStates storageState = StorageStates.NOTHING;
  private ShooterStates shooterState = ShooterStates.DRIVE;

  private double shootTopSpeed = 0.0;
  private double shootBottomSpeed = 0.0;
  private double shootFeederSpeed = 0.0;

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
    for (int i = 0; i < 25; i++) {
      ballReadings.add(false);
    }
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

  private boolean seesTarget() {
    return NtHelper.getDouble("/limelight/tv", 0.0) == 1;
  }

  private double getTargetOffset() {
    return NtHelper.getDouble("/limelight/tx", 0.0);
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

  private boolean atTarget() {
    if (seesTarget() && Math.abs(getTargetOffset()) < 2.5) {
      return true;
    }
    return false;
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("teleopInit");
  }

  public void robotPeriodic() {

    setShooterSpeeds();
    shooterStates();
    //driveJoystick();


    NtHelper.setString("/shooter/intakeState", storageState.toString());
    NtHelper.setDouble("/shooter/ballSensor", ballSensor.getValue());
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

      // transitions
      if (seesBall()) {
        storageState = StorageStates.SEESBALL;
      }
    
    } else if (storageState == StorageStates.SEESBALL) {

      // transitions
      storageState = StorageStates.FEEDBALL;
    
    } else if (storageState == StorageStates.FEEDBALL) {
      // do something
      beltMotor.setPercent(0.5);

      // transitions
      if (!seesBall()) {
        storageState = StorageStates.NOTHING;
      }
    
    }
  }

  private void setShooterSpeeds() {
    switch(getMode()) {
      case "Green Zone":
        shootFeederSpeed = 30.0;
        shootTopSpeed = 35.0;
        shootBottomSpeed = 10.0;
        break;

      case "Yellow Zone":
      case "Blue Zone":
      case "Red Zone":
        shootFeederSpeed = 11.0;
        shootTopSpeed = 27.0;
        shootBottomSpeed = 23.0;
        break;
      
      default:
        shootFeederSpeed = 0.0;
        shootTopSpeed = 0.0;
        shootBottomSpeed = 0.0;
    }
  }

  private String getMode() {
    return NtHelper.getString("/shooter/mode", "Drive");
  }

  private Boolean isShootingMode() {
    String mode = getMode();
    return (
      mode == "Green Zone"
      || mode == "Yellow Zone"
      || mode == "Blue Zone"
      || mode == "Red Zone"
    );
  }

  private void stopIntake() {
    intakeMotor.setPercent(0);
    greenWheel.setPercent(0);
    intakeValve.set(DoubleSolenoid.Value.kForward);
    beltMotor.setPercent(0);
  }

  private void stopShooter() {
    shooterFeederMotor.setSpeed(0);
    shooterBottomWheel.setSpeed(0);
    shooterTopWheel.setSpeed(0);
  }

  private void shooterStates() {
    String mode = getMode();
    switch(shooterState) {
      case DRIVE:
        stopIntake();
        stopShooter();
        driveJoystick();
        if (mode == "Collect Balls") {
          shooterState = ShooterStates.INTAKE;
        } else if (isShootingMode()) {
          shooterState = ShooterStates.AIM;
        }
        break;
      
      case INTAKE:
        intakeStates();
        intakeMotor.setPercent(.5);
        intakeValve.set(DoubleSolenoid.Value.kReverse);
        greenWheel.setPercent(0.5);
        stopShooter();
        driveJoystick();
        if (mode == "Drive") {
          shooterState = ShooterStates.DRIVE;
        } else if (isShootingMode()) {
          shooterState = ShooterStates.AIM;
        }
        break;

      case AIM:
        driveAutoAim();
        stopIntake();
        stopShooter();
        if (atTarget()) {
          shooterState = ShooterStates.REVSHOOTER;
        } else if (mode == "Drive") {
          shooterState = ShooterStates.DRIVE;
        } else if (mode == "Collect Balls") {
          shooterState = ShooterStates.INTAKE;
        }
        break;
      
      case REVSHOOTER:
        shooterFeederMotor.setSpeed(shootFeederSpeed, true);
        shooterBottomWheel.setSpeed(shootBottomSpeed, true);
        shooterTopWheel.setSpeed(shootTopSpeed, true);
        stopIntake();
        if (shooterTopWheel.getSpeed() > 0.9 * shootTopSpeed) {
          shooterState = ShooterStates.SHOOT;
        } else if (mode == "Drive") {
          shooterState = ShooterStates.DRIVE;
        } else if (mode == "Collect Balls") {
          shooterState = ShooterStates.INTAKE;
        }
        break;
      case SHOOT:
        shooterFeederMotor.setSpeed(shootFeederSpeed, true);
        shooterBottomWheel.setSpeed(shootBottomSpeed, true);
        shooterTopWheel.setSpeed(shootTopSpeed, true);
        beltMotor.setPercent(.5);
        intakeMotor.setPercent(0);
        greenWheel.setPercent(0);
        intakeValve.set(DoubleSolenoid.Value.kForward);
        arcade(0, 0);
        
        if (mode == "Drive") {
          shooterState = ShooterStates.DRIVE;
        } else if (mode == "Collect Balls") {
          shooterState = ShooterStates.INTAKE;
        }
        break;
    }
  }
}
