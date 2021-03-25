// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private NeoMotor greenWheel;
  private NeoMotor beltMotor;
  private NeoMotor shooterFeederMotor;
  private NeoMotor shooterBottomWheel;
  private NeoMotor shooterTopWheel;

  @Override
  public void robotInit() {
    System.out.println("robotInit");

    greenWheel = new NeoMotor(2);
    beltMotor = new NeoMotor(3);
    shooterFeederMotor = new NeoMotor(8);
    shooterBottomWheel = new NeoMotor(10);
    shooterTopWheel = new NeoMotor(11);

    shooterFeederMotor.setConversionFactor(1, 1);
    shooterFeederMotor.setPidf(1, 0, 0, 0);

    shooterBottomWheel.setConversionFactor(.16, 1);
    shooterBottomWheel.setPidf(.05, .0001, 0, 0);

    shooterTopWheel.setConversionFactor(.16, 1);
    shooterTopWheel.setPidf(.05, .0001, 0, 0);

  }

  private boolean getGreenWheel() {
    return NtHelper.getBoolean("/shooter/greenWheel", false);
  }

  private boolean getStorageMotor() {
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

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("teleopInit");
  }

  public void robotPeriodic() {
    if (getGreenWheel()) {
      greenWheel.setPercent(0.5);
    } else {
      greenWheel.setPercent(0);
    }
    if (getStorageMotor()) {
      beltMotor.setPercent(0.5);
    } else {
      beltMotor.setPercent(0);
    }
    shooterFeederMotor.setSpeed(getShooterFeederMotor());
    shooterBottomWheel.setSpeed(getShooterBottomMotor());
    shooterTopWheel.setSpeed(getShooterTopMotor());
  }
}
