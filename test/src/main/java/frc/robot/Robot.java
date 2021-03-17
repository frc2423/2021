// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drive; // Q
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.controllers.GalacticSearch;
import frc.robot.controllers.ShooterTesting;
import frc.robot.controllers.AutoNav;
import frc.robot.devices.NeoMotor;
import frc.robot.devices.SimMotor;
import frc.robot.devices.SimBallTracker;
import frc.robot.helpers.NtHelper;
import frc.robot.devices.BallTracker;
import frc.robot.devices.Gyro;
import frc.robot.devices.SimGyro;
import frc.robot.subsystems.Shooter;
import frc.robot.constants.Constants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends KwarqsRobot {

  @Override
  public void init(){
    //Subsystems 
    new Drive();
    new Shooter();
    new Storage();
    new Intake();

    //Devices
    new NeoMotor(7,"intakeMotor");
    new NeoMotor(2,"greenWheel");
    new NeoMotor(8,"shooterFeederMotor");
    new NeoMotor(10,"shooterBottomWheel");
    new NeoMotor(11,"shooterTopWheel");
    new NeoMotor(3,"beltMotor");
    addDriveMotors();

    if (isSimulation()){
        new SimGyro();
        new SimBallTracker();
        
    } else {
        new Gyro();
        new BallTracker();

    }

    addController("Galactic Search", new GalacticSearch());
    addController("Shooter Tester", new ShooterTesting());
    addController("Auto Nav", new AutoNav());

    setCurrController("Auto Nav");
    
    NtHelper.listen("/controllerPicker/selected", (table) -> {
      setCurrController(NtHelper.getString("/controllerPicker/selected", "Auto Nav"));
    });
    NtHelper.setString("/controllerPicker/selected", "Auto Nav");
  }

  private void addDriveMotors() {
    if (isSimulation()){
      double conversionFactor = Constants.WHEEL_CIRCUMFERENCE / Constants.SIM_ENCODER_PULSES_PER_ROTATION;

      SimMotor leftFollowerMotor = new SimMotor(1, 0, 1, "leftFollowerMotor");
      SimMotor leftLeadMotor = new SimMotor(4, 2, 3, "leftLeadMotor");
      SimMotor rightFollowerMotor = new SimMotor(6, 4, 5, "rightFollowerMotor");
      SimMotor rightLeadMotor = new SimMotor(5, 6, 7, "rightLeadMotor");

      leftFollowerMotor.follow(leftLeadMotor);
      rightFollowerMotor.follow(rightLeadMotor);

      leftLeadMotor.setConversionFactor(conversionFactor);
      rightLeadMotor.setConversionFactor(conversionFactor);

      rightLeadMotor.setInverted(true);

      leftLeadMotor.setPidf(Constants.SIM_DRIVE_KP, Constants.SIM_DRIVE_KI, Constants.SIM_DRIVE_KD, Constants.SIM_DRIVE_KF);
      rightLeadMotor.setPidf(Constants.SIM_DRIVE_KP, Constants.SIM_DRIVE_KI, Constants.SIM_DRIVE_KD, Constants.SIM_DRIVE_KF);

    } else {
      double conversionFactor = Constants.WHEEL_CIRCUMFERENCE / Constants.REAL_ENCODER_PULSES_PER_ROTATION;
      NeoMotor leftFollowerMotor = new NeoMotor(1, "leftFollowerMotor");
      NeoMotor leftLeadMotor = new NeoMotor(4, "leftLeadMotor");
      NeoMotor rightFollowerMotor = new NeoMotor(6, "rightFollowerMotor");
      NeoMotor rightLeadMotor = new NeoMotor(5, "rightLeadMotor");

      leftFollowerMotor.follow(leftLeadMotor);
      rightFollowerMotor.follow(rightLeadMotor);

      leftLeadMotor.setConversionFactor(conversionFactor);
      rightLeadMotor.setConversionFactor(conversionFactor);

      leftLeadMotor.setPidf(Constants.REAL_DRIVE_KP, Constants.REAL_DRIVE_KI, Constants.REAL_DRIVE_KD, Constants.REAL_DRIVE_KF);
      rightLeadMotor.setPidf(Constants.REAL_DRIVE_KP, Constants.REAL_DRIVE_KI, Constants.REAL_DRIVE_KD, Constants.REAL_DRIVE_KF);
    }
  }
}