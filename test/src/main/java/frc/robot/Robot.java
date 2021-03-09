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

    if (isSimulation()){
        new SimMotor(1, 0, 1, "lf_motor");
        new SimMotor(4, 2, 3, "lb_motor");
        new SimMotor(6, 4, 5, "rf_motor");
        new SimMotor(5, 6, 7, "rb_motor");
        new SimGyro();
        new SimBallTracker();
        
    } else {
        new NeoMotor(1, "lf_motor");
        new NeoMotor(4, "lb_motor");
        new NeoMotor(6, "rf_motor");
        new NeoMotor(5, "rb_motor");
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
}