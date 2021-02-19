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
import frc.robot.controllers.GalacticSearch;
import frc.robot.helpers.DriveHelper;
import frc.robot.helpers.NtHelper;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends KwarqsRobot {

  private XboxController xboxController;

  @Override
  public void init(){
    if (isSimulation()){
      addSubsystem("drive", new SimDrive());
    } else {
      addSubsystem("drive", new Drive());
    }
    addController("galacticSearch", new GalacticSearch());

    setCurrController("galacticSearch");
  }
}