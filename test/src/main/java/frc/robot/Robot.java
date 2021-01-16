// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.ControlType;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.Hand;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


    private DoubleSolenoid gear_switcher;

    private CANPIDController m_l_PIDController; 
    private CANPIDController m_r_PIDController; 
    private CANEncoder m_l_encoder; 
    private CANEncoder m_r_encoder;
    private int maxRPM = 5676;

    private CANSparkMax lf_motor;  //left front motor
    private CANSparkMax lb_motor;  //left back motor
    private CANSparkMax rf_motor;  //right front motor
    private CANSparkMax rb_motor;  //right back motor

    private boolean previous_button = false;
    private double joystickDeadband = 0.17;

    private XboxController xboxController;




  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    lf_motor = new CANSparkMax(1, MotorType.kBrushless);
    lb_motor = new CANSparkMax(4, MotorType.kBrushless);
    rf_motor = new CANSparkMax(6, MotorType.kBrushless);
    rb_motor = new CANSparkMax(5, MotorType.kBrushless);

    lf_motor.follow(lb_motor);
    rf_motor.follow(rb_motor);

    lb_motor.restoreFactoryDefaults();
    rb_motor.restoreFactoryDefaults();

    m_l_encoder = lf_motor.getEncoder();
    m_r_encoder = rf_motor.getEncoder();

    m_l_PIDController = lb_motor.getPIDController();
    m_r_PIDController = rb_motor.getPIDController();
    m_l_PIDController.setReference(0.0, ControlType.kVoltage);
    m_r_PIDController.setReference(0.0, ControlType.kVoltage);

    gear_switcher = new DoubleSolenoid(0, 1);

    xboxController = new XboxController(0);


  }

  private double[] getSpeeds(double xSpeedInput, double zRotationInput, boolean squareInputs) {
        var xSpeed = MathUtil.clamp(xSpeedInput, -1.0, 1.0);
        var zRotation = MathUtil.clamp(zRotationInput, -1.0, 1.0);
        if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }
        var maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
        double leftMotorOutput;
        double rightMotorOutput;
        if (xSpeed >= 0.0) {
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else if (zRotation >= 0.0) {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
        } else {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
        }
        var lm_speed = (MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * 1);
        var rm_speed = (MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * -1);
        
        double[] returnArray = { lm_speed, rm_speed };
        return returnArray;
    }



  private static double applyDeadband(double value, double deadband/*default should be 0.1*/){
    if(Math.abs(value) < deadband){
      return 0.0;
    }
    // if we made it here, we're outside the deadband
    final double slope = 1.0/(1-deadband);
    final double xDist = (Math.abs(value) - deadband);
    final double yVal = xDist * slope;
      
    if(value < 0){
      return -yVal;
    }
    return yVal;
  }  

  public void switchGears() {
    if (gear_switcher.get() == DoubleSolenoid.Value.kForward) {
      toLowGear();
    }
    else {
      toHighGear();
    }
  }

  private void toHighGear() {
    gear_switcher.set(DoubleSolenoid.Value.kForward);
  }

  private void toLowGear() {
    gear_switcher.set(DoubleSolenoid.Value.kReverse);
  }

  public void setSpeeds(double speed, double rot) {
    final double forwardbackNoDeadband = -speed;
    final double forwardBack = applyDeadband(forwardbackNoDeadband, joystickDeadband);
    final double rotation = applyDeadband(rot, joystickDeadband);
    var driveArray = this.getSpeeds(forwardBack, rotation, true);
    var lSpeed = driveArray[0];
    var rSpeed = driveArray[1];
    m_l_PIDController.setReference(lSpeed * maxRPM, ControlType.kVoltage);
    m_r_PIDController.setReference(rSpeed * maxRPM, ControlType.kVoltage);
    }


  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    setSpeeds(xboxController.getY(Hand.kRight), xboxController.getX(Hand.kRight));
    //nothing bad ever happens to the kennedys

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
