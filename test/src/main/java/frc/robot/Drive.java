package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.ControlType;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.Hand;

class Drive {

    private val countsPerRev = 16.35;
    private val ftPerRev = 1.57;

    private CANPIDController m_l_PIDController; 
    private CANPIDController m_r_PIDController; 
    private CANEncoder m_l_encoder; 
    private CANEncoder m_r_encoder;
    private int maxRPM = 5676;
  
    private boolean previous_button = false;

    private DoubleSolenoid gear_switcher;
    private CANSparkMax lf_motor;  //left front motor
    private CANSparkMax lb_motor;  //left back motor
    private CANSparkMax rf_motor;  //right front motor
    private CANSparkMax rb_motor;  //right back motor
  
    private double joystickDeadband = 0.17;

    private AHRS gyro = Port.kMXP;

    private double leftSpeed = 0.0;
    private double rightSpeed = 0.0;

    private double maxSpeed = 7.0;

    public void robotInit() {
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
    }

    public void init() {
        setSpeeds(0.0, 0.0);
        toLowGear();
        reset();
    }

    public void reset() {
        m_l_encoder.setPosition(0.0);
        m_r_encoder.setPosition(0.0);
        gyro.reset();
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

    private boolean isHighGear() {
        speed = NetworkTableInstance.getDefault().getEntry("/gear").getString("slow");
        return speed == "fast";
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public void setAngle(double angle) {
        gyro.reset();
        gyro.setAngleAdjustment(angle);
    }

    public double getLeftDistance() {
        return m_l_encoder.getPosition() / countsPerRev * ftPerRev;
    }

    public double getRightDistance() {
        return -m_r_encoder.getPosition() / countsPerRev * ftPerRev;
    }

    public double getLeftVelocity() {
        return m_l_encoder.getVelocity() / countsPerRev * ftPerRev / 60.0;
    }

    public double getRightVelocity() {
        return -m_r_encoder.getVelocity() / countsPerRev * ftPerRev / 60.0;
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

    public void setTankSpeeds(double leftSpeed, double rightSpeed) {
        this.leftSpeed = leftSpeed * countsPerRev / ftPerRev * 60.0;
        this.rightSpeed = -rightSpeed * countsPerRev / ftPerRev * 60.0;
    }

    private double[] getSpeeds(double xSpeedInput, double zRotationInput, boolean squareInputs) {
        double xSpeed = MathUtil.clamp(xSpeedInput, -1.0, 1.0);
        double zRotation = MathUtil.clamp(zRotationInput, -1.0, 1.0);
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

    public void execute() {
        m_l_PIDController.setReference(leftSpeed, ControlType.kVelocity);
        m_r_PIDController.setReference(rightSpeed, ControlType.kVelocity);

        if (isHighGear()) {
            gear_switcher.set(DoubleSolenoid.Value.kReverse);
        } else {
            gear_switcher.set(DoubleSolenoid.Value.kForward);
        }
    }
}