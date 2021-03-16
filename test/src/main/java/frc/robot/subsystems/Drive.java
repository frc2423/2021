package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;

import frc.robot.devices.IMotor;
import frc.robot.devices.IGyro;
import frc.robot.helpers.NtHelper;
import frc.robot.helpers.DriveHelper;
import frc.robot.DrivePosition;
import frc.robot.Manager;

import edu.wpi.first.wpilibj.RobotBase;

public class Drive extends Subsystem {

    public static final double countsPerRev = 16.35;
    public static final double ftPerRev = 1.57;
    public static final double maxSpeed = 9.0;  // feet per second
    public static final double kTrackWidth = 1.9375;
    public static final double kWheelRadius = 0.25;
    public static final double defaultP = 0.0001;
    public static final double defaultI = 0.00001;
    public static final double defaultD = 0.000015;
    public static final double defaultF = 0.0;

    private DoubleSolenoid gear_switcher;

    private IMotor lf_motor; // left front motor
    private IMotor lb_motor; // left back motor
    private IMotor rf_motor; // right front motor
    private IMotor rb_motor; // right back motor

    private IGyro gyro;

    private double leftSpeed = 0.0;
    private double rightSpeed = 0.0;

    private DrivePosition drivePosition;

    private String driveMode = "";

    public Drive() {
        super("drive");
    }
    
    public void init() {
        double conversionFactor = RobotBase.isReal()
            ? (ftPerRev / countsPerRev * 10 / 7.5)
            : (2 * Math.PI * kWheelRadius / 4096);

        lf_motor = Manager.getDevice("lf_motor", IMotor.class);
        lb_motor = Manager.getDevice("lb_motor", IMotor.class);
        rf_motor = Manager.getDevice("rf_motor", IMotor.class);
        rb_motor = Manager.getDevice("rb_motor", IMotor.class);
        gyro = Manager.getDevice("gyro", IGyro.class);

        lf_motor.setConversionFactor(conversionFactor);
        lb_motor.setConversionFactor(conversionFactor);
        rf_motor.setConversionFactor(conversionFactor);
        rb_motor.setConversionFactor(conversionFactor);

        rf_motor.setInverted(true);
        rb_motor.setInverted(true);

        lf_motor.follow(lb_motor);
        rf_motor.follow(rb_motor);

        setDefaultPIDs();
        gear_switcher = new DoubleSolenoid(0, 1);
        drivePosition = new DrivePosition(kTrackWidth, kWheelRadius, gyro.getAngle());
    }

    public double getP() {
        return lb_motor.getP();
    }

    public double getI() {
        return lb_motor.getI();
    }

    public double getD() {
        return lb_motor.getD();
    }

    public double getF() {
        return lb_motor.getF();
    }

    public void setDefaultPIDs() {
        lb_motor.setP(defaultP);
        lb_motor.setI(defaultI);
        lb_motor.setD(defaultD);
        lb_motor.setF(defaultF);
        rb_motor.setP(defaultP);
        rb_motor.setI(defaultI);
        rb_motor.setD(defaultD);
        rb_motor.setF(defaultF);
    }

    public void setPids(double kP, double kI, double kD, double kF) {
        lb_motor.setP(kP);
        lb_motor.setI(kI);
        lb_motor.setD(kD);
        lb_motor.setF(kF);
        rb_motor.setP(kP);
        rb_motor.setI(kI);
        rb_motor.setD(kD);
        rb_motor.setF(kF);
    }

    public void begin() {
        setTankPercent(0.0, 0.0);
        toLowGear();
        reset();
    }

    public void reset(Pose2d pose) {
        lb_motor.resetEncoder(0.0);
        rb_motor.resetEncoder(0.0);
        gyro.reset();
        drivePosition.reset(pose, gyro.getAngle());
    }

    public void reset() {
        reset(new Pose2d());
    }

    public void switchGears() {
        if (gear_switcher.get() == DoubleSolenoid.Value.kForward) {
            toLowGear();
        } else {
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
        String speed = NetworkTableInstance.getDefault().getEntry("/gear").getString("slow");
        return speed == "fast";
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public void setAngle(double angle) {
        gyro.setAngle(angle);
    }

    public Pose2d getPose() {
        return drivePosition.getPose();
    }

    public double getLeftDistance() {
        return lb_motor.getDistance();
    }

    public double getRightDistance() {
        return rb_motor.getDistance();
    }

    public double getLeftVelocity() {
        return lb_motor.getSpeed();
    }

    public double getRightVelocity() {
        return rb_motor.getSpeed();
    }

    public void setArcadeVoltage(double speed, double rot) {
        var driveArray = DriveHelper.getArcadeSpeeds(speed, rot, true);
        leftSpeed = driveArray[0];
        rightSpeed = driveArray[1];
        lb_motor.setPercent(leftSpeed);
        rb_motor.setPercent(rightSpeed);
        driveMode = "Arcade Voltage";
    }

    public void setArcadePercent(double speed, double rot) {
        var driveArray = DriveHelper.getArcadeSpeeds(speed, rot, true);
        setTankPercent(driveArray[0], driveArray[1]);
        driveMode = "Arcade Speed Percent";
    }

    public void setTankPercent(double leftSpeed, double rightSpeed) {
        if (leftSpeed != 0) {
            this.leftSpeed = leftSpeed * maxSpeed;
            lb_motor.setSpeed(this.leftSpeed);
        } else {
            this.leftSpeed = 0;
            lb_motor.setPercent(this.leftSpeed);
        }
        if (rightSpeed != 0) {
            this.rightSpeed = rightSpeed * maxSpeed;
            rb_motor.setSpeed(this.rightSpeed);
        } else {
            this.rightSpeed = 0;
            rb_motor.setPercent(this.rightSpeed);
        }
        driveMode = "Tank Speed Percent";
    }

    public void setTankSpeeds(double leftFeetPerSecond, double rightFeetPerSecond) {
        this.leftSpeed = leftFeetPerSecond;
        this.rightSpeed = rightFeetPerSecond;
        lb_motor.setSpeed(leftFeetPerSecond);
        rb_motor.setSpeed(rightFeetPerSecond);
        driveMode = "Tank Speed";
    }

    public void setArcadeSpeeds(double feetPerSecond, double degreesPerSecond) {
        DifferentialDriveWheelSpeeds wheelSpeeds = drivePosition.getWheelSpeeds(feetPerSecond, degreesPerSecond);
        double leftFeetPerSecond = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
        double rightFeetPerSecond = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);
    
        setTankSpeeds(leftFeetPerSecond, rightFeetPerSecond);
        driveMode = "Arcade Speed";
    }

    public double getEncoder(){
        return lb_motor.getEncoderCount();
    }

    public void execute() {
    
        if (isHighGear()) {
            gear_switcher.set(DoubleSolenoid.Value.kReverse);
        } else {
            gear_switcher.set(DoubleSolenoid.Value.kForward);
        }

        drivePosition.setInputs(lb_motor.getPercent(), rb_motor.getPercent());
        
        lb_motor.setEncoderPositionAndRate(
            drivePosition.getLeftPos(),
            drivePosition.getLeftVel()
        );
        rb_motor.setEncoderPositionAndRate(
            drivePosition.getRightPos(),
            drivePosition.getRightVel()
        );

        if (RobotBase.isSimulation()) {
            gyro.setAngle(drivePosition.getDegrees());
        }

        drivePosition.updateOdometry(gyro.getAngle(), lb_motor.getDistance(), rb_motor.getDistance());
    }

    @Override
    public void report() {
        reportValue("raw/velocity", getLeftVelocity() /maxSpeed);
        reportValue("raw/gyroAngle", gyro.getAngle());
        reportValue("raw/encoderCount", lf_motor.getEncoderCount());
        reportValue("raw/leftDistance", getLeftDistance());
        reportValue("raw/rightDistance", getRightDistance());
        reportValue("raw/leftSpeed", leftSpeed);
        reportValue("raw/rightSpeed", leftSpeed);
        reportValue("raw/kP", lb_motor.getP());
        reportValue("raw/kI", lb_motor.getI());
        reportValue("raw/kD", lb_motor.getD());
        reportValue("raw/kF", lb_motor.getF());
        reportValue("raw/driveMode", driveMode);

        reportValue("left velocity", getLeftVelocity() + " ft/s");
        reportValue("right velocity", getRightVelocity() + " ft/s");
        reportValue("robot heading", gyro.getAngle() + " degrees");
        reportValue("left distance", getLeftDistance() + " feet");
        reportValue("right distance", getRightDistance() + " feet");
        if (driveMode == "Arcade Voltage") {
            reportValue("desired left speed", leftSpeed * 100 + "% speed");
            reportValue("desired right speed", rightSpeed * 100 + "% speed");
        } else {
            reportValue("desired left speed", leftSpeed + " ft/s");
            reportValue("desired right speed", rightSpeed  + " ft/s");
        }
        reportValue("Pose/X", drivePosition.getPose().getX());
        reportValue("Pose/Y", drivePosition.getPose().getY());
        reportValue("Pose/Rotation", drivePosition.getPose().getRotation().getDegrees());

    }
}