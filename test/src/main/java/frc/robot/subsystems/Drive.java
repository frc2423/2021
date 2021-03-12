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
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Manager;

import edu.wpi.first.wpilibj.RobotBase;

public class Drive extends Subsystem {

    private double countsPerRev = 16.35;
    private double ftPerRev = 1.57;
    private double maxSpeed = 9.0;  // feet per second

    private boolean previous_button = false;

    private DoubleSolenoid gear_switcher;

    private IMotor lf_motor; // left front motor
    private IMotor lb_motor; // left back motor
    private IMotor rf_motor; // right front motor
    private IMotor rb_motor; // right back motor

    private IGyro gyro;

    private double leftSpeed = 0.0;
    private double rightSpeed = 0.0;

    private static final double kTrackWidth = 1.9375;
    private static final double kWheelRadius = 0.5;
    private DrivePosition drivePosition;

    private double defaultP = 0.0001;
    private double defaultI = 0.00001;
    private double defaultD = 0.000015;
    private double defaultF = 0.0;

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

        setPids();
        gear_switcher = new DoubleSolenoid(0, 1);

        /*NtHelper.listen("/drive/kP", (table) -> setPids());
        NtHelper.listen("/drive/kI", (table) -> setPids());
        NtHelper.listen("/drive/kD", (table) -> setPids());
        NtHelper.listen("/drive/kF", (table) -> setPids());
        NtHelper.listen("/drive/setPoint", (table) -> setSetPoints());
*/
        drivePosition = new DrivePosition(kTrackWidth, kWheelRadius, gyro.getRotation2d());
    }

    private double getSetPoint() {
        return NtHelper.getDouble("/drive/setPoint", 0);
        
    }
    
    private void setSetPoints() {
        //leftSpeed = getSetPoint();
        //rightSpeed = getSetPoint();
    }

    private double getP() {
        // return NtHelper.getDouble("/drive/kP", 6e-5);
        // return NtHelper.getDouble("/drive/kP", 0.0001);
        return NtHelper.getDouble("/drive/kP", defaultP);
        // return NtHelper.getDouble("/drive/kP", 0.0002);
    }

    private double getI() {
        return NtHelper.getDouble("/drive/kI", defaultI);
        // return NtHelper.getDouble("/drive/kI", 1e-9);
        // return NtHelper.getDouble("/drive/kI", 0);
    }

    private double getD() {
        return NtHelper.getDouble("/drive/kD", defaultD);

        // return NtHelper.getDouble("/drive/kD", 0.0);
    }

    private double getF() {
        return NtHelper.getDouble("/drive/kF", defaultF);
        // return NtHelper.getDouble("/drive/kF", 0.0);
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

    private void setPids() {
        lb_motor.setP(getP());
        lb_motor.setI(getI());
        lb_motor.setD(getD());
        lb_motor.setF(getF());
        rb_motor.setP(getP());
        rb_motor.setI(getI());
        rb_motor.setD(getD());
        rb_motor.setF(getF());
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
        drivePosition.reset(pose, gyro.getRotation2d());
    }
     public Rotation2d getRotation2d(){
        return gyro.getRotation2d();
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

        gyro.setAngle(drivePosition.getDegrees());

        drivePosition.updateOdometry(gyro.getRotation2d(), lb_motor.getDistance(), rb_motor.getDistance());
    }

    @Override
    public void report() {
        reportValue("raw/velocity", getLeftVelocity() /maxSpeed);
        reportValue("raw/gyroAngle", gyro.getRotation2d().getDegrees());
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
        reportValue("robot heading", gyro.getRotation2d().getDegrees() + " degrees");
        reportValue("left distance", getLeftDistance() + " feet");
        reportValue("right distance", getRightDistance() + " feet");
        if (driveMode == "Arcade Voltage") {
            reportValue("desired left speed", leftSpeed * 100 + "% speed");
            reportValue("desired right speed", rightSpeed * 100 + "% speed");
        } else {
            reportValue("desired left speed", leftSpeed + " ft/s");
            reportValue("desired right speed", rightSpeed  + " ft/s");
        }

    }
    
}