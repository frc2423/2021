package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;

import frc.robot.devices.IDriveMotor;
import frc.robot.devices.DriveMotor;
import frc.robot.devices.Gyro;
import frc.robot.devices.IGyro;
import frc.robot.helpers.NtHelper;
import frc.robot.helpers.DriveHelper;
import frc.robot.DrivePosition;

public class Drive implements IDrive, ISubsystem{

    private double countsPerRev = 16.35;
    private double ftPerRev = 1.57;
    private double maxSpeed = 9.0;  // feet per second

    private boolean previous_button = false;

    private DoubleSolenoid gear_switcher;

    private IDriveMotor lf_motor; // left front motor
    private IDriveMotor lb_motor; // left back motor
    private IDriveMotor rf_motor; // right front motor
    private IDriveMotor rb_motor; // right back motor

    private IGyro gyro;

    private double leftSpeed = 0.0;
    private double rightSpeed = 0.0;

    private static final double kTrackWidth = 1.9375;
    private static final double kWheelRadius = 0.5;
    private final DrivePosition drivePosition;
    
    public Drive () {

        double conversionFactor = ftPerRev / countsPerRev;

        lf_motor = new DriveMotor(1);
        lb_motor = new DriveMotor(4);
        rf_motor = new DriveMotor(6);
        rb_motor = new DriveMotor(5);
        gyro = new Gyro();


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

        NtHelper.listen("/drive/kP", (table) -> setPids());
        NtHelper.listen("/drive/kI", (table) -> setPids());
        NtHelper.listen("/drive/kD", (table) -> setPids());
        NtHelper.listen("/drive/kF", (table) -> setPids());
        NtHelper.listen("/drive/setPoint", (table) -> setSetPoints());

        drivePosition = new DrivePosition(kTrackWidth, kWheelRadius, gyro.getRotation2d());
    }

    private double getSetPoint() {
        return NtHelper.getDouble("/drive/setPoint", 0);
        
    }
    
    private void setSetPoints() {
        leftSpeed = getSetPoint();
        rightSpeed = getSetPoint();
    }

    private double getP() {
        // return NtHelper.getDouble("/drive/kP", 6e-5);
        // return NtHelper.getDouble("/drive/kP", 0.0001);
        return NtHelper.getDouble("/drive/kP", 6e-6);
        // return NtHelper.getDouble("/drive/kP", 0.0002);
    }

    private double getI() {
        return NtHelper.getDouble("/drive/kI", 1e-7);
        // return NtHelper.getDouble("/drive/kI", 1e-9);
        // return NtHelper.getDouble("/drive/kI", 0);
    }

    private double getD() {
        return NtHelper.getDouble("/drive/kD", 0);

        // return NtHelper.getDouble("/drive/kD", 0.0);
    }

    private double getF() {
        return NtHelper.getDouble("/drive/kF", 0.000015);
        // return NtHelper.getDouble("/drive/kF", 0.0);
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

    public void init() {
        setTankPercent(0.0, 0.0);
        toLowGear();
        reset();
    }

    public void reset(Pose2d pose) {
        lb_motor.resetEncoder(0.0);
        rb_motor.resetEncoder(0.0);
        gyro.reset();
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
        return null;
    }

    public double getLeftDistance() {
        return lb_motor.getDistance();
    }

    public double getRightDistance() {
        return -rb_motor.getDistance();
    }

    public double getLeftVelocity() {
        return lb_motor.getSpeed();
    }

    public double getRightVelocity() {
        return -rb_motor.getSpeed();
    }

    public void setArcadePercent(double speed, double rot) {
        var driveArray = DriveHelper.getArcadeSpeeds(speed, rot, true);
        setTankPercent(driveArray[0], driveArray[1]);
    }

    public void setTankPercent(double leftSpeed, double rightSpeed) {
        this.leftSpeed = leftSpeed * maxSpeed;
        this.rightSpeed = rightSpeed * maxSpeed;
    }

    public void setTankSpeeds(double leftFeetPerSecond, double rightFeetPerSecond) {
        leftSpeed = leftFeetPerSecond;
        rightSpeed = rightFeetPerSecond;
    }

    public void setArcadeSpeeds(double feetPerSecond, double degreesPerSecond) {
        DifferentialDriveWheelSpeeds wheelSpeeds = drivePosition.getWheelSpeeds(feetPerSecond, degreesPerSecond);
        double leftFeetPerSecond = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
        double rightFeetPerSecond = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);
        setTankSpeeds(leftFeetPerSecond, rightFeetPerSecond);
    }

    public void execute() {
        lb_motor.setSpeed(leftSpeed);
        rb_motor.setSpeed(rightSpeed);
        System.out.println("Drive velocity" + lb_motor.getSpeed());
        NtHelper.setDouble("/drive/velocity", lb_motor.getSpeed());
        NtHelper.setDouble("/gyroAngle", getAngle());

        if (isHighGear()) {
            gear_switcher.set(DoubleSolenoid.Value.kReverse);
        } else {
            gear_switcher.set(DoubleSolenoid.Value.kForward);
        }

        drivePosition.setInputs(lb_motor.getPercent(), rb_motor.getPercent());
        drivePosition.updateOdometry(gyro.getRotation2d(), lb_motor.getDistance(), rb_motor.getDistance());
    }
}