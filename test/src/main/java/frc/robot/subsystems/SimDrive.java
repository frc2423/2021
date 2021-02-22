package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.DrivePosition;
import frc.robot.devices.IMotor;
import frc.robot.devices.SimMotor;
import frc.robot.devices.IGyro;
import frc.robot.devices.SimGyro;

import frc.robot.helpers.NtHelper;
import frc.robot.helpers.DriveHelper;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;


public class SimDrive implements IDrive, ISubsystem{

    private double maxSpeed = 9.0;  // feet per second

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

    
    public SimDrive () {

        double conversionFactor = 2 * Math.PI * kWheelRadius / 4096;


        lf_motor = new SimDriveMotor(1, 0, 1);
        lb_motor = new SimDriveMotor(4, 2, 3);
        rf_motor = new SimDriveMotor(6, 4, 5);
        rb_motor = new SimDriveMotor(5, 6, 7);
        gyro = new SimGyro();

        lf_motor.setConversionFactor(conversionFactor);
        lb_motor.setConversionFactor(conversionFactor);
        rf_motor.setConversionFactor(conversionFactor);
        rb_motor.setConversionFactor(conversionFactor);

        lf_motor.follow(lb_motor);
        rf_motor.follow(rb_motor);

        drivePosition = new DrivePosition(kTrackWidth, kWheelRadius, gyro.getRotation2d());

        setPids();
        gear_switcher = new DoubleSolenoid(0, 1);

        NtHelper.listen("/drive/kP", (table) -> setPids());
        NtHelper.listen("/drive/kI", (table) -> setPids());
        NtHelper.listen("/drive/kD", (table) -> setPids());

        /*
            Link to all .json and .pngs to be used in field simulation.
            https://github.com/wpilibsuite/PathWeaver/tree/master/src/main/resources/edu/wpi/first/pathweaver
            In simulations
        */

    }

    private double getP() {
        return NtHelper.getDouble("/drive/kP", 0.0001);
    }

    private double getI() {
        return NtHelper.getDouble("/drive/kI", 1e-6);
    }

    private double getD() {
        return NtHelper.getDouble("/drive/kD", 0.0);
    }

    private void setPids() {
        lb_motor.setP(getP());
        lb_motor.setI(getI());
        lb_motor.setD(getD());
        rb_motor.setP(getP());
        rb_motor.setI(getI());
        rb_motor.setD(getD());
    }

    public void init() {
        setArcadeSpeeds(0.0, 0.0);
        toLowGear();
        reset();
    }

    public void reset() {
        reset(new Pose2d());
    }

    public void reset(Pose2d pose) {
        lb_motor.resetEncoder(0.0);
        rb_motor.resetEncoder(0.0);
        gyro.reset();
        drivePosition.reset(pose, gyro.getRotation2d());
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

    public Pose2d getPose() {
        return drivePosition.getPose();
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public Rotation2d getRotation2d(){
        return gyro.getRotation2d();
    }

    public void setAngle(double angle) {
        gyro.setAngle(angle);
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
        DifferentialDriveWheelSpeeds wheelSpeeds = drivePosition.getWheelSpeeds(feetPerSecond, degreesPerSecond); // comment (very helpful)
        double leftFeetPerSecond = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
        double rightFeetPerSecond = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);
        setTankSpeeds(leftFeetPerSecond, rightFeetPerSecond);
    }

    public void execute() {

        lb_motor.setSpeed(leftSpeed);
        rb_motor.setSpeed(rightSpeed);

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
}