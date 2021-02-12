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

public class Drive implements IDrive{

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

        lf_motor.follow(lb_motor);
        rf_motor.follow(rb_motor);

        setPids();
        gear_switcher = new DoubleSolenoid(0, 1);

        NtHelper.listen("/drive/kP", (table) -> setPids());
        NtHelper.listen("/drive/kI", (table) -> setPids());
        NtHelper.listen("/drive/kD", (table) -> setPids());

        drivePosition = new DrivePosition(kTrackWidth, kWheelRadius, gyro.getRotation2d());
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
        rb_motor.setSpeed(leftSpeed);
        lb_motor.setSpeed(rightSpeed);

        if (isHighGear()) {
            gear_switcher.set(DoubleSolenoid.Value.kReverse);
        } else {
            gear_switcher.set(DoubleSolenoid.Value.kForward);
        }
    }

    
    /** 
     * Update robot odometry.
     *  Needs encoders to work.
     */
    public void updateOdometry() { //ooooooooh
        //m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }


}