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
    private final DrivePosition drivePosition;

    private double defaultP = 0.0001;
    private double defaultI = 0.0;
    private double defaultD = 0.000015;
    private double defaultF = 0.0;

    private boolean voltageMode = false;
    
    public Drive () {
        super("drive");
        double conversionFactor = ftPerRev / countsPerRev;
        conversionFactor = conversionFactor *10 /7.5;

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

    public void init() {
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
        lb_motor.setSpeed(driveArray[0]);
        rb_motor.setSpeed(driveArray[1]);
    }

    public void setArcadePercent(double speed, double rot) {
        var driveArray = DriveHelper.getArcadeSpeeds(speed, rot, true);
        setTankPercent(driveArray[0], driveArray[1]);
    }

    public void setTankPercent(double leftSpeed, double rightSpeed) {
        if (leftSpeed != 0) {
            lb_motor.setSpeed(leftSpeed * maxSpeed);
        } else {
            lb_motor.setPercent(0);
        }
        if (rightSpeed != 0) {
            rb_motor.setSpeed(rightSpeed * maxSpeed);
        } else {
            rb_motor.setPercent(0);
        }
    }

    public void setTankSpeeds(double leftFeetPerSecond, double rightFeetPerSecond) {
        lb_motor.setSpeed(leftFeetPerSecond);
        rb_motor.setSpeed(rightFeetPerSecond);
    }

    public void setArcadeSpeeds(double feetPerSecond, double degreesPerSecond) {
        DifferentialDriveWheelSpeeds wheelSpeeds = drivePosition.getWheelSpeeds(feetPerSecond, degreesPerSecond);
        double leftFeetPerSecond = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
        double rightFeetPerSecond = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);
    
        setTankSpeeds(leftFeetPerSecond, rightFeetPerSecond);
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
        NtHelper.setDouble("/drive/velocity", getLeftVelocity() /maxSpeed);
        NtHelper.setDouble("/drive/gyroAngle", gyro.getRotation2d().getDegrees());
        NtHelper.setDouble("/drive/encoderCount", lf_motor.getEncoderCount());
        NtHelper.setDouble("/drive/leftSpeed", leftSpeed);
        NtHelper.setDouble("/drive/rightSpeed", leftSpeed);
        NtHelper.setDouble("/drive/kP", lb_motor.getP());
        NtHelper.setDouble("/drive/kI", lb_motor.getI());
        NtHelper.setDouble("/drive/kD", lb_motor.getD());
        NtHelper.setDouble("/drive/kF", lb_motor.getF());
        NtHelper.setBoolean("/drive/voltageMode", voltageMode);
    }
}