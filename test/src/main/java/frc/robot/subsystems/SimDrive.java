package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.devices.IDriveMotor;
import frc.robot.devices.SimDriveMotor;
import frc.robot.devices.IGyro;
import frc.robot.devices.SimGyro;

import frc.robot.helpers.NtHelper;
import frc.robot.helpers.DriveHelper;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;


public class SimDrive implements IDrive{

    private double countsPerRev = 16.35;
    private double ftPerRev = 1.57;
    private double maxSpeed = 9.0;  // feet per second

    private DoubleSolenoid gear_switcher;

    private IDriveMotor lf_motor; // left front motor
    private IDriveMotor lb_motor; // left back motor
    private IDriveMotor rf_motor; // right front motor
    private IDriveMotor rb_motor; // right back motor

    private IGyro gyro;

    private double leftSpeed = 0.0;
    private double rightSpeed = 0.0;
    private static final double kTrackWidth = 2;
    private static final double kWheelRadius = 0.5;
    private final LinearSystem<N2, N2, N2> drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    private final DifferentialDrivetrainSim drivetrainSimulator =
      new DifferentialDrivetrainSim(
          drivetrainSystem, DCMotor.getCIM(2), 8, kTrackWidth, kWheelRadius, null);

    
    private final DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(kTrackWidth);

    private final Field2d field = new Field2d();
    private final DifferentialDriveOdometry odometry;

    
    public SimDrive () {

        double conversionFactor = 2 * Math.PI * kWheelRadius / 4096;


        lf_motor = new SimDriveMotor(1, 0, 1);
        lb_motor = new SimDriveMotor(4, 2, 3);
        rf_motor = new SimDriveMotor(6, 4, 5);
        rb_motor = new SimDriveMotor(5, 6, 7);
        gyro = new SimGyro();

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

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

        SmartDashboard.putData("Field", field);
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
        drivetrainSimulator.setPose(pose);
        odometry.resetPosition(pose, gyro.getRotation2d());
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
        return odometry.getPoseMeters();
    }

    public double getAngle() {
        return gyro.getAngle();
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
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(
            new ChassisSpeeds(Units.feetToMeters(feetPerSecond), 0, Units.degreesToRadians(degreesPerSecond))
        );
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
        drivetrainSimulator.setInputs(
          lb_motor.getPercent() * RobotController.getInputVoltage(),
          rb_motor.getPercent() * RobotController.getInputVoltage()
        );
        drivetrainSimulator.update(0.02);
        lb_motor.setEncoderPositionAndRate(
            drivetrainSimulator.getLeftPositionMeters(),
            drivetrainSimulator.getLeftVelocityMetersPerSecond()
        );
        rb_motor.setEncoderPositionAndRate(
            drivetrainSimulator.getRightPositionMeters(),
            drivetrainSimulator.getRightVelocityMetersPerSecond()
        );

        gyro.setAngle(drivetrainSimulator.getHeading().getDegrees());
        odometry.update(gyro.getRotation2d(), lb_motor.getDistance(), rb_motor.getDistance());
        field.setRobotPose(odometry.getPoseMeters());
    }
}