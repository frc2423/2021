package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;

import frc.robot.devices.IDriveMotor;
import frc.robot.devices.DriveMotor;
import frc.robot.devices.SimDriveMotor;
import frc.robot.helpers.NtHelper;
import frc.robot.helpers.DriveHelper;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.geometry.Pose2d;

public class Drive {

    private double countsPerRev = 16.35;
    private double ftPerRev = 1.57;
    private double maxSpeed = 9.0;  // feet per second

    private boolean previous_button = false;

    private DoubleSolenoid gear_switcher;

    private IDriveMotor lf_motor; // left front motor
    private IDriveMotor lb_motor; // left back motor
    private IDriveMotor rf_motor; // right front motor
    private IDriveMotor rb_motor; // right back motor

    private AHRS gyro = new AHRS(Port.kMXP);

    private double leftSpeed = 0.0;
    private double rightSpeed = 0.0;
    private static final double kTrackWidth = 2;
    private static final double kWheelRadius = 0.5;
    private final LinearSystem<N2, N2, N2> drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    private final DifferentialDrivetrainSim drivetrainSimulator =
      new DifferentialDrivetrainSim(
          drivetrainSystem, DCMotor.getCIM(2), 8, kTrackWidth, kWheelRadius, null);
    public Drive() {

        double conversionFactor = RobotBase.isReal() ? ftPerRev / countsPerRev : 1;

        if (RobotBase.isReal()) {
            lf_motor = new DriveMotor(1);
            lb_motor = new DriveMotor(4);
            rf_motor = new DriveMotor(6);
            rb_motor = new DriveMotor(5);
        } else {
            lf_motor = new SimDriveMotor(1, 0, 1);
            lb_motor = new SimDriveMotor(4, 2, 3);
            rf_motor = new SimDriveMotor(6, 4, 5);
            rb_motor = new SimDriveMotor(5, 6, 7);
        }

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
        setSpeeds(0.0, 0.0);
        toLowGear();
        reset();
    }

    public void reset() {
        lb_motor.resetEncoder(0.0);
        rb_motor.resetEncoder(0.0);
        gyro.reset();
        drivetrainSimulator.setPose(new Pose2d());
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
        gyro.reset();
        gyro.setAngleAdjustment(angle);
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

    public void setSpeeds(double speed, double rot) {
        var driveArray = DriveHelper.getArcadeSpeeds(speed, rot, true);
        var lSpeed = driveArray[0];
        var rSpeed = driveArray[1];

        leftSpeed = lSpeed * maxSpeed;
        rightSpeed = rSpeed * maxSpeed;
    }

    public void setTankSpeeds(double leftSpeed, double rightSpeed) {
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
    }

    public void execute() {
        rb_motor.setSpeed(leftSpeed);
        lb_motor.setSpeed(rightSpeed);

        if (isHighGear()) {
            gear_switcher.set(DoubleSolenoid.Value.kReverse);
        } else {
            gear_switcher.set(DoubleSolenoid.Value.kForward);
        }
        drivetrainSimulator.setInputs(
          lb_motor.getPercent() * RobotController.getInputVoltage(),
          rb_motor.getPercent() * RobotController.getInputVoltage());
        drivetrainSimulator.update(0.02);
    }
}