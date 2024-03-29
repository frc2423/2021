package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;

import frc.robot.devices.IMotor;
import frc.robot.devices.IGyro;
import frc.robot.helpers.DriveHelper;
import frc.robot.DrivePosition;
import frc.robot.Manager;
import frc.robot.constants.Constants;

import edu.wpi.first.wpilibj.RobotBase;

public class Drive extends Subsystem {
    
    private IMotor leftMotor;
    private IMotor rightMotor;

    private IGyro gyro;

    private double leftSpeed = 0.0;
    private double rightSpeed = 0.0;

    private DrivePosition drivePosition;

    private String driveMode = "";

    public Drive() {
        super("drive");
    }
    
    public void init() {
        leftMotor = Manager.getDevice("leftLeadMotor", IMotor.class);
        rightMotor = Manager.getDevice("rightLeadMotor", IMotor.class);
        gyro = Manager.getDevice("gyro", IGyro.class);

        drivePosition = new DrivePosition(Constants.TRACK_WIDTH, Constants.WHEEL_RADIUS, gyro.getAngle());
    }

    public void setPids(double kP, double kI, double kD, double kF) {
        leftMotor.setPidf(kP, kI, kD, kF);
        rightMotor.setPidf(kP, kI, kD, kF);
    }

    public void begin() {
        setTankPercent(0.0, 0.0);
        reset();
    }

    public void reset(Pose2d pose) {
        leftMotor.resetEncoder(0.0);
        rightMotor.resetEncoder(0.0);
        gyro.reset();
        drivePosition.reset(pose, gyro.getAngle());
    }

    public void reset() {
        reset(new Pose2d());
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
        return leftMotor.getDistance();
    }

    public double getRightDistance() {
        return rightMotor.getDistance();
    }

    public double getLeftVelocity() {
        return leftMotor.getSpeed();
    }

    public double getRightVelocity() {
        return rightMotor.getSpeed();
    }

    public void setArcadeVoltage(double speed, double rot) {
        var driveArray = DriveHelper.getArcadeSpeeds(speed, rot, true);
        leftSpeed = driveArray[0];
        rightSpeed = driveArray[1];
        leftMotor.setPercent(leftSpeed);
        rightMotor.setPercent(rightSpeed);
        driveMode = "Arcade Voltage";
    }

    public void setArcadePercent(double speed, double rot) {
        var driveArray = DriveHelper.getArcadeSpeeds(speed, rot, true);
        setTankPercent(driveArray[0], driveArray[1]);
        driveMode = "Arcade Speed Percent";
    }

    public void setTankPercent(double leftSpeed, double rightSpeed) {
        if (leftSpeed != 0) {
            this.leftSpeed = leftSpeed * Constants.MAX_SPEED;
            leftMotor.setSpeed(this.leftSpeed);
        } else {
            this.leftSpeed = 0;
            leftMotor.setPercent(this.leftSpeed);
        }
        if (rightSpeed != 0) {
            this.rightSpeed = rightSpeed * Constants.MAX_SPEED;
            rightMotor.setSpeed(this.rightSpeed);
        } else {
            this.rightSpeed = 0;
            rightMotor.setPercent(this.rightSpeed);
        }
        driveMode = "Tank Speed Percent";
    }

    public void setTankSpeeds(double leftFeetPerSecond, double rightFeetPerSecond) {
        this.leftSpeed = leftFeetPerSecond;
        this.rightSpeed = rightFeetPerSecond;
        leftMotor.setSpeed(leftFeetPerSecond);
        rightMotor.setSpeed(rightFeetPerSecond);
        driveMode = "Tank Speed";
    }

    public void setArcadeSpeeds(double feetPerSecond, double degreesPerSecond) {
        DifferentialDriveWheelSpeeds wheelSpeeds = drivePosition.getWheelSpeeds(feetPerSecond, degreesPerSecond);
        double leftFeetPerSecond = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
        double rightFeetPerSecond = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);
    
        setTankSpeeds(leftFeetPerSecond, rightFeetPerSecond);
        driveMode = "Arcade Speed";
    }

    public void execute() {
    
        drivePosition.setInputs(leftMotor.getPercent(), rightMotor.getPercent());
        
        leftMotor.setEncoderPositionAndRate(
            drivePosition.getLeftPos(),
            drivePosition.getLeftVel()
        );
        rightMotor.setEncoderPositionAndRate(
            drivePosition.getRightPos(),
            drivePosition.getRightVel()
        );

        if (RobotBase.isSimulation()) {
            gyro.setAngle(drivePosition.getDegrees());
        }

        drivePosition.updateOdometry(gyro.getAngle(), leftMotor.getDistance(), rightMotor.getDistance());
    }

    @Override
    public void report() {
        reportValue("raw/velocity", getLeftVelocity() / Constants.MAX_SPEED);
        reportValue("raw/gyroAngle", gyro.getAngle());
        reportValue("raw/encoderCount", leftMotor.getEncoderCount());
        reportValue("raw/leftDistance", getLeftDistance());
        reportValue("raw/rightDistance", getRightDistance());
        reportValue("raw/leftSpeed", leftSpeed);
        reportValue("raw/rightSpeed", leftSpeed);
        reportValue("raw/kP", leftMotor.getP());
        reportValue("raw/kI", leftMotor.getI());
        reportValue("raw/kD", leftMotor.getD());
        reportValue("raw/kF", leftMotor.getF());
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