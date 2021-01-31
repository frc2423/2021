package frc.robot.subsystems;

public interface IDrive {
    
    public void init();

    public void reset();

    public void switchGears();

    public double getAngle();

    public void setAngle(double angle);

    public double getLeftDistance();

    public double getRightDistance();

    public double getLeftVelocity();

    public double getRightVelocity();

    public void setSpeeds(double speed, double rot);

    public void setTankSpeeds(double leftSpeed, double rightSpeed);

    public void execute();
}
