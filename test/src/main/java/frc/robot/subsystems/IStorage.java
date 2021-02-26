package frc.robot.subsystems;

public interface IStorage extends ISubsystem{

    public void init();

    public void turnBelt(double speed);

    public void stop();

    public void resetCount();

    public void addBall();

    public boolean seesBall();

    public void execute();
}