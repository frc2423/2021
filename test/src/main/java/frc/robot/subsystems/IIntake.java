package frc.robot.subsystems;

public interface IIntake extends ISubsystem{

    public void init();

    public void stop();

    public void intake();

    public void turnWheel(double speed);

    public boolean isDown();

    public void intakeUp();

    public void intakeDown();
   
    public void execute();
  
}