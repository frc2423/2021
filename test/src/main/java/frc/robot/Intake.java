package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

class Intake {
    enum State{
      INTAKEBALLS, OUTTAKE, STALLED, NOTHING
    }
  
    private State state = State.NOTHING;
    private Timer timer = new timer();
    private double zoom  = 0.6;
    private CANSparkMax motor;
    private Boolean running;
    private CANEncoder encoder;
    private double stallZone = 1500.0;
    private DoubleSolenoid intakeValve;
    private CANSparkMax greenWheel;

    private double motorSpeed = 0.0;
    private double greenWheelSpeed = 0.0;
    private DoubleSolenoid.Value intakeValue = DoubleSolenoid.Value.kOff;

    public Intake() {
        motor = CANSparkMax(7, MotorType.kBrushless);
        encoder = motor.getEncoder();
        running = false;
        intakeValve = DoubleSolenoid(2, 3);
        greenWheel = CANSparkMax(2, MotorType.kBrushless);
    }

    public init() {
      stop();
    }

    public stop (){
      running = false;
      runIntake();
      greenWheelSpeed = 0.0;
    }

    public intake() {
      running = true;
      runIntake();
      greenWheelSpeed = -.3;
    }

    public turnWheel(double speed) {
      greenWheelSpeed = speed;
    }

    public boolean isDown() {
      return intakeValve.get() == DoubleSolenoid.Value.kReverse;
    }
  public intakeUp(){
    intakeValue = DoubleSolenoid.Value.kForward;
  }
  public intakeDown(){
    intakeValue = DoubleSolenoid.Value.kReverse;
  }
   
    private runIntake(){
      
        switch(state){
            case NOTHING: 
              motorSpeed = 0.0;
              if(running){
                state = State.INTAKEBALLS;
              }
              break;
            case INTAKEBALLS :
                motorSpeed = zoom;
                if(encoder.getVelocity() > -stallZone && encoder.getVelocity() < stallZone){
                  state = State.STALLED;
                  timer.stop();
                  timer.reset();
                  timer.start();
                }
                else if (!running){
                  state = State.NOTHING;
                }
                break;
            case STALLED :
                motorSpeed = zoom;
                if(encoder.getVelocity() < -stallZone && encoder.getVelocity() > stallZone){
                  state = State.INTAKEBALLS;
                  timer.stop();
                  timer.reset();
                  timer.start();
                }
                else if(timer.get()>.5){
                  state = State.OUTTAKE;
                  timer.stop();
                  timer.reset();
                  timer.start();
                }
                else if (!running){
                  state = State.NOTHING;
                }
                break;
            case OUTTAKE :
                motorSpeed = -zoom;
                if(timer.get() > 1){
                  state = State.INTAKEBALLS;
                  timer.stop();
                  timer.reset();
                  timer.start();
                }
                else if(!running){
                  state = State.NOTHING;
                }
                break;
        }
    }

    public execute() {
      motor.set(motorSpeed);
      greenWheel.set(greenWheelSpeed);
      intakeValve.set(intakeValue);
    }
  
}