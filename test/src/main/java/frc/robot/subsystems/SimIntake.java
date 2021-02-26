package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.devices.SimMotor;
import frc.robot.devices.IMotor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class SimIntake implements IIntake{
    enum State{
    INTAKEBALLS, OUTTAKE, STALLED, NOTHING
    }

    private State state = State.NOTHING;
    private Timer timer = new Timer();
    private double zoom  = 0.6;
    private IMotor motor;
    private Boolean running;
    private double stallZone = 1500.0;
    private DoubleSolenoid intakeValve;
    private IMotor greenWheel;

    private double motorSpeed = 0.0;
    private double greenWheelSpeed = 0.0;
    private DoubleSolenoid.Value intakeValue = DoubleSolenoid.Value.kOff;

    public SimIntake() {
        motor = new SimMotor(7, 8, 9);
        running = false;
        intakeValve = new DoubleSolenoid(2, 3);
        greenWheel = new SimMotor(2, 10, 11);
    }

    public void init() {
    stop();
    }

    public void stop (){
    running = false;
    runIntake();
    greenWheelSpeed = 0.0;
    }

    public void intake() {
    running = true;
    runIntake();
    greenWheelSpeed = -.3;
    }

    public void turnWheel(double speed) {
    greenWheelSpeed = speed;
    }

    public boolean isDown() {
    return intakeValve.get() == DoubleSolenoid.Value.kReverse;
    }
    public void intakeUp(){
        intakeValue = DoubleSolenoid.Value.kForward;
    }
    public void intakeDown(){
        intakeValue = DoubleSolenoid.Value.kReverse;
    }
    
    private void runIntake(){
    
        switch(state){
            case NOTHING: 
            motorSpeed = 0.0;
            if(running){
                state = State.INTAKEBALLS;
            }
            break;
            case INTAKEBALLS :
                motorSpeed = zoom;
                if(motor.getSpeed() > -stallZone && motor.getSpeed() < stallZone){
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
                if(motor.getSpeed() < -stallZone && motor.getSpeed() > stallZone){
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

    public void execute() {
    motor.setSpeed(motorSpeed);
    greenWheel.setSpeed(greenWheelSpeed);
    intakeValve.set(intakeValue);
    }
    
}