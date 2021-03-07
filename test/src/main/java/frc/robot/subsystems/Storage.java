package frc.robot.subsystems;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.devices.IMotor;
import frc.robot.devices.NeoMotor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Storage implements Subsystem{
    private IMotor beltMotor;
    private AnalogInput ballSensor;
  
    private final int BALL_READING_COUNT = 25;
    private ArrayList<Boolean> ballReadings = new ArrayList<Boolean>();

    private double beltMotorSpeed = 0.0;
  
  
    public Storage (){
      beltMotor = new NeoMotor(3);
      ballSensor = new AnalogInput(0);
      for (int x = 0; x < BALL_READING_COUNT; x++) {
          ballReadings.add(false);
      }
      setBallCount(0);
    }

    public void init() {
      stop();
    }

    public void turnBelt(double speed) {
      beltMotorSpeed = speed;
    }

    public void stop() {
      turnBelt(0.0);
    }

    private int getBallCount() {
      return (int)(NetworkTableInstance.getDefault().getEntry("/ballCount").getNumber(0)); // hopefully casting wont mess anything up :)
    }

    private void setBallCount(int count) {
      NetworkTableInstance.getDefault().getEntry("/ballCount").setNumber(count);
    }
    
    public void resetCount() {
      setBallCount(0); // attempting to break the water-speed world record is the most dangerous activity one can engage in
    }

    public void addBall() {
      setBallCount(getBallCount() + 1);
    }


    public boolean seesBall() {
        final boolean ballReading = ballSensor.getValue() > 500;
        ballReadings.remove(0);
        ballReadings.add(ballReading);
    
        int trueReadings = 0;
        int falseReadings = 0;
    
        for (int x = 0; x < ballReadings.size(); x++) {
          if (ballReadings.get(x).equals(true)) {
            trueReadings++;
          } else {
            falseReadings++;
          }
        }
        return trueReadings > falseReadings;
    }

    public void execute() {
      beltMotor.setSpeed(beltMotorSpeed);
    }
}