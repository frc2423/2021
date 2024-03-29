package frc.robot.subsystems;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.devices.IMotor;
import frc.robot.Manager;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Storage extends Subsystem{
    private IMotor beltMotor;
    private AnalogInput ballSensor;
  
    private final int BALL_READING_COUNT = 25;
    private ArrayList<Boolean> ballReadings = new ArrayList<Boolean>();
  
    public Storage (){
        super("storage");
    }

    public void init() {
        ballSensor = new AnalogInput(1);
        beltMotor = Manager.getDevice("beltMotor", IMotor.class);

        for (int x = 0; x < BALL_READING_COUNT; x++) {
            ballReadings.add(false);
        }
        setBallCount(0);
        stop();
    }

    public void turnBelt(double speed) {
        beltMotor.setPercent(speed);
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
    
}