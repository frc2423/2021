package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.EntryListenerFlags;


class Storage {
    private CANSparkMax beltMotor;
    private AnalogInput ballSensor;
  
    private final int BALL_READING_COUNT = 25;
    private ArrayList ballReadings = new ArrayList<Boolean>();

    private double beltMotorSpeed = 0.0;
  
  
    public Storage (){
      beltMotor = new CANSparkMax(3, MotorType.kBrushless);
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
      beltMotor.set(beltMotorSpeed);
    }
}