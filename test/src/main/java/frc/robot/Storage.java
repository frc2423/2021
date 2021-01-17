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

    public fun turnBelt(speed : Double) {
      beltMotorSpeed = speed
    }

    public fun stop() {
      turnBelt(0.0)
    }

    fun getBallCount() : Int {
      return NetworkTableInstance.getDefault().getEntry("/ballCount").getNumber(0).toInt()
    }

    private void setBallCount(int count) {
      NetworkTableInstance.getDefault().getEntry("/ballCount").setNumber(count);
    }
    
    public fun resetCount() {
    setBallCount(0)
    }

    public fun addBall() {
      setBallCount(getBallCount()+1)
    }


    public fun seesBall() : Boolean {
        val ballReading = ballSensor.getValue() > 500
        ballReadings.removeAt(0)
        ballReadings.add(ballReading)
    
        var trueReadings = 0
        var falseReadings = 0
    
        for (reading in ballReadings) {
          if (reading) {
            trueReadings++
          } else {
            falseReadings++
          }
        }
        return trueReadings > falseReadings
    }

    public fun execute() {
      beltMotor.set(beltMotorSpeed)
    }
}