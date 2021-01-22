package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import frc.robot.IDriveMotor;

public class DriveMotor implements IDriveMotor {
    
    // private CANSparkMax motor;

    public DriveMotor() {
        
    }

    public void setReference(double speed) {
        if (RobotBase.isReal()) {
            // realMotor.setReference(speed, ControlType.kVoltage);
        } else {

        }
    }
}
