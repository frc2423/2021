package frc.robot.devices;

public interface IGyro{

    public void reset();

    /**
     * Returns angle reported by the gyro. Clockwise means positive angle.
     * 
     * @return angle
     */
    public double getAngle();

    /**
     * Sets angle of the gyro.
     * 
     * @param angle Clockwise means positive angle.
     */
    public void setAngle(double angle);

    public double getRate();

    public void calibrate();

}