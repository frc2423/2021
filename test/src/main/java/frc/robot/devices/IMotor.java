package frc.robot.devices;

public interface IMotor {

    /**
     * Sets the desired speed in feet/ second.
     * 
     * @param speed Positive means motor is spinning forward.
     */
    public void setSpeed(double speed);

    /**
     * Gets speed in feet/ second based on encoder rate.
     * 
     * @return Positive means motor is spinning forward.
     */
    public double getSpeed();

    /**
     * Sets speed as a percentage of max speed.
     * 
     * @param percent  Positive means motor is spinning forward.
     */
    public void setPercent(double percent);

    /**
     * Gets percentage of max speed based on value passed into setPercent().
     * 
     * @return Positive means motor is spinning forward
     */
    public double getPercent();

    /**
     * Sets the accumulated motor distance.
     * 
     * @param dist New motor distance, positive means forward from initial position
     */
    public void setDistance(double dist);

    /**
     * Gets the accumuated motor distance.
     * 
     * @return The current distance, positive means forward from initial position
     */
    public double getDistance();

    /**
     * Reset the encoder postion to specified value
     * 
     * @param distance positive means foward from initial position
     */
    public void resetEncoder(double distance);

    /**
     * Sets the conversion factor for position and velocity of encoders.
     * 
     * @param factor scale factor
     */
    public void setConversionFactor(double factor);

    /**
     * Gets the conversion factor for position and velocity of encoders.
     * 
     * @return current scale factor.
     */
    public double getConversionFactor();

    /**
     * Sets inversion of the motor.
     * 
     * @param isInverted true is inverted
     */
    public void setInverted(boolean isInverted);

    /**
     * Gets the inversion setting of the motor. 
     * 
     * @return true if inverted
     */
    public boolean getInverted();

    public void setPid(double kP, double kI, double kD);
    
    public void setPidf(double kP, double kI, double kD, double kF);

    public void setP(double kP);

    public void setI(double kI);

    public void setD(double kD);

    public void setF(double kF);

    public double getP();

    public double getI();

    public double getD();

    public double getF();

    public void follow(IMotor leader); // thats sir motor to you

    /**
     * Sets the encoder values for simulation
     * @param position new position
     * @param rate new rate
     */
    public void setEncoderPositionAndRate(double position, double rate);

    public double getEncoderCount();

}
