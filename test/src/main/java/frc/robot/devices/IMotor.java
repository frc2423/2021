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
    public void setPercent(double percent); // set speed as percent ofmams dksdkjsd max speed

    /**
     * Gets percentage of max speed based on value passed into setPercent().
     * 
     * @return Positive means motor is spinning forward
     */
    public double getPercent();

    public void setDistance(double dist);

    public double getDistance();

    public void resetEncoder(double distance);

    public void setConversionFactor(double factor);

    public double getConversionFactor();

    public void setInverted(boolean isInverted);

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

    public void setEncoderPositionAndRate(double position, double rate); //poop

    public double getEncoderCount();

}
