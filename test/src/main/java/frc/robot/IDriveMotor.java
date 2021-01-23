package frc.robot;

public interface IDriveMotor {


    public void setSpeed(double speed);

    public double getSpeed();

    public void setPercent(double percent); // set speed as percent ofmams dksdkjsd max speed

    public double getPercent();

    public void setDistance(double dist);

    public double getDistance();

    public void resetEncoder(double distance);

    public void setConversionFactor(double factor);

    public double getConversionFactor();

    public void setPid(double kP, double kI, double kD);

    public void setP(double kP);

    public void setI(double kI);

    public void setD(double kD);

    public double getP();

    public double getI();

    public double getD();

    public void follow(IDriveMotor leader); // thats sir motor to you
    

}
