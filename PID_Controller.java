package org.firstinspires.ftc.teamcode.RidgeviewRoboticsFTCSkystone13022;

public class PID_Controller {

    // Data Fields
    private double pCoeff;
    private double iCoeff;
    private double dCoeff;
    private double sumError, prevError;
    private double prevPrevError;

    // Constructor
    public PID_Controller(double pCoeff, double iCoeff, double dCoeff)
    {
        this.pCoeff = pCoeff;
        this.iCoeff = iCoeff;
        this.dCoeff = dCoeff;

        sumError = 0.0;
        prevError = 0.0;
    }

    // Methods
    double getSpeed(double error)
    {
        sumError += error;

        // Calculate Derivative (slope of the error)
        double dError = error - prevError;

        prevPrevError = prevError;

        prevError = error;

        return pCoeff*error + iCoeff*sumError + dCoeff*dError;
    }

    boolean checkTimeOut(double error)
    {
        if (Math.abs(error - prevPrevError) == 0)
        {
            return false;
        }
        else
        {
            return true;
        }

    }

    boolean checkTimeOutGyro(double error)
    {
        double tolerance = 5;
        if (Math.abs(error - prevPrevError) == 0 && error < tolerance)
        {
            return false;
        }
        else
        {
            return true;
        }

    }

    double printStuff(double error)
    {
        return Math.abs(error - prevPrevError);
    }
}
