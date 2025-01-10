package org.firstinspires.ftc.teamcode.utils;

public class LowPassFilter {

    private final double t;
    private double lastValue;

    public LowPassFilter (double t, double initialValue){
        this.t = t;
        this.lastValue = initialValue;
    }

    public double getValue(double rawValue){
        double newValue = lastValue + t * (rawValue - lastValue);
        this.lastValue = newValue;
        return newValue;
    }
    public double getValue(double rawValue,double oldvalue){
        double newValue = oldvalue + t * (rawValue - oldvalue);
        this.lastValue = newValue;
        return newValue;
    }
}