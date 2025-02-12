package org.firstinspires.ftc.teamcode.utils.math;

public class Angles {
    public static double normalize(double a){
        a %= 2.0 * Math.PI;
        if(a < -Math.PI) a += 2.0 * Math.PI;
        if(a > Math.PI) a -= 2.0 * Math.PI;
        return a;
    }
}