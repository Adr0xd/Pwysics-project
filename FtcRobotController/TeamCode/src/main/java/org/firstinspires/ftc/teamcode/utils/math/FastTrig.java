package org.firstinspires.ftc.teamcode.utils.math;

public class FastTrig {

    private static double[] a = new double[65536];

    static {
        for(int i = 0; i<65536;i++){
            a[i] = Math.sin((double) i * 3.141592653589793D * 2.0D / 65536.0D);
        }
    }

    public static final double sin(double f){
        return a[(int) (f*10430.378d) & '\uffff'];
    }

    public static final double cos(double f){
        return a[(int) (f*10430.378d + 16384.0d) & '\uffff'];
    }

}