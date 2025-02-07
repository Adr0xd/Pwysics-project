package org.firstinspires.ftc.teamcode.Wrappers;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.opencv.core.Mat;

public class CoolMotor {



    public enum RunMode {
        RUN, PID;
    }

    public final DcMotorFunny motor;
    private final PIDController pidController = new PIDController(0,0,0);
    private PIDCoefficients pidCoefficients = new PIDCoefficients(0,0,0);

    private RunMode runMode;

    private final boolean async;

    public CoolMotor(DcMotorFunny motor, RunMode runMode, boolean reversed){
        this.motor = motor;
        this.async = false;
        MotorConfigurationType motorConfigurationType = this.motor.motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        this.motor.motor.setMotorType(motorConfigurationType);
//        this.motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(reversed)this.motor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.runMode = runMode;
    }

    public CoolMotor(DcMotorFunny motor){
        this(motor, RunMode.RUN, false);
    }

    public CoolMotor(DcMotorFunny motor, RunMode runMode){
        this(motor, runMode, false);
    }

    public CoolMotor(DcMotorFunny motor, boolean reversed){
        this(motor, RunMode.RUN, false);
    }

    public CoolMotor(DcMotorEx motor, RunMode runMode, boolean reversed){
        this.motor = new DcMotorFunny(motor);
        this.async = false;
        MotorConfigurationType motorConfigurationType = this.motor.motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        this.motor.motor.setMotorType(motorConfigurationType);
//        this.motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(reversed)this.motor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.runMode = runMode;
    }

    public CoolMotor(DcMotorEx motor){
        this(motor, RunMode.RUN, false);
    }

    public CoolMotor(DcMotorEx motor, RunMode runMode){
        this(motor, runMode, false);
    }

    public CoolMotor(DcMotorEx motor, boolean reversed){
        this(motor, RunMode.RUN, false);
    }

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zeroPowerBehaviour){
        this.motor.motor.setZeroPowerBehavior(zeroPowerBehaviour);
    }

    public double getPower(){
        return motor.getPower();
    }

    public void setMode(RunMode runMode){
        this.runMode = runMode;
    }

    private double power;

    public void setPower(double power){
        if(runMode == RunMode.PID) return;
        this.power = power;
    }

    public void setPID(PIDCoefficients pidCoefficients){
        this.pidCoefficients = pidCoefficients;
    }

    public double feedforward;
    public void resetEncoder (){
        this.motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
    public void setPIDF(PIDCoefficients pidCoefficients, double feedforward){
        this.feedforward = feedforward;
        this.pidCoefficients.p = pidCoefficients.p;
        this.pidCoefficients.i = pidCoefficients.i;
        this.pidCoefficients.d = pidCoefficients.d;
    }


    public void calculatePower(double current, double target){
        if(runMode == RunMode.RUN) return;
        pidController.setPID(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);
        power = feedforward + pidController.calculate(current,target);
    }
    public void calculatePowerMax(double current, double target,double max){
        if(runMode == RunMode.RUN) return;
        pidController.setPID(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);
        power = feedforward + pidController.calculate(current,target);
        power =max*power;


    }

    public void update(){
        motor.setPowerAsync(power);
        if(!async) motor.updatePowerAsync();
    }

    public double getCurrentPosition(){
        return motor.getCurrentPosition();
    }

    public double getRawVelocity(){
        return motor.getRawVelocity();
    }

}