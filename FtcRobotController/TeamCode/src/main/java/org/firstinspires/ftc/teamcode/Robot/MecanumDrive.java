package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Robot.Localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.Wrappers.DcMotorFunny;
import org.firstinspires.ftc.teamcode.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.math.Angles;
import org.firstinspires.ftc.teamcode.utils.math.Vector3D;

public class MecanumDrive implements IRobotModule {

    public static boolean ENABLED = true;

    private final PinpointLocalizer localizer;
    private HardwareMap hwMap;

    public final CoolMotor frontLeft, frontRight, backLeft, backRight;
    public static boolean frontLeftMotorReversed = true, frontRightMotorReversed = false, backLeftMotorReversed = true, backRightMotorReversed = false;

    public static double headingMultiplier = 1;

    public static PIDCoefficients translationalPIDNormal = new PIDCoefficients(0.15,0 ,0),
            headingPID = new PIDCoefficients(1.3,0,0.19), translationalPIDPrediction = new PIDCoefficients(0.08,0,0);
    public final PIDController tpid= new PIDController(0,0,0), hpid = new PIDController(0,0,0);

    public static double lateralMultiplier = 2;

    public double overallMultiplier = 1;

    public double velocityThreshold = 1.5;

    public static double ks = 0.03;

    private boolean usePrediction = true;

    public enum RunMode{
        PID, Vector, VectorAndSlows
    }

    private RunMode runMode;

    public MecanumDrive(HardwareMap hardwareMap, RunMode runMode, boolean brake){
        this.runMode = runMode;
        if(!ENABLED) {
            this.localizer = null;
            frontLeft = null;
            frontRight = null;
            backLeft = null;
            backRight = null;
            return;
        }

        this.localizer = new PinpointLocalizer(hardwareMap);
        frontLeft = new CoolMotor(hardwareMap.get(DcMotorEx.class, "FL"), CoolMotor.RunMode.RUN, frontLeftMotorReversed);
        frontRight = new CoolMotor(hardwareMap.get(DcMotorEx.class, "FR"), CoolMotor.RunMode.RUN, frontRightMotorReversed);
        backLeft = new CoolMotor(hardwareMap.get(DcMotorEx.class, "BL"), CoolMotor.RunMode.RUN, backLeftMotorReversed);
        backRight = new CoolMotor(hardwareMap.get(DcMotorEx.class, "BR"), CoolMotor.RunMode.RUN, backRightMotorReversed);

        if(brake){
            frontLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.hwMap = hardwareMap;

        this.runMode = runMode;
    }

    public Vector3D powerVector = new Vector3D();
    private Pose targetPose = new Pose();
    public Vector3D targetVector = new Vector3D();

    public void setTargetPose(Pose pose, boolean usePrediction){
        this.targetPose = pose;
        this.usePrediction = usePrediction;
    }

    public void setTargetVector(Vector3D Vector){
        this.targetVector = Vector;
    }

    public RunMode getRunMode() {
        return runMode;
    }

    public PinpointLocalizer getLocalizer(){
        return localizer;
    }

    public Pose getTargetPose(){
        return targetPose;
    }

    public void setRunMode(RunMode runMode){
        this.runMode = runMode;
    }

    public boolean reachedTarget(double tolerance){
        if(runMode == RunMode.Vector) return false;
        return localizer.getPose().getDistance(targetPose) <= tolerance;
    }


    public double diff;

    public boolean reachedHeading(double tolerance){
        if(runMode == RunMode.Vector) return false;
        return Math.abs(Angles.normalize(targetPose.getHeading() - localizer.getHeading())) <= tolerance;
    }

    public boolean stopped(){
        return localizer.getVelocityVector().getMagnitude() <= velocityThreshold;
    }

    public void setTargetVectorAndSlows(Vector3D vector, double slowTurn, boolean slowBack, boolean slowForward) {

        double yComp = 0, xComp = 0;

        if (slowBack) yComp = -0.2;
        if (slowForward) yComp = 0.2;

        if (Math.abs(slowTurn) > 0.05) {
            xComp = 0.4 * (slowTurn / Math.abs(slowTurn));
        }

        this.targetVector = new Vector3D(xComp != 0 ? xComp : vector.getX() * 1.1, yComp != 0 ? yComp : vector.getY(), vector.getZ());
    }

    private void updatePowerVector(){
        switch (runMode){
            case Vector:
                powerVector = new Vector3D(targetVector.getX(), targetVector.getY(), targetVector.getZ());
                powerVector = Vector3D.rotateBy(powerVector, localizer.getHeading());
                powerVector = new Vector3D(powerVector.getX(), powerVector.getY() * lateralMultiplier, targetVector.getZ());
                break;
            case VectorAndSlows:
                powerVector = targetVector;
                break;
            case PID:
                Pose currentPose;

                if(usePrediction){
                    currentPose = localizer.getPredictedPose();
                    tpid.setPID(translationalPIDPrediction.p, translationalPIDPrediction.i, translationalPIDPrediction.d);
                } else {
                    currentPose = localizer.getPose();
                    tpid.setPID(translationalPIDNormal.p, translationalPIDNormal.i, translationalPIDNormal.d);
                }

                double xDiff = targetPose.getX() - currentPose.getX();
                double yDiff = targetPose.getY() - currentPose.getY();

                double distance = Math.sqrt(xDiff * xDiff + yDiff * yDiff);

                double calculatedCos = xDiff/distance;
                double calculatedSin = yDiff/distance;

                double translationalPower = tpid.calculate(-distance, 0);

                powerVector = new Vector3D(translationalPower * calculatedCos, translationalPower * calculatedSin);
                powerVector = Vector3D.rotateBy(powerVector, currentPose.getHeading());

                double headingDiff = Angles.normalize(targetPose.getHeading() - currentPose.getHeading());

                hpid.setPID(headingPID.p, headingPID.i, headingPID.d);

                double headingPower = hpid.calculate(-headingDiff, 0) * headingMultiplier;

                powerVector= new Vector3D(powerVector.getX(),powerVector.getY() * lateralMultiplier, headingPower);
                break;
        }
        if(Math.abs(powerVector.getX()) + Math.abs(powerVector.getY()) + Math.abs(powerVector.getZ()) > 1)
            powerVector.scaleToMagnitude(1);
        powerVector.scaleBy(overallMultiplier);
    }

    private void updateMotors(){
        if(runMode != RunMode.VectorAndSlows) {
            double voltage = hwMap.voltageSensor.iterator().next().getVoltage();
            double actualKs = ks * 13.8 / voltage;

            frontLeft.setPower((powerVector.getX() - powerVector.getY() - powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() - powerVector.getZ()));
            frontRight.setPower((powerVector.getX() + powerVector.getY() + powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() + powerVector.getY() + powerVector.getZ()));
            backLeft.setPower((powerVector.getX() + powerVector.getY() - powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() + powerVector.getY() - powerVector.getZ()));
            backRight.setPower((powerVector.getX() - powerVector.getY() + powerVector.getZ()) * (1 - actualKs) + actualKs * Math.signum(powerVector.getX() - powerVector.getY() + powerVector.getZ()));
        } else {
            double x = powerVector.getX(),
                    y = powerVector.getY(),
                    rot = powerVector.getZ();

            frontLeft.setPower(y + x + rot);
            frontRight.setPower(y - x - rot);
            backLeft.setPower(y - x + rot);
            backRight.setPower(y + x - rot);
        }

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }

    @Override
    public void update() {
        if(!ENABLED) return;

        updatePowerVector();
        updateMotors();
    }

    @Override
    public void emergencyStop() {
        powerVector = new Vector3D();
        updateMotors();
    }
}