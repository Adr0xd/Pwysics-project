package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Robot.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "Forward Backward")
public class ForwardBackward extends LinearOpMode {

    PinpointLocalizer localizer;
    MecanumDrive drive;

    State currentState = State.FORWARD;

    boolean isAtTarget = false;

    @Override
    public void runOpMode() throws InterruptedException {

        localizer = new PinpointLocalizer(hardwareMap);
        drive = new MecanumDrive(hardwareMap, MecanumDrive.RunMode.PID, true);


        waitForStart();


        while(opModeIsActive()){
            switch (currentState){
                case FORWARD:
                    isAtTarget = false;
                    drive.setTargetPose(forwardPose, true);
                    if(drive.reachedTarget(2) && drive.stopped())
                    {
                        isAtTarget = true;
                        currentState = State.WAIT;
                    }
                    break;
                case BACKWARD:
                    drive.setTargetPose(backwardPose, true);
                    if(drive.reachedTarget(2) && drive.stopped())
                    {
                        isAtTarget = true;
                        currentState = State.WAIT;
                    }
                    break;
                case WAIT:
                    if(gamepad1.a){
                        currentState = State.FORWARD;
                    }
                    if(gamepad1.b){
                        currentState = State.BACKWARD;
                    }
                    break;
            }

            drive.update();
            localizer.update();
            localizer.telemetry(telemetry);
        }
    }

    Pose forwardPose = new Pose(60, 0, 0);
    Pose backwardPose = new Pose(0, 0, 0);

    enum State{
        FORWARD, BACKWARD, WAIT
    }

}
