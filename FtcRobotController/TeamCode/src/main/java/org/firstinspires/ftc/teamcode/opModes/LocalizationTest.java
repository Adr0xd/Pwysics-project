package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Robot.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.gamepads.MisuDrivetrain;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "Localization Test")
public class LocalizationTest extends LinearOpMode {

    PinpointLocalizer localizer;
    MecanumDrive drive;
    MisuDrivetrain drivetrainControl;

    @Override
    public void runOpMode() throws InterruptedException {

        localizer = new PinpointLocalizer(hardwareMap);
        drive = new MecanumDrive(hardwareMap, MecanumDrive.RunMode.VectorAndSlows, true);
        drivetrainControl = new MisuDrivetrain(gamepad1, drive);


        waitForStart();
        localizer.resetIMU();
        localizer.setPose(new Pose(0,0,0));

        while(opModeIsActive()){
            drive.update();
            localizer.update();
            drivetrainControl.update();

            localizer.telemetry(telemetry);
        }
    }
}
