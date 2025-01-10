package org.firstinspires.ftc.teamcode.Robot.gamepads;

import android.sax.StartElementListener;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.MecanumDrive;
import org.firstinspires.ftc.teamcode.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.utils.math.Vector3D;

public class MisuDrivetrain implements IRobotModule {

    private final Gamepad gamepad;
    private final StickyGamepad stickyGamepad;
    private final MecanumDrive drive;
    public MisuDrivetrain(Gamepad gamepad, MecanumDrive drive) {
        this.gamepad = gamepad;
        this.stickyGamepad = new StickyGamepad(gamepad);
        this.drive = drive;

    }

    @Override
    public void update() {
        drive.setTargetVectorAndSlows
                (
                        new Vector3D(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x),
                        gamepad.right_trigger - gamepad.left_trigger,
                        gamepad.left_bumper, gamepad.right_bumper
                );
        stickyGamepad.update();
    }
}