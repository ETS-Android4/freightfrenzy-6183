package org.firstinspires.ftc.teamcode.Dreamville.SystemTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Drivetrain Experiment", group="test")
public class DrivetrainExperimentOpMode extends LinearOpMode {

    private DcMotor fl;
    private DcMotor bl;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "m1");
        bl = hardwareMap.get(DcMotor.class, "m2");

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive()) {
            fl.setPower(gamepad1.right_stick_y);
            bl.setPower(-gamepad1.right_stick_y);
        }
    }
}