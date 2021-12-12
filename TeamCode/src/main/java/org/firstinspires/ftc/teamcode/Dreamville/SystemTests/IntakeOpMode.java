package org.firstinspires.ftc.teamcode.Dreamville.SystemTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Intake Test", group="test")
public class IntakeOpMode extends LinearOpMode {

    private DcMotor intake1;
    private DcMotor intake2;

    @Override
    public void runOpMode()
    {
        intake1 = hardwareMap.get(DcMotor.class, "m1");
        intake2 = hardwareMap.get(DcMotor.class, "m2");

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive())
        {
            intake1.setPower(gamepad1.right_trigger);
            intake2.setPower(gamepad1.right_trigger);
        }
    }
}