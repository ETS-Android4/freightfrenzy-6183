package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Intake Test", group="test")
public class IntakeOpMode extends LinearOpMode {

    private DcMotor intake;

    ElapsedTime eTime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        intake = hardwareMap.get(DcMotor.class, "m");

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive())
        {
            intake.setPower(gamepad1.right_trigger);
        }
    }
}