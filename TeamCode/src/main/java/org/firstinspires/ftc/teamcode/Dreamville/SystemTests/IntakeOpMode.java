package org.firstinspires.ftc.teamcode.Dreamville.SystemTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Intake Test", group="test")
public class IntakeOpMode extends LinearOpMode {

    private DcMotor intake1;
    private int initialPos;
    //private DcMotor intake2;

    @Override
    public void runOpMode()
    {
        intake1 = hardwareMap.get(DcMotor.class, "elevator");
        intake1.setDirection(DcMotor.Direction.REVERSE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake2 = hardwareMap.get(DcMotor.class, "m2");

        intake1.setDirection(DcMotor.Direction.REVERSE);
        initialPos = intake1.getCurrentPosition();

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive())
        {
            intake1.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

            telemetry.addData("rightTrigger", gamepad1.right_trigger);
            telemetry.addData("leftTrigger", gamepad1.left_trigger);

            telemetry.addData("currentPos", intake1.getCurrentPosition());
            telemetry.addData("positionDelta", intake1.getCurrentPosition()-initialPos);

            telemetry.update();

            //intake2.setPower(gamepad1.right_trigger);
        }
    }
}