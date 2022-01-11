package org.firstinspires.ftc.teamcode.Dreamville.SystemTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test", group="test")
public class ServoTestOpMode extends LinearOpMode {

    private Servo ls, rs;
    private boolean oldg1a = false, oldg1b = false;

    @Override
    public void runOpMode()
    {
        ls = hardwareMap.get(Servo.class, "ls");
        rs = hardwareMap.get(Servo.class, "rs");

        rs.setPosition(0);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("lsPos", ls.getPosition());
            telemetry.addData("rsPos", rs.getPosition());

            if (gamepad1.a && !oldg1a) {
                if (rs.getPosition()<=0.9) {
                    rs.setPosition(rs.getPosition()+0.1);
                    oldg1a = true;
                }
            } else if (!gamepad1.a) {
                oldg1a = false;
            }

            if (gamepad1.b && !oldg1b) {
                if (rs.getPosition()>=0.1) {
                    rs.setPosition(rs.getPosition()-0.1);
                    oldg1b = true;
                }
            } else if (!gamepad1.b) {
                oldg1b = false;
            }

            telemetry.update();
        }
    }
}