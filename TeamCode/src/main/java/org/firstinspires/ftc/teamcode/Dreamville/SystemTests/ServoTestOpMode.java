package org.firstinspires.ftc.teamcode.Dreamville.SystemTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "test")
public class ServoTestOpMode extends LinearOpMode {

    private Servo ls, rs, ms;
    private boolean oldg1a = false, oldg1b = false, oldg1y = false, oldg1x = false;

    @Override
    public void runOpMode() {
        ls = hardwareMap.get(Servo.class, "ls");
        rs = hardwareMap.get(Servo.class, "rs");
        ms = hardwareMap.get(Servo.class, "ms");

        rs.setPosition(0.4);
        ls.setPosition(0.8);
        ms.setPosition(0.5);

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        //Wait for the start button to be pressed.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("lsPos", ls.getPosition());
            telemetry.addData("rsPos", rs.getPosition());

            if (gamepad1.y && !oldg1y) {
                rs.setPosition(0.4);
                ls.setPosition(0.8);
                ms.setPosition(0.5);
                oldg1y = true;
            } else if (!gamepad1.y) {
                oldg1y = false;
            }

            if (gamepad1.x && !oldg1x) {
                rs.setPosition(1);
                ls.setPosition(0.2);
                ms.setPosition(1);
                oldg1x = true;
            } else if (!gamepad1.x) {
                oldg1x = false;
            }

            if (gamepad1.a && !oldg1a) {
                if (ls.getPosition()<=0.9) {
                    ls.setPosition(ls.getPosition()+0.1);
                    oldg1a = true;
                }
            } else if (!gamepad1.a) {
                oldg1a = false;
            }

            if (gamepad1.b && !oldg1b) {
                if (ls.getPosition()>=0.1) {
                    ls.setPosition(ls.getPosition()-0.1);
                    oldg1b = true;
                }
            } else if (!gamepad1.b) {
                oldg1b = false;
            }

            telemetry.update();
        }
    }
}