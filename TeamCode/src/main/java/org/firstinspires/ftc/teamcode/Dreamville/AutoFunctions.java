package org.firstinspires.ftc.teamcode.Dreamville;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;

public class AutoFunctions extends LinearOpMode {
    private Servo ls, rs;

    public static double carouselPower = 0.26;
    public static double brakingTime = 1;
    public static int carouselEncoder = 400;
    public static double fastTime = 0.4;

    private boolean carouselDone = false;
    private int carouselStartPos = 0, carouselSavedPos = 0;

    private DcMotor c;

    private enum carouselMode {
        BRAKING,
        IDLE,
        SLOW,
        FAST
    }

    private carouselMode carouselState = carouselMode.IDLE;

    private final ElapsedTime carouselTime = new ElapsedTime();

    private OpenCvCamera camera;

    @Override
    public void runOpMode() {
        c = hardwareMap.get(DcMotor.class, "carousel");

        ls = hardwareMap.get(Servo.class, "ls");
        rs = hardwareMap.get(Servo.class, "rs");

        c.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Robot initialized.");
        telemetry.update();
    }

    public void spinCarousel() {
        switch (carouselState) {
            case IDLE:
                carouselStartPos = c.getCurrentPosition();
                carouselState = carouselMode.SLOW;
                break;
            case SLOW:
                if (Math.abs(c.getCurrentPosition()-carouselStartPos) < carouselEncoder) {
                    c.setPower(carouselPower);
                } else {
                    carouselState = carouselMode.FAST;
                    carouselTime.reset();
                }
                break;
            case FAST:
                if (carouselTime.time()<fastTime) {
                    c.setPower(1);
                } else {
                    c.setPower(0);
                    carouselDone = true;

                    carouselState = carouselMode.IDLE;
                }
                break;
        }
    }

    public void autoSpinCarousel() {
        while (!carouselDone) {
            spinCarousel();
        }
        carouselDone = false;
    }

    public void dropOdometry() {
        rs.setPosition(1);
        ls.setPosition(0.2);
    }

    public void raiseOdometry() {
        rs.setPosition(0.4);
        ls.setPosition(0.8);
    }
}
