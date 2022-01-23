package org.firstinspires.ftc.teamcode.Dreamville.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Elevator {
    public static int bottomPos = -500;
    public static int middlePos = -1900;
    public static int topPos = -2500;
    public static double P = 0.005;
    public static double ltDelay = 0.5;

    private int error = 0;
    private double pwr = 0;

    private double g1rt, g1lt;
    private boolean g1y, g1b, g1a;
    private Telemetry telemetry;

    private static DcMotor elevator;

    private boolean ltHeld = false;
    private static final ElapsedTime ltTimer = new ElapsedTime();

    private enum elevatorMode {
        GROUND,
        BOTTOM,
        MIDDLE,
        TOP,
        LTDELAY
    }

    private elevatorMode elevatorState = elevatorMode.BOTTOM;

    public Elevator(HardwareMap hardwareMap) {
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void lift(double g1rt, double g1lt, boolean g1y, boolean g1b, boolean g1a, Telemetry telemetry) {
        this.g1rt = g1rt;
        this.g1lt = g1lt;
        this.g1y = g1y;
        this.g1b = g1b;
        this.g1a = g1a;

        this.telemetry = telemetry;

        controls();
    }

    public void controls() {
        telemetry.addData("elevator error", error);
        telemetry.addData("elevator power", pwr);

        switch (elevatorState) {
            case GROUND:
                pwr = P * -elevator.getCurrentPosition();
                elevator.setPower(pwr);
                if (g1rt==0) {
                    elevatorState = elevatorMode.BOTTOM;
                }
                break;
            case BOTTOM:
                error = bottomPos - elevator.getCurrentPosition();
                pwr = P * error;
                elevator.setPower(pwr);
                if (g1rt>0) {
                    elevatorState = elevatorMode.GROUND;
                }
                if (g1y) {
                    elevatorState = elevatorMode.TOP;
                } else if (g1b) {
                    elevatorState = elevatorMode.MIDDLE;
                }
                break;
            case MIDDLE:
                error = middlePos - elevator.getCurrentPosition();
                pwr = P * error;
                elevator.setPower(pwr);
                if (g1rt>0) {
                    elevatorState = elevatorMode.GROUND;
                }
                if (g1y) {
                    elevatorState = elevatorMode.TOP;
                } else if (g1a) {
                    elevatorState = elevatorMode.BOTTOM;
                }
                if (g1lt>0) {
                    ltHeld = true;
                }
                if (ltHeld && g1lt==0) {
                    ltHeld = false;
                    ltTimer.reset();
                    elevatorState = elevatorMode.LTDELAY;
                }
                break;
            case TOP:
                error = topPos - elevator.getCurrentPosition();
                pwr = P * error;
                elevator.setPower(pwr);
                if (g1rt>0) {
                    elevatorState = elevatorMode.GROUND;
                }
                if (g1a) {
                    elevatorState = elevatorMode.BOTTOM;
                } else if (g1b) {
                    elevatorState = elevatorMode.MIDDLE;
                }
                if (g1lt>0) {
                    ltHeld = true;
                }
                if (ltHeld && g1lt==0) {
                    ltHeld = false;
                    ltTimer.reset();
                    elevatorState = elevatorMode.LTDELAY;
                }
                break;
            case LTDELAY:
                if (ltTimer.time() > ltDelay) {
                    elevatorState = elevatorMode.BOTTOM;
                }
                break;
        }
    }
}