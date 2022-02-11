package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class AutoElevator {
    private static final Map<String, Object> telemetry = new LinkedHashMap<>();

    public static int bottomPos = -1600;
    public static int middlePos = -3600;
    public static int topPos = -5200;
    public static int errorTolerance = 10;
    public static double P = 0.005;

    private static int error = 0;

    private static DcMotor elevator;
    private elevatorMode elevatorState = elevatorMode.BOTTOM;

    public AutoElevator(HardwareMap hardwareMap) {
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        telemetry.put("elevator error", error);
        telemetry.put("elevator power", P * error);

        switch (elevatorState) {
            case GROUND:
                error = -elevator.getCurrentPosition();
                elevator.setPower(P * error);

                break;
            case BOTTOM:
                error = bottomPos - elevator.getCurrentPosition();
                elevator.setPower(P * error);

                break;
            case MIDDLE:
                error = middlePos - elevator.getCurrentPosition();
                elevator.setPower(P * error);

                break;
            case TOP:
                error = topPos - elevator.getCurrentPosition();
                elevator.setPower(P * error);

                break;
        }
    }

    public void goToGround() {
        elevatorState = elevatorMode.GROUND;
    }

    public void goToBottom() {
        elevatorState = elevatorMode.BOTTOM;
    }

    public void goToMiddle() {
        elevatorState = elevatorMode.MIDDLE;
    }

    public void goToTop() {
        elevatorState = elevatorMode.TOP;
    }

    public boolean isBusy() {
        return Math.abs(error) >= errorTolerance;
    }

    public Map<String, Object> getTelemetry() {
        return telemetry;
    }

    private enum elevatorMode {
        GROUND,
        BOTTOM,
        MIDDLE,
        TOP
    }
}