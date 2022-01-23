package org.firstinspires.ftc.teamcode.Dreamville.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake {
    public static double ltDivisor = 2;

    private double g1rt, g1lt;
    private Telemetry telemetry;

    private static DcMotor intake;
    private static RevColorSensorV3 colorSensor;

    private enum intakeMode {
        INTAKE,
        DEPOSIT,
        STOP,
        COLOR
    }

    private intakeMode intakeState = intakeMode.STOP;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");
    }

    public void intake(double g1rt, double g1lt, Telemetry telemetry) {
        this.g1rt = g1rt;
        this.g1lt = g1lt;

        this.telemetry = telemetry;

        controls();
    }

    public void controls() {
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        telemetry.addData("Distance (cm)", "%.3f", distance);

        switch (intakeState) {
            case STOP:
                intake.setPower(0);
                if (g1rt!=0) {
                    if (distance > 4.5) {
                        intakeState = intakeMode.INTAKE;
                    }
                } else if (g1lt!=0) {
                    if (distance < 4.5) {
                        intakeState = intakeMode.DEPOSIT;
                    }
                }
                break;
            case INTAKE:
                intake.setPower(g1rt);
                if (distance < 4.5) {
                    intakeState = intakeMode.COLOR;
                }
                if (g1rt == 0) {
                    intakeState = intakeMode.STOP;
                }
                if (g1lt != 0) {
                    intakeState = intakeMode.DEPOSIT;
                }
                break;
            case DEPOSIT:
                intake.setPower(-(g1lt/ltDivisor));
                if (distance > 4.5) {
                    intakeState = intakeMode.COLOR;
                }
                if (g1lt == 0) {
                    intakeState = intakeMode.STOP;
                }
                if (g1rt != 0) {
                    intakeState = intakeMode.INTAKE;
                }
                break;
            case COLOR:
                intake.setPower(0);
                if (g1lt==0 && g1rt==0) {
                    intakeState = intakeMode.STOP;
                }
                break;
        }
    }
}