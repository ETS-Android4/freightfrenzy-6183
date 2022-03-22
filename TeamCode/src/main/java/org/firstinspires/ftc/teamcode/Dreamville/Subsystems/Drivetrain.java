package org.firstinspires.ftc.teamcode.Dreamville.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.PoseStorage;

@Config
public class Drivetrain {
    private Telemetry telemetry;

    private double g1lx = 0, g1ly = 0, g1rx = 0;
    private boolean g1rb, g1lb, g1dl, g1dr, g1dd, g1du;

    private static Servo rs, ms, ls;

    public double leftStickY;
    public double leftStickX;
    public double rightStickX;
    public double FL_power, FR_power, RL_power, RR_power;

    public static double P = 0.04;
    public static double I = 0;
    public static double D = 0;

    private double integral, previous_error = 0;

    public double newForward, newStrafe, denominator;

    private static BNO055IMU imu;
    private static DcMotor fr, rr, fl, rl;

    public Orientation angles;

    private double error;
    private double desiredAngle = 0;
    private String turnState = "auto";

    private enum driveMode {
        DRIVER_CONTROLLED,
        AUTO_CONTROL
    }

    private driveMode driveState = driveMode.AUTO_CONTROL;

    private final ElapsedTime eTime = new ElapsedTime();

    public Drivetrain(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample op mode
        parameters.mode = BNO055IMU.SensorMode.IMU;
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        fr = hardwareMap.get(DcMotor.class, "frMotor");
        rr = hardwareMap.get(DcMotor.class, "rrMotor");
        fl = hardwareMap.get(DcMotor.class, "flMotor");
        rl = hardwareMap.get(DcMotor.class, "rlMotor");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotor.Direction.REVERSE);

        ls = hardwareMap.get(Servo.class, "ls");
        rs = hardwareMap.get(Servo.class, "rs");
        ms = hardwareMap.get(Servo.class, "ms");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(null, null, 1000);

        eTime.reset();
    }

    public void drive(double g1lx, double g1ly, double g1rx, boolean g1lb, boolean g1rb, boolean g1dl,
                      boolean g1dr, boolean g1dd, boolean g1du, Telemetry telemetry) {
        this.g1lx = g1lx;
        this.g1ly = g1ly;
        this.g1rx = g1rx;
        this.g1lb = g1lb;
        this.g1rb = g1rb;

        this.g1dl = g1dl;
        this.g1dr = g1dr;
        this.g1dd = g1dd;
        this.g1du = g1du;

        this.telemetry = telemetry;

        holonomicFormula();
        eTime.reset();
    }

    public void holonomicFormula() {
        rs.setPosition(0.4);
        ls.setPosition(0.8);
        ms.setPosition(0.5);

        double time = eTime.time();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        leftStickY = g1ly;
        leftStickX = g1lx * 1.1;
        rightStickX = g1rx;

        float pi = 3.1415926f;

        double gyro_degrees = angles.firstAngle + PoseStorage.currentHeading;
        double gyro_radians = gyro_degrees * pi / 180;
        newForward = leftStickY * Math.cos(gyro_radians) + leftStickX * Math.sin(gyro_radians);
        newStrafe = -leftStickY * Math.sin(gyro_radians) + leftStickX * Math.cos(gyro_radians);

        double errorMin;
        if (desiredAngle - angles.firstAngle < 0) {
            errorMin = Math.min(Math.abs(desiredAngle - angles.firstAngle), Math.abs(desiredAngle - angles.firstAngle + 360));
        } else {
            errorMin = Math.min(Math.abs(desiredAngle - angles.firstAngle), Math.abs(desiredAngle - angles.firstAngle - 360));
        }

        if (errorMin == Math.abs(desiredAngle - angles.firstAngle)) {
            error = desiredAngle - angles.firstAngle;
        } else if (errorMin == Math.abs(desiredAngle - angles.firstAngle - 360)) {
            error = desiredAngle - angles.firstAngle - 360;
        } else if (errorMin == Math.abs(desiredAngle - angles.firstAngle + 360)) {
            error = desiredAngle - angles.firstAngle + 360;
        }

        telemetry.addData("Turn Error", error);

        integral += (error * time);
        eTime.reset();

        double derivative = (error - previous_error) / time;
        double rcw = P * -error + I * integral + D * derivative;

        previous_error = error;

        telemetry.addData("saved heading", PoseStorage.currentHeading);
        telemetry.addData("Read angle", angles.firstAngle);
        telemetry.addData("RCW", rcw);
        telemetry.addData("Desired Angle", desiredAngle);
        telemetry.addData("rightStickX", rightStickX);

        switch (driveState) {
            case AUTO_CONTROL:
                if (!g1rb) {
                    if (g1du) {
                        desiredAngle = 0 + PoseStorage.currentHeading;
                    }
                    if (g1dr) {
                        desiredAngle = 270 + PoseStorage.currentHeading;
                    }
                    if (g1dd) {
                        desiredAngle = 180 + PoseStorage.currentHeading;
                    }
                    if (g1dl) {
                        desiredAngle = 90 + PoseStorage.currentHeading;
                    }
                }
                if (rightStickX != 0) {
                    driveState = driveMode.DRIVER_CONTROLLED;
                }
                turnState = "auto";
                //denominator = Math.max(Math.abs(newForward) + Math.abs(newStrafe) + Math.abs(rcw), 1);
                FL_power = (-newForward + newStrafe + rcw);// / denominator;
                RL_power = (-newForward - newStrafe + rcw);// / denominator;
                FR_power = (-newForward - newStrafe - rcw);// / denominator;
                RR_power = (-newForward + newStrafe - rcw);// / denominator;
                break;
            case DRIVER_CONTROLLED:
                turnState = "driver";
                denominator = Math.max(Math.abs(newForward) + Math.abs(newStrafe) + Math.abs(rightStickX), 1);
                FL_power = (-newForward + newStrafe + rightStickX) / denominator;
                RL_power = (-newForward - newStrafe + rightStickX) / denominator;
                FR_power = (-newForward - newStrafe - rightStickX) / denominator;
                RR_power = (-newForward + newStrafe - rightStickX) / denominator;
                if (!g1rb) {
                    if (g1du) {
                        desiredAngle = 0 + PoseStorage.currentHeading;
                        driveState = driveMode.AUTO_CONTROL;
                    }
                    if (g1dr) {
                        desiredAngle = 270 + PoseStorage.currentHeading;
                        driveState = driveMode.AUTO_CONTROL;
                    }
                    if (g1dd) {
                        desiredAngle = 180 + PoseStorage.currentHeading;
                        driveState = driveMode.AUTO_CONTROL;
                    }
                    if (g1dl) {
                        desiredAngle = 90 + PoseStorage.currentHeading;
                        driveState = driveMode.AUTO_CONTROL;
                    }
                }
                break;
        }

        telemetry.addData("turnState", turnState);

        if (g1lb) {
            FL_power /= 4;
            FR_power /= 4;
            RL_power /= 4;
            RR_power /= 4;
        }

        telemetry.addData("FLpower", FL_power);
        telemetry.addData("FRpower", FR_power);
        telemetry.addData("RLpower", RL_power);
        telemetry.addData("RRpower", RR_power);

        fl.setPower(FL_power);
        fr.setPower(FR_power);
        rl.setPower(RL_power);
        rr.setPower(RR_power);
    }
}