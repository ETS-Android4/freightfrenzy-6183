package org.firstinspires.ftc.teamcode.Dreamville.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Dreamville.Subsystems.Carousel;
import org.firstinspires.ftc.teamcode.Dreamville.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Dreamville.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Dreamville.Subsystems.Intake;

@TeleOp(name = "MainTeleOp", group = "tele")
public class MainBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Carousel carousel = new Carousel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Elevator elevator = new Elevator(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            carousel.spin(gamepad1.right_bumper, multiTelemetry);
            drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_down, gamepad1.dpad_up, multiTelemetry);
            intake.intake(gamepad1.right_trigger, gamepad1.left_trigger, multiTelemetry);
            elevator.lift(gamepad1.right_trigger, gamepad1.left_trigger, gamepad1.y, gamepad1.b, gamepad1.a, multiTelemetry);

            multiTelemetry.update();
        }
    }
}