package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoElevator;
import org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto.AutoSubsystems.AutoIntake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class CoolMainAutoTester extends LinearOpMode {

    enum State {
        TRAJECTORY_1,
        PICKUP_1,
        DEPOSIT_2,
        PARK,
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    Pose2d startPose = new Pose2d();

    ElapsedTime eTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        AutoElevator elevator = new AutoElevator(hardwareMap);
        AutoIntake intake = new AutoIntake(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d targetQuickPose = new Pose2d(0, 0, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(15, 0, Math.toRadians(0)))
                .build();

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(0)))
                .build();

        /*
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(180)))
                .build();
         */

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;
        drive.followTrajectorySequenceAsync(trajectory1);

        elevator.goToBottom();

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            switch (currentState) {
                case TRAJECTORY_1:
                    if (!drive.isBusy()) {
                        currentState = State.PICKUP_1;
                        drive.followTrajectorySequenceAsync(trajectory2);
                        elevator.goToGround();
                        intake.intake();
                    }
                    break;
                case PICKUP_1:
                    if (!intake.isBusy()) {
                        currentState = State.DEPOSIT_2;
                        Trajectory quickTrajectory = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(targetQuickPose)
                                .build();
                        drive.followTrajectoryAsync(quickTrajectory);
                        intake.stop();
                        elevator.goToTop();
                    }
                    break;
                case DEPOSIT_2:
                    if (!elevator.isBusy() && !drive.isBusy()) {
                        intake.deposit();
                        currentState = State.PARK;
                        //eTime.reset();
                    }
                    break;
                case PARK:
                    if (!intake.isBusy()) {
                        intake.stop();
                        elevator.goToBottom();
                        drive.turnAsync(Math.toRadians(180));
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            drive.update();
            elevator.update();
            intake.update();

            TelemetryPacket packet = new TelemetryPacket();

            packet.put("x", poseEstimate.getX());
            packet.put("y", poseEstimate.getY());
            packet.put("heading", poseEstimate.getHeading());

            packet.putAll(elevator.getTelemetry());
            packet.putAll(intake.getTelemetry());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}