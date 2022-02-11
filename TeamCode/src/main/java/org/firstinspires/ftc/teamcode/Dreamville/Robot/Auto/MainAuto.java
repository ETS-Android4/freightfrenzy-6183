package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Dreamville.AutoFunctions;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class MainAuto extends AutoFunctions {
    @Override
    public void runOpMode() {
        //dropOdometry();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-23.75, 62.75, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-51, 56, Math.toRadians(0)))
                .build();

        /*
        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .lineToConstantHeading(new Vector2d(-54, 24))
                .splineToConstantHeading(new Vector2d(-30, 24), Math.toRadians(0))
                .build();

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .lineToConstantHeading(new Vector2d(-30, 16))
                .splineToConstantHeading(new Vector2d(10, 16), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(10, 64))
                .lineToConstantHeading(new Vector2d(56, 64))
                .build();

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
                .lineToConstantHeading(new Vector2d(18, 64))
                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(7, 24, Math.toRadians(180)), Math.toRadians(270))
                .build();

        TrajectorySequence trajectory5 = drive.trajectorySequenceBuilder(trajectory4.end())
                .splineToSplineHeading(new Pose2d(10, 64, Math.toRadians(0)), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(56, 64))
                .build();

        TrajectorySequence trajectory6 = drive.trajectorySequenceBuilder(trajectory5.end())
                .lineToConstantHeading(new Vector2d(18, 64))
                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(7, 24, Math.toRadians(180)), Math.toRadians(270))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(trajectory6.end())
                .splineToSplineHeading(new Pose2d(10, 64, Math.toRadians(0)), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(40, 64))
                .build();

         */

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory1);

        /*
        drive.followTrajectorySequence(trajectory2);
        drive.followTrajectorySequence(trajectory3);
        drive.followTrajectorySequence(trajectory4);
        drive.followTrajectorySequence(trajectory5);
        drive.followTrajectorySequence(trajectory6);
        drive.followTrajectorySequence(park);

         */

        //raiseOdometry();
    }
}
