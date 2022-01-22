package org.firstinspires.ftc.teamcode.Dreamville.Robot.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Dreamville.AutoFunctions;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class MainAuto extends AutoFunctions {
    @Override
    public void runOpMode() {
        //dropOdometry();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-23, 62.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-54, 59, Math.toRadians(0)), Math.toRadians(180))
                .build();

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .lineToConstantHeading(new Vector2d(-54, 24))
                .splineToConstantHeading(new Vector2d(-30, 24), Math.toRadians(0))
                .build();

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .lineToConstantHeading(new Vector2d(-30, 16))
                .splineToConstantHeading(new Vector2d(10, 16), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(10, 62))
                .lineToConstantHeading(new Vector2d(56, 62))
                .build();

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
                .lineToConstantHeading(new Vector2d(18, 62))
                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(7, 24, Math.toRadians(180)), Math.toRadians(270))
                .build();

        TrajectorySequence trajectory5 = drive.trajectorySequenceBuilder(trajectory4.end())
                .splineToSplineHeading(new Pose2d(10, 62, Math.toRadians(0)), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(56, 62))
                .build();

        TrajectorySequence trajectory6 = drive.trajectorySequenceBuilder(trajectory5.end())
                .lineToConstantHeading(new Vector2d(18, 62))
                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(7, 24, Math.toRadians(180)), Math.toRadians(270))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(trajectory6.end())
                .splineToSplineHeading(new Pose2d(10, 62, Math.toRadians(0)), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(40, 62))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajectory1);
        drive.followTrajectorySequence(trajectory2);
        drive.followTrajectorySequence(trajectory3);
        drive.followTrajectorySequence(trajectory4);
        drive.followTrajectorySequence(trajectory5);
        drive.followTrajectorySequence(trajectory6);
        drive.followTrajectorySequence(park);

        //raiseOdometry();
    }
}
