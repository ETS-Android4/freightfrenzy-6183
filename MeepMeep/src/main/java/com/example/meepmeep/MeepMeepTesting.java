package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(10, 15)
                .setConstraints(35, 35, Math.toRadians(360), Math.toRadians(360), 10.1)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.75, -62.5, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(-8, -44), Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(10, -65, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(64, -65))
                                .lineToConstantHeading(new Vector2d(10, -65))
                                .splineToSplineHeading(new Pose2d(-8, -44, Math.toRadians(90)), Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(10, -65, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(64, -65))
                                .lineToConstantHeading(new Vector2d(10, -65))
                                .splineToSplineHeading(new Pose2d(-8, -44, Math.toRadians(90)), Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(10, -65, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(40, -65))
                                .lineToLinearHeading(new Pose2d(40, -45, Math.toRadians(90)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}