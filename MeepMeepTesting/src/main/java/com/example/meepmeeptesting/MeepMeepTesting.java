package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.14)
                .build();
        int x = 2;
        if (x == 1)
        {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(7.61, 62.8, Math.toRadians(-90)))
                    .strafeTo(new Vector2d(4.8, 34.5))
                    .strafeTo(new Vector2d(4.8, 33))
                    .strafeTo(new Vector2d(50, 46))
                    .strafeToLinearHeading(new Vector2d(54, 50), Math.toRadians(45))
                    .strafeToLinearHeading(new Vector2d(60, 45.5), Math.toRadians(-92.5))
                    .strafeToLinearHeading(new Vector2d(54, 50), Math.toRadians(45))
                    .strafeToLinearHeading(new Vector2d(55, 40), Math.toRadians(-45))
                    .strafeToLinearHeading(new Vector2d(55, 50), Math.toRadians(45))
                    .strafeToLinearHeading(new Vector2d(35, 8),Math.toRadians(180))
                    .strafeTo(new Vector2d(28, 8))
                    .build());

            meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
        else if(x==2)
        {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-14, 62.8, Math.toRadians(-90)))
//                    .strafeToLinearHeading(new Vector2d(-3.8, 33.3), -89.54)
//                    .strafeTo(new Vector2d(-31, 43))
//                    .setTangent(60)
//                    .splineToSplineHeading(new Pose2d(-46, 13, 89.54), Math.PI)
//                            .strafeTo(new Vector2d(-45, 55))
//                            .setTangent(-1)
//                            .splineToLinearHeading(new Pose2d(-58, 13, 89.54), Math.PI)
//                            .strafeTo(new Vector2d(-58, 50))
//                            .setTangent(-1)
//                            .splineToLinearHeading(new Pose2d(-60, 13, 89.54), Math.PI)
//                            .strafeTo(new Vector2d(-60, 50))
//                            .strafeTo(new Vector2d(-37,50))
//                            .strafeTo(new Vector2d(-37,55))


                    .strafeTo(new Vector2d(-4.8, 34.5))
                    .strafeTo(new Vector2d(-31, 43))
                    .setTangent(60)
                    .splineToSplineHeading(new Pose2d(-48, 13, Math.toRadians(0)), Math.PI*1.15)
                    .strafeTo(new Vector2d(-46, 55))
                    .strafeToLinearHeading(new Vector2d(-56,13),Math.toRadians(0))
                    .strafeTo(new Vector2d(-60,13))
                    .strafeTo(new Vector2d(-60, 50))
                    .strafeToLinearHeading(new Vector2d(-61, 13), 0)
                    .strafeTo(new Vector2d(-63,13))
                    .strafeTo(new Vector2d(-63, 50))
                    .strafeTo(new Vector2d(-23.33, 57.5))
                    .strafeToLinearHeading(new Vector2d(-4.8, 34.5), Math.toRadians(-90))
                    .strafeToLinearHeading(new Vector2d(-23.33, 57.5), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-4.8, 34.5), Math.toRadians(-90))
                    .strafeToLinearHeading(new Vector2d(-23.33, 57.5), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-4.8, 34.5), Math.toRadians(-90))
                    .strafeTo(new Vector2d(-49, 60))
//                    .strafeTo(new Vector2d(-37, 50))
//                    .strafeTo(new Vector2d(-37, 55))







//                    .build()
//                    .strafeTo(new Vector2d(-46, 13))
//                    .strafeTo(new Vector2d(-46, 54))
//                    .waitSeconds(3)
//                    //.setTangent(1)
//                    //.splineToSplineHeading(new Pose2d(-3.8, 33.3,-90), Math.PI/ 2)
//                    .strafeToLinearHeading(new Vector2d(-3.8, 33.3), -89.54)
//                    .setTangent(1)
//                    .splineToSplineHeading(new Pose2d(-48, 54, 89.54), Math.PI / 2)
//                    .strafeToLinearHeading(new Vector2d(-3.8, 33.3), -89.54)
//                    .strafeTo(new Vector2d(-54, 59))

//              .turn(Math.toRadians(90))
//              .lineToY(30)
//              .turn(Math.toRadians(90))
//              .lineToX(0)
//              .turn(Math.toRadians(90))
                    .waitSeconds(999999999)
                            .turn(Math.toRadians(90))








                    .build());

            meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
        else if (x==3)
        {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-11.6666667, -61.2, 0))
                    .turn(Math.toRadians(90))
                    .strafeTo(new Vector2d(-3.8, -33.3))
//              .waitSeconds(3)
                    .setTangent(-1)
                    .splineToSplineHeading(new Pose2d(-53, -53, -46.3), 1.5*Math.PI )
//              .waitSeconds(3)
                    .turnTo(Math.toRadians(90))
                    .strafeTo(new Vector2d(-48.3, -39))
//              .waitSeconds(3)
                    .turnTo(Math.toRadians(-135))
                    .strafeTo(new Vector2d(-54, -53))
//              .waitSeconds(3)
                    .turnTo(Math.toRadians(90))
                    .strafeTo(new Vector2d(-56.5, -40))
//              .waitSeconds(3)
                    .turnTo(Math.toRadians(-135))
                    .strafeTo(new Vector2d(-54, -53))
//              .waitSeconds(3)
                    .turnTo(Math.toRadians(180))
                    .strafeTo(new Vector2d(-55, -26))
//              .waitSeconds(3)
                    .turnTo(Math.toRadians(-135))
                    .strafeTo(new Vector2d(-54, -53))
//              .waitSeconds(3)
                    .strafeTo(new Vector2d(-33, -10))
                    .turnTo(Math.toRadians(180
                    ))
//              .lineToX(61.2)
//              .turn(Math.toRadians(90))
//              .lineToY(30)
//              .turn(Math.toRadians(90))
//              .lineToX(0)
//              .turn(Math.toRadians(90))
//              .lineToY(0)
                    .waitSeconds(999999999)
                    .turn(Math.toRadians(90))
                    .build());

            meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
}