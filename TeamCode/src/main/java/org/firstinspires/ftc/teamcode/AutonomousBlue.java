package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class AutonomousBlue extends LinearOpMode {

    public DcMotor slides;

//    RevBlinkinLedDriver lights;

    public DigitalChannel touch;
    public DistanceSensor sensorRange;

    public RevBlinkinLedDriver coolLights;

    //    public DcMotor slides
    public Servo leftCarry;
    public Servo rightCarry;

    double ServoPosition = 0.2;
    public ElapsedTime timer = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        slides = hardwareMap.dcMotor.get("slides");

        leftCarry = hardwareMap.servo.get("leftCarry");
        rightCarry = hardwareMap.servo.get("rightCarry");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        Pose2d startPose = new Pose2d( -35, 62, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose.plus(new Pose2d(0,0,Math.toRadians(90))))
                .strafeTo(new Vector2d(-30,-2),  SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(14)
                .addTemporalMarker(3, () -> {

                    slides.setPower(-0.5);
                })
                .addTemporalMarker(5.5, () -> {

                    slides.setPower(0);
                })
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .splineTo(new Vector2d(-58,11), Math.toRadians(180))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(25)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0,0, Math.toRadians(180))))
                .strafeRight(12)
                .build();

        Trajectory traj5a = drive.trajectoryBuilder(traj5.end())
                .forward(3)
                .build();




        waitForStart();



        timer.reset();
        timer.startTime();


        leftCarry.setPosition(ServoPosition);
        rightCarry.setPosition(0);

        sleep(700);
        slides.setPower(1);



        drive.turn(Math.toRadians(90));


//

        drive.followTrajectory(traj1);



            timer.reset();
            timer.startTime();

            while (timer.seconds() < 1){

                leftCarry.setPosition(0);
                rightCarry.setPosition(ServoPosition);

//

            }


        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setPower(0);




        drive.followTrajectory(traj2);



//        timer.reset();
//        timer.startTime();
//
//        while (timer.seconds() < 2.5){
//
//            slides.setPower(-0.5);
//
//
//        }
//
//
//
//        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slides.setPower(0);

        drive.turn(Math.toRadians(180));

        drive.followTrajectory(traj3);


        leftCarry.setPosition(ServoPosition);
        rightCarry.setPosition(0);

        sleep(700);
        slides.setPower(1);

        drive.followTrajectory(traj4);

        drive.turn(Math.toRadians(180));

        drive.followTrajectory(traj5);

        drive.followTrajectory(traj5a);

        leftCarry.setPosition(0);
        rightCarry.setPosition(ServoPosition);

        slides.setPower(0);


    }


}
