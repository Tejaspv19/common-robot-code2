package org.firstinspires.ftc.teamcode;

import android.speech.tts.TextToSpeech;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import kotlin.reflect.KCallable;


@TeleOp
public class SlidesTest extends LinearOpMode {


    public DcMotor slides;

//    RevBlinkinLedDriver lights;

    public DigitalChannel touch;

    //    public DcMotor slides
    public Servo leftCarry;
    public Servo rightCarry;

    double ServoPosition = 0.2;
    DistanceSensor dis;

    public ElapsedTime timer = new ElapsedTime();

    public void runOpMode() throws InterruptedException {



        touch = hardwareMap.get(DigitalChannel.class, "touch");

        slides = hardwareMap.dcMotor.get("slides");

        leftCarry = hardwareMap.servo.get("leftCarry");
        rightCarry = hardwareMap.servo.get("rightCarry");
//        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        touch.setMode(DigitalChannel.Mode.INPUT);


        waitForStart();



        while (opModeIsActive()) {




            boolean dpadU1 = gamepad1.dpad_up;
            boolean dpadD1 = gamepad1.dpad_down;
            boolean dpadL1 = gamepad1.dpad_left;
            boolean dpadR1 = gamepad1.dpad_right;

            boolean leftBump1 = gamepad1.left_bumper;
            boolean rightBump1 = gamepad1.right_bumper;

            boolean a1 = gamepad1.a;
            boolean b1 = gamepad1.b;
            boolean x1 = gamepad1.x;
            boolean y1 = gamepad1.y;

            double leftTrig1 = gamepad1.left_trigger;
            double rightTrig1 = gamepad1.right_trigger;


            boolean dpadU2 = gamepad2.dpad_up;
            boolean dpadD2 = gamepad2.dpad_down;
            boolean dpadL2 = gamepad2.dpad_left;
            boolean dpadR2 = gamepad2.dpad_right;

            boolean leftBump2 = gamepad2.left_bumper;
            boolean rightBump2 = gamepad2.right_bumper;

            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;
            boolean x2 = gamepad2.x;
            boolean y2 = gamepad2.y;

            double leftTrig2 = gamepad2.left_trigger;
            double rightTrig2 = gamepad2.right_trigger;


            //annoying trig drivetrain stuff

            timer.startTime();

//            int position = slides.getCurrentPosition();
//            telemetry.addData("Encoder Position", position);
//            telemetry.update();
            //inching each direction
            if (a1) { // Closed Arm Largest
//                telemetry.update();

                leftCarry.setPosition(ServoPosition);
                rightCarry.setPosition(0);

                sleep(700);
                slides.setPower(0.75);
//                telemetry.update();
                timer.reset();



                if (timer.time() > 2.5)
                {

                    slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

            }
//
            if (b1) { //closed Arm
//                telemetry.update();
                leftCarry.setPosition(0);
                rightCarry.setPosition(ServoPosition);

                sleep(700);

                slides.setPower(-0.5);

                timer.reset();
                timer.startTime();


                if (timer.equals(time))
                {
                    slides.setPower(0);
                    slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

//                telemetry.update();
            }


            if(x1){ //Closed Arm Medium

                leftCarry.setPosition(ServoPosition);
                rightCarry.setPosition(0);

                sleep(700);

                slides.setPower(1);
                timer.reset();
                timer.startTime();


                if(timer.time() > 0.5)
                {
                    slides.setPower(0);
                    slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }

            if(y1){ //Closed Arm Smallest
                leftCarry.setPosition(ServoPosition);
                rightCarry.setPosition(0);

                sleep(700);

                slides.setPower(1);
                timer.reset();
                timer.startTime();


                if (timer.seconds() > .01)
                {

                    slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                }
            }

            if (touch.getState() == true){

                telemetry.addData("touch","is not Pressed");


            }else{
                telemetry.addData("touch","is Pressed");
            }



            telemetry.update();
        }

    }
}