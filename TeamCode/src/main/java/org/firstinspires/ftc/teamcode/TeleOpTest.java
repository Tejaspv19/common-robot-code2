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
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import kotlin.reflect.KCallable;


@TeleOp
public class TeleOpTest extends LinearOpMode {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor backLeft;

    public DcMotor slides;

//    RevBlinkinLedDriver lights;

    public DigitalChannel touch;
    public DistanceSensor sensorRange;
//
//    public RevBlinkinLedDriver coolLights;

    //    public DcMotor slides
    public Servo leftCarry;
    public Servo rightCarry;

    double ServoPosition = 0.35;



//    DistanceSensor dis;

    public ElapsedTime timer = new ElapsedTime();

    public void runOpMode() throws InterruptedException {


        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensorRange");
//        touch = hardwareMap.get(DigitalChannel.class, "touch");

        slides = hardwareMap.dcMotor.get("slides");

        leftCarry = hardwareMap.servo.get("leftCarry");
        rightCarry = hardwareMap.servo.get("rightCarry");
//        coolLights = hardwareMap.get(RevBlinkinLedDriver.class, "coolLights");
//        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

//        touch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();



        while (opModeIsActive()) {

            telemetry.addData("deviceName",sensorRange.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));


            telemetry.update();


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
            double r = Math.hypot(gamepad1.left_stick_x, -1 * gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-1 * gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            double s = Math.hypot(gamepad2.left_stick_x, -1 * gamepad2.left_stick_y);
            double robotAngle2 = Math.atan2(-1 * gamepad2.left_stick_y, gamepad2.left_stick_x) - Math.PI / 4;
            double rightY = gamepad2.right_stick_x;

            final double v1 = r * Math.cos(robotAngle) + 0.5 * rightX;
            final double v2 = r * Math.sin(robotAngle) - 0.5 * rightX;
            final double v3 = r * Math.sin(robotAngle) + 0.5 * rightX;
            final double v4 = r * Math.cos(robotAngle) - 0.5 * rightX;






            final double v5 = s * Math.cos(robotAngle2) + 0.5 * rightY;
//            final double v6 = r * Math.sin(robotAngle2) - 0.5 * rightY;
//            final double v7 = r * Math.sin(robotAngle2) + 0.5 * rightY;
//            final double v8 = r * Math.cos(robotAngle2) - 0.5 * rightY;

//            set drivetrain velocities
            frontLeft.setPower(0.75*v1);
            frontRight.setPower(0.75*v2);
            backLeft.setPower(0.75*v3);
            backRight.setPower(0.75*v4);

            slides.setPower(0.8*v5);

            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            timer.startTime();

//            int position = slides.getCurrentPosition();
//            telemetry.addData("Encoder Position", position);
//            telemetry.update();
            //inching each direction


//            if (sensorRange.getDistance()) {
//            }


            if (sensorRange.getDistance(DistanceUnit.INCH) < 3 && (!rightBump1 || !rightBump2)){

                leftCarry.setPosition(ServoPosition);
                rightCarry.setPosition(0);

            }

            if(leftBump1 || leftBump2){

                leftCarry.setPosition(ServoPosition);
                rightCarry.setPosition(0);

            }

            if(rightBump1 || rightBump2){

                leftCarry.setPosition(0);
                rightCarry.setPosition(ServoPosition);

            }




            if (y1  || y2) { // Closed Arm Largest
//                telemetry.update();

                timer.reset();
                timer.startTime();

                while (timer.seconds() < 3){
                    leftCarry.setPosition(ServoPosition);
                    rightCarry.setPosition(0);

                    sleep(700);
                    slides.setPower(1);
                }


//
                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slides.setPower(0);


            }
//
            if (b1 || b2) { //Open Arm
//                telemetry.update();

                timer.reset();
                timer.startTime();

                while (timer.seconds() < 2.5){

                    leftCarry.setPosition(0);
                    rightCarry.setPosition(ServoPosition);

//                    sleep(700);

                    slides.setPower(-0.5);
                }


                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slides.setPower(0);

            }


            if(x1 || x2){ //Closed Arm Medium

                timer.reset();
                timer.startTime();

                while (timer.seconds() < 2){


                    leftCarry.setPosition(ServoPosition);
                    rightCarry.setPosition(0);

                    sleep(700);

                    slides.setPower(1);

                }

                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slides.setPower(0);

            }

            if(a1 || a2){ //Closed Arm Smallest

                timer.reset();
                timer.startTime();

                while (timer.seconds() < 0.8) {
                    leftCarry.setPosition(ServoPosition);
                    rightCarry.setPosition(0);

                    sleep(700);

                    slides.setPower(1);
                }


                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slides.setPower(0);


            }

            if (dpadR1){

                frontLeft.setPower(0.75);
                frontRight.setPower(-0.75);
                backLeft.setPower(-0.75);
                backRight.setPower(0.75);

            }

            if (dpadL1){
                frontLeft.setPower(-0.75);
                frontRight.setPower(0.75);
                backLeft.setPower(0.75);
                backRight.setPower(-0.75);
            }
//
//            if (sensorRange.getDistance(DistanceUnit.INCH) < 3) {
//
//                telemetry.addData("distance", "is close");
//
//            }else {
//                telemetry.addData("distance", "is not close");
//            }
//
//
//
//            if (touch.getState() == true){
//
//                telemetry.addData("touch","is not Pressed");
//
//
//            }else{
//                telemetry.addData("touch","is Pressed");
//            }



            telemetry.update();
        }

    }
}