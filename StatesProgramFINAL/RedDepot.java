package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

@Autonomous(name = "RedDepot")
public class RedDepot extends LinearOpMode{

    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor PULLEY = null;
    private DigitalChannel TopSensor = null;

    private Servo MARKER = null;
    private GoldAlignDetector detector;



    private ElapsedTime runtime = new ElapsedTime();

    static final double     TICKS_PER_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.9 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    static final double     TICKS_PER_REV_TORQUE60    = 1680 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION60    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES60   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH60         = (TICKS_PER_REV_TORQUE60 * DRIVE_GEAR_REDUCTION60) /
            (WHEEL_DIAMETER_INCHES60 * 3.1415);

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException


    {
        //Initialize motors
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
        BL = hardwareMap.dcMotor.get("BL");
        PULLEY = hardwareMap.dcMotor.get("PULLEY");
        MARKER = hardwareMap.servo.get("MARKER");
        TopSensor = hardwareMap.digitalChannel.get("TopSensor");

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PULLEY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        PULLEY.setMode(DcMotor.RunMode.RUN_TO_POSITION);





        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        PULLEY.setDirection(DcMotor.Direction.REVERSE);

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();


        // Optional Tuning
        detector.alignSize = 600; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        //changed from 200 to 400 then 600
        // 800 does not work
        detector.alignPosOffset = 5; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.6; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        telemetry.addData("Status" , "Running BlueDepot"); // Gold X pos.

        telemetry.update();

        waitForStart();
        pulleyDrive(1.0,-110,30);
        strafeDrive(0.5,-3,3,30);

        String location = getMineralPos();
        telemetry.addData("MineralPosition",location);
        telemetry.addData("MineralPosition", "Running to %s : %s", location, location);
        telemetry.update();
        detector.disable();

        if ( location.equals("middle")) {
            encoderDrive(.6,15,15,30);// move forward
            strafeDrive(.5,5,-5,30);//strafe to mineral
            encoderDrive(.7,38,38,30);//hit mineral
            MARKER.setPosition(0); // drop marker
            sleep(1000);
            encoderDrive(.6,-35,-35,30);// go back
            encoderDrive(.5,-21,21,30); // turn to crater
            encoderDrive(.8,52,52,30);//drive to crater
            encoderDrive(.5,-6,6,30);// turn to crater 4 to 6
            encoderDrive(.8,16,16,30); //hit crater

        } else if(location.equals("left")) {
            encoderDrive(.7,13,13,30); // going forward
            encoderDrive(.5,-10,10,30); // turns to mineral
            encoderDrive(.8,25,25,30); // hit to mineral changed from 30 to 25
            encoderDrive(.8,-22,-22,30); // going back changed from 27 to 22
            encoderDrive(.5,-10,10,30);// turn left to crater
            encoderDrive(.8,42,42,30); // drive forward
            encoderDrive(.5,32,-32,30); // turn to depot changed from 31.5 - 32.5
            strafeDrive(.5,-7,7,30); // strafe to wall from 5 to 7
            encoderDrive(.7,50,50,30); // drive to depot
            MARKER.setPosition(0);// marker drop
            sleep(1000); //wai
            encoderDrive(1,-70,-70,30); // driving back into crater change 62 to 70
        } else { //moving to right
            encoderDrive(.6,8,8,30); // move forward
            encoderDrive(.5,10,-10,30); // turn to mineral
            encoderDrive(.8,26,26,30); // hit mineral
            encoderDrive(.8,-17,-17,30); //moving backward
            encoderDrive(.5,12,-12,30); // turn
            encoderDrive(.8,-53,-53,30); // drive forward
            encoderDrive(.5,-11,11,30); // turn to depot
            strafeDrive(.5,-9.5,9.5,30);//Strafe to wall from 5 to 9.5
            encoderDrive(.7,51,51,30); // drive to depot
            MARKER.setPosition(0);// marker drop
            sleep(1000); //wait
            encoderDrive(1,-66,-66,30); // driving back into crater 62 to 66 */




        }
        sleep(10000);

        // put (-x) for inches
        // 9.5 of negative
        //encoderDrive(.3,6,6,10); Move robot forward
        //encoderDrive(.3,-6,-6,10); Move robot backward.
        //encoderDrive(.3,9.5,-9.5,10);  Turn Right 90 Degree       9.5 is the real number for the
        //encoderDrive(.3,-9.5,9.5,10);  Turn Left 90 Degree        turning of 90 Degree.


// Positive goes down
        // pulleyDrive(.2, 26, 10);


        // strafeDrive(.3, 2, -2, 5);







        // Initialize Servo


    }
    // (power)0.5 (time) 1000 is a almost perfect 90 degree turn.
    // (power) 1 (time) 1275 is almost a perfect 180 degree turn.



    //public void runOpModethrows InterruptedException

      /* public void encoderDrive(double speedleft,double speedright,
                                double leftInches, double rightInches,
                                double timeoutS)*/

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)
    {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);


            FR.setTargetPosition(newRightFrontTarget);
            FL.setTargetPosition(newLeftFrontTarget);
            BR.setTargetPosition(newRightBackTarget);
            BL.setTargetPosition(newLeftBackTarget);


            // Turn On RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          /* runtime.reset();
           BL.setPower(Math.abs(speedleft));
           BR.setPower(Math.abs(speedright));
           FL.setPower(Math.abs(speedleft));
           FR.setPower(Math.abs(speedright));*/

            // reset the timeout time and start motion.
            runtime.reset();
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 31.0) &&
                    (FR.isBusy() && FL.isBusy() && (BR.isBusy() && (BL.isBusy())))) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);


                // FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                telemetry.update();





            }

            // Stop all motion;
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);



            // Turn off RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void pulleyDrive(double speed,
                            double Inches,
                            double timeoutS) {

        int PulleyDistance;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            PulleyDistance = PULLEY.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH60);


            PULLEY.setTargetPosition(PulleyDistance);


            // Turn On RUN_TO_POSITION
            PULLEY.setMode(DcMotor.RunMode.RUN_TO_POSITION);


          /* runtime.reset();
           BL.setPower(Math.abs(speedleft));
           BR.setPower(Math.abs(speedright));
           FL.setPower(Math.abs(speedleft));
           FR.setPower(Math.abs(speedright));*/

            // reset the timeout time and start motion.
            runtime.reset();
            PULLEY.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 16) && TopSensor.getState()==true &&
                    (PULLEY.isBusy())) {

                // Display it for the driver.
                //  telemetry.addData("Path1",  "Running to %7d :%7d", PulleyDistance);
                //telemetry.addData("Path2",  "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);


                // FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                telemetry.update();


            }

            // Stop all motion;
            PULLEY.setPower(0);


            // Turn off RUN_TO_POSITION
            PULLEY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void strafeDrive(double speed,
                            double leftInches, double rightInches,
                            double timeoutS)
    {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = BL.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = BR.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);


            FR.setTargetPosition(newRightFrontTarget);
            FL.setTargetPosition(newLeftFrontTarget);
            BR.setTargetPosition(newRightBackTarget);
            BL.setTargetPosition(newLeftBackTarget);


            // Turn On RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          /* runtime.reset();
           BL.setPower(Math.abs(speedleft));
           BR.setPower(Math.abs(speedright));
           FL.setPower(Math.abs(speedleft));
           FR.setPower(Math.abs(speedright));*/

            // reset the timeout time and start motion.
            runtime.reset();
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 31.0) &&
                    (FR.isBusy() && FL.isBusy() && (BR.isBusy() && (BL.isBusy())))) {

                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                //telemetry.addData("Path2",  "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);


                // FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                telemetry.update();





            }

            // Stop all motion;
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);



            // Turn off RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public String getMineralPos() {
        String location = "";
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {


            if (detector.getAligned() == true) {
                if (detector.getXPosition() > 300) {
                    location = "middle";
                } else {
                    location = "left";
                }

            } else {
                location = "right";
            }
            // telemetry.addData("IsAligned", SampleFinder.init_detect(hardwareMap)); // Is the bot aligned with the gold mineral
            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos.
            telemetry.addData("Location", location); // Gold X pos.

            telemetry.update();
        }
        return location;


    }


}







