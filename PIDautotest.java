package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "PID AUTO")
public class PIDautotest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor fL;
    private DcMotor fR;

    private DcMotor bL;

    private DcMotor bR;


    private DcMotorEx elbow;

    private DcMotorEx elbow2;


    Servo clawL;
    Servo clawR;

    Servo wrist;


    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77952;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.85;
    static final double TURN_SPEED = 0.5;
    double diameter = 15.4;
    double arc90 = Math.PI * diameter / 2;


    private final double ticks_in_degrees = 5281.1 / 180;
    public static double p = 0.02, i = 0.15, d = 0.000;
    public static double f = 0.1;



    private PIDController controller;


    @Override
    public void init() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // Initialize the drive system variables.
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        elbow2 = hardwareMap.get(DcMotorEx.class, "elbow2");


        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");


        wrist = hardwareMap.servo.get("wrist");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);


        elbow2.setDirection(DcMotorSimple.Direction.REVERSE);


       fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // elbow2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       elbow2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Send telemetry message to indicate successful Encoder reset

    }


    @Override
    public void loop() {




        encoderDrive(DRIVE_SPEED, 20,20);



        armMove(34);

    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    public void openClaw() {
        clawL.setPosition(0);
        clawR.setPosition(0.4);
    }


    public void armMove(double target) {
        controller.setPID(p, i, d);
        int armPos = elbow.getCurrentPosition();
        int armPos2 = elbow2.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double pid2 = controller.calculate(armPos2, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = pid + ff;

        double power2 = pid2 + ff;

        elbow.setPower(power);
        elbow2.setPower(power2);

        telemetry.addData("pos", armPos);
        telemetry.addData("pos2", armPos2);
        telemetry.addData("target", target);
        telemetry.update();
    }


    public void encoderDrive(double speed,
                             double leftDrive, double rightDrive) {
        int newLeftTar;
        int newRightTar;
        int newBackLeftTar;
        int newBackRightTar;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTar = fL.getCurrentPosition() + (int) (leftDrive * COUNTS_PER_INCH);
        newRightTar = fR.getCurrentPosition() + (int) (rightDrive * COUNTS_PER_INCH);
        newBackRightTar = bR.getCurrentPosition() + (int) (rightDrive * COUNTS_PER_INCH);
        newBackLeftTar = bL.getCurrentPosition() + (int) (leftDrive * COUNTS_PER_INCH);
        fL.setTargetPosition(newLeftTar);
        fR.setTargetPosition(newRightTar);
        bL.setTargetPosition(newBackLeftTar);
        bR.setTargetPosition(newBackRightTar);

        // Turn On RUN_TO_POSITION
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        fL.setPower(Math.abs(speed));
        fR.setPower(Math.abs(speed));
        bL.setPower(Math.abs(speed));
        bR.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        // Display it for the driver.
        telemetry.addData("Running to", " %7d :%7d", newLeftTar, newRightTar, newBackLeftTar, newBackRightTar);
        telemetry.addData("Currently at", " at %7d :%7d",
                fL.getCurrentPosition(), fR.getCurrentPosition(),
                bL.getCurrentPosition(), bR.getCurrentPosition());
        telemetry.update();


        // Turn off RUN_TO_POSITION
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);

    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    public void strafeRight(double speed, double frontLeft,
                            double frontRight, double backLeft,
                            double backRight, double timeoutS) {
        int newLeftTar;
        int newRightTar;
        int newBackLeftTar;
        int newBackRightTar;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTar = fL.getCurrentPosition() + (int) (frontLeft * COUNTS_PER_INCH);
        newRightTar = fR.getCurrentPosition() - (int) (frontRight * COUNTS_PER_INCH);
        newBackRightTar = bR.getCurrentPosition() + (int) (backRight * COUNTS_PER_INCH);
        newBackLeftTar = bL.getCurrentPosition() - (int) (backLeft * COUNTS_PER_INCH);
        fL.setTargetPosition(newLeftTar);
        fR.setTargetPosition(newRightTar);
        bL.setTargetPosition(newBackLeftTar);
        bR.setTargetPosition(newBackRightTar);

        // Turn On RUN_TO_POSITION
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        fL.setPower(Math.abs(speed));
        fR.setPower(Math.abs(speed));
        bL.setPower(Math.abs(speed));
        bR.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        // Display it for the driver.
        telemetry.addData("Running to", " %7d :%7d", newLeftTar, newRightTar, newBackLeftTar, newBackRightTar);
        telemetry.addData("Currently at", " at %7d :%7d",
                fL.getCurrentPosition(), fR.getCurrentPosition(),
                bL.getCurrentPosition(), bR.getCurrentPosition());
        telemetry.update();



    // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);

}





    public void strafeLeft(double speed, double frontLeft,
                           double frontRight, double backLeft,
                           double backRight, double timeoutS) {
        int newLeftTar;
        int newRightTar;
        int newBackLeftTar;
        int newBackRightTar;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            newLeftTar = fL.getCurrentPosition() + (int) (frontLeft * COUNTS_PER_INCH);
            newRightTar = fR.getCurrentPosition() + (int) (frontRight * COUNTS_PER_INCH);
            newBackRightTar = bR.getCurrentPosition() + (int) (backRight * COUNTS_PER_INCH);
            newBackLeftTar = bL.getCurrentPosition() + (int) (backLeft * COUNTS_PER_INCH);
            fL.setTargetPosition(newLeftTar);
            fR.setTargetPosition(newRightTar);
            bL.setTargetPosition(newBackLeftTar);
            bR.setTargetPosition(newBackRightTar);

            // Turn On RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            fL.setPower(Math.abs(-speed));
            fR.setPower(Math.abs(speed));
            bL.setPower(Math.abs(speed));
            bR.setPower(Math.abs(-speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTar, newRightTar, newBackLeftTar, newBackRightTar);
                telemetry.addData("Currently at", " at %7d :%7d",
                        fL.getCurrentPosition(), fR.getCurrentPosition(),
                        bL.getCurrentPosition(), bR.getCurrentPosition());
                telemetry.update();



            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);

        }



    public void elbowmove(double speed,
                          double elbowenco, double timeoutS) {

        int newElbow;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller

            newElbow = elbow.getCurrentPosition() + (int) (elbowenco * COUNTS_PER_INCH);

            elbow.setTargetPosition(newElbow);

            // Turn On RUN_TO_POSITION


            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            elbow.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

                // Display it for the driver.
                telemetry.addData("Running to", " %7d", newElbow);
                telemetry.addData("Currently at", " at %7d",

                        elbow.getCurrentPosition());
                telemetry.update();



            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            elbow.setPower(0);

        }
    }

///////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////