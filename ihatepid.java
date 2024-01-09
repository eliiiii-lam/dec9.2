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
@Autonomous

public class ihatepid extends LinearOpMode {

    DcMotorEx fL;
    DcMotorEx fR;
    DcMotorEx bL;
    DcMotorEx bR;

    DcMotorEx elbow;
    DcMotorEx elbow2;


    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.77952;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    double integralSum = 0;
    double Kp = 0.02;
    double Ki = 0.15;
    double Kd = 0;
    double Kf = 0.1;
    double target = 0;


    private final double ticks_in_degrees = 5281.1 / 180.0;

    private PIDController controller;



    ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException{

        controller = new PIDController(Kp, Ki, Kd);


        fL = hardwareMap.get(DcMotorEx.class, "fL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bL = hardwareMap.get(DcMotorEx.class, "bR");
        bR = hardwareMap.get(DcMotorEx.class, "bR");

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        elbow2 = hardwareMap.get(DcMotorEx.class, "elbow2");

        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);

        elbow2.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);






        waitForStart();



        encoderDrive(0.6,12,12);

        target = 400;


        while (opModeIsActive()){

            controller.setPID(Kp, Ki, Kd);
            int armPos = elbow.getCurrentPosition();
            int armPos2 = elbow2.getCurrentPosition();
            double pid = controller.calculate(armPos,target);
            double pid2 = controller.calculate(armPos2,target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * Kf;

            double power = pid + ff;

            double power2 = pid2 + ff;

            elbow.setPower(power);
            elbow2.setPower(power2);

            telemetry.addData("pos", armPos);
            telemetry.addData("pos2", armPos2);
            telemetry.addData("target", target);
            telemetry.update();


            //arm(0,1);

           // elbow.setPower(0);
            //elbow2.setPower(0);
            //encoderDrive(0.6,10,10);
            //arm(200,1);

          //  arm(0);
           // elbow2.setPower(PIDControl(100,elbow2.getCurrentPosition()));




        }
    }


    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();


        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }

    public double arm(double target, double timeoutS){
        controller.setPID(Kp, Ki, Kd);
        int armPos = elbow.getCurrentPosition();
        int armPos2 = elbow2.getCurrentPosition();

        double pid = controller.calculate(armPos,target);
        double pid2 = controller.calculate(armPos2,target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * Kf;

        double power = pid + ff;

        double power2 = pid2 + ff;

        elbow.setPower(power);

        elbow2.setPower(power2);

        telemetry.addData("pos", armPos);
        telemetry.addData("pos2", armPos2);
        telemetry.addData("target", target);
        telemetry.update();

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && (elbow.isBusy() && elbow2.isBusy())) {
            telemetry.addData("Running to", "%7d:%7d", armPos, armPos2);
            telemetry.addData("Currently at", "at %7d:%7d", elbow.getCurrentPosition(), elbow2.getCurrentPosition());
            telemetry.update();
        }

        return ff;



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





}
