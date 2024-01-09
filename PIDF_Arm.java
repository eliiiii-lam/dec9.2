package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDF_Arm extends OpMode {
    private PIDController controller;
    public static double p = 0.02, i = 0.15, d = 0.000;
    public static double f = 0.1;
    public static int target = 0;
    private final double ticks_in_degrees = 5281.1 / 180.0;
    private DcMotorEx elbow;
    private DcMotorEx elbow2;



    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        elbow2 = hardwareMap.get(DcMotorEx.class, "elbow2");

        elbow2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = elbow.getCurrentPosition();
        int armPos2 = elbow2.getCurrentPosition();
        double pid = controller.calculate(armPos,target);
        double pid2 = controller.calculate(armPos2,target);
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
}