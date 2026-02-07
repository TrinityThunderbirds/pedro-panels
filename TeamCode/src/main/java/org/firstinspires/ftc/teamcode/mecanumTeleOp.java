package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ErrorCalculator;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;


@SuppressWarnings("unused")
@TeleOp
public class mecanumTeleOp extends LinearOpMode {

    boolean leftBumperPreviouslyPressed = false;
    boolean rightBumperPreviouslyPressed = false;
    boolean firstIntake = false;
    boolean secondIntakeOn = false;

    // Predefined outtake speed levels in RPM (max 6000)
    double[] outtakeRPMs = {0, 1500, 2000, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200, 4300, 4400, 4500, 4600, 4700, 4800, 4900, 5000, 5100, 5200, 5300, 5400, 5500, 5600, 5700, 5800, 5900, 6000};
    static final double MAX_RPM = 6000.0;
    int speedIndex = 0;

    // Define the motor directions for "forward" motion
    final DcMotor.Direction FL_FORWARD = DcMotor.Direction.REVERSE;
    final DcMotor.Direction BL_FORWARD = DcMotor.Direction.REVERSE;
    final DcMotor.Direction FR_FORWARD = DcMotor.Direction.FORWARD;
    final DcMotor.Direction BR_FORWARD = DcMotor.Direction.FORWARD;

    @Override
    public void runOpMode() {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor leftOuttake = hardwareMap.dcMotor.get("leftOuttake");
        DcMotor rightOuttake = hardwareMap.dcMotor.get("rightOuttake");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        DcMotor SecondIntake = hardwareMap.dcMotor.get("SecondIntake");

        frontLeftMotor.setDirection(FL_FORWARD);
        backLeftMotor.setDirection(BL_FORWARD);
        frontRightMotor.setDirection(FR_FORWARD);
        backRightMotor.setDirection(BR_FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        rightOuttake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPowerVal = (y + x + rx) / denominator;
            double backLeftPowerVal = (y - x + rx) / denominator;
            double frontRightPowerVal = (y - x - rx) / denominator;
            double backRightPowerVal = (y + x - rx) / denominator;

            // --- Motor Power and Direction Logic ---

            if (frontLeftPowerVal < 0) {
                frontLeftMotor.setDirection(FL_FORWARD.inverted());
                frontLeftMotor.setPower(Math.abs(frontLeftPowerVal));
            } else {
                frontLeftMotor.setDirection(FL_FORWARD);
                frontLeftMotor.setPower(frontLeftPowerVal);
            }

            if (backLeftPowerVal < 0) {
                backLeftMotor.setDirection(BL_FORWARD.inverted());
                backLeftMotor.setPower(Math.abs(backLeftPowerVal));
            } else {
                backLeftMotor.setDirection(BL_FORWARD);
                backLeftMotor.setPower(backLeftPowerVal);
            }

            if (frontRightPowerVal < 0) {
                frontRightMotor.setDirection(FR_FORWARD.inverted());
                frontRightMotor.setPower(Math.abs(frontRightPowerVal));
            } else {
                frontRightMotor.setDirection(FR_FORWARD);
                frontRightMotor.setPower(frontRightPowerVal);
            }

            if (backRightPowerVal < 0) {
                backRightMotor.setDirection(BR_FORWARD.inverted());
                backRightMotor.setPower(Math.abs(backRightPowerVal));
            } else {
                backRightMotor.setDirection(BR_FORWARD);
                backRightMotor.setPower(backRightPowerVal);
            }


            if(gamepad1.left_bumper && !leftBumperPreviouslyPressed){
                speedIndex--;
                if(speedIndex < 0){
                    speedIndex = 0;
                }
            }
            if(gamepad1.right_bumper && !rightBumperPreviouslyPressed){
                speedIndex++;
                if(speedIndex >= outtakeRPMs.length){
                    speedIndex = outtakeRPMs.length - 1;
                }
            }

            leftBumperPreviouslyPressed = gamepad1.left_bumper;
            rightBumperPreviouslyPressed = gamepad1.right_bumper;

            double OuttakeSpeed = outtakeRPMs[speedIndex] / MAX_RPM;

            if(gamepad1.x){
                if(firstIntake){
                    firstIntake = false;
                    intake.setPower(0);
                }
                else{
                    firstIntake = true;
                    intake.setPower(1);
                }
            }

            // second intake controls
            if(gamepad1.b){
                if(secondIntakeOn){
                    secondIntakeOn = false;
                    SecondIntake.setPower(0);
                }
                else{
                    secondIntakeOn = true;
                    SecondIntake.setPower(1);
                }
            }

            rightOuttake.setPower(OuttakeSpeed);
            leftOuttake.setPower(OuttakeSpeed);

            telemetry.addData("Outtake RPM: ", outtakeRPMs[speedIndex]);
            telemetry.addData("Outtake Power: ", OuttakeSpeed);
            telemetry.addData("Speed Level: ", speedIndex + " / " + (outtakeRPMs.length - 1));
            telemetry.update();
        }
    }
}
