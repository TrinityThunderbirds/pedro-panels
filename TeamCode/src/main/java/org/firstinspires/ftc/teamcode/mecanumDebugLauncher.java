package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@SuppressWarnings("unused")
@TeleOp
public class mecanumDebugLauncher extends LinearOpMode {

    boolean leftBumperPreviouslyPressed = false;
    boolean rightBumperPreviouslyPressed = false;
    boolean firstIntake = false;
    boolean secondIntakeOn = false;

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


        double OuttakeSpeed = 0.000;
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        rightOuttake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator);
            double backLeftPower = ((y - x + rx) / denominator);
            double frontRightPower = ((y - x - rx) / denominator);
            double backRightPower = ((y + x - rx) / denominator);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            if(gamepad1.left_bumper && !leftBumperPreviouslyPressed){
                OuttakeSpeed -= 0.050;
            }
            if(gamepad1.right_bumper && !rightBumperPreviouslyPressed){
                OuttakeSpeed += 0.050;
            }

            leftBumperPreviouslyPressed = gamepad1.left_bumper;
            rightBumperPreviouslyPressed = gamepad1.right_bumper;

            if(OuttakeSpeed > 1.000){
                OuttakeSpeed = 1.000;
            }
            if(OuttakeSpeed < 0.250){
                OuttakeSpeed = .250;
            }

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
//idk lol
            rightOuttake.setPower(OuttakeSpeed);
            leftOuttake.setPower(OuttakeSpeed);

            telemetry.addData("Outtake RPM: ", (OuttakeSpeed * 6000));
            telemetry.update();
        }
    }
}