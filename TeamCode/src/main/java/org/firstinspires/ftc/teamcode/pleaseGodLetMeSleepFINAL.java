package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@SuppressWarnings("unused")
@TeleOp
public class pleaseGodLetMeSleepFINAL extends LinearOpMode {

    boolean leftBumperPreviouslyPressed = false;
    boolean rightBumperPreviouslyPressed = false;
    boolean firstIntake = false;
    boolean secondIntakeOn = false;

    // Predefined outtake speed levels in RPM (max 6000)
    double[] outtakeRPMs = {0, 1500, 2000, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200, 4300, 4400, 4500, 4600, 4700, 4800, 4900, 5000, 5100, 5200, 5300, 5400, 5500, 5600, 5700, 5800, 5900, 6000};
    static final double MAX_RPM = 6000.0;
    int speedIndex = 0;

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
            if(gamepad1.b)
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