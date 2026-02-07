package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@SuppressWarnings("unused")
@TeleOp
public class totallyFunctioning extends LinearOpMode {

    // --- TRACKING VARIABLES (DEBOUNCING) ---
    boolean leftBumperPreviouslyPressed = false;
    boolean rightBumperPreviouslyPressed = false;
    boolean xPreviouslyPressed = false;
    boolean bPreviouslyPressed = false;

    // --- STATE VARIABLES ---
    boolean firstIntakeActive = false;
    boolean secondIntakeActive = false;

    // --- OUTTAKE CONFIGURATION ---
    // Max RPM of your motor (Gobilda 6000RPM Yellow Jacket?)
    static final double MAX_RPM = 6000.0;
    
    // The specific speeds you wanted to cycle through
    double[] outtakeRPMs = {
        0, 1500, 2000, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 
        2900, 3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 
        3900, 4000, 4100, 4200, 4300, 4400, 4500, 4600, 4700, 4800, 
        4900, 5000, 5100, 5200, 5300, 5400, 5500, 5600, 5700, 5800, 
        5900, 6000
    };
    
    int speedIndex = 0; // Starts at 0 RPM

    @Override
    public void runOpMode() {

        // --- HARDWARE MAPPING ---
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        
        DcMotor leftOuttake = hardwareMap.dcMotor.get("leftOuttake");
        DcMotor rightOuttake = hardwareMap.dcMotor.get("rightOuttake");
        
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor SecondIntake = hardwareMap.dcMotor.get("SecondIntake");

        // --- MOTOR DIRECTIONS ---
        // Standard Mecanum Setup: Reverse Left side
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        
        // Auxiliary Directions
        intake.setDirection(DcMotor.Direction.REVERSE);
        rightOuttake.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse one so they spin together

        // Ensure motors brake when power is 0 (optional but recommended for drive)
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready to start!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            if (gamepad1.left_bumper && !leftBumperPreviouslyPressed) {
                speedIndex--;
                if (speedIndex < 0) {
                    speedIndex = 0;
                }
            }
            leftBumperPreviouslyPressed = gamepad1.left_bumper;

            if (gamepad1.right_bumper && !rightBumperPreviouslyPressed) {
                speedIndex++;
                if (speedIndex >= outtakeRPMs.length) {
                    speedIndex = outtakeRPMs.length - 1;
                }
            }
            rightBumperPreviouslyPressed = gamepad1.right_bumper;

            double currentRPM = outtakeRPMs[speedIndex];
            double outtakePower = currentRPM / MAX_RPM;

            leftOuttake.setPower(outtakePower);
            rightOuttake.setPower(outtakePower);


            if (gamepad1.x && !xPreviouslyPressed) {
                firstIntakeActive = !firstIntakeActive;
                if (firstIntakeActive) {
                    intake.setPower(1.0);
                } else {
                    intake.setPower(0.0);
                }
            }
            xPreviouslyPressed = gamepad1.x;

            if (gamepad1.b && !bPreviouslyPressed) {
                secondIntakeActive = !secondIntakeActive;
                if (secondIntakeActive) {
                    SecondIntake.setPower(1.0);
                } else {
                    SecondIntake.setPower(0.0);
                }
            }
            bPreviouslyPressed = gamepad1.b;


            telemetry.addData("Target RPM", currentRPM);
            telemetry.addData("Motor Power", "%.2f", outtakePower);
            telemetry.addData("Speed Level", "%d / %d", speedIndex, outtakeRPMs.length - 1);
            telemetry.addLine();
            telemetry.addData("Intake 1 (X)", firstIntakeActive ? "ON" : "OFF");
            telemetry.addData("Intake 2 (B)", secondIntakeActive ? "ON" : "OFF");
            telemetry.update();
        }
    }
}
