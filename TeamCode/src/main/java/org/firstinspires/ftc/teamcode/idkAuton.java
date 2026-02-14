package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto: Forward 1 Sec (Mecanum Calc)", group = "Autonomous")
public class idkAuton extends LinearOpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    @Override
    public void runOpMode() {

        // 1. Hardware Map (Matches your TeleOp)
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // 2. Directions (Matches your TeleOp)
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // 3. Set Zero Power Behavior (Crucial for Auto accuracy)
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // --- COMMAND 1: MOVE FORWARD ---
            // y = 0.5 (Forward power)
            // x = 0.0 (No strafe)
            // rx = 0.0 (No turn)
            driveMecanum(-0.5, -0.5, 0);
            
            // Run for 1 second
            sleep(1000);

            // --- COMMAND 2: STOP ---
            driveMecanum(0, 0, 0);
        }
    }

    /**
     * This function applies the Mecanum kinematic equation.
     * * @param forward (y) - Negative is backward, Positive is forward
     * @param strafe  (x) - Positive is right, Negative is left
     * @param turn    (rx) - Positive is right turn, Negative is left turn
     */
    public void driveMecanum(double forward, double strafe, double turn) {
        
        // This is the same math from your TeleOp
        // Note: We don't need the 1.1 strafe correction for simple auto moves, 
        // but you can add it back if strafing distance is inconsistent.

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        
        double frontLeftPower = (forward + strafe + turn) / denominator;
        double backLeftPower = (forward - strafe + turn) / denominator;
        double frontRightPower = (forward - strafe - turn) / denominator;
        double backRightPower = (forward + strafe - turn) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}
