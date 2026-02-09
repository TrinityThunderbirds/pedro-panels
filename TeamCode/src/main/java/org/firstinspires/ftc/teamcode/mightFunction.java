package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

/**
 * FINAL COMPETITION CODE
 * Field-Centric Mecanum TeleOp with Control Hub mounted on 45-degree ramp.
 *
 * CONFIGURATION:
 * - Hub Logo: DOWN
 * - Hub USB:  LEFT
 *
 * CONTROLS:
 * Left Stick         — Translate
 * Right Stick X      — Rotate
 * Triggers (L/R)     — Precision Mode (Slower speed for fine movements)
 * Options (Start)    — Reset Field-Centric Heading (Square up against wall first!)
 * BACK Button        — EMERGENCY SWITCH: Toggle between Field-Centric and Robot-Centric
 * Bumpers (L/R)      — Outtake Speed Control
 * X / B Buttons      — Intake Toggles
 */
@SuppressWarnings("unused")
@TeleOp(name = "Field Centric FINAL", group = "Competition")
public class mightFunction extends LinearOpMode {

    // =====================================================================
    // CONSTANTS
    // =====================================================================
    private static final double MAX_OUTTAKE_RPM    = 6000.0;
    private static final double STRAFE_CORRECTION  = 1.1;
    private static final double JOYSTICK_DEADZONE  = 0.05;
    private static final long   IMU_SETTLE_MS      = 3000;
    private static final double PRECISION_MIN_SCALE = 0.5; // 50% speed when trigger pressed

    private static final double[] OUTTAKE_RPM_PRESETS = {
            0, 1500, 2000, 2200, 2300, 2400, 2500, 2600, 2700, 2800,
         2900, 3000, 3100, 3200, 3300
    };

    // =====================================================================
    // STATE VARIABLES
    // =====================================================================
    private Quaternion qMount        = null;
    private double     yawZeroOffset = 0.0;
    private int        outtakeSpeedIndex    = 0;
    private boolean    primaryIntakeActive   = false;
    private boolean    secondaryIntakeActive = false;
    
    // Drive Mode State
    private boolean    useFieldCentric = true; // Starts in Field-Centric

    // Button Toggle Flags
    private boolean prevLeftBumper  = false;
    private boolean prevRightBumper = false;
    private boolean prevX           = false;
    private boolean prevB           = false;
    private boolean prevOptions     = false;
    private boolean prevBack        = false;

    @Override
    public void runOpMode() {

        // -----------------------------------------------------------------
        // 1. PERFORMANCE: MANUAL BULK CACHING
        // -----------------------------------------------------------------
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // -----------------------------------------------------------------
        // 2. HARDWARE MAPPING
        // -----------------------------------------------------------------
        DcMotor frontLeft  = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft   = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight  = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor leftOuttake     = hardwareMap.dcMotor.get("leftOuttake");
        DcMotor rightOuttake    = hardwareMap.dcMotor.get("rightOuttake");
        DcMotor primaryIntake   = hardwareMap.dcMotor.get("intake");
        DcMotor secondaryIntake = hardwareMap.dcMotor.get("SecondIntake");

        // -----------------------------------------------------------------
        // 3. IMU INITIALIZATION
        // -----------------------------------------------------------------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));

        // -----------------------------------------------------------------
        // 4. MOTOR CONFIGURATION
        // -----------------------------------------------------------------
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        primaryIntake.setDirection(DcMotor.Direction.REVERSE);
        rightOuttake.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // -----------------------------------------------------------------
        // 5. CALIBRATION
        // -----------------------------------------------------------------
        telemetry.addData("STATUS", "CALIBRATING - DO NOT TOUCH");
        telemetry.addData("INFO",   "Place robot FLAT on floor.");
        telemetry.update();
        sleep(IMU_SETTLE_MS);

        qMount = imu.getRobotOrientationAsQuaternion();

        telemetry.addData("STATUS", "READY");
        telemetry.addData("Mount", formatQuaternion(qMount));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        ElapsedTime loopTimer = new ElapsedTime();

        // =================================================================
        // MAIN LOOP
        // =================================================================
        while (opModeIsActive()) {

            // Clear cache at start of loop (Essential for Manual Mode)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // -------------------------------------------------------------
            // DRIVE MODE TOGGLE (BACK BUTTON)
            // -------------------------------------------------------------
            if (gamepad1.back && !prevBack) {
                useFieldCentric = !useFieldCentric;
            }
            prevBack = gamepad1.back;

            // -------------------------------------------------------------
            // SENSOR READING & HEADING CALC
            // -------------------------------------------------------------
            Quaternion qHub   = imu.getRobotOrientationAsQuaternion();
            Quaternion qRobot = multiply(qHub, inverse(qMount));

            double fwdX = 1.0 - 2.0 * (qRobot.y * qRobot.y + qRobot.z * qRobot.z);
            double fwdY = 2.0 * (qRobot.x * qRobot.y + qRobot.w * qRobot.z);
            double botHeading = Math.atan2(fwdY, fwdX);

            // Yaw Reset (Options Button)
            if (gamepad1.options && !prevOptions) {
                yawZeroOffset = botHeading;
            }
            prevOptions = gamepad1.options;

            double fieldHeading = botHeading - yawZeroOffset;

            // -------------------------------------------------------------
            // DRIVETRAIN MATH
            // -------------------------------------------------------------
            // 1. Read Inputs + Deadzone
            double rawDrive  = applyDeadzone(-gamepad1.left_stick_y,  JOYSTICK_DEADZONE);
            double rawStrafe = applyDeadzone( gamepad1.left_stick_x,  JOYSTICK_DEADZONE);
            double rawTurn   = applyDeadzone(-gamepad1.right_stick_x, JOYSTICK_DEADZONE);

            // 2. Square Inputs (Precision curve)
            double drive  = Math.copySign(rawDrive  * rawDrive,  rawDrive);
            double strafe = Math.copySign(rawStrafe * rawStrafe, rawStrafe) * STRAFE_CORRECTION;
            double turn   = Math.copySign(rawTurn   * rawTurn,   rawTurn);

            // 3. Precision Mode (Triggers)
            double triggerValue = Math.max(gamepad1.left_trigger, gamepad1.right_trigger);
            double speedScale   = 1.0 - triggerValue * (1.0 - PRECISION_MIN_SCALE);
            
            drive  *= speedScale;
            strafe *= speedScale;
            turn   *= speedScale;

            // 4. Field Centric Rotation (Conditional)
            double rotatedX = strafe;
            double rotatedY = drive;

            if (useFieldCentric) {
                rotatedX = strafe * Math.cos(-fieldHeading) - drive * Math.sin(-fieldHeading);
                rotatedY = strafe * Math.sin(-fieldHeading) + drive * Math.cos(-fieldHeading);
            }

            // 5. Motor Powers
            double denom = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(turn), 1.0);
            frontLeft.setPower( (rotatedY + rotatedX + turn) / denom);
            backLeft.setPower(  (rotatedY - rotatedX + turn) / denom);
            frontRight.setPower((rotatedY - rotatedX - turn) / denom);
            backRight.setPower( (rotatedY + rotatedX - turn) / denom);

            // -------------------------------------------------------------
            // MECHANISMS (Outtake & Intakes)
            // -------------------------------------------------------------
            if (gamepad1.left_bumper && !prevLeftBumper) {
                outtakeSpeedIndex = Math.max(0, outtakeSpeedIndex - 1);
            }
            prevLeftBumper = gamepad1.left_bumper;

            if (gamepad1.right_bumper && !prevRightBumper) {
                outtakeSpeedIndex = Math.min(OUTTAKE_RPM_PRESETS.length - 1, outtakeSpeedIndex + 1);
            }
            prevRightBumper = gamepad1.right_bumper;

            double outtakeRPM   = OUTTAKE_RPM_PRESETS[outtakeSpeedIndex];
            leftOuttake.setPower(outtakeRPM / MAX_OUTTAKE_RPM);
            rightOuttake.setPower(outtakeRPM / MAX_OUTTAKE_RPM);

            if (gamepad1.x && !prevX) {
                primaryIntakeActive = !primaryIntakeActive;
                primaryIntake.setPower(primaryIntakeActive ? 1.0 : 0.0);
            }
            prevX = gamepad1.x;

            if (gamepad1.b && !prevB) {
                secondaryIntakeActive = !secondaryIntakeActive;
                secondaryIntake.setPower(secondaryIntakeActive ? 1.0 : 0.0);
            }
            prevB = gamepad1.b;

            // -------------------------------------------------------------
            // TELEMETRY (Clean & Data Focused)
            // -------------------------------------------------------------
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            telemetry.addData("DRIVE MODE", useFieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addData("Heading",    "%.1f deg", Math.toDegrees(fieldHeading));
            telemetry.addData("Loop Time",  "%.1f ms", loopMs);
            telemetry.addData("Speed Scale","%.0f%%", speedScale * 100);
            telemetry.addData("Outtake",    "%.0f RPM (%d)", outtakeRPM, outtakeSpeedIndex);
            telemetry.addData("Intake 1",   primaryIntakeActive ? "ON" : "OFF");
            telemetry.addData("Intake 2",   secondaryIntakeActive ? "ON" : "OFF");
            telemetry.update();
        }
    }

    // =====================================================================
    // MATH HELPERS
    // =====================================================================
    private Quaternion multiply(Quaternion a, Quaternion b) {
        return new Quaternion(
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                System.nanoTime()
        );
    }

    private Quaternion inverse(Quaternion q) {
        return new Quaternion(q.w, -q.x, -q.y, -q.z, q.acquisitionTime);
    }

    private String formatQuaternion(Quaternion q) {
        return String.format("(%.2f, %.2f, %.2f, %.2f)", q.w, q.x, q.y, q.z);
    }

    private double applyDeadzone(double input, double threshold) {
        if (Math.abs(input) < threshold) return 0.0;
        return Math.copySign((Math.abs(input) - threshold) / (1.0 - threshold), input);
    }
}
