package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/**
 * Field-Centric Mecanum TeleOp for an FTC robot with a Control Hub / IMU
 * mounted on a 45-degree ramp (logo facing DOWN, USB port facing LEFT
 * relative to the robot's forward direction).
 *
 * This code uses raw quaternion math to remove the fixed mount tilt from
 * the IMU reading so that only the robot's yaw on the field remains,
 * enabling accurate field-centric driving even with a non-flat IMU mount.
 *
 * SETUP INSTRUCTIONS:
 *   1. Place the robot FLAT on the floor (do not prop it up).
 *   2. Press INIT on the Driver Station.
 *   3. Wait for "Calibration Complete" on telemetry (~3 seconds). Do NOT
 *      touch or bump the robot during this time.
 *   4. Press START.
 *
 * CONTROLS (gamepad1):
 *   Left Stick        — Translate (field-centric)
 *   Right Stick X     — Rotate
 *   Options Button    — Reset field-centric heading (press once)
 *   Left Bumper       — Decrease outtake speed
 *   Right Bumper      — Increase outtake speed
 *   X Button          — Toggle primary intake on/off
 *   B Button          — Toggle secondary intake on/off
 */
@SuppressWarnings("unused")
@TeleOp(name = "Field Centric Drive", group = "TeleOp")
public class maybeFunctioning extends LinearOpMode {

    // =====================================================================
    // CONSTANTS
    // =====================================================================

    /** Maximum RPM the outtake motors can achieve (used to normalize power). */
    private static final double MAX_OUTTAKE_RPM = 6000.0;

    /** Strafe correction multiplier — compensates for mecanum rollers having
     *  less grip in the lateral direction than fore/aft. */
    private static final double STRAFE_CORRECTION = 1.1;

    /** How long (ms) to wait after IMU initialization for the sensor fusion
     *  to converge before capturing the calibration quaternion. */
    private static final long IMU_SETTLE_MS = 3000;

    /** Selectable outtake RPM presets.  Index 0 = OFF. */
    private static final double[] OUTTAKE_RPM_PRESETS = {
            0, 1500, 2000, 2200, 2300, 2400, 2500, 2600, 2700, 2800,
         2900, 3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800,
         3900, 4000, 4100, 4200, 4300, 4400, 4500, 4600, 4700, 4800,
         4900, 5000, 5100, 5200, 5300, 5400, 5500, 5600, 5700, 5800,
         5900, 6000
    };

    // =====================================================================
    // STATE
    // =====================================================================

    /** Quaternion captured at calibration representing the fixed mount tilt. */
    private Quaternion qMount = null;

    /** Heading offset used to zero the field-centric direction on demand. */
    private double yawZeroOffset = 0.0;

    /** Current index into OUTTAKE_RPM_PRESETS. */
    private int outtakeSpeedIndex = 0;

    /** Toggle states for the two intakes. */
    private boolean primaryIntakeActive  = false;
    private boolean secondaryIntakeActive = false;

    // Edge-detection flags (true = button was pressed on the previous loop).
    private boolean prevLeftBumper  = false;
    private boolean prevRightBumper = false;
    private boolean prevX           = false;
    private boolean prevB           = false;
    private boolean prevOptions     = false;

    // =====================================================================
    // MAIN OP MODE
    // =====================================================================

    @Override
    public void runOpMode() {

        // -----------------------------------------------------------------
        // HARDWARE MAPPING
        // -----------------------------------------------------------------
        DcMotor frontLeft  = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft   = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight  = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor leftOuttake  = hardwareMap.dcMotor.get("leftOuttake");
        DcMotor rightOuttake = hardwareMap.dcMotor.get("rightOuttake");
        DcMotor primaryIntake   = hardwareMap.dcMotor.get("intake");
        DcMotor secondaryIntake = hardwareMap.dcMotor.get("SecondIntake");

        // -----------------------------------------------------------------
        // IMU INITIALIZATION
        // The logo faces DOWN and the USB port faces LEFT relative to the
        // robot's forward direction. This tells the SDK how to remap the
        // IMU's internal axes to match the robot's coordinate frame.
        // -----------------------------------------------------------------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));

        // -----------------------------------------------------------------
        // MOTOR CONFIGURATION
        // Left-side motors are reversed so that positive power = forward
        // for the whole drivetrain.
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
        // IMU CALIBRATION
        // The robot MUST be flat on the floor and perfectly still.
        // We capture the IMU quaternion here; it represents only the fixed
        // 45-degree mount tilt (no robot yaw yet).
        // -----------------------------------------------------------------
        telemetry.addLine("╔══════════════════════════════════╗");
        telemetry.addLine("║  CALIBRATING   DO NOT TOUCH BOT  ║");
        telemetry.addLine("║  Keep robot FLAT on the floor.   ║");
        telemetry.addLine("╚══════════════════════════════════╝");
        telemetry.update();
        sleep(IMU_SETTLE_MS);

        qMount = imu.getRobotOrientationAsQuaternion();

        telemetry.addLine("Calibration complete.");
        telemetry.addData("Mount quaternion", formatQuaternion(qMount));
        telemetry.addLine("Press ▶ START to drive.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =================================================================
        // MAIN LOOP
        // =================================================================
        while (opModeIsActive()) {

            // -------------------------------------------------------------
            // 1. READ IMU & COMPUTE FIELD HEADING
            // -------------------------------------------------------------
            // qHub is the total orientation the IMU currently reads.
            // It is the composition of the robot's yaw (world-frame) and
            // the fixed mount tilt (body-frame):
            //
            //     qHub  =  qRobotYaw  *  qMountTilt
            //
            // To isolate the yaw we right-multiply by the mount inverse:
            //
            //     qRobot  =  qHub  *  qMount⁻¹
            //
            // This removes the body-fixed tilt while preserving the
            // world-frame yaw — exactly what field-centric drive needs.
            // -------------------------------------------------------------
            Quaternion qHub   = imu.getRobotOrientationAsQuaternion();
            Quaternion qRobot = multiply(qHub, inverse(qMount));

            // Project the robot's forward axis [1,0,0] through qRobot and
            // take only the X-Y (floor-plane) components.
            double fwdX = 1.0 - 2.0 * (qRobot.y * qRobot.y + qRobot.z * qRobot.z);
            double fwdY = 2.0 * (qRobot.x * qRobot.y + qRobot.w * qRobot.z);
            double botHeading = Math.atan2(fwdY, fwdX);

            // Allow the driver to re-zero the heading at any time.
            if (gamepad1.options && !prevOptions) {
                yawZeroOffset = botHeading;
            }
            prevOptions = gamepad1.options;

            double fieldHeading = botHeading - yawZeroOffset;

            // -------------------------------------------------------------
            // 2. FIELD-CENTRIC MECANUM DRIVE
            // -------------------------------------------------------------
            double drive  = -gamepad1.left_stick_y;               // forward / back
            double strafe =  gamepad1.left_stick_x * STRAFE_CORRECTION; // left / right
            double turn   = -gamepad1.right_stick_x;              // rotation

            // Rotate the translational vector by the negative heading so
            // that "forward on the stick" always means "toward the far wall"
            // regardless of which way the robot is facing.
            double rotatedX = strafe * Math.cos(-fieldHeading) - drive * Math.sin(-fieldHeading);
            double rotatedY = strafe * Math.sin(-fieldHeading) + drive * Math.cos(-fieldHeading);

            // Standard mecanum power equations, normalized so no motor
            // exceeds full power.
            double denom = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(turn), 1.0);
            frontLeft.setPower( (rotatedY + rotatedX + turn) / denom);
            backLeft.setPower(  (rotatedY - rotatedX + turn) / denom);
            frontRight.setPower((rotatedY - rotatedX - turn) / denom);
            backRight.setPower( (rotatedY + rotatedX - turn) / denom);

            // -------------------------------------------------------------
            // 3. OUTTAKE SPEED CONTROL (bumpers cycle through presets)
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
            double outtakePower = outtakeRPM / MAX_OUTTAKE_RPM;
            leftOuttake.setPower(outtakePower);
            rightOuttake.setPower(outtakePower);

            // -------------------------------------------------------------
            // 4. INTAKE TOGGLES
            // -------------------------------------------------------------
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
            // 5. TELEMETRY
            // -------------------------------------------------------------
            telemetry.addData("Mode",        "FIELD-CENTRIC");
            telemetry.addData("Heading (°)", "%.1f", Math.toDegrees(fieldHeading));
            telemetry.addLine();
            telemetry.addData("Outtake",     "%.0f RPM  [%d/%d]",
                    outtakeRPM, outtakeSpeedIndex, OUTTAKE_RPM_PRESETS.length - 1);
            telemetry.addData("Intake 1",    primaryIntakeActive   ? "ON" : "off");
            telemetry.addData("Intake 2",    secondaryIntakeActive ? "ON" : "off");
            telemetry.update();
        }
    }

    // =====================================================================
    // QUATERNION MATH HELPERS
    // =====================================================================

    /**
     * Hamilton product of two quaternions.
     * FTC SDK Quaternion constructor order: (w, x, y, z, acquisitionTime).
     */
    private Quaternion multiply(Quaternion a, Quaternion b) {
        return new Quaternion(
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,   // w
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,   // x
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,   // y
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,   // z
                System.nanoTime()
        );
    }

    /**
     * Returns the inverse (conjugate) of a unit quaternion.
     * For a unit quaternion, q⁻¹ = (w, -x, -y, -z).
     */
    private Quaternion inverse(Quaternion q) {
        return new Quaternion(q.w, -q.x, -q.y, -q.z, q.acquisitionTime);
    }

    /** Pretty-print a quaternion for telemetry. */
    private String formatQuaternion(Quaternion q) {
        return String.format("(w=%.3f, x=%.3f, y=%.3f, z=%.3f)", q.w, q.x, q.y, q.z);
    }
}
