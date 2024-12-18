package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class StarterBot2025TeleOpJava extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    private DcMotor wrist = null;
    private CRServo claw = null;
    private Servo intake = null;
    private DcMotor linearSlide = null; // Declare linear slide motor
    private DcMotor rotationMotor = null; // Declare motor for rotation

    // Arm and Wrist target positions for each state
    private static final int ARM_POSITION_INIT = 0;
    private static final int ARM_POSITION_INTAKE = 0;
    private static final int ARM_POSITION_WALL_GRAB = 0;
    private static final int ARM_POSITION_WALL_UNHOOK = 0;
    private static final int ARM_POSITION_HOVER_HIGH = 0;
    private static final int ARM_POSITION_CLIP_HIGH = 0;
    private static final int ARM_POSITION_LOW_BASKET = 0;

    private static final int WRIST_POSITION_INIT = 0;
    private static final int WRIST_POSITION_SAMPLE = 270;
    private static final int WRIST_POSITION_SPEC = 10;

    // Claw positions
    private static final double CLAW_OPEN_POSITION = 0.55;
    private static final double CLAW_CLOSED_POSITION = 0.7;

    // Enum for state machine
    private enum RobotState {
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        HOVER_HIGH,
        CLIP_HIGH,
        LOW_BASKET,
        MANUAL
    }

    // Initial state
    private RobotState currentState = RobotState.INIT;

    // Claw toggle state
    private boolean clawOpen = true;
    private boolean lastBump = false;
    private boolean lastHook = false;
    private boolean lastGrab = false;
    
    // Target positions
    private int targetArm = 130;
    private int targetWrist = 0;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "motorR");
        rightDrive = hardwareMap.get(DcMotor.class, "motorL");
        arm = hardwareMap.get(DcMotor.class, "uArmMotor");
        wrist = hardwareMap.get(DcMotor.class, "lArmMotor");
        claw = hardwareMap.get(CRServo.class, "y");
        intake = hardwareMap.get(Servo.class, "intake");
        linearSlide = hardwareMap.get(DcMotor.class, "ExtenderMotor"); // Initialize linear slide motor
        rotationMotor = hardwareMap.get(DcMotor.class, "ExtenderAngle"); // Initialize rotation motor

        // Stop and reset encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset linear slide encoder
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset rotation motor encoder

        // Set motors to use encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set motor direction
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotor.Direction.FORWARD); // Set direction for linear slide
        rotationMotor.setDirection(DcMotor.Direction.FORWARD); // Set direction for rotation motor

        // Set zero power behavior
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Set zero power behavior for linear slide
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Set zero power behavior for rotation motor

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Direct control of arm and wrist
            if (gamepad1.dpad_up) {
                targetWrist += 3;
            } else if (gamepad1.dpad_down) {
                targetWrist -= 3;
            }
            if (gamepad1.dpad_left) {
                targetWrist += 1;
            } else if (gamepad1.dpad_right) {
                targetWrist -= 1;
            }

            // Claw control
            if (gamepad1.right_bumper && !lastBump) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    claw.setPosition(CLAW_OPEN_POSITION);
                } else {
                    claw.setPosition(CLAW_CLOSED_POSITION);
                }
            }
            lastBump = gamepad1.right_bumper;

            // Control intake servo with triggers
            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(10.0);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-10.0);
            } else {
                intake.setPower(0);
            }

            // Drive control (split arcade)
            double drive = -gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;
            double leftPower = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Linear slide control (Y to extend, A to retract)
            if (gamepad1.y) {
                linearSlide.setPower(1.0);  // Extend the linear slide
            } else if (gamepad1.a) {
                linearSlide.setPower(-1.0); // Retract the linear slide
            } else {
                linearSlide.setPower(0);    // Stop the linear slide
            }

            // Rotation control (B to rotate forward, X to rotate backward)
            if (gamepad1.b) {
                rotationMotor.setPower(1.0);  // Rotate forward
            } else if (gamepad1.x) {
                rotationMotor.setPower(-1.0); // Rotate backward
            } else {
                rotationMotor.setPower(0);    // Stop rotation
            }

            // Set target positions and power for arm and wrist
            arm.setTargetPosition(targetArm);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);

            wrist.setTargetPosition(targetWrist);
            wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPower(1);

            // Send telemetry data
            telemetry.addData("Linear Slide Power", linearSlide.getPower());
            telemetry.addData("Rotation Motor Power", rotationMotor.getPower());
            telemetry.addData("Arm Target", targetArm);
            telemetry.addData("Wrist Target", targetWrist);
            telemetry.update();
        }
    }
}
