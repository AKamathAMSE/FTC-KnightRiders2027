package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "KnightShiftTeleop")
public class KnightShiftTeleop extends LinearOpMode {
    private DcMotorEx LB, LF, RB, RF;
    private DcMotorEx Shooter, Intake;

    double lbPower, lfPower, rbPower, rfPower;

    public static double SHOOTER_RPM = 2000.0;
    public static double INTAKE_RPM = 800.0;
    public static double SHOOTER_TICKS_PER_REV = 28.0;
    public static double INTAKE_TICKS_PER_REV = 28.0;

    public static double shooterP = 0.001;
    public static double shooterI = 0.0;
    public static double shooterD = 0.0;
    public static double shooterF = 0.0;

    public static double intakeP = 0.001;
    public static double intakeI = 0.0;
    public static double intakeD = 0.0;
    public static double intakeF = 0.0;

    private double targetShooterVelocity = 0.0;
    private double targetIntakeVelocity = 0.0;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        LB = hardwareMap.get(DcMotorEx.class, "LB");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);

        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        BasicPIDController shooterPID = new BasicPIDController(shooterP, shooterI, shooterD);
        BasicPIDController intakePID = new BasicPIDController(intakeP, intakeI, intakeD);

        while (opModeIsActive()) {
            shooterPID.setPID(shooterP, shooterI, shooterD);
            intakePID.setPID(intakeP, intakeI, intakeD);

            intakeMethod(intakePID);
            shooterMethod(shooterPID);

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            lbPower = forward - strafe + turn;
            lfPower = forward + strafe + turn;
            rbPower = forward + strafe - turn;
            rfPower = forward - strafe - turn;

            double max = Math.max(
                    Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower))
            );

            if (max > 1.0) {
                lbPower /= max;
                lfPower /= max;
                rbPower /= max;
                rfPower /= max;
            }

            LB.setPower(lbPower);
            LF.setPower(lfPower);
            RB.setPower(rbPower);
            RF.setPower(rfPower);

            telemetry.addData("Run Time", runtime);
            telemetry.addData("Shooter Target RPM", toRpm(targetShooterVelocity, SHOOTER_TICKS_PER_REV));
            telemetry.addData("Shooter RPM", toRpm(Shooter.getVelocity(), SHOOTER_TICKS_PER_REV));
            telemetry.addData("Intake Target RPM", toRpm(targetIntakeVelocity, INTAKE_TICKS_PER_REV));
            telemetry.addData("Intake RPM", toRpm(Intake.getVelocity(), INTAKE_TICKS_PER_REV));
            telemetry.update();
        }
    }

    private void shooterMethod(BasicPIDController pid) {
        if (gamepad1.b) {
            targetShooterVelocity = toTicksPerSecond(SHOOTER_RPM, SHOOTER_TICKS_PER_REV);
        } else {
            targetShooterVelocity = 0.0;
        }

        if (targetShooterVelocity == 0.0) {
            pid.reset();
            Shooter.setPower(0.0);
            return;
        }

        double power = pid.update(targetShooterVelocity, Shooter.getVelocity()) + (targetShooterVelocity * shooterF);
        Shooter.setPower(Range.clip(power, -1.0, 1.0));
    }

    private void intakeMethod(BasicPIDController pid) {
        double intakeVelocity = toTicksPerSecond(INTAKE_RPM, INTAKE_TICKS_PER_REV);
        if (gamepad1.right_trigger > 0.1) {
            targetIntakeVelocity = intakeVelocity;
        } else if (gamepad1.left_trigger > 0.1) {
            targetIntakeVelocity = -intakeVelocity;
        } else {
            targetIntakeVelocity = 0.0;
        }

        if (targetIntakeVelocity == 0.0) {
            pid.reset();
            Intake.setPower(0.0);
            return;
        }

        double power = pid.update(targetIntakeVelocity, Intake.getVelocity()) + (targetIntakeVelocity * intakeF);
        Intake.setPower(Range.clip(power, -1.0, 1.0));
    }

    private static double toTicksPerSecond(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }

    private static double toRpm(double ticksPerSecond, double ticksPerRev) {
        return ticksPerSecond * 60.0 / ticksPerRev;
    }

    public static class BasicPIDController {
        private double Kp, Ki, Kd;
        private double lastError = 0;
        private double integral = 0;
        private ElapsedTime timer = new ElapsedTime();

        public BasicPIDController(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public void setPID(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public void reset() {
            integral = 0;
            lastError = 0;
            timer.reset();
        }

        public double update(double reference, double state) {
            double error = reference - state;
            double dt = timer.seconds();
            timer.reset();

            integral += error * dt;
            double derivative = (dt != 0) ? (error - lastError) / dt : 0.0;
            lastError = error;

            return (Kp * error) + (Ki * integral) + (Kd * derivative);
        }
    }
}
