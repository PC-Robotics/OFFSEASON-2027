package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.getMotorVelocityRPM;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.Locale;

@Configurable
public class FlywheelShooter implements Subsystem {
    public enum State {
        STOPPED,
        SPINNING
    }

    public enum SpinPosition {
        CLOSE,
        FAR
    }

    public enum LedColor {
        OFF(0.0),
        GREEN(0.5),
        RED(0.29),
        YELLOW(0.388),
        BLUE(0.65),
        WHITE(1.0);

        private final double value;

        LedColor(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    private static final PIDFCoefficients COEFFICIENTS = new PIDFCoefficients(
            0.0005,
            0.0,
            0.0,
            1.0
    );

    private final LinearOpMode opMode;
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    public ServoImplEx light;

    private State state = State.STOPPED;
    private boolean readyToShoot = false;
    private boolean readyCandidate = false;

    private SpinPosition spinPosition = SpinPosition.CLOSE;

    private double commandedPower = 0.0;
    private double currentRPM = 0.0;
    private LedColor commandedLedColor = LedColor.OFF;

    private int closeSpinningRPM = 3000;
    private int farSpinningRPM = 5500;

    private double settlingTolerance = 75.0;
    private double settlingDerivativeTolerance = 50.0;
    private int settlingTimeThreshold = 300; // ms

    private final ElapsedTime settlingTimer = new ElapsedTime();

    private final PIDFController controller;

    public FlywheelShooter(LinearOpMode opMode) {
        this.opMode = opMode;
        controller = new PIDFController(COEFFICIENTS);
    }

    @Override
    public void init() {
        leftMotor = opMode.hardwareMap.get(DcMotorEx.class, "flywheelleft");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor = opMode.hardwareMap.get(DcMotorEx.class, "flywheelright");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        light = opMode.hardwareMap.get(ServoImplEx.class, "light");

        state = State.STOPPED;
        spinPosition = SpinPosition.CLOSE;
        readyToShoot = false;
        readyCandidate = false;
        commandedPower = 0.0;
        currentRPM = 0.0;
        commandedLedColor = LedColor.OFF;

        controller.reset();
        controller.setTargetPosition(getTargetRPM());

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        settlingTimer.reset();
        light.setPosition(LedColor.OFF.getValue());
    }

    @Override
    public void update() {
        calculateRPM();
        updateReadyToShoot();

        switch (state) {
            case STOPPED:
                commandedPower = 0.0;
                commandedLedColor = LedColor.OFF;
                controller.reset();
                break;

            case SPINNING:
                if (controller.getTargetPosition() != getTargetRPM()) {
                    controller.setTargetPosition(getTargetRPM());
                }
                controller.updatePosition(currentRPM);
                controller.updateFeedForwardInput(0.5);

                commandedPower = clamp(controller.run(), -1.0, 1.0);
                commandedLedColor = readyToShoot ? LedColor.GREEN : LedColor.RED;
                break;
        }

        leftMotor.setPower(commandedPower);
        rightMotor.setPower(commandedPower);
        light.setPosition(commandedLedColor.getValue());
    }

    private void updateReadyToShoot() {
        boolean sus =
                state == State.SPINNING
                && Math.abs(controller.getError()) <= settlingTolerance
                && Math.abs(controller.getErrorDerivative()) <= settlingDerivativeTolerance;

        if (sus) {
            if (!readyCandidate) {
                readyCandidate = true;
                settlingTimer.reset();
            }

            readyToShoot = settlingTimer.milliseconds() >= settlingTimeThreshold;
        } else {
            readyCandidate = false;
            readyToShoot = false;
        }
    }

    private void calculateRPM() {
        currentRPM = (getMotorVelocityRPM(leftMotor) + getMotorVelocityRPM(rightMotor)) * 0.5;
    }

    private int getTargetRPM() {
        switch (spinPosition) {
            case CLOSE:
                return closeSpinningRPM;
            case FAR:
                return farSpinningRPM;
        }

        return closeSpinningRPM;
    }

    public void startSpinning() {
        state = State.SPINNING;
    }

    public void stopSpinning() {
        state = State.STOPPED;
    }

    @Override
    public void stop() {
        state = State.STOPPED;
        spinPosition = SpinPosition.CLOSE;
        readyToShoot = false;
        readyCandidate = false;
        commandedPower = 0.0;
        currentRPM = 0.0;
        commandedLedColor = LedColor.OFF;

        controller.reset();

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        settlingTimer.reset();
        light.setPosition(LedColor.OFF.getValue());
    }

    @Override
    public List<String> getSimpleTelemetry() {
        return List.of(
                "Flywheel State: " + state,
                "Spin Position: " + spinPosition,
                "Ready To Shoot: " + readyToShoot,
                "Power: " + String.format(Locale.US, "%.2f", commandedPower)
        );
    }

    @Override
    public List<String> getDetailedTelemetry() {
        return List.of(
                "Flywheel State: " + state,
                "Spin Position: " + spinPosition,
                "Commanded Power: " + String.format(Locale.US, "%.2f", commandedPower),
                "Left Motor Power: " + String.format(Locale.US, "%.2f", leftMotor.getPower()),
                "Right Motor Power: " + String.format(Locale.US, "%.2f", rightMotor.getPower()),
                "Current RPM: " + String.format(Locale.US, "%.2f", currentRPM),
                "Target RPM: " + getTargetRPM(),
                "PID Error: " + String.format(Locale.US, "%.2f", controller.getError()),
                "PID Error Derivative: " + String.format(Locale.US, "%.2f", controller.getErrorDerivative()),
                "Ready Candidate: " + readyCandidate,
                "Ready To Shoot: " + readyToShoot,
                "Settling Timer (ms): " + String.format(Locale.US, "%.1f", settlingTimer.milliseconds()),
                "Settling Tolerance: " + String.format(Locale.US, "%.2f", settlingTolerance),
                "Settling Derivative Tolerance: " + String.format(Locale.US, "%.2f", settlingDerivativeTolerance),
                "Settling Time Threshold (ms): " + String.format(Locale.US, "%.1f", settlingTimeThreshold),
                "LED Color: " + commandedLedColor
        );
    }

    public boolean isReadyToShoot() {
        return readyToShoot;
    }

    public SpinPosition getSpinPosition() {
        return spinPosition;
    }

    public void setSpinPosition(@NonNull SpinPosition spinPosition) {
        this.spinPosition = spinPosition;
        controller.setTargetPosition(getTargetRPM());
        readyToShoot = false;
        readyCandidate = false;
        settlingTimer.reset();
    }

    public State getState() {
        return state;
    }

    public void setCloseSpinningRPM(int closeSpinningRPM) {
        this.closeSpinningRPM = clamp(closeSpinningRPM, 0, 6000);
    }

    public void setFarSpinningRPM(int farSpinningRPM) {
        this.farSpinningRPM = clamp(farSpinningRPM, 0, 6000);
    }

    public void setSettlingTolerance(double settlingTolerance) {
        this.settlingTolerance = Math.max(0.0, settlingTolerance);
    }

    public void setSettlingDerivativeTolerance(double settlingDerivativeTolerance) {
        this.settlingDerivativeTolerance = Math.max(0.0, settlingDerivativeTolerance);
    }

    public void setSettlingTimeThreshold(int settlingTimeThreshold) {
        this.settlingTimeThreshold = Math.max(0, settlingTimeThreshold);
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public double getCommandedPower() {
        return commandedPower;
    }
}