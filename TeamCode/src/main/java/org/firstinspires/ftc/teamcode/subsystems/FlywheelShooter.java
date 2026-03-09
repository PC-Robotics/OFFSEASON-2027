package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.getMotorVelocityRPM;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.Locale;

@Configurable
public class FlywheelShooter implements Subsystem {
    public enum State {
        STOPPED,
        SPINNING
    }

    private final LinearOpMode opMode;
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;

    private State state = State.STOPPED;
    private boolean readyToShoot = false;
    private boolean readyCandidate = false;

    private double commandedPower = 0.0;
    private int currentRPM = 0;

    //
    private int closeSpinningRPM = 3000;
    private int farSpinningRPM = 5500;

    private int settlingTolerance = 75;
    private int settlingDerivativeTolerance = 50;
    private int settlingTimeThreshold = 300; // ms

    private final ElapsedTime settlingTimer = new ElapsedTime();

    private final PIDFController controller;

    private static final PIDFCoefficients coefficients = new PIDFCoefficients(
            0.0005,
            0,
            0,
            1.0
    );


    public FlywheelShooter(LinearOpMode opMode) {
        this.opMode = opMode;
        controller = new PIDFController(coefficients);
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

        state = State.STOPPED;
        readyToShoot = false;
        readyCandidate = false;
        commandedPower = 0.0;
        settlingTimer.reset();
    }

    @Override
    public void update() {
        calculateRPM();

        switch (state) {
            case STOPPED:
                commandedPower = 0.0;
                break;
                // TODO - close and far
            case SPINNING:
                updateReadyToShoot();
                commandedPower = clamp(controller.run(), -1.0, 1.0);
                break;
        }

        leftMotor.setPower(commandedPower);
        rightMotor.setPower(commandedPower);
    }

    private void updateReadyToShoot() {
        boolean sus =
                Math.abs(controller.getError()) <= settlingTolerance
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
        currentRPM = (int) (
                (getMotorVelocityRPM(leftMotor) +
                 getMotorVelocityRPM(rightMotor)
                )
                * 0.5
        );
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
        readyToShoot = false;
        readyCandidate = false;
        commandedPower = 0.0;
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        settlingTimer.reset();
    }

    @Override
    public List<String> getSimpleTelemetry() {
        return List.of(
                "Flywheel State: " + state,
                "Ready To Shoot: " + readyToShoot,
                "Power: " + String.format(Locale.US, "%.2f", commandedPower)
        );
    }

    // TODO - actually finish im just too lazy
    @Override
    public List<String> getDetailedTelemetry() {
        return List.of(
                "Flywheel State: " + state,
                "Commanded Power: " + String.format(Locale.US, "%.2f", commandedPower),
                "Left Motor Power: " + String.format(Locale.US, "%.2f", leftMotor.getPower()),
                "Right Motor Power: " + String.format(Locale.US, "%.2f", rightMotor.getPower()),
                "Current RPM: " + currentRPM,
                "Ready To Shoot: " + readyToShoot
        );
    }

    public boolean isReadyToShoot() {
        return readyToShoot;
    }

    public void setCloseSpinningRPM(int closeSpinningRPM) {
        this.closeSpinningRPM = clamp(closeSpinningRPM, 0, 6000);
    }

    public void setFarSpinningRPM(int farSpinningRPM) {
        this.farSpinningRPM = clamp(farSpinningRPM, 0, 6000);
    }

    public void setSettlingTolerance(int settlingTolerance) {
        this.settlingTolerance = settlingTolerance;
    }

    public void setSettlingDerivativeTolerance(int settlingDerivativeTolerance) {
        this.settlingDerivativeTolerance = settlingDerivativeTolerance;
    }

    public void setSettlingTimeThreshold(int settlingTimeThreshold) {
        this.settlingTimeThreshold = settlingTimeThreshold;
    }
}