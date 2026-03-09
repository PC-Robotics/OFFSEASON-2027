package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Utility.clamp;
import static org.firstinspires.ftc.teamcode.Utility.getMotorVelocityRPM;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
import java.util.Locale;

@Configurable
public class Intake implements Subsystem {
    // state machine :)
    public enum State {
        STOPPED, HOLDING, INTAKING, OUTTAKING, JAM_CLEARING
    }

    // TODO - transition away from linearopmode
    private final LinearOpMode opMode;
    public DcMotorEx motor;

    // using desiredState because
    // sometimes driver switches modes during jam clear
    private State desiredState = State.STOPPED;
    private State state = State.STOPPED;

    private double commandedPower = 0;

    // powers are inverted in code
    private double holdingPower = 0.05;
    private double intakingPower = 1.0;
    private double outtakingPower = 0.3;
    private double jamClearingPower = 0.7;

    // jamming
    private double jamCurrentThreshold = 5; // amps
    private double jamVelocityThreshold = 50; // rpm
    private double jamTimeThreshold = 250; // ms
    private double jamMinPower = 0.4; // minimum motor power

    // jam clearing
    private double jamClearDuration = 250; // ms

    private boolean jamCandidate = false;
    private boolean jammed = false;

    private final ElapsedTime jamTimer = new ElapsedTime();
    private final ElapsedTime jamClearTimer = new ElapsedTime();

    // item detection
    private DistanceSensor distanceSensor;
    private boolean hasItem = false;
    private boolean itemCandidate = false;
    private double itemDistanceThreshold = 5.0; // cm
    private int itemDetectionLoops = 3;

    private int itemDetectedLoops = 0;

    private final ElapsedTime itemDetectionTimer = new ElapsedTime();


    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init() {
        motor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "intakeSensor");


        // idiot (me) proof
        desiredState = State.STOPPED;
        state = State.STOPPED;
        commandedPower = 0.0;
        jamCandidate = false;
        jammed = false;
        jamTimer.reset();
        jamClearTimer.reset();

        hasItem = false;
        itemCandidate = false;
        itemDetectionTimer.reset();
    }


    @Override
    public void update() {
        detectJam();
        detectItem();

        // dunno if this is the right way to do it
        if (state == State.JAM_CLEARING) {
            if (jamClearTimer.milliseconds() >= jamClearDuration) {
                state = desiredState;
            }
        } else {
            state = desiredState;

            if (desiredState == State.INTAKING && jammed) {
                state = State.JAM_CLEARING;
                jamClearTimer.reset();
                jamCandidate = false;
                jammed = false;
            }
        }

        switch (state) {
            case STOPPED:
                commandedPower = 0.0;
                break;
            case HOLDING:
                commandedPower = -holdingPower;
                break;
            case INTAKING:
                commandedPower = intakingPower;
                break;
            case OUTTAKING:
                commandedPower = -outtakingPower;
                break;
            case JAM_CLEARING:
                commandedPower = -jamClearingPower;
                break;
        }

        motor.setPower(commandedPower);
    }

    private void detectJam() {
        boolean sus =
                desiredState == State.INTAKING
                && Math.abs(commandedPower) >= jamMinPower
                && motor.getCurrent(CurrentUnit.AMPS) >= jamCurrentThreshold
                && getMotorVelocityRPM(motor) <= jamVelocityThreshold;

        if (sus) {
            if (!jamCandidate) { // rising edge to avoid resetting timer over and over again
                jamCandidate = true;
                jamTimer.reset();
            }

            jammed = jamTimer.milliseconds() >= jamTimeThreshold;
        } else {
            jamCandidate = false;
            jammed = false;
        }
    }

    private void detectItem() {
        boolean sus = distanceSensor.getDistance(DistanceUnit.CM) <= itemDistanceThreshold;

        if (sus) {
            if (!itemCandidate) {
                itemCandidate = true;
                itemDetectedLoops = 0;
            }

            hasItem = ++itemDetectedLoops >= itemDetectionLoops;
        } else {
            itemCandidate = false;
            hasItem = false;
        }

    }

    public void startIntake() {
        desiredState = State.INTAKING;
    }

    public void stopIntake() {
        desiredState = State.STOPPED;
    }

    public void startOuttake() {
        desiredState = State.OUTTAKING;
    }

    public void startHolding() {
        desiredState = State.HOLDING;
    }

    public void startJamClearing() {
        desiredState = State.JAM_CLEARING;
        state = State.JAM_CLEARING;
        jamClearTimer.reset();
    }

    public void setHoldingPower(double power) {
        this.holdingPower = Math.abs(clamp(power, -1.0, 1.0));
    }

    public void setIntakingPower(double power) {
        this.intakingPower = Math.abs(clamp(power, -1.0, 1.0));
    }

    public void setOuttakingPower(double power) {
        this.outtakingPower = Math.abs(clamp(power, -1.0, 1.0));
    }

    public void setJamClearingPower(double power) {
        this.jamClearingPower = Math.abs(clamp(power, -1.0, 1.0));
    }

    public void setJamClearDuration(double jamClearDuration) {
        this.jamClearDuration = Math.max(0.0, jamClearDuration);
    }


    @Override
    public void stop() {
        desiredState = State.STOPPED;
        state = State.STOPPED;
        commandedPower = 0.0;
        motor.setPower(0.0);
        jamCandidate = false;
        jammed = false;
        jamTimer.reset();
        jamClearTimer.reset();
    }


    @Override
    public List<String> getSimpleTelemetry() {
        return List.of(
                "Intake State: " + state,
                "Desired State: " + desiredState,
                "Has Item: " + hasItem,
                "Jammed: " + jammed,
                "Power: " + String.format(Locale.US, "%.2f", commandedPower)
        );
    }

    @Override
    public List<String> getDetailedTelemetry() {
        return List.of(
                "Intake State: " + state,
                "Desired State: " + desiredState,
                "Commanded Power: " + String.format(Locale.US, "%.2f", commandedPower),
                "Motor Power: " + String.format(Locale.US, "%.2f", motor.getPower()),
                "Motor Current (A): " + String.format(Locale.US, "%.2f", motor.getCurrent(CurrentUnit.AMPS)),
                "Motor Velocity (RPM): " + String.format(Locale.US, "%.2f", getMotorVelocityRPM(motor)),
                "Has Item: " + hasItem,
                "Item Candidate: " + itemCandidate,
                "Item Distance (cm): " + String.format(Locale.US, "%.2f", distanceSensor.getDistance(DistanceUnit.CM)),
                "Item Detect Distance (cm): " + String.format(Locale.US, "%.2f", itemDistanceThreshold),
                "Loops Detected: " + itemDetectedLoops,
                "Jam Candidate: " + jamCandidate,
                "Jammed: " + jammed,
                "Jam Timer (ms): " + String.format(Locale.US, "%.1f", jamTimer.milliseconds()),
                "Jam Clear Timer (ms): " + String.format(Locale.US, "%.1f", jamClearTimer.milliseconds()),
                "Jam Current Threshold: " + String.format(Locale.US, "%.2f", jamCurrentThreshold),
                "Jam Velocity Threshold (RPM): " + String.format(Locale.US, "%.2f", jamVelocityThreshold),
                "Jam Time Threshold (ms): " + String.format(Locale.US, "%.1f", jamTimeThreshold)
        );
    }


    public State getState() {
        return state;
    }

    public State getDesiredState() {
        return desiredState;
    }

    public double getMotorPower() {
        return motor.getPower();
    }

    public double getCommandedPower() {
        return commandedPower;
    }

    public boolean isJammed() {
        return jammed;
    }

    public boolean hasItem() {
        return hasItem;
    }

    public double getItemDistanceCm() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
}
