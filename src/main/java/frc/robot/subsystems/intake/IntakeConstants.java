// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class IntakeConstants {
    
    public static final int MOTOR_ID = 20;

    public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

    public static final MotorType MOTOR = MotorType.KrakenX44;

    public static final double ROLLER_MOI = 0.5 * 0.3 * Math.pow(Units.inchesToMeters(1.0), 2);

    public static final double GEAR_RATIO = 1.0;

    public static final double WHEEL_RADIUS = Units.inchesToMeters(2);

    // Feed Forwards
    public static final double kV = 0;
    public static final double kA = 0;

    // PID
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double accelerationmps = 2;

    public static final double targetVelocitymps = 1;

    public static final IntakeIO.Gains GAINS =
        new IntakeIO.Gains(kV, kA, kP, kI, kD, targetVelocitymps, accelerationmps);

    public static final double INTAKE_SPEED_Mps = 20; //in meters per second if it wasn't clear enough
}
