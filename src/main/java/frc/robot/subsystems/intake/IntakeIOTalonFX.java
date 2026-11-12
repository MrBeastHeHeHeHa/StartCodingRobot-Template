// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
    protected final TalonFX intakeMotor = new TalonFX(IntakeConstants.MOTOR_ID);

    private final MotionMagicVelocityVoltage voltRequestMM;
    private final TalonFXConfiguration motorConfig;
    private final VoltageOut voltageRequest;

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;

    public IntakeIOTalonFX() {
        motorConfig = new TalonFXConfiguration();

        velocity = intakeMotor.getVelocity(false);
        appliedVolts = intakeMotor.getMotorVoltage(false);
        current = intakeMotor.getStatorCurrent(false);

        setGains(IntakeConstants.GAINS);

        voltRequestMM = new MotionMagicVelocityVoltage(0);
        voltageRequest = new VoltageOut(0.0);
        }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        intakeMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setVelocity(double velocity) {
            intakeMotor.setControl(
                voltRequestMM.withVelocity(
                    Units.rotationsToRadians(velocity) * IntakeConstants.WHEEL_RADIUS));
    }

    @Override
    public void setGains(Gains gains) {
        final Slot0Configs config = new Slot0Configs();
        config.kV = gains.kV();
        config.kA = gains.kA();
        config.kP = gains.kP();
        config.kI = gains.kI();
        config.kD = gains.kD();
        motorConfig.MotorOutput =
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(IntakeConstants.MOTOR_DIRECTION);
        motorConfig.MotionMagic =
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(
                    Units.radiansToRotations(
                        gains.accelerationmps() / IntakeConstants.WHEEL_RADIUS));
        motorConfig.Feedback =
            new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.GEAR_RATIO);
        motorConfig.Slot0 = config;
        intakeMotor.getConfigurator().apply(motorConfig);
    }
}
