/*
 * Copyright (C) 2023, Team 3602. All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;

import edu.wpi.first.math.util.Units;

import frc.team3602.robot.subsystems.DrivetrainSubsytem;

public final class Constants {
  public final class OperatorInterfaceConstants {
    public final static int XBOX_CONTROLLER_PORT = 0;
  }

  public final class DrivetrainConstants {
    public static final int PIGEON_ID = 52;
    public static final String CAN_BUS_NAME = "";

    public static final double MAX_SPEED = 6.0; // 6 meters per second desired top speed.
    public static final double MAX_ANGULAR_RATE = Math.PI; // Half a roation per second max angular velocity.

    private static final double DRIVE_GEAR_RATIO = 6.122448979591837;
    private static final double TURN_GEAR_RATIO = 21.428571428571427;
    private static final double WHEEL_RADIUS_INCHES = 2.0;

    /*
     * The stator current at which the wheels start to slip.
     * This needs to be tuned to your individual robot.
     */
    private static final double SLIP_CURRENT_AMPS = 300.0;

    /*
     * Both sets of gains need to be tuned to your individual robot.
     * When using closed-loop control, the drive motor uses:
     * - VelocityVoltage, if DRIVETRAIN_CONSTANTS.SupportsPro is false (default)
     * - VelocityTorqueCurrentFOC, if DRIVETRAIN_CONSTANTS.SupportsPro is true
     */
    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
        .withKP(3.0).withKI(0.0).withKD(0.0)
        .withKS(0.0).withKV(1.5).withKA(0.0);

    // The turning motor uses MotionMagicVoltage control.
    private static final Slot0Configs TURN_GAINS = new Slot0Configs()
        .withKP(100.0).withKI(0.0).withKD(0.05)
        .withKS(0.0).withKV(1.5).withKA(0.0);

    /*
     * Theoretical free speed (m/s) at 12v applied output.
     * This needs to be tuned to your individual robot.
     */
    private static final double SPEED_AT_12_VOLTS_MPS = 6.0;

    // These are only used for simulation.
    private static final double DRIVE_INTERTIA = 0.001;
    private static final double TURN_INTERIA = 0.00001;

    /*
     * Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns.
     * This may need to be tuned to your individual robot.
     */
    private static final double COUPLE_RATIO = 3.5714285714285716;

    private static final boolean TURN_MOTOR_REVERSED = true;
    private static final boolean INVERT_LEFT_SIDE = false;
    private static final boolean INVERT_RIGHT_SIDE = true;

    private static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
        .withPigeon2Id(PIGEON_ID)
        .withCANbusName(CAN_BUS_NAME)
        .withSupportsPro(false);

    private static final SwerveModuleConstantsFactory MODULE_CONSTANTS_FACTORY = new SwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
        .withSteerMotorGearRatio(TURN_GEAR_RATIO)
        .withWheelRadius(WHEEL_RADIUS_INCHES)
        .withSlipCurrent(SLIP_CURRENT_AMPS)
        .withDriveMotorGains(DRIVE_GAINS)
        .withSteerMotorGains(TURN_GAINS)
        .withSpeedAt12VoltsMps(SPEED_AT_12_VOLTS_MPS)
        .withDriveInertia(DRIVE_INTERTIA)
        .withSteerInertia(TURN_INTERIA)
        .withFeedbackSource(SwerveModuleSteerFeedbackType.FusedCANcoder)
        .withCouplingGearRatio(COUPLE_RATIO)
        .withSteerMotorInverted(TURN_MOTOR_REVERSED);

    // Front left (Module 0)
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 46;
    private static final int FRONT_LEFT_TURN_MOTOR_ID = 49;
    private static final int FRONT_LEFT_ENCODER_ID = 41;

    private static final double FRONT_LEFT_ENCODER_OFFSET = 0.112548828125;

    private static final double FRONT_LEFT_X_POS_INCHES = 12.875;
    private static final double FRONT_LEFT_Y_POS_INCHES = 12.875;

    private static final SwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS = MODULE_CONSTANTS_FACTORY
        .createModuleConstants(
            FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_TURN_MOTOR_ID, FRONT_LEFT_ENCODER_ID,
            FRONT_LEFT_ENCODER_OFFSET,
            Units.inchesToMeters(FRONT_LEFT_X_POS_INCHES), Units.inchesToMeters(FRONT_LEFT_Y_POS_INCHES),
            INVERT_LEFT_SIDE);

    // Front right (Module 1)
    private static final int FRONT_RIGHT_DRVE_MOTOR_ID = 44;
    private static final int FRONT_RIGHT_TURN_MOTOR_ID = 45;
    private static final int FRONT_RIGHT_ENCODER_ID = 42;

    private static final double FRONT_RIGHT_ENCODER_OFFSET = -0.4921875;

    private static final double FRONT_RIGHT_X_POS_INCHES = 12.875;
    private static final double FRONT_RIGHT_Y_POS_INCHES = -12.875;

    private static final SwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS = MODULE_CONSTANTS_FACTORY
        .createModuleConstants(FRONT_RIGHT_DRVE_MOTOR_ID, FRONT_RIGHT_TURN_MOTOR_ID, FRONT_RIGHT_ENCODER_ID,
            FRONT_RIGHT_ENCODER_OFFSET,
            Units.inchesToMeters(FRONT_RIGHT_X_POS_INCHES), Units.inchesToMeters(FRONT_RIGHT_Y_POS_INCHES),
            INVERT_RIGHT_SIDE);

    // Back left (Module 2)
    private static final int BACK_LEFT_DRVE_MOTOR_ID = 51;
    private static final int BACK_LEFT_TURN_MOTOR_ID = 50;
    private static final int BACK_LEFT_ENCODER_ID = 40;

    private static final double BACK_LEFT_ENCODER_OFFSET = 0.429931640625;

    private static final double BACK_LEFT_X_POS_INCHES = -12.875;
    private static final double BACK_LEFT_Y_POS_INCHES = 12.875;

    private static final SwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS = MODULE_CONSTANTS_FACTORY
        .createModuleConstants(BACK_LEFT_DRVE_MOTOR_ID, BACK_LEFT_TURN_MOTOR_ID, BACK_LEFT_ENCODER_ID,
            BACK_LEFT_ENCODER_OFFSET,
            Units.inchesToMeters(BACK_LEFT_X_POS_INCHES), Units.inchesToMeters(BACK_LEFT_Y_POS_INCHES),
            INVERT_LEFT_SIDE);

    // Back right (Module 3)
    private static final int BACK_RIGHT_DRVE_MOTOR_ID = 48;
    private static final int BACK_RIGHT_TURN_MOTOR_ID = 47;
    private static final int BACK_RIGHT_ENCODER_ID = 43;

    private static final double BACK_RIGHT_ENCODER_OFFSET = 0.26904296875;

    private static final double BACK_RIGHT_X_POS_INCHES = -12.875;
    private static final double BACK_RIGHT_Y_POS_INCHES = -12.875;

    private static final SwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS = MODULE_CONSTANTS_FACTORY
        .createModuleConstants(BACK_RIGHT_DRVE_MOTOR_ID, BACK_RIGHT_TURN_MOTOR_ID, BACK_RIGHT_ENCODER_ID,
            BACK_RIGHT_ENCODER_OFFSET, Units.inchesToMeters(BACK_RIGHT_X_POS_INCHES),
            Units.inchesToMeters(BACK_RIGHT_Y_POS_INCHES), INVERT_RIGHT_SIDE);

    public static final DrivetrainSubsytem DRIVETRAIN_SUBSYTEM = new DrivetrainSubsytem(DRIVETRAIN_CONSTANTS,
        FRONT_LEFT_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_CONSTANTS, BACK_LEFT_MODULE_CONSTANTS,
        BACK_RIGHT_MODULE_CONSTANTS);
  }

  public static final class AutonomousConstants {
    public static final double KP_X_CONTROLLER = 1.0;
    public static final double KP_Y_CONTROLLER = 1.0;
    public static final double KP_THETA_CONTROLLER = 1.0;
  }
}