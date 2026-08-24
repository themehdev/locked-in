package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;

public class Arm {
    public final SparkMax arm = new SparkMax(5, MotorType.kBrushed);

    public final Servo wrist = new Servo(0);

    public Arm() {
        SparkMaxConfig armconfig = new SparkMaxConfig();
        armconfig.encoder.countsPerRevolution(Constants.ENCODER_COUNTS_PER_REVOLUTION)
            .inverted(Constants.ENCODER_INVERTED);

        armconfig.closedLoop.
        p(Constants.ARM_kP)
        .i(Constants.ARM_kI)
        .d(Constants.ARM_kD);

        armconfig.closedLoop.feedForward.
        kS(Constants.ARM_kS)
        .kV(Constants.ARM_kV)
        .kA(Constants.ARM_kA);

        armconfig.closedLoop.maxMotion.
        cruiseVelocity(Constants.ARM_cruiseVelocity)
        .maxAcceleration(Constants.ARM_acceleration)
        .allowedProfileError(Constants.ARM_allowedProfileError);

        armconfig.smartCurrentLimit(9);

        armconfig.inverted(true);

        armconfig.idleMode(IdleMode.kBrake);

        arm.configure(armconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private double getArmPosition() {
        return arm.getEncoder().getPosition() * 360;
    }


    private void setArmPosition(double position) {
        double multiplier = getArmPosition() < position ? 1.0 : -1.0;
        if(Math.abs(getArmPosition() - position) > Constants.ARM_HOLDING_THRESHOLD) {
            arm.setVoltage(Constants.ARM_MOVING_VOLTAGE * multiplier);
        } else if (Math.abs(getArmPosition() - position) > Constants.ARM_DONE_THRESHOLD) {
            arm.setVoltage(Constants.ARM_HOLDING_VOLTAGE * multiplier);
        } else {
            arm.setVoltage(0.0);
        }
    }

    public void setPosition(Constants.POSITIONS position) {
        wrist.set(position.wristPos * 1.0/300.0);
        setArmPosition(position.armPos);
    }
}
