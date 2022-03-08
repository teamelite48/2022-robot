// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterfeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.subsystems.ShooterFeedConfig;
import frc.robot.subsystems.ShooterFeedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.CoolDownTimer;

public class ShooterFeedUpV2 extends CommandBase {

  ShooterFeedSubsystem shooterFeedSubsystem;
  ShooterSubsystem shooterSubsystem;
  CoolDownTimer coolDownTimer = new CoolDownTimer(ShooterFeedConfig.ballCoolDownTimer);

  boolean currentBallSensorValue;
  boolean lastBallSensorValue;

  public ShooterFeedUpV2(
    ShooterFeedSubsystem shooterFeedSubsystem,
    ShooterSubsystem shooterSubsystem
  ) {
    addRequirements(shooterFeedSubsystem);

    this.shooterFeedSubsystem = shooterFeedSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {

    currentBallSensorValue = shooterFeedSubsystem.getBallSensorValue();
    lastBallSensorValue = currentBallSensorValue;
  }

  @Override
  public void execute() {

    if (shooterSubsystem.isShooterOn() == false) {
      if (shooterFeedSubsystem.getBallSensorValue() == !ShooterFeedConfig.ballSensedValue) {
        shooterFeedSubsystem.up();
      }

      return;
    }

    currentBallSensorValue = shooterFeedSubsystem.getBallSensorValue();

    if (thereWasABallAndThereIsStillABall() && coolDownTimer.isCool()) {
      shooterFeedSubsystem.up();
    }

    else if (thereWasNoBallAndNowThereIsABall()) {
      shooterFeedSubsystem.stop();
      lastBallSensorValue = currentBallSensorValue;
    }

    else if (thereWasABallAndNowThereIsNoBall()) {
      coolDownTimer.start();
      shooterFeedSubsystem.up();
      lastBallSensorValue = currentBallSensorValue;
    }

    else if (thereWasNoBallAndThereIsNoBall()){
      shooterFeedSubsystem.up();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterFeedSubsystem.stop();
  }

  @Override
  public boolean isFinished() {

    if (shooterSubsystem.isShooterOn() == false) {
      return shooterFeedSubsystem.getBallSensorValue() == ShooterFeedConfig.ballSensedValue;
    }

    return false;
  }

  private boolean thereWasABallAndNowThereIsNoBall() {
    return lastBallSensorValue == ShooterFeedConfig.ballSensedValue && currentBallSensorValue == !ShooterFeedConfig.ballSensedValue;
  }

  private boolean thereWasNoBallAndNowThereIsABall() {
    return lastBallSensorValue == !ShooterFeedConfig.ballSensedValue && currentBallSensorValue == ShooterFeedConfig.ballSensedValue;
  }

  private boolean thereWasABallAndThereIsStillABall() {
    return lastBallSensorValue == ShooterFeedConfig.ballSensedValue && currentBallSensorValue == ShooterFeedConfig.ballSensedValue;
  }

  private boolean thereWasNoBallAndThereIsNoBall() {
    return lastBallSensorValue == !ShooterFeedConfig.ballSensedValue && currentBallSensorValue == !ShooterFeedConfig.ballSensedValue;
  }
}
