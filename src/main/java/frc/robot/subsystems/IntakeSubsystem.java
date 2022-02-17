// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.roborio.PneumaticChannel;
import frc.robot.config.roborio.PwmPort;

public class IntakeSubsystem extends SubsystemBase {

  private final PWMSparkMax motor = new PWMSparkMax(PwmPort.IntakeMotor);
  private final Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticChannel.IntakeForward);

  private final double motorSpeed = 0.5;

  public IntakeSubsystem() {
    intakeSolenoid.set(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Deployed", isIntakeDeployed());
    SmartDashboard.putNumber("Intake Speed", motor.get());
  }

  public void intake() {
    if (isIntakeDeployed() == false) {
      deploy();
    }

    motor.set(motorSpeed);
  }

  public void outtake() {
    if (isIntakeDeployed() == true){
      motor.set(motorSpeed * -1);
    }
    else {
      stop();
    }
  }

  public void stop() {
      motor.set(0);
  }

  public void deploy(){
    intakeSolenoid.set(true);
  }

  public void retract(){
    stop();
    intakeSolenoid.set(false);
  }

  public boolean isIntakeDeployed() {
    return intakeSolenoid.get();
  }
}
