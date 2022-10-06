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
import frc.robot.config.subsystems.IntakeConfig;

public class IntakeSubsystem extends SubsystemBase {

  final PWMSparkMax motor = new PWMSparkMax(PwmPort.IntakeMotor);
  final Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticChannel.IntakeDeploy);

  public IntakeSubsystem() {
    intakeSolenoid.set(IntakeConfig.retractValue);
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

    motor.set(IntakeConfig.intakeSpeed);
  }

  public void outtake() {
    if (isIntakeDeployed() == true){
      motor.set(IntakeConfig.outtakeSpeed);
    }
    else {
      stop();
    }
  }

  public void stop() {
      motor.set(0);
  }

  public void deploy(){
    intakeSolenoid.set(IntakeConfig.deployValue);
  }

  public void retract(){
    stop();
    intakeSolenoid.set(IntakeConfig.retractValue);
  }

  public boolean isIntakeDeployed() {
    return intakeSolenoid.get() == IntakeConfig.deployValue;
  }
}
