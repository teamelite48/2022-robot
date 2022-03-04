// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.roborio.CanBusId;
import frc.robot.config.roborio.PneumaticChannel;
import frc.robot.config.subsystems.ClimberConfig;

public class ClimberSubsystem extends SubsystemBase {

  private final WPI_TalonFX leftArmMotor = new WPI_TalonFX(CanBusId.LeftClimberMotor);
  private final WPI_TalonFX rightArmMotor = new WPI_TalonFX(CanBusId.RightClimberMotor);

  private TalonFXSimCollection leftArmSim;
  private TalonFXSimCollection rightArmSim;

  private final DoubleSolenoid leftArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.LeftArmForward, PneumaticChannel.LeftArmReverse);
  private final DoubleSolenoid rightArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.RightArmForward, PneumaticChannel.RightArmReverse);

  private final DoubleSolenoid leftLockSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.LeftClimbLockForward, PneumaticChannel.LeftClimbLockReverse);
  private final DoubleSolenoid rightLockSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannel.RightClimbLockForward, PneumaticChannel.RightClimbLockReverse);

  private boolean isClimberEnabled = false;

  public ClimberSubsystem() {
    leftArmSolenoid.set(ClimberConfig.initialArmPosition);
    rightArmSolenoid.set(ClimberConfig.initialArmPosition);

    leftArmMotor.configFactoryDefault();
    rightArmMotor.configFactoryDefault();

    leftArmMotor.configForwardSoftLimitThreshold(ClimberConfig.armExtensionLimit, 30);
    leftArmMotor.configReverseSoftLimitThreshold(ClimberConfig.armRetractionLimit, 30);
    leftArmMotor.configForwardSoftLimitEnable (true, 30);
    leftArmMotor.configReverseSoftLimitEnable(true, 30);

    rightArmMotor.configForwardSoftLimitThreshold(ClimberConfig.armExtensionLimit, 30);
    rightArmMotor.configReverseSoftLimitThreshold(ClimberConfig.armRetractionLimit, 30);
    rightArmMotor.configForwardSoftLimitEnable (true, 30);
    rightArmMotor.configReverseSoftLimitEnable(true, 30);

    // TODO: We need to invert one of the motors, but I'm not sure which one yet.
    // leftArmMotor.setInverted(TalonFXInvertType.Clockwise);
    // or rightArmMotor.setInverted(TalonFXInvertType.Clockwise);

    if (RobotBase.isSimulation() == true) {
      leftArmSim = leftArmMotor.getSimCollection();
      rightArmSim = rightArmMotor.getSimCollection();
    }
  }

  @Override
  public void periodic() {
    
    if (leftArmMotor.isFwdLimitSwitchClosed() == 1 || leftArmMotor.isRevLimitSwitchClosed() == 1) {
      leftLockSolenoid.set(ClimberConfig.lockValue);
    }
    else if (rightArmMotor.isFwdLimitSwitchClosed() == 1 || rightArmMotor.isRevLimitSwitchClosed() == 1) {
      rightLockSolenoid.set(ClimberConfig.lockValue);
    }

    SmartDashboard.putBoolean("Climber Enabled", isClimberEnabled);
    SmartDashboard.putString("Left Arm Position", leftArmSolenoid.get() == ClimberConfig.upTilt ? "Up" : "Down");
    SmartDashboard.putString("Right Arm Position", rightArmSolenoid.get() == ClimberConfig.upTilt ? "Up" : "Down");
    SmartDashboard.putNumber("Left Arm Length", leftArmMotor.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putNumber("Right Arm Length",rightArmMotor.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putString("Left Lock", leftLockSolenoid.get() == ClimberConfig.lockValue ? "Lock" : "Unlock");
    SmartDashboard.putString("Right Lock", rightLockSolenoid.get() == ClimberConfig.lockValue ? "Lock" : "Unlock");
  }

  public void simulationPeriodic() {

    int ticksPerPeriodic = 1;

    double currentLeftArmPosition = leftArmMotor.getSensorCollection().getIntegratedSensorPosition();
    int newLeftArmPosition = (int) (currentLeftArmPosition + leftArmSim.getMotorOutputLeadVoltage() * ticksPerPeriodic);

    double currentRightArmPosition = rightArmMotor.getSensorCollection().getIntegratedSensorPosition();
    int newRightArmPosition = (int) (currentRightArmPosition + rightArmSim.getMotorOutputLeadVoltage() * ticksPerPeriodic);

    leftArmSim.setIntegratedSensorRawPosition(newLeftArmPosition);
    rightArmSim.setIntegratedSensorRawPosition(newRightArmPosition);

  }

  public void toggleClimberEnabled() {
    isClimberEnabled = !isClimberEnabled;
  }

  public void tiltArmsDown() {
    if (isClimberEnabled){
      leftArmSolenoid.set(ClimberConfig.downTilt);
      rightArmSolenoid.set(ClimberConfig.downTilt);
    }
  }

  public void tiltArmsUp() {
    if (isClimberEnabled){
      leftArmSolenoid.set(ClimberConfig.upTilt);
      rightArmSolenoid.set(ClimberConfig.upTilt);
    }
  }

  public void extendArms() {
    if (isClimberEnabled) {
      leftLockSolenoid.set(ClimberConfig.unlockValue);
      rightLockSolenoid.set(ClimberConfig.unlockValue);
      leftArmMotor.set(ClimberConfig.extendArmSpeed);
      rightArmMotor.set(ClimberConfig.extendArmSpeed);
    }
  }

  public void retractArms() {
    if (isClimberEnabled) {
      leftLockSolenoid.set(ClimberConfig.unlockValue);
      rightLockSolenoid.set(ClimberConfig.unlockValue);
      leftArmMotor.set(ClimberConfig.retractArmSpeed);
      rightArmMotor.set(ClimberConfig.retractArmSpeed);
    }
  }

  public void stopArms() {
    leftLockSolenoid.set(ClimberConfig.lockValue);
    rightLockSolenoid.set(ClimberConfig.lockValue);
    leftArmMotor.set(0);
    rightArmMotor.set(0);
  }
}
