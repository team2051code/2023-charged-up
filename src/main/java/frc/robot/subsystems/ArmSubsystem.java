package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.LimitedMotor;
import frc.robot.subsystems.simulated.AnalogPotentiometerSimulation;
import frc.robot.subsystems.simulated.ArmSimulation;
import frc.robot.subsystems.simulated.CANSparkMaxSimulated;

public class ArmSubsystem extends SubsystemBase {
    public final static double kArmP = 0.4 / 5;public final static double kArmI = 0;public final static double kArmD = 0;
    public final static double kextenderP = 1.4 / 1.3;public final static double kextenderI = 0;public final static double kextenderD = 0;
    public final static double kgripperP = 0.5;public final static double kgripperI = 0;public final static double kgripperD = 0;
    public final static double kgripperRotatorP = 0.1;public final static double kgripperRotatorI = 0;public final static double kgripperRotatorD = 0;
    public final static double kintakeLeftP = 0.1;public final static double kintakeLeftI = 0; public final static double kintakeLeftD = 0;  
    public final static double kintakeRightP = 0.1;public final static double kintakeRightI = 0; public final static double kintakeRightD = 0;  
    public final static double ksolidArmDistance = 28;
    public final static double MAX_ARM_EXTENSION_LENGTH_INCHES = 40;
    // reverse one of the sides of the intake and the arm before we make them a
    // group 
    private final PIDController m_gripperRotatorPIDController;
    private final PIDController m_armPIDController;
    private final PIDController m_extenderPIDController;
    private final PIDController m_gripperPivotPIDController;
    private final PIDController m_intakeLeftPIDController;
    private final PIDController m_intakeRightPIDController;
    private Solenoid m_gripperSolenoid;
    private Solenoid m_breakSolenoid;
    private RelativeEncoder m_armPivotEncoder;
    private RelativeEncoder m_extenderEncoder;
    private RelativeEncoder m_gripperRotatorEncoder;
    private RelativeEncoder m_gripperPivotEncoder;
    private RelativeEncoder m_intakeLeftEncoder;
    private RelativeEncoder m_intakeRightEncoder;
    private CANSparkMax m_ArmPivot1;
    private final CANSparkMax m_ArmPivot2;
    private CANSparkMax m_Extender;
    private final CANSparkMax m_GripperRotator;
    private CANSparkMax m_GripperPivot;
    private final CANSparkMax m_IntakeLeft;
    private final CANSparkMax m_IntakeRight;
    private final MotorControllerGroup m_intake;
    private final MotorControllerGroup m_armPivot;
    public final static double POWER_LIMIT = 1;
    private AnalogPotentiometer m_absArmPivotEncoder;
    private AnalogPotentiometer m_absExtenderEncoder;
    private AnalogPotentiometer m_absGripperPivotEncoder;
    private double m_armAngle;
    private boolean m_brake = false;

    private enum Quadrant {
        Q1, Q2, Q3, Q4
    }

    public enum IntakeMode {
        OFF, FORWARD, BACKWARD, SLOW,
    }

    private IntakeMode m_intakeMode = IntakeMode.OFF;

    // These are non-null in simulation only
    private ArmSimulation m_ArmSimulation;

    public ArmSubsystem() {
        // If we're simulating the robot, set up a bunch of simulated components
        // to replace the normal ones.
        if (RobotBase.isSimulation()) {
            var simulatedArmPivotEncoder = new AnalogPotentiometerSimulation(0, 360, 0);
            simulatedArmPivotEncoder.set(90);
            m_absArmPivotEncoder = simulatedArmPivotEncoder;
            var simulatedExtenderEncoder = new AnalogPotentiometerSimulation(1, MAX_ARM_EXTENSION_LENGTH_INCHES, 0);
            m_absExtenderEncoder = simulatedExtenderEncoder;
            var simulatedGripperEncoder = new AnalogPotentiometerSimulation(2, 360, 0);
            m_absGripperPivotEncoder = simulatedGripperEncoder;
            m_ArmPivot1 = new CANSparkMaxSimulated(CompetitionDriveConstants.kArmPivotMotorPort1, MotorType.kBrushless);
            m_Extender = new CANSparkMaxSimulated(CompetitionDriveConstants.kExtenderMotorPort, MotorType.kBrushless);
            m_GripperPivot = new CANSparkMaxSimulated(CompetitionDriveConstants.kGripperPivotMotorPort,
                    MotorType.kBrushless);
            m_ArmSimulation = new ArmSimulation(
                    simulatedArmPivotEncoder,
                    m_ArmPivot1,
                    simulatedExtenderEncoder,
                    m_Extender,
                    simulatedGripperEncoder,
                    m_GripperPivot);
        }

        m_ArmPivot2 = new LimitedMotor(CompetitionDriveConstants.kArmPivotMotorPort2, MotorType.kBrushless,
                POWER_LIMIT);
        m_GripperRotator = new LimitedMotor(CompetitionDriveConstants.kGripperRotatorMotorPort, MotorType.kBrushless,
                POWER_LIMIT);
        m_IntakeLeft = new LimitedMotor(CompetitionDriveConstants.kIntakeLeft, MotorType.kBrushless, POWER_LIMIT);
        m_IntakeRight = new LimitedMotor(CompetitionDriveConstants.kIntakeRight, MotorType.kBrushless, POWER_LIMIT);
        m_breakSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, CompetitionDriveConstants.kBrakeSolenoid);
        m_gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, CompetitionDriveConstants.kGripperSolenoid);
        m_intakeLeftEncoder = m_IntakeLeft.getEncoder();
        m_intakeRightEncoder = m_IntakeRight.getEncoder();

        // Some components got replaced for simulation, so don't set them up here if
        // they're
        // simulated.
        if (!RobotBase.isSimulation()) {
            m_ArmPivot1 = new LimitedMotor(CompetitionDriveConstants.kArmPivotMotorPort1, MotorType.kBrushless,
                    POWER_LIMIT);
            m_Extender = new LimitedMotor(CompetitionDriveConstants.kExtenderMotorPort, MotorType.kBrushless,
                    POWER_LIMIT);
            m_GripperPivot = new LimitedMotor(CompetitionDriveConstants.kGripperPivotMotorPort, MotorType.kBrushless,
                    POWER_LIMIT);
            m_absArmPivotEncoder = new AnalogPotentiometer(0, 285, 36.6);
            m_absExtenderEncoder = new AnalogPotentiometer(1, MAX_ARM_EXTENSION_LENGTH_INCHES * 1.749, -3.229);
            m_absGripperPivotEncoder = new AnalogPotentiometer(2, 360 * 0.665, 69.32);
        }

        m_armPIDController = new PIDController(kArmP, kArmI, kArmD);
        m_extenderPIDController = new PIDController(kextenderP, kextenderI, kextenderD);
        m_gripperPivotPIDController = new PIDController(kgripperP, kgripperI, kgripperD);
        m_gripperRotatorPIDController = new PIDController(kgripperRotatorP, kgripperRotatorI, kgripperRotatorD);
        m_intakeLeftPIDController = new PIDController(kintakeLeftP,kintakeLeftI,kintakeLeftD);
        m_intakeRightPIDController = new PIDController(kintakeRightP,kintakeRightI,kintakeRightD);
        

        m_ArmPivot1.restoreFactoryDefaults();
        m_ArmPivot2.restoreFactoryDefaults();
        m_GripperPivot.restoreFactoryDefaults();
        m_GripperRotator.restoreFactoryDefaults();
        m_IntakeLeft.restoreFactoryDefaults();
        m_IntakeRight.restoreFactoryDefaults();

        m_ArmPivot1.setSmartCurrentLimit(40);
        m_ArmPivot2.setSmartCurrentLimit(40);
        m_Extender.setSmartCurrentLimit(30);
        m_GripperPivot.setSmartCurrentLimit(40);
        m_GripperRotator.setSmartCurrentLimit(40);
        m_IntakeLeft.setSmartCurrentLimit(40);
        m_IntakeRight.setSmartCurrentLimit(40);

        m_ArmPivot1.setIdleMode(IdleMode.kBrake);
        m_ArmPivot2.setIdleMode(IdleMode.kBrake);
        m_Extender.setIdleMode(IdleMode.kBrake);
        m_GripperPivot.setIdleMode(IdleMode.kBrake);
        m_GripperRotator.setIdleMode(IdleMode.kCoast);
        m_IntakeLeft.setIdleMode(IdleMode.kBrake);
        m_IntakeRight.setIdleMode(IdleMode.kBrake);

        m_ArmPivot1.setInverted(false);
        m_ArmPivot2.setInverted(false);
        m_Extender.setInverted(false);
        m_GripperPivot.setInverted(true);
        m_GripperRotator.setInverted(false);
        m_IntakeLeft.setInverted(false);
        m_IntakeRight.setInverted(true);

        m_ArmPivot1.burnFlash();
        m_ArmPivot2.burnFlash();
        m_Extender.burnFlash();
        m_GripperPivot.burnFlash();
        m_GripperRotator.burnFlash();
        m_IntakeLeft.burnFlash();
        m_IntakeRight.burnFlash();

        m_intake = new MotorControllerGroup(m_IntakeLeft, m_IntakeRight);
        m_armPivot = new MotorControllerGroup(m_ArmPivot1, m_ArmPivot2);

        m_armPivotEncoder = m_ArmPivot1.getEncoder();
        m_extenderEncoder = m_Extender.getEncoder();
        m_gripperRotatorEncoder = m_GripperRotator.getEncoder();
        m_gripperPivotEncoder = m_GripperPivot.getEncoder();

        m_gripperPivotPIDController.setSetpoint(0);

        SmartDashboard.putData("Arm PID", m_armPIDController);
        SmartDashboard.putData("Extender PID", m_extenderPIDController);
        SmartDashboard.putData("Gripper PID", m_gripperPivotPIDController);
        SmartDashboard.putData("Arm/Intake/leftPid", m_intakeLeftPIDController);
        SmartDashboard.putData("Arm/Intake/rightPid", m_intakeRightPIDController);

    }

    @Override
    public void periodic() {
        while (true) {
            m_armAngle = m_absArmPivotEncoder.get();
            double relativeAngle;
            // updateIntake();
            // We read in arm PID values and take advantage of them.

            SmartDashboard.putNumber("Extension motor", m_Extender.get());
            SmartDashboard.putNumber("Gripper motor", m_GripperPivot.get());
            SmartDashboard.putNumber("Gripper encoder", m_absGripperPivotEncoder.get());

            Quadrant quadrant;
            var armPivotVoltage = m_armPIDController.calculate(m_armAngle);
            var extenderVoltage = m_extenderPIDController.calculate(m_absExtenderEncoder.get());
            var gripperPivotVoltage = m_gripperPivotPIDController.calculate(m_absGripperPivotEncoder.get());
            var gripperRotationVoltage = m_gripperRotatorPIDController.calculate(m_gripperRotatorEncoder.getPosition());
            var armPivotPosition = m_armAngle;
            var armPivotSetpoint = m_armPIDController.getSetpoint();
            var extenderPosition = m_absExtenderEncoder.get();
            var extenderSetpoint = m_extenderPIDController.getSetpoint();
            var gripperPivotPosition = m_absGripperPivotEncoder.get();
            double lastArmSetPoint = Integer.MIN_VALUE;
            SmartDashboard.putNumber("GripperRotatorEnc",m_gripperRotatorEncoder.getPosition());
            SmartDashboard.putNumber("PIDarmPivotVoltage", armPivotVoltage);
            SmartDashboard.putNumber("extenderBusVoltage", m_Extender.get());
            SmartDashboard.putNumber("extenderVoltage", extenderVoltage);
            SmartDashboard.putNumber("armPivotSetpoint", m_armPIDController.getSetpoint());
            SmartDashboard.putNumber("extenderSetpoint", m_extenderPIDController.getSetpoint());
            SmartDashboard.putNumber("extender potentiometer", extenderPosition);
            SmartDashboard.putNumber("arm potentiometer", armPivotPosition);
            SmartDashboard.putNumber("gripper potentiometer", gripperPivotPosition);
            SmartDashboard.putBoolean("Brake", getBreakSol());
            SmartDashboard.putNumber("extenderOutputCurrent", m_Extender.getOutputCurrent());
            SmartDashboard.putNumber("gripperVoltage", gripperPivotVoltage);
            SmartDashboard.putNumber("gripperSetpoint", m_gripperPivotPIDController.getSetpoint());
            SmartDashboard.putNumber("gripperOutputCurrent", m_GripperPivot.getOutputCurrent());
            SmartDashboard.putNumber("gripper rotator enc", m_gripperRotatorEncoder.getPosition());
            // SmartDashboard.putBoolean("GripperSol", m_gripperSolenoid.get());
            if (m_armAngle < 90) {
                relativeAngle = 90 - m_armAngle;
                quadrant = Quadrant.Q1;
            } else if (m_armAngle < 180) {
                relativeAngle = m_armAngle - 90;
                quadrant = Quadrant.Q2;
            } else if (m_armAngle < 270) {
                relativeAngle = 270 - m_armAngle;
                quadrant = Quadrant.Q3;
            } else {
                relativeAngle = m_armAngle - 270;
                quadrant = Quadrant.Q4;
            }
            SmartDashboard.putNumber("RelativeAngle", relativeAngle);
            SmartDashboard.putNumber("Height",
                    (extenderPosition + ksolidArmDistance) * Math.sin(Units.degreesToRadians(relativeAngle)));
            SmartDashboard.putNumber("X",
                    (extenderPosition + ksolidArmDistance) * Math.cos(Units.degreesToRadians(relativeAngle)));
            var distanceFromPivotPointHorizontalInches = (extenderSetpoint + ksolidArmDistance) * Math.cos(Units.degreesToRadians(relativeAngle));
            if (quadrant.equals(Quadrant.Q2) || quadrant.equals(Quadrant.Q3)) {
                if ((extenderSetpoint + ksolidArmDistance) * Math.sin(Units.degreesToRadians(relativeAngle)) > 52) {
                    setExtenderSetpoint((52 / Math.sin(Units.degreesToRadians(relativeAngle)) - ksolidArmDistance) - 2);
                    m_Extender.setVoltage(m_extenderPIDController.calculate(m_absExtenderEncoder.get()));
                    // m_armPivot.setVoltage(0);
                    SmartDashboard.putBoolean("Debug", true);
                    break;
                }

                if (distanceFromPivotPointHorizontalInches > 63.5) {
                    setExtenderSetpoint((63.5 / Math.cos(Units.degreesToRadians(relativeAngle)) - ksolidArmDistance) - 0.5);
                    m_Extender.setVoltage(m_extenderPIDController.calculate(m_absExtenderEncoder.get()));
                    SmartDashboard.putBoolean("Debug", true);
                    // m_armPivot.setVoltage(0);
                    break;
                }
            } else { // we are in the bottom quadrants Q1 || Q4
             if ((extenderSetpoint + ksolidArmDistance) * Math.sin(Units.degreesToRadians(relativeAngle)) > 21) {
                lastArmSetPoint = m_armPIDController.getSetpoint();
                setArmPivotSetpoint(armPivotPosition);
                setExtenderSetpoint((22 / Math.sin(Units.degreesToRadians(relativeAngle)) - ksolidArmDistance) - 11);
                m_armPivot.setVoltage(0);
                m_Extender.setVoltage(m_extenderPIDController.calculate(m_absExtenderEncoder.get()));
                SmartDashboard.putBoolean("Debug", true);
                break;
                }
            
                if (distanceFromPivotPointHorizontalInches > 62) {
                    setExtenderSetpoint((62 / Math.cos(Units.degreesToRadians(relativeAngle)) - ksolidArmDistance) - 2);
                    m_Extender.setVoltage(m_extenderPIDController.calculate(m_absExtenderEncoder.get()));
                    SmartDashboard.putBoolean("Debug", true);
                    // m_armPivot.setVoltage(0);
                    break;
                }
            }
            if (lastArmSetPoint != Integer.MIN_VALUE)
                setArmPivotSetpoint(lastArmSetPoint);

            //Limiters
            var LIMITER_MODE = 1;//use 1
            switch (LIMITER_MODE) {
                case 1:
                    if (armPivotSetpoint < 60)// imagining 0 means vertically down
                        setArmPivotSetpoint(60);
                    if (armPivotSetpoint > 300)
                        setArmPivotSetpoint(300);
                    m_armPivot.setVoltage(armPivotVoltage);
                    break;
                case 2:
                    //Do not use
                    if (armPivotPosition > 300 && armPivotVoltage > 0)
                        m_armPivot.setVoltage(0);
                    else if (armPivotPosition < 60 && armPivotVoltage < 0)
                        m_armPivot.setVoltage(0);
                    else
                        m_armPivot.setVoltage(armPivotVoltage);
                    break;
                case 3:
                    //Do not use
                    if (armPivotPosition > 60 && armPivotPosition < 300)// imagining 0 means vertically down
                        m_armPivot.setVoltage(armPivotVoltage);
                    else
                        m_armPivot.setVoltage(0);
                    break;
            }

            SmartDashboard.putString("Pivot1kError", m_ArmPivot1.getLastError().toString());
            SmartDashboard.putString("Pivot2Error", m_ArmPivot2.getLastError().toString());

            if (extenderPosition > 39)
                setExtenderSetpoint(39);
            if (extenderSetpoint < 3)
                setExtenderSetpoint(3);

            m_Extender.setVoltage(extenderVoltage);

            // m_Extender.setVoltage(extenderVoltage);

            // if (gripperPivotPosition<55&&gripperPivotPosition>-55){
            // m_GripperPivot.setVoltage(gripperPivotVoltage*10/10);
            // }
            // else{
            // m_GripperPivot.setVoltage(0);
            // }

            if (gripperPivotPosition > (180 + 45) && gripperPivotVoltage > 0) // imagining 0 means vertically down
                m_GripperPivot.setVoltage(0);
            else if (gripperPivotPosition < (180 - 45) && gripperPivotVoltage < 0)
                m_GripperPivot.setVoltage(0);
            else
                m_GripperPivot.setVoltage(gripperPivotVoltage);

            SmartDashboard.putNumber("gripperRotatorVoltage", gripperPivotVoltage);
            m_GripperRotator.setVoltage(gripperRotationVoltage);

            if (!m_breakSolenoid.get())
                m_armPivot.setVoltage(0);

            updateIntake();

            // if(Math.abs(armPivotPosition - m_armPIDController.getSetpoint())<1){
            // m_breakSolenoid.set(false);
            // m_armPivot.setVoltage(0);
            // }
            // else{
            // m_armPivot.setVoltage(armPivotVoltage);
            // m_breakSolenoid.set(true);
            // }

            // if(armPivotPosition<160)
            // setGripperPivotSetpoint(0);
            // if(armPivotPosition>200)
            // setGripperPivotSetpoint(93);

            SmartDashboard.putBoolean("Debug", false);
            break;
        }

        updateDashboard();
    }

    @Override
    public void simulationPeriodic() {
        m_ArmSimulation.calculate();
    }

    public double getExtenderSetpoint(){
        return m_extenderPIDController.getSetpoint();
    }
    // Subroutine: update the intake based on current mode
    private void updateIntake() {
        switch (m_intakeMode) {
            // case OFF:
            //     m_intake.set(0);
            //     // m_intakeRightPIDController.setSetpoint(0);
            //     // m_intakeLeftPIDController.setSetpoint(0);
            //     break;
            case FORWARD:
                m_intake.set(1);
                // m_intakeRightPIDController.setSetpoint(1);
                // m_intakeLeftPIDController.setSetpoint(1);
                break;
            case BACKWARD:
                m_intake.set(-1);
                // m_intakeRightPIDController.setSetpoint(-1);
                // m_intakeLeftPIDController.setSetpoint(-1);
                break;
            case SLOW:
                m_intake.set(-0.25);
                // m_intakeRightPIDController.setSetpoint(-.125);
                // m_intakeLeftPIDController.setSetpoint(-.125);
                break;
        }
        // SmartDashboard.putString("IntakeMode", m_intakeMode.toString());
        // var intakeLeftVelocity = m_intakeLeftEncoder.getVelocity();
        // var intakeRightVelocity = m_intakeRightEncoder.getVelocity();
        // var intakeLeftVoltage = m_intakeLeftPIDController.calculate(intakeLeftVelocity);
        // var intakeRightVoltage = m_intakeRightPIDController.calculate(intakeRightVelocity);

        // if(m_intakeMode == IntakeMode.OFF){
        //     m_IntakeRight.setVoltage(0);
        //     m_IntakeLeft.setVoltage(0);
        // }else{
        //     m_IntakeLeft.setVoltage(intakeLeftVoltage);
        //     m_IntakeRight.setVoltage(intakeRightVoltage);
        // }
        

        // SmartDashboard.putNumber("Arm/Intake/leftVoltage", intakeLeftVoltage);
        // SmartDashboard.putNumber("Arm/Intake/rightVoltage", intakeRightVoltage);
        // SmartDashboard.putNumber("Arm/Intake/rightVelocity", intakeRightVelocity);
        // SmartDashboard.putNumber("Arm/Intake/leftVelocity", intakeLeftVelocity);

    }

    public IntakeMode getIntakeMode() {
        return m_intakeMode;
    }

    public void incrementArmPivotSetpoint(double increment) {
        double setpoint = m_armPIDController.getSetpoint();
        setArmPivotSetpoint(setpoint + (increment / 50.0));
    }

    public void incrementExtenderSetpoint(double increment) {
        double setpoint = m_extenderPIDController.getSetpoint();
        m_extenderPIDController.setSetpoint(setpoint + (increment / 50.0));
    }

    public void incrementGripperPivotSetpoint(double increment) {
        double setpoint = m_gripperPivotPIDController.getSetpoint();
        m_gripperPivotPIDController.setSetpoint(setpoint + (increment / 50.0));
    }

    public void setArmPivotSetpoint(double setpoint) {
        m_armPIDController.setSetpoint(setpoint);
    }

    public void setExtenderSetpoint(double setpoint) {
        m_extenderPIDController.setSetpoint(setpoint);
    }

    public void setGripperPivotSetpoint(double setpoint) {
        m_gripperPivotPIDController.setSetpoint(setpoint);
    }

    public void toggleGripperRotator() {
        if (m_gripperRotatorPIDController.getSetpoint() == 0)
            setGripperPivotSetpoint(93);
        else
            setGripperRotatorSetpoint(0);
    }

    private void setGripperRotatorSetpoint(double setpoint) {
        m_gripperRotatorPIDController.setSetpoint(setpoint);
    }

    public double getGripperRotatorSetpoint() {
        return m_gripperPivotPIDController.getSetpoint();
    }

    public double getArmPivotSetpoint() {
        return m_armPIDController.getSetpoint();
    }

    public void resetExtenderEnc() {
        m_extenderEncoder.setPosition(0);
    }

    public void resetArmPivotEnc() {
        m_armPivotEncoder.setPosition(0);
    }

    public void resetGripperRotatorEnc() {
        m_armPivotEncoder.setPosition(0);
    }

    public void resetGripperPivotEnc() {
        m_gripperPivotEncoder.setPosition(0);
    }

    public void resetIntakeEnc() {
        m_intakeLeftEncoder.setPosition(0);
        m_intakeRightEncoder.setPosition(0);
    }

    public double getExtendorEnc() {
        return m_extenderEncoder.getPosition();
    }

    public double getArmPivotEnc() {
        return m_armPivotEncoder.getPosition();
    }

    public double getGripperRotatorEnc() {
        return m_gripperRotatorEncoder.getPosition();
    }

    public double getGripperPivotEnc() {
        return m_gripperPivotEncoder.getPosition();
    }

    public double getIntakeEnc() {
        return m_intakeLeftEncoder.getPosition();
    }

    public double getArmPivotAbs() {
        return m_armAngle;
    }

    public double getExtendorAbs() {
        return m_absExtenderEncoder.get();
    }

    public double getGripperPivotAbs() {
        return m_absGripperPivotEncoder.get();
    }

    public void setOveride(boolean overide) {
        m_brake = overide;
    }

    public boolean getOveride() {
        return m_brake;
    }

    public void toggleBreak() {
        m_breakSolenoid.toggle();
    }

    public void setBreak(boolean brake) {
        m_breakSolenoid.set(brake);
    }

    public boolean getBreakSol() {
        return m_breakSolenoid.get();
    }

    public void toggleGripper() {
        m_gripperSolenoid.toggle();
    }

    public void toggleGripper(boolean openGripper) {
        m_gripperSolenoid.set(openGripper);
    }

    public boolean getGripperSol() {
        return m_gripperSolenoid.get();
    }

    public void setIntakeMode(IntakeMode mode) {
        m_intakeMode = mode;
    }

    // only use once in robotInit
    public void resetEncoders() {
        resetArmPivotEnc();
        resetExtenderEnc();
        resetGripperPivotEnc();
        resetGripperRotatorEnc();
        resetIntakeEnc();
    }

    private void updateDashboard(){
        SmartDashboard.putNumber("gripper/pivot angle", m_absGripperPivotEncoder.get());
        SmartDashboard.putNumber("gripper/rotation", m_gripperRotatorEncoder.getPosition());

        SmartDashboard.putNumber("arm/extender", m_extenderEncoder.getPosition());
        SmartDashboard.putNumber("arm/angle", m_absArmPivotEncoder.get());
    }

}
