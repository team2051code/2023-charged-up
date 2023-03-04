package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
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
    public final static double kArmP = 1; public final static double kArmI = 0; public final static double kArmD = .1;
    public final static double kextenderP = .1; public final static double kextenderI = 0; public final static double kextenderD = 0;
    public final static double kgripperP = .1; public final static double kgripperI = 0; public final static double kgripperD = 0;
    public final static double kgripperRotatorP = 0; public final static double kgripperRotatorI = 0; public final static double kgripperRotatorD = 0;
    public final static double ksolidArmDistance = 28;
    public final static double MAX_ARM_EXTENSION_LENGTH_INCHES = 40;
    //reverse one of the sides of the intake and the arm before we make them a group
    private final PIDController m_gripperRotatorPIDController;
    private final PIDController m_armPIDController;
    private final PIDController m_extenderPIDController;
    private final PIDController m_gripperPivotPIDController;
    private Solenoid m_gripperSolenoid;
    private Solenoid m_breakSolenoid;
    private RelativeEncoder m_armPivotEncoder;
    private RelativeEncoder m_extenderEncoder;
    private RelativeEncoder m_gripperRotatorEncoder;
    private RelativeEncoder m_gripperPivotEncoder;
    private RelativeEncoder m_intakeEncoder;
    private CANSparkMax m_ArmPivot1;
    private final CANSparkMax m_ArmPivot2;
    private CANSparkMax m_Extender;
    private final CANSparkMax m_GripperRotator;
    private CANSparkMax m_GripperPivot;
    private final CANSparkMax m_IntakeLeft;
    private final CANSparkMax m_IntakeRight;
    private final MotorControllerGroup m_intake;
    private final MotorControllerGroup m_armPivot;
    public final static double POWER_LIMIT = 0;
    private AnalogPotentiometer m_absArmPivotEncoder;
    private AnalogPotentiometer m_absExtenderEncoder;
    private AnalogPotentiometer m_absGripperPivotEncoder;
    private enum Quadrant{
        Q1,Q2,Q3,Q4
    }
    public enum IntakeMode {
        FORWARD, OFF, BACKWARD,
    }
    private IntakeMode m_intakeMode = IntakeMode.OFF;

    // These are non-null in simulation only
    private ArmSimulation m_ArmSimulation;

    public ArmSubsystem(){
        // If we're simulating the robot, set up a bunch of simulated components
        // to replace the normal ones.
        if (RobotBase.isSimulation()) {
            var simulatedArmPivotEncoder = new AnalogPotentiometerSimulation(0, 360, 0);
            m_absArmPivotEncoder = simulatedArmPivotEncoder;
            var simulatedExtenderEncoder = new AnalogPotentiometerSimulation(1, MAX_ARM_EXTENSION_LENGTH_INCHES, 0);
            m_absExtenderEncoder = simulatedExtenderEncoder;
            var simulatedGripperEncoder = new AnalogPotentiometerSimulation(2, 360, 0);
            m_absGripperPivotEncoder = simulatedGripperEncoder;
            m_ArmPivot1 = new CANSparkMaxSimulated(CompetitionDriveConstants.kArmPivotMotorPort1, MotorType.kBrushless);
            m_Extender = new CANSparkMaxSimulated(CompetitionDriveConstants.kExtenderMotorPort, MotorType.kBrushless);
            m_GripperPivot = new CANSparkMaxSimulated(CompetitionDriveConstants.kGripperPivotMotorPort, MotorType.kBrushless);
            m_ArmSimulation = new ArmSimulation(
                simulatedArmPivotEncoder, 
                m_ArmPivot1, 
                simulatedExtenderEncoder, 
                m_Extender,
                simulatedGripperEncoder,
                m_GripperPivot);
        }

        m_ArmPivot2 = new LimitedMotor(CompetitionDriveConstants.kArmPivotMotorPort2, MotorType.kBrushless, POWER_LIMIT);
        m_GripperRotator = new LimitedMotor(CompetitionDriveConstants.kGripperRotatorMotorPort, MotorType.kBrushless, POWER_LIMIT);
        m_IntakeLeft = new LimitedMotor(CompetitionDriveConstants.kIntakeLeft, MotorType.kBrushless, POWER_LIMIT);
        m_IntakeRight = new LimitedMotor(CompetitionDriveConstants.kIntakeRight, MotorType.kBrushless, POWER_LIMIT);
        m_breakSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
        m_gripperSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
        m_intakeEncoder = m_IntakeRight.getEncoder();

        // Some components got replaced for simulation, so don't set them up here if they're
        // simulated.
        if (!RobotBase.isSimulation()) {
            m_ArmPivot1 = new LimitedMotor(CompetitionDriveConstants.kArmPivotMotorPort1, MotorType.kBrushless, POWER_LIMIT);
            m_Extender = new LimitedMotor(CompetitionDriveConstants.kExtenderMotorPort, MotorType.kBrushless, POWER_LIMIT);
            m_GripperPivot = new LimitedMotor(CompetitionDriveConstants.kGripperPivotMotorPort, MotorType.kBrushless, POWER_LIMIT);
            m_absArmPivotEncoder = new AnalogPotentiometer(0, 360, 0);
            m_absExtenderEncoder = new AnalogPotentiometer(1, MAX_ARM_EXTENSION_LENGTH_INCHES, 0);
            m_absGripperPivotEncoder = new AnalogPotentiometer(2,360, 0);
        }

        m_armPIDController = new PIDController(kArmP, kArmI, kArmD);
        m_extenderPIDController = new PIDController(kextenderP, kextenderI, kextenderD);
        m_gripperPivotPIDController = new PIDController(kgripperP, kgripperI, kgripperD);
        m_gripperRotatorPIDController = new PIDController(kgripperRotatorP, kgripperRotatorI, kgripperRotatorD);
        m_Extender.restoreFactoryDefaults();
        m_GripperPivot.restoreFactoryDefaults();
        m_GripperRotator.restoreFactoryDefaults();
        m_ArmPivot1.restoreFactoryDefaults();
        m_ArmPivot2.restoreFactoryDefaults();
        m_IntakeLeft.restoreFactoryDefaults();
        m_IntakeRight.restoreFactoryDefaults();
        m_IntakeRight.setInverted(true);
        m_ArmPivot2.setInverted(true);
        m_intake = new MotorControllerGroup(m_IntakeLeft, m_IntakeRight);
        m_armPivot = new MotorControllerGroup(m_ArmPivot1, m_ArmPivot2);

        m_armPivotEncoder = m_ArmPivot1.getEncoder();
        m_extenderEncoder = m_Extender.getEncoder();
        m_gripperRotatorEncoder = m_GripperRotator.getEncoder();
        m_gripperPivotEncoder = m_GripperPivot.getEncoder();

        m_gripperPivotPIDController.setSetpoint(180);

        SmartDashboard.putData("Arm PID", m_armPIDController);
        SmartDashboard.putData("Extender PID",m_extenderPIDController);
        SmartDashboard.putData("Gripper PID", m_gripperPivotPIDController);


    }

    @Override
    public void periodic() {
        double relativeAngle;
        updateIntake();
        // We read in arm PID values and take advantage of them.

        SmartDashboard.putNumber("Extension motor", m_Extender.get());
        SmartDashboard.putNumber("Gripper motor", m_GripperPivot.get());
        SmartDashboard.putNumber("Gripper encoder", m_absGripperPivotEncoder.get());

        Quadrant quadrant;
        var armPivotVoltage = m_armPIDController.calculate(m_absArmPivotEncoder.get());
        var extenderVoltage = m_extenderPIDController.calculate(m_absExtenderEncoder.get());
        var gripperPivotVoltage = m_gripperPivotPIDController.calculate(m_absGripperPivotEncoder.get());
        var gripperRotationVoltage = m_gripperRotatorPIDController.calculate(m_gripperRotatorEncoder.getPosition());
        var armPivotPosition = m_absArmPivotEncoder.get();
        var extenderPosition = m_absExtenderEncoder.get();
        var gripperPivotPosition = m_absGripperPivotEncoder.get();
        SmartDashboard.putNumber("armPivotVoltage", armPivotVoltage);
        SmartDashboard.putNumber("armPivotSetpoint", m_armPIDController.getSetpoint());
        SmartDashboard.putNumber("arm potentiometer", armPivotPosition);
        SmartDashboard.putBoolean("Brake", getBreakSol());
        if(m_absArmPivotEncoder.get()<90)
        {
            relativeAngle = 90 - m_absArmPivotEncoder.get();
            quadrant = Quadrant.Q1;
        }
        else if(m_absArmPivotEncoder.get()<180)
        {
            relativeAngle = m_absArmPivotEncoder.get() - 90;
            quadrant = Quadrant.Q2;
        }
        else if(m_absArmPivotEncoder.get()<270)
        {
            relativeAngle = 270-m_absArmPivotEncoder.get();
            quadrant = Quadrant.Q3;
        }
        else 
        {
            relativeAngle = m_absArmPivotEncoder.get()-270;
            quadrant = Quadrant.Q4;
        }
        if(quadrant.equals(Quadrant.Q2)||quadrant.equals(Quadrant.Q3))
        {
            if((extenderPosition+ksolidArmDistance)*Math.sin(relativeAngle)>52)
                setExtenderSetpoint((52/Math.sin(relativeAngle)-ksolidArmDistance)-2);
            if((extenderPosition+ksolidArmDistance)*Math.cos(relativeAngle)>62)
                setExtenderSetpoint((62/Math.cos(relativeAngle)-ksolidArmDistance)-2);
        }else
            if((extenderPosition+ksolidArmDistance)*Math.sin(relativeAngle)>24)
                setExtenderSetpoint((24/Math.sin(relativeAngle)-ksolidArmDistance)-2);

        if(armPivotPosition>45 && armPivotPosition < 325) //imagining 0 means vertically down
            m_armPivot.setVoltage(armPivotVoltage);
        else
            m_armPivot.setVoltage(0);
         m_Extender.setVoltage(extenderVoltage);
        
        //if(gripperPivotPosition>35 && gripperPivotPosition<145) //imagining 0 means vertically down
            m_GripperPivot.setVoltage(gripperPivotVoltage);
        //else
        //    m_GripperPivot.setVoltage(0);
        m_GripperRotator.setVoltage(gripperRotationVoltage);
        
        if(Math.abs(armPivotPosition - m_armPIDController.getSetpoint())<0.4999){
            m_breakSolenoid.set(false);
            m_armPivot.setVoltage(0);
        }
        else{
            m_armPivot.setVoltage(armPivotVoltage);
            m_breakSolenoid.set(true);
        }

    }

    @Override
    public void simulationPeriodic() {
        m_ArmSimulation.calculate();
    }

    // Subroutine: update the intake based on current mode
    private void updateIntake() {
        m_intakeMode = IntakeMode.FORWARD;
        switch(m_intakeMode) {
            case OFF:
                m_intake.set(0);
                break;
            case FORWARD:
                m_intake.set(.5);
                break;
            case BACKWARD:
                m_intake.set(-0.5);
                break;
        }
    }

    public void incrementArmPivotSetpoint(double increment){
        double setpoint = m_armPIDController.getSetpoint();
        m_armPIDController.setSetpoint(setpoint+(increment/50.0));
    }

    public void incrementExtenderSetpoint(double increment){
        double setpoint = m_extenderPIDController.getSetpoint();
        m_extenderPIDController.setSetpoint(setpoint+(increment/50.0));
    }

    public void incrementGripperPivotSetpoint(double increment){
        double setpoint = m_gripperPivotPIDController.getSetpoint();
        m_gripperPivotPIDController.setSetpoint(setpoint+(increment/50.0));
    }

    public void incrementGripperRotatorSetpoint(double increment){
        double setpoint = m_gripperRotatorPIDController.getSetpoint();
        m_gripperRotatorPIDController.setSetpoint(setpoint+(increment/50.0));
    }

    public void setArmPivotSetpoint(double setpoint){
        m_armPIDController.setSetpoint(setpoint);
    }

    public void setExtenderSetpoint(double setpoint){
        m_extenderPIDController.setSetpoint(setpoint);
    }
    
    public void setGripperPivotSetpoint(double setpoint){
        m_gripperPivotPIDController.setSetpoint(setpoint);
    }

    public void setGripperRotatorSetpoint(double setpoint){
        m_gripperRotatorPIDController.setSetpoint(setpoint);
    }

    public void resetExtenderEnc(){
        m_extenderEncoder.setPosition(0);
    }

    public void resetArmPivotEnc(){
        m_armPivotEncoder.setPosition(0);
    }

    public void resetGripperRotatorEnc(){
        m_armPivotEncoder.setPosition(0);
    }

    public void resetGripperPivotEnc(){
        m_gripperPivotEncoder.setPosition(0);
    }

    public void resetIntakeEnc(){
        m_intakeEncoder.setPosition(0);
    }

    public double getExtendorEnc(){
        return m_extenderEncoder.getPosition();
    }

    public double getArmPivotEnc(){
        return m_armPivotEncoder.getPosition();
    }

    public double getGripperRotatorEnc(){
        return m_gripperRotatorEncoder.getPosition();
    }

    public double getGripperPivotEnc(){
        return m_gripperPivotEncoder.getPosition();
    }

    public double getIntakeEnc(){
        return m_intakeEncoder.getPosition();
    }

    public double getArmPivotAbs(){
        return m_absArmPivotEncoder.get();
    }

    public double getExtendorAbs(){
        return m_absExtenderEncoder.get();
    }

    public double getGripperPivotAbs(){
        return m_absGripperPivotEncoder.get();
    }

    public void toggleBreak(){
        m_breakSolenoid.toggle();
    }

    public boolean getBreakSol(){
        return m_breakSolenoid.get();
    }
    
    public void toggleGripper(){
        m_gripperSolenoid.toggle();
    }

    public boolean getGripperSol(){
        return m_gripperSolenoid.get();
    }

    public void setIntakeMode(IntakeMode mode) {
        m_intakeMode = mode;
    }

}
