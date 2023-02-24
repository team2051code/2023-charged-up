package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
    public final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

    public boolean compressorEnabled(){
        return m_compressor.isEnabled();
    }

    public boolean fullPressure(){
        return m_compressor.getPressureSwitchValue();
    }

}
