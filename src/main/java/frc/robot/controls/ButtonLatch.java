package frc.robot.controls;

import java.util.function.Supplier;

public class ButtonLatch {
    private Supplier<Boolean> m_getButton;
    private boolean m_lastPressed = false;

    public ButtonLatch(Supplier getButton) {
        m_getButton = getButton;
    }

    public boolean wasPressed() {
        var wasPressed = m_getButton.get();
        var returnValue = wasPressed && !m_lastPressed;
        m_lastPressed = wasPressed;
        return returnValue;
    }
    
}
