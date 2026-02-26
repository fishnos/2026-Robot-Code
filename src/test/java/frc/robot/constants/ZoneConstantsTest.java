package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ZoneConstantsTest {
    @Test
    void getZone_returnsNamedTowerZone() {
        ZoneConstants.RectangleZone zone = ZoneConstants.getZone("tower_left").orElseThrow();
        assertEquals(ZoneConstants.Tower.LEFT, zone);
    }

    @Test
    void zones_containsTowerDefinitions() {
        assertTrue(ZoneConstants.ZONES.containsKey("tower_left"));
        assertTrue(ZoneConstants.ZONES.containsKey("tower_right"));
    }
}
