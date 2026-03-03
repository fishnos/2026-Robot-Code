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
        assertTrue(ZoneConstants.ZONES.containsKey("tower"));
        assertTrue(ZoneConstants.ZONES.containsKey("tower_left"));
        assertTrue(ZoneConstants.ZONES.containsKey("tower_right"));
        assertTrue(ZoneConstants.ZONES.containsKey("alliance_lower"));
        assertTrue(ZoneConstants.ZONES.containsKey("alliance_lower_transition"));
        assertTrue(ZoneConstants.ZONES.containsKey("alliance_middle"));
        assertTrue(ZoneConstants.ZONES.containsKey("alliance_upper"));
        assertTrue(ZoneConstants.ZONES.containsKey("hub_exclusion"));
        assertTrue(ZoneConstants.ZONES.containsKey("neutral_lower"));
        assertTrue(ZoneConstants.ZONES.containsKey("neutral_lower_transition"));
        assertTrue(ZoneConstants.ZONES.containsKey("neutral_middle"));
        assertTrue(ZoneConstants.ZONES.containsKey("neutral_upper_transition"));
        assertTrue(ZoneConstants.ZONES.containsKey("neutral_upper"));
    }

    @Test
    void allianceComposite_containsExpectedRectangleCount() {
        assertEquals(4, ZoneConstants.Alliance.COMPOSITE.size());
    }

    @Test
    void getZone_returnsHubExclusionZone() {
        ZoneConstants.RectangleZone zone = ZoneConstants.getZone("hub_exclusion").orElseThrow();
        assertEquals(ZoneConstants.Hub.EXCLUSION, zone);
    }

    @Test
    void getZone_returnsTowerExclusionZone() {
        ZoneConstants.RectangleZone zone = ZoneConstants.getZone("tower").orElseThrow();
        assertEquals(ZoneConstants.Tower.EXCLUSION, zone);
    }

    @Test
    void neutralComposite_containsExpectedRectangleCount() {
        assertEquals(5, ZoneConstants.Neutral.COMPOSITE.size());
    }

    @Test
    void getZone_returnsNeutralMiddleZone() {
        ZoneConstants.RectangleZone zone = ZoneConstants.getZone("neutral_middle").orElseThrow();
        assertEquals(ZoneConstants.Neutral.MIDDLE, zone);
    }

    @Test
    void opposingAllianceComposite_aliasesAllianceComposite() {
        assertEquals(ZoneConstants.Alliance.COMPOSITE, ZoneConstants.OpposingAlliance.COMPOSITE);
    }
}
