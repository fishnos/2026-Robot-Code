package frc.robot.lib.util;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

import com.google.gson.Gson;
import com.google.gson.JsonSyntaxException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.Constants;

public final class ConfigLoader {
    private static final Gson GSON = new Gson();
    private static final Map<String, Object> CACHE = new HashMap<>();

    private ConfigLoader() {
    }

    public static <T> T load(String subsystemName, Class<T> configClass) {
        return load(subsystemName, getModeName(), configClass);
    }

    public static String getModeFolder(boolean simOnlyFlag) {
        if (Constants.shouldUseSimulation(simOnlyFlag)) {
            return Constants.Mode.SIM.name();
        }
        if (Constants.currentMode == Constants.Mode.REPLAY) {
            return Constants.Mode.COMP.name();
        }
        return Constants.currentMode.name();
    }

    public static <T> T load(String subsystemName, String modeName, Class<T> configClass) {
        String normalizedMode = modeName.toLowerCase(Locale.ROOT);
        String cacheKey = subsystemName + ":" + normalizedMode;
        if (CACHE.containsKey(cacheKey)) {
            return configClass.cast(CACHE.get(cacheKey));
        }

        Path configPath = Filesystem.getDeployDirectory().toPath()
            .resolve("configs")
            .resolve(subsystemName)
            .resolve(normalizedMode + ".json");

        try {
            String json = Files.readString(configPath);
            T config = GSON.fromJson(json, configClass);
            CACHE.put(cacheKey, config);
            return config;
        } catch (IOException | JsonSyntaxException ex) {
            String message = "Failed to load config: " + configPath + " (" + ex.getMessage() + ")";
            DriverStation.reportError(message, ex.getStackTrace());
            throw new RuntimeException(message, ex);
        }
    }

    private static String getModeName() {
        if (Constants.currentMode == Constants.Mode.REPLAY) {
            return Constants.Mode.COMP.name();
        }
        return Constants.currentMode.name();
    }
}
