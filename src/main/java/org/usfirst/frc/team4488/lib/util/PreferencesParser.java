package org.usfirst.frc.team4488.lib.util;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.usfirst.frc.team4488.robot.Constants;

public class PreferencesParser {

  private FileReader fileReader;
  private JSONObject json;
  private JSONParser jsonParser;

  private static PreferencesParser sInstance = null;

  public static PreferencesParser getInstance() {
    if (sInstance == null) {
      sInstance = new PreferencesParser();
    }

    return sInstance;
  }

  public PreferencesParser() {
    jsonParser = new JSONParser();
    update();
  }

  public String getString(String key) throws PreferenceDoesNotExistException {
    update();
    if (keyExists(key)) {
      return (String) json.get(key);
    }

    System.out.println("Key " + key + " does not exist in preferences!");
    throw new PreferenceDoesNotExistException();
  }

  public int getInt(String key) throws PreferenceDoesNotExistException {
    update();
    if (keyExists(key)) {
      return (int) json.get(key);
    }

    System.out.println("Key " + key + " does not exist in preferences!");
    throw new PreferenceDoesNotExistException();
  }

  public double getDouble(String key) throws PreferenceDoesNotExistException {
    update();
    if (keyExists(key)) {
      return ((Number) json.get(key)).doubleValue();
    }

    System.out.println("Key " + key + " does not exist in preferences!");
    throw new PreferenceDoesNotExistException();
  }

  private boolean keyExists(String key) {
    if (json.get(key) == null) {
      return false;
    }

    return true;
  }

  private void update() {
    try {
      fileReader = new FileReader(Constants.prefsPath);

      Object obj = jsonParser.parse(fileReader);
      json = (JSONObject) obj;

      fileReader.close();
    } catch (FileNotFoundException e) {
      System.out.println("Could not find preferences file at " + Constants.prefsPath);
    } catch (IOException e) {
      System.out.println("IOException in PreferencesParser");
    } catch (ParseException e) {
      e.printStackTrace();
    } catch (NullPointerException e) {
      System.out.println("NullPointerException in PreferencesParser");
    }
  }
}
