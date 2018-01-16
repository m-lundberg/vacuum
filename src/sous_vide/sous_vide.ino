#include <OneWire.h>
#include <DallasTemperature.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#include <MenuSystem.h>
#include <EEPROMex.h>
#include <PID_v1.h>

// EEPROM
#define CONFIG_VERSION "sv2"
#define CONFIG_START 32

// Settings
struct StoreStruct {
    double setpoint;
    double p, i, d;
    char version[4];
} settings = {
    50.0,
    40, 15, 10,
    CONFIG_VERSION
};

// OLED Display
SSD1306AsciiAvrI2c oled;

// Rotary encoder
// Top view of rotary encoder:
// Rotary B   -|   |-   Rotary Button
// Ground     -| O |
// Rotary A   -|___|-   Ground
const int rotaryA = 2;
const int rotaryB = 3;
const int rotaryButton = 4;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
const byte RIGHT = 2;
const byte LEFT = 1;

// SSR
const int SSR = 13;  // TODO: change to 5

// Thermocouple (DS18B20)
const int ds18b20 = 6;
OneWire oneWire(ds18b20);
DallasTemperature thermo(&oneWire);
DeviceAddress dsAddress;
const int dsResolution = 10;
unsigned long lastTempRequest = 0;
int tempReadDelay = 0;

// PID
double input, output;
PID pid(&input, &output, &(settings.setpoint), settings.p, settings.i, settings.d, REVERSE);
int windowSize = 1000;
unsigned long windowStartTime;

// Heating turned on/off
bool heating = false;

// Menu system
bool inMenu = false;
void onSelectedNumeric(MenuComponent* component);
void onSelected(MenuComponent* component);

class Renderer : public MenuComponentRenderer {
public:
    void render(Menu const& menu) const {
        if (!inMenu) {
            return;
        }
      
        oled.clear();
        
        for (int i = 0; i < menu.get_num_components(); ++i) {
            const MenuComponent* component = menu.get_menu_component(i);
            component->render(*this);

            if (component->is_current()) {
                oled.print(" <<<");
            }
            oled.println("");
        }
    }

    void render_menu_item(MenuItem const& menu_item) const {
        oled.print(menu_item.get_name());
    }

    void render_back_menu_item(BackMenuItem const& menu_item) const {
        oled.print(menu_item.get_name());
    }

    void render_numeric_menu_item(NumericMenuItem const& menu_item) const {
        oled.print(menu_item.get_name());
        oled.print(": ");
        oled.print(menu_item.get_formatted_value());
    }

    void render_menu(Menu const& menu) const {
        oled.print(menu.get_name());
    }
};

Renderer renderer;
MenuSystem ms(renderer);
MenuItem toggleHeatingItem("Heating", &onSelected);
NumericMenuItem tempItem("Target", &onSelectedNumeric, settings.setpoint, 0, 90);
Menu pidMenu("PID Settings");
NumericMenuItem pItem("P", &onSelectedNumeric, settings.p, 0, 100);
NumericMenuItem iItem("I", &onSelectedNumeric, settings.i, 0, 100);
NumericMenuItem dItem("D", &onSelectedNumeric, settings.d, 0, 100);
BackMenuItem backItem("Back", &onSelected, &ms);

void setup() {
    Serial.begin(9600);
    loadConfig();

    // Set up OLED display with I2C address 0x3C and fast mode = true
    oled.begin(&Adafruit128x64, 0x3C, true);
    oled.setFont(System5x7);

    oled.clear();
    oled.set2X();
    oled.println("Booting");
    oled.println("Sous Vide");
    oled.println("2.0");
    oled.set1X();

    // Set up thermocouple
    thermo.begin();
    thermo.getAddress(dsAddress, 0);
    thermo.setResolution(dsAddress, dsResolution);
    thermo.setWaitForConversion(false);
    tempReadDelay = 750 / (1 << (12-dsResolution));

    windowStartTime = millis();

    pid.SetOutputLimits(0, windowSize);
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(400);  // set pid to compute new output every 400 ms to allow temp readings to happen
    input = readTemp();

    pinMode(SSR, OUTPUT);
    
    pinMode(rotaryA, INPUT_PULLUP);
    pinMode(rotaryB, INPUT_PULLUP);
    pinMode(rotaryButton, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rotaryA), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rotaryB), updateEncoder, CHANGE);

    // Define menu
    ms.get_root_menu().add_item(&toggleHeatingItem);
    ms.get_root_menu().add_item(&tempItem);
    pidMenu.add_item(&pItem);
    pidMenu.add_item(&iItem);
    pidMenu.add_item(&dItem);
    pidMenu.add_item(&backItem);
    ms.get_root_menu().add_menu(&pidMenu);
    ms.get_root_menu().add_item(&backItem);

    delay(1400);
    oled.clear();

    Serial.println("Sous Vide started");
}

void loop() {
    // Read commands from serial port (HC-05 bluetooth)
    readSerialCommands();
    
    byte rotation = encoderRotated();
    if (rotation == RIGHT) {
        // rotated right
        if (inMenu) {
            ms.next();
            ms.display();
        } else {
            settings.setpoint += 1;
        }
    } else if (rotation == LEFT) {
        // rotated left
        if (inMenu) {
            ms.prev();
            ms.display();
        } else {
            settings.setpoint -= 1;
        }
    }
    
    // Handle rotary button
    if (buttonPressed()) {
        // button pressed and debounced
        if (!inMenu) {
            inMenu = true;
            ms.reset();
            ms.display();
        } else {
            ms.select();
            ms.display();
        }
    }

    input = readTemp();

    if (heating) {
        pid.Compute();
    } else {
        digitalWrite(SSR, LOW);
    }

    // Control SSR using time proporional control
    unsigned long now = millis();
    if (now-windowStartTime > windowSize) {
        windowStartTime += windowSize;
    }
    if (heating && output < now-windowStartTime) {
        digitalWrite(SSR, HIGH);
    } else {
        digitalWrite(SSR, LOW);
    }

    draw();
}

void draw() {
    // Keep track of some state to avoid updating screen unnecessarily
    static unsigned long lastUpdate = 0;
    static int lastTemp = 0;
    static int lastSetpoint = 0;
    static bool lastHeating = false;
    static bool lastInMenu = false;
    
    if (inMenu) {
        lastInMenu = true;
        return;
    }

    // Only update if state changed and enough time has passed since last update
    if (millis()-lastUpdate < 440
        || (input == lastTemp && settings.setpoint == lastSetpoint && heating == lastHeating && !lastInMenu)) {
        return;
    }

    // Draw current temperature
    oled.clear();
    oled.set2X();
    int compensate = 5 - numberOfDigits((int)input);
    for (int i = 0; i < compensate; ++i) oled.print(' ');
    oled.print((int)input);
    oled.set1X();

    // Draw setpoint
    oled.setCursor(70, 1);
    oled.print("/ ");
    oled.print((int)settings.setpoint);

    // Draw heating indicatior
    oled.setCursor(0, 5);
    if (heating) {
        oled.print("Heating");
    }
    
    lastUpdate = millis();
    lastTemp = input;
    lastSetpoint = settings.setpoint;
    lastHeating = heating;
    lastInMenu = false;
}

// ISR for rotary encoder
void updateEncoder() {
    int MSB = digitalRead(rotaryA);  //MSB = most significant bit
    int LSB = digitalRead(rotaryB);  //LSB = least significant bit
    
    int encoded = (MSB << 1) | LSB;  //converting the 2 pin value to single number
    int sum  = (lastEncoded << 2) | encoded;  //adding it to the previous encoded value
    
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
    
    lastEncoded = encoded;  //store this value for next time
}

// Returns 1 for left, 2 for right and 0 for no rotation
byte encoderRotated() {
    static long lastEncoderValue = 0;
    if (encoderValue/4 > lastEncoderValue) {
        lastEncoderValue = encoderValue/4;
        return RIGHT;
    } else if (encoderValue/4 < lastEncoderValue) {
        lastEncoderValue = encoderValue/4;
        return LEFT;
    }
    return 0;
}

bool buttonPressed() {
    static bool pressable = true;
    static unsigned long lastPress;
    
    if (!digitalRead(rotaryButton) && pressable && millis()-lastPress > 200) {
        pressable = false;
        lastPress = millis();
        return true;
    } else if (digitalRead(rotaryButton)) {
        pressable = true;
        return false;
    }
}

void onSelectedNumeric(MenuComponent* component) {
    Serial.println(component->get_name());
    
    if (component->get_name() == pItem.get_name()) {
        Serial.println("Saving p");
        settings.p = ((NumericMenuItem*)component)->get_value();
    } else if (component->get_name() == iItem.get_name()) {
        settings.i = ((NumericMenuItem*)component)->get_value();
    } else if (component->get_name() == dItem.get_name()) {
        settings.d = ((NumericMenuItem*)component)->get_value();
    } else if (component->get_name() == tempItem.get_name()) {
        settings.setpoint = ((NumericMenuItem*)component)->get_value();
    }
    
    pid.SetTunings(settings.p, settings.i, settings.d);
    saveConfig();
    ms.display();
}

void onSelected(MenuComponent* component) {
    if (component->get_name() == backItem.get_name() && ms.get_current_menu() == &ms.get_root_menu()) {
        // in root menu, exit menu
        inMenu = false;
        ms.reset();
        oled.clear();
    } else if (component->get_name() == toggleHeatingItem.get_name()) {
        // toggle heating and exit from menu
        heating = !heating;
        inMenu = false;
        ms.reset();
        oled.clear();
    }
}

void readSerialCommands() {
    while (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        Serial.println(command);

        if (command.startsWith("set")) {
            // set - set a value for a setting
            int settingPos = command.indexOf(' ') + 1;
            int valuePos = command.indexOf(' ', settingPos) + 1;
            
            String setting = command.substring(settingPos, valuePos-1);
            String value = command.substring(valuePos);
            
            if (setting == "p") {
                settings.p = value.toFloat();
            } else if (setting == "i") {
                settings.i = value.toFloat();
            } else if (setting == "d") {
                settings.d = value.toFloat();
            } else if (setting == "setpoint" || setting == "target") {
                settings.setpoint = value.toFloat();
            } else {
                Serial.println("Unknown setting: " + setting);
            }
            Serial.println(value.toFloat());

            pid.SetTunings(settings.p, settings.i, settings.d);
            saveConfig();
            
        } else if (command.startsWith("get")) {
            // get - get the value of a setting
            int settingPos = command.indexOf(' ') + 1;
            String setting = command.substring(settingPos);
            
            if (setting == "p") {
                Serial.println(settings.p);
            } else if (setting == "i") {
                Serial.println(settings.i);
            } else if (setting == "d") {
                Serial.println(settings.d);
            } else if (setting == "setpoint" || setting == "target") {
                Serial.println(settings.setpoint);
            } else {
                Serial.println("Unknown setting: " + setting);
            }
            
        } else if (command.startsWith("target")) {
            int valuePos = command.indexOf(' ');
            if (valuePos == -1) {
                // no target given, print current
                Serial.println(settings.setpoint);
            } else {
                settings.setpoint = command.substring(valuePos+1).toFloat();
                saveConfig();
            }

        } else if (command.startsWith("on")) {
            Serial.println("Turning heating ON");
            heating = true;

        } else if (command.startsWith("off")) {
            Serial.println("Turning heating OFF");
            heating = false;

        } else {
            printHelpSerial();
        }
    }
}

void printHelpSerial() {
    Serial.println("Sous Vide commands:");
    Serial.println("  set SETTING VALUE - set the value of SETTING to VALUE");
    Serial.println("  get SETTING - get the current value of SETTING");
    Serial.println("  target [VALUE] - if VALUE is specified, set target to it; otherwise return current target");
    Serial.println("  on - turn heating on");
    Serial.println("  off - turn heating off");
    Serial.println("  help - show this help");
}

void saveConfig() {
    for (unsigned int t = 0; t < sizeof(settings); ++t) {
        EEPROM.update(CONFIG_START + t, *((char*)&settings + t));
    }
}

void loadConfig() {
    if (EEPROM.read(CONFIG_START + sizeof(settings) - 2) == settings.version[2] &&
        EEPROM.read(CONFIG_START + sizeof(settings) - 3) == settings.version[1] &&
        EEPROM.read(CONFIG_START + sizeof(settings) - 4) == settings.version[0]) { 
        for (unsigned int t = 0; t < sizeof(settings); ++t) {
            *((char*)&settings + t) = EEPROM.read(CONFIG_START + t);
        }
    } else {
        // settings invalid, overwrite with default
        Serial.println("Loaded invalid config");
        saveConfig();
    }
}

float readTemp() {
    static float temp = 0;
    
    if (millis() - lastTempRequest >= tempReadDelay) {  // check if conversion time is done
        temp = thermo.getTempCByIndex(0);
        thermo.requestTemperatures();
    }
    
    // temp will be the new value if conversion is done and the last value otherwise
    return temp;
}

// Returns the number of digits number has
int numberOfDigits(int number) {
    if (number < -9999) {
        return 6;
    } else if (number > 9999 || number < -999) {
        return 5;
    } else if (number > 999 || number < -99) {
        return 4;
    } else if (number > 99 || number < -9) {
        return 3;
    } else if (number > 9 || number < 0) {
        return 2;
    } else {
        return 1;
    }
}

