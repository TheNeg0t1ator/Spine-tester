#include <LiquidCrystal_I2C.h>

class LCD_Display {
private:
    LiquidCrystal_I2C lcd;

public:
    LCD_Display() : lcd(0x27, 20, 4) {}

    LiquidCrystal_I2C get_lcd() {
        return lcd;
    }

    void init_display() {
        lcd.init();
        lcd.backlight();
    }
    void print(int x, int y, const char* message) {
        lcd.setCursor(x, y);
        lcd.print(message);
    }
    void clear() {
        lcd.clear();
    }
    void set_cursor(int x, int y) {
        lcd.setCursor(x, y);
    }
    void print(const char* message) {
        lcd.print(message);
    }
    void writeString(int x, int y, const char* format, ...) {
        char buffer[128];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        lcd.setCursor(x, y);
        lcd.print(buffer);
    }
    void write(char character) {
        lcd.write(character);
    }
};