#include <MistLCD.h>


// class MistLCD
// {
//   int lcdCols;
//   int lcdRows;
//   hd44780_I2Cexp lcd;
//   boolean isReady;

MistLCD::MistLCD(int cols, int rows)
{
  lcdCols = cols;
  lcdRows = rows;
}

void MistLCD::print(int row)
{

}
//   boolean display(int row, String message) {
//     if (!isReady) {
//       return false;
//     }
//
//     lcd.clear();
//     lcd.setCursor(0, row);
//     lcd.print(message);
//     return true;
//   }
// };
