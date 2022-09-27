#include "LedController.h"

namespace LedController
{
  // We use the strip instead of the matrix to ensure different dimensions; Convesion of the pattern has to be done on the cliet side!
  Adafruit_NeoPixel *matrix;
  PINDEF *pins;
  bool DEBUG = false;
  bool isBusy;
  bool isOn = false;

  int NLED4x4 = 16;
  int NLED8x8 = 64;

  int LED_PATTERN_DPC_TOP_8x8[64] = {1, 1, 1, 1, 1, 1, 1, 1,
                                     1, 1, 1, 1, 1, 1, 1, 1,
                                     1, 1, 1, 1, 1, 1, 1, 1,
                                     1, 1, 1, 1, 1, 1, 1, 1,
                                     0, 0, 0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0, 0, 0};

  int LED_PATTERN_DPC_LEFT_8x8[64] = {1, 1, 1, 1, 0, 0, 0, 0,
                                      0, 0, 0, 0, 1, 1, 1, 1,
                                      1, 1, 1, 1, 0, 0, 0, 0,
                                      0, 0, 0, 0, 1, 1, 1, 1,
                                      1, 1, 1, 1, 0, 0, 0, 0,
                                      0, 0, 0, 0, 1, 1, 1, 1,
                                      1, 1, 1, 1, 0, 0, 0, 0,
                                      0, 0, 0, 0, 1, 1, 1, 1};

  int LED_PATTERN_DPC_TOP_4x4[16] = {1, 1, 1, 1,
                                     1, 1, 1, 1,
                                     0, 0, 0, 0,
                                     0, 0, 0, 0};

  int LED_PATTERN_DPC_LEFT_4x4[16] = {1, 1, 0, 0,
                                      0, 0, 1, 1,
                                      1, 1, 0, 0,
                                      0, 0, 1, 1};

  void setup(PINDEF *pi, bool debug)
  {
    // LED Matrix
    pins = pi;
    DEBUG = debug;
    matrix = new Adafruit_NeoPixel(pins->LED_ARRAY_NUM, pins->LED_ARRAY_PIN, NEO_GRB + NEO_KHZ800);
    log_i( "setup matrix is null:%s", boolToChar(matrix == nullptr));
    log_i( "LED_ARRAY_PIN: %i", pins->LED_ARRAY_PIN);
    matrix->begin();
    matrix->setBrightness(255);
    if (!isOn)
      set_all(0, 0, 0);
    else
      set_all(255, 255, 255);
    matrix->show(); //  Update strip to match
  }

  // Custom function accessible by the API
  void act()
  {
    // here you can do something
    log_i( "start parsing json matrix is null:%s", boolToChar(matrix == nullptr));

    if (WifiController::getJDoc()->containsKey(keyLed))
    {
      LedModes LEDArrMode = static_cast<LedModes>((*WifiController::getJDoc())[keyLed][keyLEDArrMode]); // "array", "full", "single", "off", "left", "right", "top", "bottom",
      int NLeds = 0;
      log_i( "LEDArrMode : %i", LEDArrMode);
      if ((*WifiController::getJDoc())[keyLed].containsKey(keyNLeds))
        NLeds = (*WifiController::getJDoc())[keyLed][keyNLeds];
      log_i( "NLeds : %i", NLeds);

      log_i( "containsKey : led_array %s", boolToChar((*WifiController::getJDoc())[keyLed].containsKey(key_led_array)));
      // individual pattern gets adressed
      // PYTHON: send_LEDMatrix_array(self, led_pattern, timeout=1)
      if (LEDArrMode == LedModes::array || LEDArrMode == LedModes::multi)
      {
        for (int i = 0; i < (*WifiController::getJDoc())[keyLed][key_led_array].size(); i++)
        {
          set_led_RGB(
              (*WifiController::getJDoc())[keyLed][key_led_array][i][keyid],
              (*WifiController::getJDoc())[keyLed][key_led_array][i][keyRed],
              (*WifiController::getJDoc())[keyLed][key_led_array][i][keyGreen],
              (*WifiController::getJDoc())[keyLed][key_led_array][i][keyBlue]);
        }
      }
      // only if a single led will be updated, all others stay the same
      // PYTHON: send_LEDMatrix_single(self, indexled=0, intensity=(255,255,255), timeout=1)
      else if (LEDArrMode == LedModes::single)
      {
        set_led_RGB(
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyid],
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyRed],
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyGreen],
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyBlue]);
      }
      // turn on all LEDs
      // PYTHON: send_LEDMatrix_full(self, intensity = (255,255,255),timeout=1)
      else if (LEDArrMode == LedModes::full)
      {
        matrix->clear();
        log_i( "set all start");
        u_int8_t r = (*WifiController::getJDoc())[keyLed][key_led_array][0][keyRed];
        u_int8_t g = (*WifiController::getJDoc())[keyLed][key_led_array][0][keyGreen];
        u_int8_t b = (*WifiController::getJDoc())[keyLed][key_led_array][0][keyBlue];
        isOn = r == 0 && g == 0 && b == 0 ? true : false;
        
        log_i( "rgb %i %i %i", r, g, b);
        set_all(r, g, b);
        log_i( "set all start end");
      }
      // turn off all LEDs
      else if (LEDArrMode == LedModes::left)
      {
        set_left(
            NLeds,
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyRed],
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyGreen],
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyBlue]);
      }
      // turn off all LEDs
      else if (LEDArrMode == LedModes::right)
      {
        set_right(
            NLeds,
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyRed],
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyGreen],
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyBlue]);
      }
      // turn off all LEDs
      else if (LEDArrMode == LedModes::top)
      {
        set_top(
            NLeds,
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyRed],
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyGreen],
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyBlue]);
      }
      // turn off all LEDs
      else if (LEDArrMode == LedModes::bottom)
      {
        set_bottom(
            NLeds,
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyRed],
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyGreen],
            (*WifiController::getJDoc())[keyLed][key_led_array][0][keyBlue]);
      }
      else if (LEDArrMode == LedModes::off)
      {
        matrix->clear();
      }
    }
    else
    {
      log_i( "failed to parse json. required keys are led_array,LEDArrMode");
    }

    WifiController::getJDoc()->clear();
    //(*WifiController::getJDoc())[F("return")] = 1;
    //(*WifiController::getJDoc())[keyLEDArrMode] = LEDArrMode;
    isBusy = false;
  }

  void set()
  {
    WifiController::getJDoc()->clear();
    (*WifiController::getJDoc())[F("return")] = 1;
  }

  // Custom function accessible by the API
  void get()
  {
    WifiController::getJDoc()->clear();
    (*WifiController::getJDoc())[keyNLeds] = pins->LED_ARRAY_NUM;
    (*WifiController::getJDoc())[keyLEDArrMode].add(0);
    (*WifiController::getJDoc())[keyLEDArrMode].add(1);
    (*WifiController::getJDoc())[keyLEDArrMode].add(2);
    (*WifiController::getJDoc())[keyLEDArrMode].add(3);
    (*WifiController::getJDoc())[keyLEDArrMode].add(4);
    (*WifiController::getJDoc())[keyLEDArrMode].add(5);
    (*WifiController::getJDoc())[keyLEDArrMode].add(6);
    (*WifiController::getJDoc())[keyLEDArrMode].add(7);
    (*WifiController::getJDoc())[key_led_isOn] = isOn;
    //(*jsonDocument)[F("LED_ARRAY_PIN")] = pins->LED_ARRAY_PIN;
  }

  /***************************************************************************************************/
  /*******************************************  LED Array  *******************************************/
  /***************************************************************************************************/
  /*******************************FROM OCTOPI ********************************************************/

  void set_led_RGB(u_int8_t iLed, u_int8_t R, u_int8_t G, u_int8_t B)
  {
    matrix->setPixelColor(iLed, matrix->Color(R, G, B)); //  Set pixel's color (in RAM)
  }

  void set_all(u_int8_t R, u_int8_t G, u_int8_t B)
  {
    for (int i = 0; i < matrix->numPixels(); i++)
    {
      set_led_RGB(i, R, G, B);
    }
    matrix->show(); //  Update strip to match
  }

  void set_left(u_int8_t NLed, u_int8_t R, u_int8_t G, u_int8_t B)
  {
    if (NLed == NLED4x4)
    {
      for (int i = 0; i < (NLED4x4); i++)
      {
        set_led_RGB(i, LED_PATTERN_DPC_LEFT_4x4[i] * R, LED_PATTERN_DPC_LEFT_4x4[i] * G, LED_PATTERN_DPC_LEFT_4x4[i] * B);
      }
    }
    if (NLed == NLED8x8)
    {
      for (int i = 0; i < (NLED8x8); i++)
      {
        set_led_RGB(i, LED_PATTERN_DPC_LEFT_8x8[i] * R, LED_PATTERN_DPC_LEFT_8x8[i] * G, LED_PATTERN_DPC_LEFT_8x8[i] * B);
      }
    }
    matrix->show(); //  Update strip to match
  }

  void set_right(u_int8_t NLed, u_int8_t R, u_int8_t G, u_int8_t B)
  {
    if (NLed == NLED4x4)
    {
      for (int i = 0; i < (NLED4x4); i++)
      {
        set_led_RGB(i, (1 - LED_PATTERN_DPC_LEFT_4x4[i]) * R, (1 - LED_PATTERN_DPC_LEFT_4x4[i]) * G, (1 - LED_PATTERN_DPC_LEFT_4x4[i]) * B);
      }
    }
    if (NLed == NLED8x8)
    {
      for (int i = 0; i < (NLED8x8); i++)
      {
        set_led_RGB(i, (1 - LED_PATTERN_DPC_LEFT_8x8[i]) * R, (1 - LED_PATTERN_DPC_LEFT_8x8[i]) * G, (1 - LED_PATTERN_DPC_LEFT_8x8[i]) * B);
      }
    }
    matrix->show(); //  Update strip to match
  }

  void set_top(u_int8_t NLed, u_int8_t R, u_int8_t G, u_int8_t B)
  {
    if (NLed == NLED4x4)
    {
      for (int i = 0; i < (NLED4x4); i++)
      {
        set_led_RGB(i, (LED_PATTERN_DPC_TOP_4x4[i]) * R, (LED_PATTERN_DPC_TOP_4x4[i]) * G, (LED_PATTERN_DPC_TOP_4x4[i]) * B);
      }
    }
    if (NLed == NLED8x8)
    {
      for (int i = 0; i < (NLED8x8); i++)
      {
        set_led_RGB(i, (LED_PATTERN_DPC_TOP_8x8[i]) * R, (LED_PATTERN_DPC_TOP_8x8[i]) * G, (LED_PATTERN_DPC_TOP_8x8[i]) * B);
      }
    }
    matrix->show(); //  Update strip to match
  }

  void set_bottom(u_int8_t NLed, u_int8_t R, u_int8_t G, u_int8_t B)
  {
    if (NLed == NLED4x4)
    {
      for (int i = 0; i < (NLED4x4); i++)
      {
        set_led_RGB(i, (1 - LED_PATTERN_DPC_TOP_4x4[i]) * R, (1 - LED_PATTERN_DPC_TOP_4x4[i]) * G, (1 - LED_PATTERN_DPC_TOP_4x4[i]) * B);
      }
    }
    if (NLed == NLED8x8)
    {
      for (int i = 0; i < (NLED8x8); i++)
      {
        set_led_RGB(i, (1 - LED_PATTERN_DPC_TOP_8x8[i]) * R, (1 - LED_PATTERN_DPC_TOP_8x8[i]) * G, (1 - LED_PATTERN_DPC_TOP_8x8[i]) * B);
      }
    }
    matrix->show(); //  Update strip to match
  }

  void set_center(u_int8_t R, u_int8_t G, u_int8_t B)
  {
    /*
    matrix.fillScreen(matrix.Color(0, 0, 0));
    matrix.drawPixel(4, 4, matrix.Color(R,   G,   B));
    matrix.show();
    */
  }
}
