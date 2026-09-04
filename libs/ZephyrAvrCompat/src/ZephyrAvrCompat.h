// ZephyrAvrCompat — 讓 AVR 時代的 Arduino 函式庫能在 arduino:zephyr 核心上編譯。
//
// 為什麼需要這個：
//   Adafruit_ILI9341 1.6.3 的 Adafruit_ILI9341.cpp:51 無條件 include 了
//   "pins_arduino.h"，接著又 include "wiring_private.h"，兩者只被
//   #ifndef ARDUINO_STM32_FEATHER 包住。arduino:zephyr 核心（UNO Q）
//   兩個標頭都沒有提供，於是編譯直接失敗：
//       fatal error: pins_arduino.h: No such file or directory
//
// 為什麼空的就夠：
//   那兩個標頭會提供的符號（digitalPinToPort / portOutputRegister / SREG …）
//   在 Adafruit 的程式碼裡只出現在 __AVR__ 與 SAMD 的專用路徑，這個目標
//   不會走到。核心的 Arduino.h 本身也已經把 digitalPinToPort 等巨集定義好了。
//   實測整包連結乾淨，證明沒有東西真的需要從這裡拿。
//
// 這個函式庫必須被 sketch 明確 include，否則 arduino-cli 不會把它視為
// 「用到的函式庫」，也就不會把 src/ 加進 include path，Adafruit 的 .cpp
// 仍然找不到標頭。所以 sketch.ino 開頭那行 include 不能拿掉。
//
// 何時可以刪掉這整個目錄：
//   當 Adafruit_ILI9341 上游把那兩個 include 改成有條件（例如加上
//   !defined(ARDUINO_ARCH_ZEPHYR)），或改用其他支援 Zephyr 的顯示驅動時。
#pragma once

#include "pins_arduino.h"
#include "wiring_private.h"
