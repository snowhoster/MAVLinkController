// Shim：arduino:zephyr 核心未提供此標頭。詳見 ZephyrAvrCompat.h 的說明。
// 刻意留空 —— 核心的 Arduino.h 已定義 digitalPinToPort / portOutputRegister
// 等相容巨集，而真正 AVR 專屬的內容只在不會走到的 __AVR__ 路徑被使用。
#pragma once
