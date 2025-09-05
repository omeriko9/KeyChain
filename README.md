
<img src="https://github.com/user-attachments/assets/b5a5049d-5f7d-4604-a6b0-9eb1aa825fb7" width="300" alt="bigone">
<img src="https://github.com/user-attachments/assets/28077eb6-7637-490f-af5f-6888c6834309" width="300" alt="IMG_8065">
<img src="https://github.com/user-attachments/assets/42d94e75-f32e-4476-a4ad-b457da8ffe35" width="300" alt="IMG_8069">


# KeyChain — ESP32‑C3 Mini TFT Image Slideshow

This project turns a tiny ESP32‑C3 board with a small SPI TFT into a simple image slideshow. It targets pocket “trinket” displays like:

- Spotpear ESP32‑C3 Desktop Trinket (1.44" ST7735): https://spotpear.com/wiki/ESP32-C3-desktop-trinket-Mini-TV-Portable-Pendant-LVGL-1.44inch-LCD-ST7735.html
- Waveshare ESP32‑C3‑LCD‑0.71 (0.71" GC9D01): https://www.waveshare.com/esp32-c3-lcd-0.71.htm

Images are stored on SPIFFS and displayed full‑screen in a loop. A built‑in web page lets you upload images and set how long each image stays on screen.

## Board selection (menuconfig)

Choose your device in menuconfig so the correct pins, driver, and resolution are used:

- menuconfig → Board Selection →
    - TFT1.44 for ST7735‑based 1.44" modules (e.g., Spotpear mini TV)
    - TFT0.71 for GC9D01‑based 0.71" modules (e.g., Waveshare 0.71)

These map to `CONFIG_BOARD_TFT144` and `CONFIG_BOARD_TFT071` and configure SPI pins, LCD resolution, and driver.

## Web interface (upload and settings)

- After the device joins your Wi‑Fi, browse to: `http://[ESP IP]:8080/`
- The page shows the current images, lets you upload new ones, and change “seconds per image”.
- Settings are applied immediately and persisted in NVS.

Notes:
- If no known Wi‑Fi is saved, the device opens a setup AP named `ESP32-Setup`. Connect to it and you’ll be redirected to a simple portal to save your Wi‑Fi credentials.
- Uploaded files are stored under `/spiffs/img`. JPEG is recommended; very large images are skipped to avoid memory issues.

## Build and flash (ESP‑IDF)

Typical flow with ESP‑IDF:
1) `idf.py set-target esp32c3`
2) `idf.py menuconfig` → select your board under Board Selection
3) `idf.py build flash monitor`

Once the device prints its IP (or after provisioning via `ESP32-Setup`), open the web UI at `http://[ESP IP]:8080/`.
