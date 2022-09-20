# Vulintus_Board_Definitions

This repository contains Vulintus board definition files for the Arduino IDE.

To install these board definitions, take the following steps:

1. Open the Arduino IDE. Then select File -> Preferences.
2. In the preferences window, find the "Additional Board Manager URLs" near the bottom of the window. Click the icon next to it to pop out a new window that shows all board manager URLs.
3. On a new line in that window, add the following URL: "https://raw.githubusercontent.com/Vulintus/Vulintus_Board_Definitions/main/package_vulintus_index.json" (without quotes)
4. Press OK to close the window.
5. Press OK to close the preferences window.
6. In the Tools menu, select Board -> Boards Manager. The "Boards Manager" window will display on your screen.
7. In the list of boards, search for "Vulintus". The search results should show both "Vulintus SAMD Devices" and "Vulintus AVR Devices".
8. Based on your needs, select and install either the "Vulintus SAMD Devices" boards, "Vulintus AVR Devices" boards, or both.
9. Close the "Boards Manager" window.
10. In the Tools menu, select Board. In the list of boards that show up, you should now see the Vulintus boards. If they do not appear, try restarting the Arduino IDE.
