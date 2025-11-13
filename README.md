# 🥚TaserDerby Website

## Overview

TaserDerby is a high-stakes, single-player race betting application designed to interface directly with an ESP32-C3 microcontroller connected to a relay module. 

The application runs a simulated microscopic race. If the user's chosen racer **fails to finish within the Top X** (where X is the selected Difficulty), a specific API endpoint (`/lose`) is called on the ESP32. This triggers the relay, administering a momentary "penalty" pulse to a connected load (like a light or a low-voltage device).

<img width="1919" height="992" alt="image" src="https://github.com/user-attachments/assets/9a5bb126-29f2-465a-adb6-dba7af6d6509" />
<img width="1919" height="984" alt="image" src="https://github.com/user-attachments/assets/94e13cbd-f902-4a10-a81e-c77b07da6697" />

---

## 🚀 Features

* **Microscopic Race Simulation:** Real-time, physics-based movement visualized on a canvas. The race starts in the bottom-left and finishes in the top-right quarter-circle goal zone.
* **Difficulty Scaling:** Adjust the required finishing rank (Top 1 to Top 8). The race ends when the number of finished racers equals the selected difficulty level.
* **Minimalist UI:** The entire interface, including betting and results, is handled via modals that appear over the fullscreen canvas for an immersive "microscope" aesthetic.
* **Direct Hardware Integration:** Uses a `fetch` POST request to trigger a physical relay pulse upon losing.

---

## 🛠️ System Components

### Hardware
* **ESP32-C3 Development Board**
* **Relay Module** (5V coil, 3.3V logic)
* **External 5V Power Supply** (For stable, isolated relay coil power)
* **Load** (e.g., LED + 220 Ohm resistor)

### Software / Files
| File | Description | Language |
| :--- | :--- | :--- |
| `DigitalDerbyApp.jsx` | The main React frontend application. Contains the game logic, UI, and API call. **Requires IP update.** | React / JS |
| `wifi_relay_control.cpp` | Arduino sketch for the ESP32. Sets up the Wi-Fi server, handles the `/lose` endpoint, and pulses the relay. | C++ / Arduino |

---

## ⚙️ Setup and Installation

### Step 1: ESP32 Microcontroller Configuration

1.  Open the `wifi_relay_control.cpp` file in the Arduino IDE.
2.  **Update Wi-Fi Credentials:** Change the following lines to match your local Wi-Fi network:
    ```cpp
    const char* ssid = "YOUR_WIFI_SSID";
    const char* password = "YOUR_WIFI_PASSWORD";
    ```
3.  Upload the sketch to your ESP32-C3.
4.  Open the Serial Monitor (115200 baud). Once connected to Wi-Fi, the ESP32 will print its **local IP address**. **Write this down.**

### Step 2: React App Configuration

1.  Open the `DigitalDerbyApp.jsx` file.
2.  **CRITICAL STEP:** Update the IP address placeholder at the top of the file with the IP address printed by the ESP32 in Step 1.

    ```javascript
    // IMPORTANT: Replace with the actual IP address of your ESP32 server
    const ESP32_IP_ADDRESS = 'YOUR_ESP32_IP'; // e.g., '192.168.1.100'
    const API_LOSE_ENDPOINT = `http://${ESP32_IP_ADDRESS}/lose`; 
    ```
    
### Step 3: Running the Web Application

This project was bootstrapped with **Vite** and uses **React** with **Tailwind CSS** classes for styling.

**Prerequisites:** Node.js (v16+) and npm/yarn/pnpm must be installed.

1. **Clone and Navigate:** Clone your repository and change into the project directory.
```
git clone https://github.com/your-username/GoonHacks.git cd GoonHacks
```

2. **Install Dependencies:** Install React, Vite, and Tailwind CSS (which is assumed to be available via configuration).
```
npm install
```
The key packages are 'react' (v18+), 'vite' (v4+), and 'tailwindcss' (v3.3.3+)


3. **Start the Development Server:**
```
npm run dev
```
---

## 🕹️ Usage

1.  **Start the Race:** The app displays the **Betting Modal**.
2.  **Set Difficulty:** Adjust the slider to determine your required finishing rank (e.g., Top 4).
3.  **Place Your Bet:** Click one of the eight colored racer buttons to hide the modal and start the race.
4.  **Race:** The race simulation runs until the number of finishers matches your chosen difficulty level.
5.  **Result:** The **Results Modal** appears.
    * **SAFE:** If your racer finishes within the Top X, the game displays a "SAFE" message.
    * **DEFEAT:** If your racer finishes outside the Top X, the game sends a POST request to the ESP32's `/lose` endpoint, triggering the physical penalty pulse.

---

*Note: If the penalty is not triggering, first check the IP address in `DigitalDerbyApp.jsx` and ensure both your computer running the app and the ESP32 are on the same local Wi-Fi network.*
