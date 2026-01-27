App code goes here
# RoboMow Control App - Testing Package

## What's Included
- **RoboMow.apk** - Android app installer
- **RoboMowTestServer/** - Simulated mower backend for testing

## Prerequisites
- Android phone
- Computer with Node.js installed (download from https://nodejs.org)
- Both on same network OR use mobile hotspot

---

## Installation & Setup

### Step 1: Install the App on Your Phone

1. Transfer **RoboMow.apk** to your Android phone
2. Open the APK file
3. Enable "Install from Unknown Sources" if prompted
4. Install the app

### Step 2: Set Up the Test Server on Your Computer

1. Extract the **RoboMowTestServer** folder to your computer
2. Open PowerShell (or Terminal) in that folder
3. Run these commands:
```powershell
   npm install
   node test-server.js
```
4. You should see: "Test server running on port 3000"
5. **Keep this window open** - the server must stay running

---

## Connecting the App to Test Server

### METHOD 1: Mobile Hotspot (RECOMMENDED - Always Works)

**On your computer:**
1. Press `Win + I` to open Settings
2. Go to **Network & Internet** → **Mobile hotspot**
3. Turn it **ON**
4. Note the network name and password

**On your phone:**
1. Disconnect from current WiFi
2. Connect to your computer's hotspot
3. Open the **RoboMow** app
4. Enter IP address: `192.168.137.1:3000`
5. Tap **Connect**

You should see the mower control screen! ✅

---

### METHOD 2: Same WiFi Network (May Not Work on All Routers)

**Find your computer's IP address:**
```powershell
ipconfig
```
Look for "IPv4 Address" under WiFi (e.g., `192.168.0.104`)

**Connect:**
1. Ensure phone and computer are on the SAME WiFi network
2. In the RoboMow app, enter: `YOUR_IP_ADDRESS:3000`
   - Example: `192.168.0.104:3000`
3. Tap **Connect**

**If this doesn't work:** Use METHOD 1 (Mobile Hotspot) instead.

## Testing the Connection

### Quick Browser Test:
Before using the app, verify the server is accessible:

**On your phone's browser**, go to:
- Mobile Hotspot: `http://192.168.137.1:3000/api/status`
- Same WiFi: `http://YOUR_IP_ADDRESS:3000/api/status`

You should see JSON data like:
```json
{"isMowing":true,"batteryLevel":75,...}
```
## Using the App

Once connected, you can:
- ⏹ **Emergency Stop** - Immediately stop the mower
- ⏸ **Pause/Resume** - Control mowing operation
- 📊 **Monitor** - Battery level, progress, distance, runtime
- 📤 **Upload Patterns** - Custom mowing patterns

The test server simulates a real mower, so you can see all features in action!

---
## Technical Notes

**Test Server Details:**
- Simulates a robotic lawnmower with realistic data
- Updates progress, battery, and stats automatically
- Responds to commands (pause, resume, emergency stop)
- Runs on port 3000

**Network Requirements:**
- Mobile Hotspot: Creates direct connection (always works)
- Same WiFi: Requires router to allow device-to-device communication


Happy Testing! 🚀
