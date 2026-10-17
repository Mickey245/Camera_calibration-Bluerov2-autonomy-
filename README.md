# Calibration Setup

## Applications Needed
- OBS on your laptop: https://obsproject.com/download
- DroidCam on your iPhone
- DroidCam extension for OBS Studio: https://droidcam.app/
- Print checkerboard: https://markhedleyjones.com/projects/calibration-checkerboard-collection

**Default checkerboard:** 9 columns, 7 rows, and 25 mm squares, as shown below.

![Checkerboard](image/checkerboard_default.png)

## Steps

### 1. Open OBS
Open OBS, add a new Source, and select **DroidCam**. You can name the new source anything.

### 2. Set the Resolution
Double-click the new Source and make sure the resolution is **1920 × 1080**.

### 3. Connect Your iPhone
Open DroidCam on your iPhone. Enter the network IP shown on the iPhone into the **WiFi IP** section in OBS.

### 4. Start Virtual Camera
Once connected, click **Start Virtual Camera** in OBS.

### 5. Edit the Calibration Script
Edit the `calibrate_iphone.sh` code in the Calibration folder shared on GitHub.
- Add your phone profile, similar to how it is done for `iphone14`. You can use one of the other profiles if it fits your phone and lens.
- If you use a different checkerboard for calibration, update the code based on the board used.

### 6. Run the Calibration Script
Under the Calibration folder, run:

```bash
./usb_cam/calibrate_iphone.sh iphone14
