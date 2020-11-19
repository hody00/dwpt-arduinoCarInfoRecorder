******************** Dynamic Wireless Power Transper Project **********************

The code I wrote in "dwpt_v2" intended to collect data of a moving car.

I used Arduino Mega ADK board, connected to it a gps module (Neo-7M), and a USB thumb drive.

The data I wanted to save was: Latitude (deg), Longitude (deg), Altitude (m), Speed (km/h), CMG (deg), Date, Time, ShuntVoltage (V), Current (A), BattaryVoltage (V), Power (Watt).

One of the struggles I had during this project was to save the data I collected into .csv files inside the USB thumb drive.

In order to do this, I needed to use 2 libraries: USB_Host_Shield_2.0 , UsbFat. (the TinyGPS++ library used for collect gps data)

From those 2 libraries I copied into my project only the relevant .h and .cpp files.

I hope that the next person that would like to save info on USB flash drive using Mega ADK board, would find my project useful.

credit to: greiman for the "UsbFat" library, you can find more information about it in here: https://github.com/greiman/UsbFat .
