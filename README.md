
BNNB: Bluetooth Near is Not Blueproximity
--

A Small script to lock/unlock (or any other action if you wish)
Had a lot of issues with blueproximity so I started to search for alternatives and finally wrote this.
The main idea is to have a lock/unlock within 1 meter like some proprietary software like NearLock (https://nearlock.me/) can do.
The main alternative for linux is blueproximity (https://sourceforge.net/projects/blueproximity/) that use RSSI as a distance.

This is the result of documenting myself on different techniques and implementation to get distance from RSSI, 

It's a very crude script working for now only with xscreensaver (I like GLmatrix screensaver ...)
Even if  it's not recommended to kill xscreensaver ... it's the only way I found to quickly unlock xscreensaver without messing with pam ... 

## Usage

### bluetooth-near.py

This is a simple script that take MAC address as input 
Use `Ctrl + C` to exit the script or kill it's process.

```
./bluetooth-near.py <bluetooth-address>
```