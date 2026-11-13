# PX4 SD card configs

PX4 runs `etc/extras.txt` from the SD card at boot. Here it starts the MAVLink instance for the onboard computer and sets the stream rates that MAVROS and the MRS HW API need.

To use one, copy the board's `etc` folder to the root of the SD card.

The instance runs in the `custom` MAVLink mode, so PX4 sends only the streams listed in `extras.txt` (plus STATUSTEXT and command messages), the same on every PX4 version. A MAVROS plugin that needs another message needs its stream added here.
