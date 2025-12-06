## TODO

- Add listeners to update IMU/GPS data
- Describe how this app was build
- Are our topics now compatible with ROS? See if there is some demo app that publishes IMU/GPS topics. Might be issues
-	that the IDL files are not exactly like the String.idl we created for previous demo app

## How it was built

- Install FastDDS (TODO: Link)
- Generate type files based on IDL with fastddsgen
-	Before generating: make sure to comment out the major (TODO: better description of what) @verbatim blocks
-	Command: fastddsgen -d c:\temp\fastdds-remove sensor_msgs\msg\NavSatFix.idl -I . -typeros2 -de final