## TODO

- Add listeners to update IMU/GPS data
- Describe how this app was built
- Are our topics now compatible with ROS? See if there is some demo app that publishes IMU/GPS topics. Might be issues
-	that the IDL files are not exactly like the String.idl we created for previous demo app

## How it was built

- Install FastDDS (TODO: Link)
- Generate type files based on IDL with fastddsgen
-	Before generating: make sure to comment out the major (TODO: better description of what) @verbatim blocks
-	Command: fastddsgen -d c:\temp\fastdds-remove sensor_msgs\msg\NavSatFix.idl -I . -typeros2 -de final

## Notes

- To be ROS2 compatible we add "rt/" before the topic name (TODO: verify that this is actually required)
- Asking ROS2 to publish a topic of a certain kind is useful when debuggin. Using this command will publish a topic of kind NavSatFix continuously:
	- ros2 topic pub /fix sensor_msgs/msg/NavSatFix "{}"
- Using eProsima Fast DDS Monitor is useful for seeing if publisher and subscriber actually match. TODO: expand this note. Write that QoS also matters, how to fix etc