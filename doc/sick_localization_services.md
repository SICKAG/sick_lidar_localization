# REST API services

LiDAR-LOC can be configured using a JSON REST API. This API is available using ROS services (on ROS-1 and ROS-2) or commandline tool `gen_service_call` (on all target systems). See the operation manual for details on the commands supported by the JSON REST API.

## gen_service_call

The tool `gen_service_call` sends requests to the localization server via http REST API and returns its response. It supports Linux and Windows, native and ROS.

Usage:
```
gen_service_call <command> <method> <jsondata> [options]
```
with the following commandline options:
```
<command>: name of command (service request), f.e. LocIsSystemReady or LocStart
<method>: GET or POST
<jsondata>: parameter as json string, f.e. {}
--hostname=<ip-address>: ip address of the localization server, default: 192.168.0.1
--verbose=0: silent mode, default: 1 (print response)
--output=<outputfile>: write response to <outputfile>, default: no outputfile
--append=<outputfile>: append response to <outputfile>, default: no outputfile
--help: print usage and options
```

The following table lists supported commands using `gen_service_call` (supported on Linux and Windows, native and ROS):

| **command** | **example** | **description** |
|-------------|-------------|-----------------|
| LocGetErrorLevel | gen_service_call<br/> LocGetErrorLevel POST "\{\}" | Returns the error level of the software |
| LocIsSystemReady | gen_service_call<br/> LocIsSystemReady POST "\{\}" | Checks if the localization controller is booted and ready to process commands |
| LocAutoStartSavePose | gen_service_call<br/> LocAutoStartSavePose POST "\{\}" | Saves the current pose on-request for the automatic start of the application software |
| LocClearMapCache | gen_service_call<br/> LocClearMapCache POST "\{\}" | Removes the cached maps from the RAM of the device |
| LocGetMap | gen_service_call<br/> LocGetMap POST "\{\}" | Returns the current map path relative to maps folder or absolute if it is not in maps folder |
| LocGetSystemState | gen_service_call<br/> LocGetSystemState POST "\{\}" | Get the current system state |
| LocInitializeAtPose | gen_service_call<br/> LocInitializeAtPose POST <br/>"\{\\"data\\": \{\\"x\\":1000, \\"y\\":1000, \\"yaw\\":1000, \\"searchRadius\\":1000\}\}" | Initializes the localization automatically at the given pose by matching against the mapped contours |
| LocLoadMapToCache | gen_service_call<br/> LocLoadMapToCache POST <br/>"\{\\"data\\": \{\\"mapPath\\": \\"test.vmap\\"\}\}" | Loads a map to RAM of the device if not already loaded |
| LocRequestTimestamp | gen_service_call<br/> LocRequestTimestamp POST "\{\}" | Request the current system time of the localization controller and generate a hardware pulse |
| LocResumeAtPose | gen_service_call<br/> LocResumeAtPose POST <br/>"\{\\"data\\": \{\\"x\\":1000, \\"y\\":1000, \\"yaw\\":1000\}\}" | Resets the pose estimate to the given pose |
| LocSaveRingBufferRecording | gen_service_call<br/> LocSaveRingBufferRecording POST <br/>"\{\\"data\\": \{\\"reason\\": \\"pose quality low\\"\}\}" | Saves the most recent data from the continuous ring buffer recording |
| LocSetKinematicVehicle ModelActive | gen_service_call<br/> LocSetKinematicVehicle ModelActive POST "\{\\"data\\": \{\\"active\\":true\}\}" | Activates or deactivates usage of the kinematic model to improve localization |
| LocSetLinesForSupportActive | gen_service_call<br/> LocSetLinesForSupportActive POST <br/>"\{\\"data\\": \{\\"active\\":true\}\}" | Actives / deactivates the use of the magnetic / optical line sensor for localization |
| LocSetMap | gen_service_call<br/> LocSetMap POST <br/>"\{\\"data\\": \{\\"mappath\\":\\"test.vmap\\"\}\}" | Load a map |
| LocSetMappingActive | gen_service_call<br/> LocSetMappingActive POST <br/>"\{\\"data\\": \{\\"active\\":true\}\}" | Activates / deactivates mapping while localizing |
| LocSetOdometryActive | gen_service_call<br/> LocSetOdometryActive POST <br/>"\{\\"data\\": \{\\"active\\":true\}\}" | Enable or disable the support of odometry to enhance the robustness of the contour localization algorithm |
| LocSetRecordingActive | gen_service_call<br/> LocSetRecordingActive POST <br/>"\{\\"data\\": \{\\"active\\":true\}\}" | Starts or stops the recording of sensor data |
| LocSetRingBufferRecordingActive | gen_service_call<br/> LocSetRingBufferRecordingActive POST <br/>"\{\\"data\\": \{\\"active\\":true\}\}" | Enable or disable the continuous recording of data for improved support in critical situations |
| LocStart | gen_service_call<br/> LocStart POST "\{\}" | Start the localization |
| LocStop | gen_service_call<br/> LocStop POST "\{\}" | Stop the localization or the demo mapping and return to IDLE state |
| LocSwitchMap | gen_service_call<br/> LocSwitchMap POST <br/>"\{\\"data\\": \{\\"subMapName\\": \\"test.vmap\\"\}\}" | Transits to the given sub map if close enough to a transition point between the current and the given sub map |
| LocGetLocalizationStatus | gen_service_call<br/> LocGetLocalizationStatus POST "\{\}" | Returns the localization status |
| LocGetSoftwareVersion | gen_service_call<br/> LocGetSoftwareVersion POST "\{\}" | Returns the software version |
| LocLoadPersistentConfig | gen_service_call<br/> LocLoadPersistentConfig POST "\{\}" | Reloads the persistent lidarloc configuration file |

Note: The table above lists the commands in their recommended long format, with "POST" in second and a json string in the third command line argument.
* Argument "POST" is optional. If not specified, the method is determined from the service name.
* Json argument "\{\}" is optional. If no json data given, "\{\}" is assumed.
* The json string can be shorten to ROS-format without quotation or keyword "data". F.e. `./gen_service_call LocInitializeAtPose "{x: 1000, y: 1000, yaw: 1000, searchRadius: 1000}"`

The following table lists supported commands in their shortest form:

| **command** | **example** | **description** |
|-------------|-------------|-----------------|
| LocGetErrorLevel | gen_service_call<br/> LocGetErrorLevel | Returns the error level of the software |
| LocIsSystemReady | gen_service_call<br/> LocIsSystemReady | Checks if the localization controller is booted and ready to process commands |
| LocAutoStartSavePose | gen_service_call<br/> LocAutoStartSavePose | Saves the current pose on-request for the automatic start of the application software |
| LocClearMapCache | gen_service_call<br/> LocClearMapCache | Removes the cached maps from the RAM of the device |
| LocGetMap | gen_service_call<br/> LocGetMap | Returns the current map path relative to maps folder or absolute if it is not in maps folder |
| LocGetSystemState | gen_service_call<br/> LocGetSystemState | Get the current system state |
| LocInitializeAtPose | gen_service_call<br/> LocInitializeAtPose <br/>"\{x: 1000, y: 1000, yaw: 1000, searchRadius: 1000\}" | Initializes the localization automatically at the given pose by matching against the mapped contours |
| LocLoadMapToCache | gen_service_call<br/> LocLoadMapToCache <br/>"\{mapPath: \\"test.vmap\\"\}" | Loads a map to RAM of the device if not already loaded |
| LocRequestTimestamp | gen_service_call<br/> LocRequestTimestamp | Request the current system time of the localization controller and generate a hardware pulse |
| LocResumeAtPose | gen_service_call<br/> LocResumeAtPose <br/>"\{x: 1000, y: 1000, yaw: 1000\}" | Resets the pose estimate to the given pose |
| LocSaveRingBufferRecording | gen_service_call<br/> LocSaveRingBufferRecording <br/>"\{reason: \\"pose quality low\\"\}\}" | Saves the most recent data from the continuous ring buffer recording |
| LocSetKinematicVehicleModelActive | gen_service_call<br/> LocSetKinematicVehicleModelActive <br/>"\{active: true\}" | Activates or deactivates usage of the kinematic model to improve localization |
| LocSetLinesForSupportActive | gen_service_call<br/> LocSetLinesForSupportActive <br/>"\{active: true\}" | Actives / deactivates the use of the magnetic / optical line sensor for localization |
| LocSetMap | gen_service_call<br/> LocSetMap <br/>"\{mappath: \\"test.vmap\\"\}\}" | Load a map |
| LocSetMappingActive | gen_service_call<br/> LocSetMappingActive <br/>"\{active: true\}" | Activates / deactivates mapping while localizing |
| LocSetOdometryActive | gen_service_call<br/> LocSetOdometryActive <br/>"\{active: true\}" | Enable or disable the support of odometry to enhance the robustness of the contour localization algorithm |
| LocSetRecordingActive | gen_service_call<br/> LocSetRecordingActive <br/>"\{active: true\}" | Starts or stops the recording of sensor data |
| LocSetRingBufferRecordingActive | gen_service_call<br/> LocSetRingBufferRecordingActive <br/>"\{active: true\}" | Enable or disable the continuous recording of data for improved support in critical situations |
| LocStart | gen_service_call<br/> LocStart | Start the localization |
| LocStop | gen_service_call<br/> LocStop | Stop the localization or the demo mapping and return to IDLE state |
| LocSwitchMap | gen_service_call<br/> LocSwitchMap <br/>"\{subMapName: \\"test.vmap\\"\}\}" | Transits to the given sub map if close enough to a transition point between the current and the given sub map |
| LocGetLocalizationStatus | gen_service_call<br/> LocGetLocalizationStatus | Returns the localization status |
| LocGetSoftwareVersion | gen_service_call<br/> LocGetSoftwareVersion | Returns the software version |
| LocLoadPersistentConfig | gen_service_call<br/> LocLoadPersistentConfig | Reloads the persistent lidarloc configuration file |


Note: For clarity reasons, the long form with explicit "POST" and json arguments is recommended.

gen_service_call examples:

```
# long format (recommend)
./build/gen_service_call LocIsSystemReady POST "{}" -d=2
./build/gen_service_call LocGetMap POST "{}" -d=2
./build/gen_service_call LocGetSystemState POST "{}" -d=2
./build/gen_service_call LocSetMappingActive POST "{\"data\":{\"active\": true}}" -d=2
./build/gen_service_call LocSetOdometryActive POST "{\"data\":{\"active\": true}}" -d=2
./build/gen_service_call LocStop POST "{}" -d=2
./build/gen_service_call LocStart POST "{}" -d=2
# short form
./build/gen_service_call LocIsSystemReady
./build/gen_service_call LocGetMap
./build/gen_service_call LocGetSystemState
./build/gen_service_call LocSetMappingActive "{active: true}"
./build/gen_service_call LocSetOdometryActive "{active: true}"
./build/gen_service_call LocStop
./build/gen_service_call LocStart
```

Output examples (test against SIM1000FX):
```
./build/gen_service_call LocIsSystemReady POST "{}" -d=2
gen_service_call LocIsSystemReady POST {} --hostname=192.168.0.1 --verbose=1
INFO : curl send http POST request "IsSystemReady", response: "{"header":{"status":0,"message":"Ok"},"data":{"success":true}}"
gen_service_call request: IsSystemReady, response: {"header":{"status":0,"message":"Ok"},"data":{"success":true}}
curl command: 
curl -i -H "Content-Type: application/json" -X POST -d "{}" http://192.168.0.1/api/IsSystemReady

./build/gen_service_call LocGetMap POST "{}" -d=2
gen_service_call LocGetMap POST {} --hostname=192.168.0.1 --verbose=1
INFO : curl send http POST request "LocGetMap", response: "{"header":{"status":0,"message":"Ok"},"data":{"mapPath":"2020-12-21_CNord_40m_v0.vmap"}}"
gen_service_call request: LocGetMap, response: {"header":{"status":0,"message":"Ok"},"data":{"mapPath":"2020-12-21_CNord_40m_v0.vmap"}}
curl command: 
curl -i -H "Content-Type: application/json" -X POST -d "{}" http://192.168.0.1/api/LocGetMap

./build/gen_service_call LocGetSystemState POST "{}" -d=2
gen_service_call LocGetSystemState POST {} --hostname=192.168.0.1 --verbose=1
INFO : curl send http POST request "LocGetSystemState", response: "{"header":{"status":0,"message":"Ok"},"data":{"systemState":"Localization"}}"
gen_service_call request: LocGetSystemState, response: {"header":{"status":0,"message":"Ok"},"data":{"systemState":"Localization"}}
curl command: 
curl -i -H "Content-Type: application/json" -X POST -d "{}" http://192.168.0.1/api/LocGetSystemState

./build/gen_service_call LocSetMappingActive POST "{\"data\":{\"active\":true}}" -d=2
gen_service_call LocSetMappingActive POST {"data":{"active":true}} --hostname=192.168.0.1 --verbose=1
INFO : curl send http POST request "LocSetMappingActive", response: "{"header":{"status":0,"message":"Ok"},"data":{"success":true}}"
gen_service_call request: LocSetMappingActive, response: {"header":{"status":0,"message":"Ok"},"data":{"success":true}}
curl command: 
curl -i -H "Content-Type: application/json" -X POST -d "{\"data\":{\"active\":true}}" http://192.168.0.1/api/LocSetMappingActive

./build/gen_service_call LocSetOdometryActive POST "{\"data\":{\"active\":true}}" -d=2
gen_service_call LocSetOdometryActive POST {"data":{"active":true}} --hostname=192.168.0.1 --verbose=1
INFO : curl send http POST request "LocSetOdometryActive", response: "{"header":{"status":0,"message":"Ok"},"data":{"success":true}}"
gen_service_call request: LocSetOdometryActive, response: {"header":{"status":0,"message":"Ok"},"data":{"success":true}}
curl command: 
curl -i -H "Content-Type: application/json" -X POST -d "{\"data\":{\"active\":true}}" http://192.168.0.1/api/LocSetOdometryActive

./build/gen_service_call LocStop POST "{}" -d=2
gen_service_call LocStop POST {} --hostname=192.168.0.1 --verbose=1
INFO : curl send http POST request "LocStop", response: "{"header":{"status":0,"message":"Ok"},"data":{"success":true}}"
gen_service_call request: LocStop, response: {"header":{"status":0,"message":"Ok"},"data":{"success":true}}
curl command: 
curl -i -H "Content-Type: application/json" -X POST -d "{}" http://192.168.0.1/api/LocStop

./build/gen_service_call LocStart POST "{}" -d=2
gen_service_call LocStart POST {} --hostname=192.168.0.1 --verbose=1
INFO : curl send http POST request "LocStart", response: "{"header":{"status":0,"message":"Ok"},"data":{"success":true}}"
gen_service_call request: LocStart, response: {"header":{"status":0,"message":"Ok"},"data":{"success":true}}
curl command: 
curl -i -H "Content-Type: application/json" -X POST -d "{}" http://192.168.0.1/api/LocStart
```

Note: A REST-API can be served with tool curl, too. `gen_service_call` with option `-d=2` prints the corresponding curl command, f.e.:
```
curl -i -H "Content-Type: application/json" -X POST -d "{}" http://192.168.0.1/api/IsSystemReady
curl -i -H "Content-Type: application/json" -X POST -d "{}" http://192.168.0.1/api/LocGetMap
curl -i -H "Content-Type: application/json" -X POST -d "{}" http://192.168.0.1/api/LocGetSystemState
curl -i -H "Content-Type: application/json" -X POST -d "{\"data\":{\"active\":true}}" http://192.168.0.1/api/LocSetMappingActive
curl -i -H "Content-Type: application/json" -X POST -d "{\"data\":{\"active\":true}}" http://192.168.0.1/api/LocSetOdometryActive
curl -i -H "Content-Type: application/json" -X POST -d "{}" http://192.168.0.1/api/LocStop
curl -i -H "Content-Type: application/json" -X POST -d "{}" http://192.168.0.1/api/LocStart
```

## ROS-1 services

The following table lists the same commands using ROS1 services:

| **command** | **example** |
|-------------|-------------|
| LocGetErrorLevel | rosservice call LocGetErrorLevel "\{\}" |
| LocIsSystemReady | rosservice call LocIsSystemReady "\{\}" |
| LocAutoStartSavePose | rosservice call LocAutoStartSavePose "\{\}" |
| LocClearMapCache | rosservice call LocClearMapCache "\{\}" |
| LocGetMap | rosservice call LocGetMap "\{\}" |
| LocGetSystemState | rosservice call LocGetSystemState "\{\}" |
| LocInitializeAtPose | rosservice call LocInitializeAtPose <br/>"\{x: 1000, y: 1000, yaw: 1000, searchradius: 1000\}" |
| LocLoadMapToCache | rosservice call LocLoadMapToCache <br/>"\{mappath: \\"test.vmap\\"\}" |
| LocRequestTimestamp | rosservice call LocRequestTimestamp "\{\}" |
| LocResumeAtPose | rosservice call LocResumeAtPose <br/>"\{x: 1000, y: 1000, yaw: 1000\}" |
| LocSaveRingBufferRecording | rosservice call LocSaveRingBufferRecording <br/>"\{reason: \\"pose quality low\\"\}" |
| LocSetKinematicVehicle ModelActive | rosservice call LocSetKinematicVehicle ModelActive "\{active: true\}" |
| LocSetLinesForSupportActive | rosservice call LocSetLinesForSupportActive <br/>"\{active: true\}" |
| LocSetMap | rosservice call LocSetMap <br/>"\{mappath: \\"test.vmap\\"\}" |
| LocSetMappingActive | rosservice call LocSetMappingActive <br/>"\{active: true\}" |
| LocSetOdometryActive | rosservice call LocSetOdometryActive <br/>"\{active: true\}" |
| LocSetRecordingActive | rosservice call LocSetRecordingActive <br/>"\{active: true\}" |
| LocSetRingBufferRecordingActive | rosservice call LocSetRingBufferRecordingActive <br/>"\{active: true\}" |
| LocStart | rosservice call LocStart "\{\}" |
| LocStop | rosservice call LocStop "\{\}" |
| LocSwitchMap | rosservice call LocSwitchMap <br/>"\{submapname: \\"test.vmap\\"\}" |
| LocGetLocalizationStatus | rosservice call LocGetLocalizationStatus "\{\}" |
| LocGetSoftwareVersion | rosservice call LocGetSoftwareVersion "\{\}" |
| LocLoadPersistentConfig | rosservice call LocLoadPersistentConfig "\{\}" |

ROS-1 examples:
```
source ./install/setup.bash 
roslaunch sick_lidar_localization sick_lidar_localization.launch &
rosservice call LocIsSystemReady "{}"
rosservice call LocGetMap "{}"
rosservice call LocGetSystemState "{}"
rosservice call LocSetMappingActive "{active: true}"
rosservice call LocSetOdometryActive "{active: true}"
rosservice call LocGetSystemState "{}"
rosservice call LocStop "{}"
rosservice call LocStart "{}"
```

Output examples (test examples against SIM1000FX):
```
source ./install/setup.bash 
roslaunch sick_lidar_localization sick_lidar_localization.launch &
[ INFO] [1626875179.228541385]: sick_lidar_localization hostname:=192.168.0.1 serverpath:=api verbose:=0 started.
[ INFO] [1626875179.236288641]: SiclLocIsSystemReady: 1
[ INFO] [1626875293.929300215]: SickServices::serviceCb("LocIsSystemReady", "POST", "{}"): success=1
success: True
rosservice call LocGetMap "{}"
[ INFO] [1626875294.835960400]: SickServices::serviceCb("LocGetMap", "POST", "{}"): mapPath="2020-12-21_CNord_40m_v0.vmap"
mappath: "2020-12-21_CNord_40m_v0.vmap"
success: True
rosservice call LocGetSystemState "{}"
[ INFO] [1626875295.768499718]: SickServices::serviceCb("LocGetSystemState", "POST", "{}"): systemState="Localization"
systemstate: "Localization"
success: True
rosservice call LocSetMappingActive "{active: true}"
[ INFO] [1626875296.680080532]: SickServices::serviceCb("LocSetMappingActive", "POST", "{"data":{"active":true}}"): success=1
success: True
rosservice call LocSetOdometryActive "{active: true}"
[ INFO] [1626875297.605719365]: SickServices::serviceCb("LocSetOdometryActive", "POST", "{"data":{"active":true}}"): success=1
success: True
rosservice call LocGetSystemState "{}"
[ INFO] [1626875299.437070523]: SickServices::serviceCb("LocGetSystemState", "POST", "{}"): systemState="Localization"
systemstate: "Localization"
success: True
rosservice call LocStop "{}"
[ INFO] [1626875300.376469527]: SickServices::serviceCb("LocStop", "POST", "{}"): success=1
success: True
rosservice call LocStart "{}"
[ INFO] [1626875302.384810233]: SickServices::serviceCb("LocStart", "POST", "{}"): success=1
success: True
```

## ROS-2 services

The following table lists the same commands using ROS2 services:

| **command** | **example** |
|-------------|-------------|
| LocGetErrorLevel | ros2 service call LocGetErrorLevel sick_lidar_localization/srv/LocGetErrorLevelSrv "\{\}" |
| LocIsSystemReady | ros2 service call LocIsSystemReady sick_lidar_localization/srv/LocIsSystemReadySrv "\{\}" |
| LocAutoStartSavePose | ros2 service call LocAutoStartSavePose sick_lidar_localization/srv/LocAutoStartSavePoseSrv "\{\}" |
| LocClearMapCache | ros2 service call LocClearMapCache sick_lidar_localization/srv/LocClearMapCacheSrv "\{\}" |
| LocGetMap | ros2 service call LocGetMap sick_lidar_localization/srv/LocGetMapSrv "\{\}" |
| LocGetSystemState | ros2 service call LocGetSystemState sick_lidar_localization/srv/LocGetSystemStateSrv "\{\}" |
| LocInitializeAtPose | ros2 service call LocInitializeAtPose sick_lidar_localization/srv/LocInitializeAtPoseSrv <br/>"\{x: 1000, y: 1000, yaw: 1000, searchradius: 1000\}" |
| LocLoadMapToCache | ros2 service call LocLoadMapToCache sick_lidar_localization/srv/LocLoadMapToCacheSrv <br/>"\{mappath: \\"test.vmap\\"\}" |
| LocRequestTimestamp | ros2 service call LocRequestTimestamp sick_lidar_localization/srv/LocRequestTimestampSrv "\{\}" |
| LocResumeAtPose | ros2 service call LocResumeAtPose sick_lidar_localization/srv/LocResumeAtPoseSrv <br/>"\{x: 1000, y: 1000, yaw: 1000\}" |
| LocSaveRingBufferRecording | ros2 service call LocSaveRingBufferRecording sick_lidar_localization/srv/LocSaveRingBufferRecordingSrv <br/>"\{reason: \\"pose quality low\\"\}" |
| LocSetKinematicVehicle ModelActive | ros2 service call LocSetKinematicVehicle ModelActive sick_lidar_localization/srv/LocSetKinematicVehicleModelActiveSrv "\{active: true\}" |
| LocSetLinesForSupportActive | ros2 service call LocSetLinesForSupportActive sick_lidar_localization/srv/LocSetLinesForSupportActiveSrv <br/>"\{active: true\}" |
| LocSetMap | ros2 service call LocSetMap sick_lidar_localization/srv/LocSetMapSrv <br/>"\{mappath: \\"test.vmap\\"\}" |
| LocSetMappingActive | ros2 service call LocSetMappingActive sick_lidar_localization/srv/LocSetMappingActiveSrv <br/>"\{active: true\}" |
| LocSetOdometryActive | ros2 service call LocSetOdometryActive sick_lidar_localization/srv/LocSetOdometryActiveSrv <br/>"\{active: true\}" |
| LocSetRecordingActive | ros2 service call LocSetRecordingActive sick_lidar_localization/srv/LocSetRecordingActiveSrv <br/>"\{active: true\}" |
| LocSetRingBufferRecordingActive | ros2 service call LocSetRingBufferRecordingActive sick_lidar_localization/srv/LocSetRingBufferRecordingActiveSrv <br/>"\{active: true\}" |
| LocStart | ros2 service call LocStart sick_lidar_localization/srv/LocStartSrv "\{\}" |
| LocStop | ros2 service call LocStop sick_lidar_localization/srv/LocStopSrv "\{\}" |
| LocSwitchMap | ros2 service call LocSwitchMap sick_lidar_localization/srv/LocSwitchMapSrv <br/>"\{submapname: \\"test.vmap\\"\}" |
| LocGetLocalizationStatus | ros2 service call LocGetLocalizationStatus sick_lidar_localization/srv/LocGetLocalizationStatusSrv "\{\}" |
| LocGetSoftwareVersion | ros2 service call LocGetSoftwareVersion sick_lidar_localization/srv/LocGetSoftwareVersionSrv "\{\}" |
| LocLoadPersistentConfig | ros2 service call LocLoadPersistentConfig sick_lidar_localization/srv/LocLoadPersistentConfigSrv "\{\}" |

ROS-2 examples:
```
source ./install/setup.bash 
ros2 run sick_lidar_localization sick_lidar_localization ./src/sick_lidar_localization/launch/sick_lidar_localization.launch &
ros2 service call LocIsSystemReady sick_lidar_localization/srv/LocIsSystemReadySrv "{}"
ros2 service call LocGetMap sick_lidar_localization/srv/LocGetMapSrv "{}"
ros2 service call LocGetSystemState sick_lidar_localization/srv/LocGetSystemStateSrv "{}"
ros2 service call LocSetMappingActive sick_lidar_localization/srv/LocSetMappingActiveSrv "{active: true}"
ros2 service call LocSetOdometryActive sick_lidar_localization/srv/LocSetOdometryActiveSrv "{active: true}"
ros2 service call LocStop sick_lidar_localization/srv/LocStopSrv "{}"
ros2 service call LocStart sick_lidar_localization/srv/LocStartSrv "{}"
```

Output examples (test examples against SIM1000FX):
```
source ./install/setup.bash 
ros2 run sick_lidar_localization sick_lidar_localization ./src/sick_lidar_localization/launch/sick_lidar_localization.launch &
[INFO] [1626874790.998263796] [sick_lidar_localization]: sick_lidar_localization hostname:=192.168.0.1 serverpath:=api verbose:=0 started.
[INFO] [1626874791.008931314] [sick_lidar_localization]: LocIsSystemReady: 1
ros2 service call LocIsSystemReady sick_lidar_localization/srv/LocIsSystemReadySrv "{}"
[INFO] [1626874807.690028213] [sick_lidar_localization]: SickServices::serviceCb("IsSystemReady", "POST", "{}"): success=1
response: sick_lidar_localization.srv.LocIsSystemReadySrv_Response(success=True)
ros2 service call LocGetMap sick_lidar_localization/srv/LocGetMapSrv "{}"
[INFO] [1626874808.446064372] [sick_lidar_localization]: SickServices::serviceCb("LocGetMap", "POST", "{}"): mapPath="2020-12-21_CNord_40m_v0.vmap"
response: sick_lidar_localization.srv.LocGetMapSrv_Response(mappath='2020-12-21_CNord_40m_v0.vmap', success=True)
ros2 service call LocGetSystemState sick_lidar_localization/srv/LocGetSystemStateSrv "{}"
[INFO] [1626874809.237483802] [sick_lidar_localization]: SickServices::serviceCb("LocGetSystemState", "POST", "{}"): systemState="Localization"
response: sick_lidar_localization.srv.LocGetSystemStateSrv_Response(systemstate='Localization', success=True)
ros2 service call LocSetMappingActive sick_lidar_localization/srv/LocSetMappingActiveSrv "{active: true}"
[INFO] [1626874809.783162484] [sick_lidar_localization]: SickServices::serviceCb("LocSetMappingActive", "POST", "{"data":{"active":true}}"): success=1
response: sick_lidar_localization.srv.LocSetMappingActiveSrv_Response(success=True)
ros2 service call LocSetOdometryActive sick_lidar_localization/srv/LocSetOdometryActiveSrv "{active: true}"
[INFO] [1626874810.621453095] [sick_lidar_localization]: SickServices::serviceCb("LocSetOdometryActive", "POST", "{"data":{"active":true}}"): success=1
response: sick_lidar_localization.srv.LocSetOdometryActiveSrv_Response(success=True)
ros2 service call LocStop sick_lidar_localization/srv/LocStopSrv "{}"
[INFO] [1626874812.001814744] [sick_lidar_localization]: SickServices::serviceCb("LocStop", "POST", "{}"): success=1
response: sick_lidar_localization.srv.LocStopSrv_Response(success=True)
ros2 service call LocStart sick_lidar_localization/srv/LocStartSrv "{}"
[INFO] [1626874813.582692773] [sick_lidar_localization]: SickServices::serviceCb("LocStart", "POST", "{}"): success=1
response: sick_lidar_localization.srv.LocStartSrv_Response(success=True)
```
