# sick_rest_server.py implements a simple rest server emulating the responses 
# of a localization server. It just receives http get requests and
# sends some predefined responses to the client.
# To be used for debugging and offline testing only!

# Sources and links:
# https://stackoverflow.com/questions/60537557/how-to-make-a-simple-python-rest-server-and-client
# https://blog.miguelgrinberg.com/post/designing-a-restful-api-with-python-and-flask

import argparse
import time
from flask import Flask, jsonify, request

app = Flask(__name__)

# Global options
time_sync = 0
timestamp_ms_start = int(time.time() * 1000000)

# definition of response header
std_header_ok           = { "status": 0, "message": "Ok" }
std_header_invalid_data = { "status": 2, "message": "Invalid data given for variable" }
std_header_not_found    = { "status": 5, "message": "Variable or method could not be found" }
response_invalid_data   = { "header": std_header_invalid_data, "data": { "success": False, "message": "invalid data, json request expected" } }
response_not_found      = { "header": std_header_not_found, "data": { "success": False, "message": "service request unknown or not supported" } }

# definition of response message (header and data)
responses = {
        "GetErrorLevel"                               : { "header": std_header_ok, "data": { "level": 0, "description": "No error" } },
        "IsSystemReady"                               : { "header": std_header_ok, "data": { "success": True } },
        "LocAutoStartSavePose"                        : { "header": std_header_ok, "data": { "success": True } },
        "LocClearMapCache"                            : { "header": std_header_ok, "data": { "success": True } },
        "LocGetMap"                                   : { "header": std_header_ok, "data": { "mapPath": "test.vmap" } },
        "LocGetSystemState"                           : { "header": std_header_ok, "data": { "systemState": "LOCALIZING" } },
        "LocInitializeAtPose"                         : { "header": std_header_ok, "data": { "success": True } },
        "LocLoadMapToCache"                           : { "header": std_header_ok, "data": { "success": True } },
        "LocRequestTimestamp"                         : { "header": std_header_ok, "data": { "timestamp": 620842067 } },
        "LocResumeAtPose"                             : { "header": std_header_ok, "data": { "success": True } },
        "LocSaveRingBufferRecording"                  : { "header": std_header_ok, "data": { "success": True } },
        "LocSetKinematicVehicleModelActive"           : { "header": std_header_ok, "data": { "success": True } },
        "LocSetLinesForSupportActive"                 : { "header": std_header_ok, "data": { "success": True } },
        "LocSetMap"                                   : { "header": std_header_ok, "data": { "success": True } },
        "LocSetMappingActive"                         : { "header": std_header_ok, "data": { "success": True } },
        "LocSetOdometryActive"                        : { "header": std_header_ok, "data": { "success": True } },
        "LocSetRecordingActive"                       : { "header": std_header_ok, "data": { "success": True } },
        "LocSetRingBufferRecordingActive"             : { "header": std_header_ok, "data": { "success": True } },
        "LocStartLocalizing"                          : { "header": std_header_ok, "data": { "success": True } },
        "LocStop"                                     : { "header": std_header_ok, "data": { "success": True } },
        "LocSwitchMap"                                : { "header": std_header_ok, "data": { "success": True } },
        "LocGetLocalizationStatus"                    : { "header": std_header_ok, "data": { "locStatus": 1, "details": "LOCALIZING" } },
        "GetSoftwareVersion"                          : { "header": std_header_ok, "data": { "version": "LLS 2.0.0.14R" } },
        "LoadPersistentConfig"                        : { "header": std_header_ok, "data": { "success": True, "response": "config loaded" } },
        "SavePermanent"                               : { "header": std_header_ok, "data": { "success": True } }
}

@app.route("/api/<string:servicename>", methods=["GET", "POST"])

# callback after http get or post request to "http://localhost/api/<servicename>",
# returns a json response depending on the request
def get_api(servicename):
    # Update timestamp
    if time_sync > 0:
        timestamp_ms = int(time.time() * 1000000) - timestamp_ms_start + 1000000
        responses["LocRequestTimestamp"] = { "header": std_header_ok, "data": { "timestamp": timestamp_ms } }
    # Get request data in case of a http post request
    if request.json is None: # http get request
        post_request = False;
        post_json_data = "";
    else: # http post request
        post_request = True;
        post_json_data = request.json;
    # Create response
    response = responses.get(servicename)
    if response is None:
        response = response_not_found
    # Debugging
    if post_request:
        print("sick_rest_server.py POST request: servicename: \"{}\", post_json_data: \"{}\", response: \"{}\"".format(servicename, post_json_data, response))
    else:
        print("sick_rest_server.py GET request: servicename: \"{}\", response: \"{}\"".format(servicename, response))        
    # Return response
    return jsonify(response)

if __name__ == "__main__":
    
    # Command line args
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--time_sync", help="generate LocRequestTimestamps in microseconds", default=time_sync, type=int)
    cli_args = arg_parser.parse_args()
    time_sync = cli_args.time_sync
    print("sick_rest_server --time_sync={} starting ...".format(time_sync))
    
    # Run rest api server
    app.run(debug=True, port=80)
    