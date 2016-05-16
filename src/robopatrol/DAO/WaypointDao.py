import requests
from requests.exceptions import ConnectionError
import json
from jsonschema import validate, ValidationError


class WaypointDao:

    schema_waypoint = {
        "properties": {
            "id": {"type": "string"},
            "name": {"type": "string"},
            "x": {"type": "number"},
            "y": {"type": "number"},
            "lastVisited": {"type": "date-time"}
        },
        "required": ["id", "x", "y"]
    }

    configURL = 'http://localhost:9998/route'

    def __init__(self):
        pass

    def get_waypoints(self):
        try:
            myResponse = requests.get(self.configURL)
            if myResponse.ok:
                JData = json.loads(myResponse.content)
                #Validate and remove invalid
                # for waypoint in JData:
                #     try:
                #         print('validating')
                #         validate(waypoint, self.schema_waypoint)
                #         print('validated')
                #     except ValidationError as e:
                #         print 'there was a validation error {0}'.format(e.message)
                #         JData.remove(waypoint)
            else:
                print('response was not ok')
                JData = None
        except (ValueError, ConnectionError):
            print('there was an error')
            JData = None
        print('returning data')
        return JData

    def post_job(self, waypoint):
        headers = {'Content-type': 'application/json'}
        response = None
        try:
            response = requests.post(self.configURL, data=json.dumps(waypoint), headers=headers)
        except ConnectionError:
            response = False
        return response
