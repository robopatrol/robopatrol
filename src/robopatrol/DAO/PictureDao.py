import requests
from requests.exceptions import ConnectionError
import json
from jsonschema import validate, ValidationError


class PictureDao:

    configURL = 'http://localhost:9998/pictures'

    def __init__(self):
        pass

    def post_picture(self, picture):
        headers = {'Content-type': 'application/json'}
        response = None

        try:
            print json.dumps(picture)
            response = requests.post(self.configURL, data=json.dumps(picture), headers=headers)
        except ConnectionError:
            response = False

        return response





























































































