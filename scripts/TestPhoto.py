# from robopatrol.Media.Camera import Camera

# camera = Camera()
# myphoto = camera.save_photo("myfile.jpg")
# print myphoto

from robopatrol.DAO.PictureDao import PictureDao
from robopatrol.Media.Camera import Camera
from time import time
import base64
import datetime
import cv2

picture = dict()
camera = Camera()

timestamp = time()
timestamp = datetime.datetime.fromtimestamp(timestamp).strftime('%Y%m%d_%H%M%S')

picture["name"] = timestamp + ".jpeg"
# image = cv2.imread("myfile.jpeg", cv2.IMREAD_UNCHANGED)
image = camera.get_photo()
cnt = cv2.imencode('.jpeg', image)[1].tostring()
b64 = base64.b64encode(cnt)
picture["image"] = "data:image/jpeg;base64,"+b64
pictureDao = PictureDao()
print pictureDao.post_picture(picture)
