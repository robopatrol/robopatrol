from TakePhoto import TakePhoto
import os
import cv2
import rospy

class Camera:

    def __init__(self):
        self.takePhoto = TakePhoto()

    def _get_photo_sequence(self):
        #TODO: PhotoSequencing by reading database
        return 123

    def save_photo(self, photopath):
        img_title = self._get_photo_sequence()
        image = self.get_photo()
        cv2.imwrite(photopath, image)

    def get_photo(self):
        return self.takePhoto.take_picture()
