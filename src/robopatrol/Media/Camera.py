from TakePhoto import TakePhoto
import os
import cv2


class Camera:

    def __init__(self):
        self.takePhoto = TakePhoto()

    def _get_photo_sequence(self):
        return 123

    def save_photo(self, photopath):
        if os.path.exists(photopath):
            img_title = self._get_photo_sequence()
            cv2.imwrite(img_title, self.image)
