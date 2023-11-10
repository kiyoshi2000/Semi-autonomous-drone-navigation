# coding: utf-8

# This file is opencv_video_tools.py

import cv2
import numpy as np
import copy

class VideoProcessor:
    def __init__(self, source=0, frame_wait = 1, display_name = 'display'):
        """
        source = 0 : the source is the webcam
        source = <filename> : the source is a video file.

        frame_wait = nb of miliseconds between end of computation and next frame. 
        """
        self.frame_wait = frame_wait
        self._frame = None
        self.wait_time = frame_wait # actual wait between 2 frames... it is set to 0 in step by step mode.
        self.step_by_step = False
        self.display_name = display_name    
        self.video = cv2.VideoCapture(source)                   # We connect to the video device
        cv2.namedWindow(self.display_name, cv2.WINDOW_AUTOSIZE) # We create a GUI window named self.display_name

        print()
        print()
        print('Commands:')
        print('-------- ')
        print('    ESC or q : quit.')
        print('    <space>  : step-by-step mode, show next frame.')
        print('    <any>    : quit step-by-step mode.')
        print()

    def _refresh_if_needed(self, cb):
        """
        For internal use. This "decorates" a callback so that it refreshes
        the image when the value changes. With this, you can stall on
        an image, and adjust parameters from the slider. The image is
        then recomputed from the current frame, and displayed.

        """
        def callback(value):
            cb(value)
            res = self._on_image(copy.deepcopy(self._frame))
            if res is not None:
                cv2.imshow(self.display_name, res)
                cv2.waitKey(1)
        return callback
            

    def close(self):
        self.video.release()    # We disconnect properly from the video device
        cv2.destroyAllWindows() # We realease cv2 GUI stuff.

    def __iadd__(self, arg):
        """
        This adds a trackbar
        """
        label, init, upper, callback = arg
        cv2.createTrackbar(label, self.display_name, init, upper, self._refresh_if_needed(callback))
        return self

    def _on_image(self, frame):
        """
        This is the default image process (identity). Override it.
        Return None if you want nothing to be displayed, return an image otherwise.
        """
        return frame
    
    def __call__(self):
        """
        self() gets an image, processes it (by calling _on_image), 
        and shows the result.
        """
        _, self._frame = self.video.read()
        if self._frame is not None:
            res = self._on_image(copy.deepcopy(self._frame))
            if res is not None:
                cv2.imshow(self.display_name, res)
                key = cv2.waitKey(self.wait_time)
                if key == ord(' '):
                    self.wait_time = 0
                else:
                    self.wait_time = self.frame_wait
                return key not in [27, ord('q')] # Press ESC or q to exit
        return False
