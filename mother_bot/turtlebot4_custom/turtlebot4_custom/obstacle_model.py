import cv2
import numpy as np
import torch
from ament_index_python.packages import get_package_share_directory

class Inference:
    def __init__(self):
        self.pt_path = get_package_share_directory('turtlebot4_custom') + '/pt_files/obstacle.pt'
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.pt_path, force_reload=True)
        self.model.conf = 0.8

    def inference(self, source):
        raw_results = self.model(source)
        raw_results_xyxy = raw_results.xyxy[0]
        refined_results = np.array(raw_results_xyxy.to('cpu'))
        return refined_results

    def get_center_point(x1, y1, x2, y2):
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        return center_x, center_y
    
# def main():
#     inf = Inference()

# if __name__== '__main__':
#     main()

