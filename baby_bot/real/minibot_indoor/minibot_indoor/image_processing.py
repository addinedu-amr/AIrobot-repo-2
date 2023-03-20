import cv2
import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt

class pgm_to_matrix:
    def __init__(self, pgm_path, yaml_path, hegiht, width, origin_x, origin_y):
        self.pgm_path = pgm_path
        self.yaml_path = yaml_path
        self.hegiht = hegiht
        self.width = width
        self.origin_x = origin_x
        self.origin_y = origin_y

        self.matrix = None
        self.cell_height = None
        self.cell_width = None
        self.resolution = None
        self.garo_m = None
        self.sero_m = None

        

    def change_pgmtopng(self):
        pgm_img = cv2.imread(self.pgm_path, cv2.IMREAD_GRAYSCALE)
        # PNG로 저장
        self.pgm_path = self.pgm_path[:-3]+"png"
        print(self.pgm_path)
        cv2.imwrite(self.pgm_path, pgm_img)

    def image_processing(self):
        map_img = cv2.imread(self.pgm_path,cv2.COLOR_BGR2GRAY )
        img_height, img_width = map_img.shape
        grid_size = (self.hegiht, self.width)  # 세로, 가로
        
        # 그리드 영역 크기 계산
        self.cell_height = img_height // grid_size[0]
        self.cell_width = img_width // grid_size[1]

        map_cost = []
        map_col = []
        
        #각각의 그리드 영역에서 흰색 픽셀 비율 계산
        for i in range(grid_size[0]):
            map_col = []
            for j in range(grid_size[1]):
                # 그리드 영역의 범위를 지정하는거고
                cell_top = i * self.cell_height
                cell_left = j * self.cell_width
                cell_bottom = cell_top + self.cell_height
                cell_right = cell_left + self.cell_width
                
                
                if i == self.hegiht-1:
                    cell_bottom = img_height
                    
                
                # 그 맵의 모든 픽셀을 가져와서
                cell = map_img[cell_top:cell_bottom, cell_left:cell_right]

                wall = 0
                load = 0
                for row in range(len(cell)):
                    for col in range(len(cell[0])):
                    
                        if cell[row][col] <= 210:
                            wall += 1;
                        else:
                            load += 1
                
                    
                if load > wall:
                    map_col.append(0)
                else:
                    map_col.append(1)
            
            map_cost.append(map_col)
                    
        self.matrix = map_cost

    def get_resolution(self):
        with open(self.yaml_path, "r") as stream:
            yaml_data = yaml.safe_load(stream)
        # resolution 값 가져오기
        self.resolution = yaml_data["resolution"]
        self.garo_m = self.resolution * self.cell_width
        self.sero_m = self.resolution * self.cell_height


    def run(self):
        self.change_pgmtopng()
        self.image_processing()
        self.get_resolution()
        
        


if __name__ == '__main__':
    a = pgm_to_matrix("/home/du/mini_bot/baby_map.pgm", "/home/du/mini_bot/baby_map.yaml" ,3 , 12, -1.1, 2.2)
    a.run()
    print(a.matrix)
    print(a.resolution)
    print(a.garo_m)
    print(a.sero_m)

