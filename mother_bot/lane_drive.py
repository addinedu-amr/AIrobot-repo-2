import cv2
from yolo_segmentation import YOLOSegmentation
import numpy as np

# PD Control
def compensate_x(center_x, target_x):    
    p_x = 0.001
    center_x = center_x
    target_x = target_x
    e_x = center_x - target_x
    control_x = p_x * e_x
    return control_x 

def compensate_y(position_y, target_y):
    p_y = 0.0003
    position_y = position_y
    target_y = target_y
    e_y = position_y - target_y
    control_y = p_y * e_y
    return control_y

# seg
ys = YOLOSegmentation("last.pt")

# 동영상 파일 경로
video_path = "test1.mp4"

# Lane center
center_pt_x_orange = 0
center_pt_x_blue = 0

# lane_num_det
lane_num = 0
lane_num_det = True

# 동영상 파일 열기
cap = cv2.VideoCapture(video_path)

# 동영상 저장
width_ = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height_ = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter('output.avi', fourcc, 30.0, (int(width_), int(height_)))

# 영역 지정

# 동영상 프레임 읽어오기
while(cap.isOpened()):
    ret, frame = cap.read()

    # 프레임을 성공적으로 읽었을 경우
    if ret:
        height, width = frame.shape[:2]

        # 영역 추출
        roi = frame[height//2:height, 0:width]

        # 프레임 처리 코드
        try:
            bboxes, classes, segmentations, scores = ys.detect(roi)
            for bbox, class_id, seg, score in zip(bboxes, classes, segmentations, scores):
                if class_id == 0:

                    cv2.polylines(roi, [seg], True, (0, 255, 0), 4)
                    
                    condition = (seg[:, 1] >= 0) & (seg[:, 1] < 10)
                    filtered_arr_x = seg[condition, 0]
                    try:
                        max_pt_x_orange = np.max(filtered_arr_x)
                        min_pt_x_orange = np.min(filtered_arr_x)
                        center_pt_x_orange = (max_pt_x_orange + min_pt_x_orange) // 2
                    except:
                        pass
                    
                    pt_y_orange = np.min(seg[:, 1])

                if class_id == 1:

                    cv2.polylines(roi, [seg], True, (0, 255, 0), 4)

                    condition = (seg[:, 1] >= 0) & (seg[:, 1] < 10)
                    filtered_arr_x = seg[condition, 0]
                    try:
                        max_pt_x_blue = np.max(filtered_arr_x)
                        min_pt_x_blue = np.min(filtered_arr_x)
                        center_pt_x_blue = (max_pt_x_blue + min_pt_x_blue) // 2
                    except:
                        pass

                    pt_y_blue = np.min(seg[:, 1])
            
            # what number of lane
            if lane_num_det and (center_pt_x_orange < center_pt_x_blue):
                lane_num = 1
                lane_num_det = False
            elif lane_num_det and (center_pt_x_orange > center_pt_x_blue):
                lane_num = 2 
                lane_num_det = False
            else:
                pass

            # drop data
            if lane_num == 1:
                pass
            elif lane_num == 2:
                pass
            else:
                pass
    
            target_point_y = max(pt_y_orange, pt_y_blue)
            # target_point_y = (pt_y_orange + pt_y_blue) // 2

            # turn sign
            if (pt_y_blue > 200) and (pt_y_orange > 200):
                turn_status = True
                turn_direction = "TURN"
                    # if :
                    #     turn_direction = "LEFT"
                    # elif :
                    #     turn_direction = "RIGHT"
            else:
                turn_status = False
                turn_direction = "STRAIGHT"
            
            # Center of Lane
            target_point_x = (center_pt_x_orange + center_pt_x_blue) // 2

            # 앵글 타겟 포인트
            cv2.circle(frame, (target_point_x, height//2), 10, (0, 255, 0), 3)
            
            # 가로 중앙선
            cv2.line(frame, (0, height//2), (width, height//2), (255, 255, 0), 5)
            cv2.line(roi, (0, target_point_y), (width, target_point_y), (255, 0, 0), 5)
            
            # 로봇 중심점
            cv2.circle(frame, (int(width/2), int(height/2)), 10, (50, 50, 50), -1)
            
            # Twist msg
            cv2.putText(frame, "linear_x : {:.2f} m/s".format(compensate_y(height//2, target_point_y)), (width//2 - 150, 340), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, "angular_z : {:.2f} rad/s".format(compensate_x(width//2, target_point_x)), (width//2 - 150, 370), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            # cv2.putText(frame, str(target_point_y), (width//2 - 150, 400), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            # cv2.putText(frame, str(height//2), (width//2 - 150, 430), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, "direction : " + str(turn_direction), (width//2 - 150, 430), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            # cv2.putText(frame, "blue : {:.2f} ".format(pt_y_blue), (width//2 - 150, 460), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            # cv2.putText(frame, "orange : {:.2f} ".format(pt_y_orange), (width//2 - 150, 480), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, "blue_x : {:.2f} ".format(center_pt_x_blue), (width//2 - 150, 460), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, "orange_x : {:.2f} ".format(center_pt_x_orange), (width//2 - 150, 480), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            # 동영상 화면에 표시
            cv2.imshow('frame', frame)
#           저장
#            out.write(frame)
        except:
            pass
        # q 키를 누르면 종료
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    # 프레임을 읽어오지 못했을 경우
    else:
        break

# 동영상 파일과 OpenCV 창 닫기
cap.release()
cv2.destroyAllWindows()
