import cv2
from yolo_segmentation import YOLOSegmentation
import numpy as np

# differential 
def compensate_x(center_x, target_x):
    p_x = 0.001
    center_x = center_x
    target_x = target_x
    e_x = center_x - target_x
    control_x = p_x * e_x
    return control_x

def compensate_y(center_y, target_y):
    p_y = 0.01
    control_y = p_y
    return control_y

# seg
ys = YOLOSegmentation("last.pt")

# 동영상 파일 경로
video_path = "test2.mp4"

# Lane center
center_pt_orange = 0
center_pt_blue = 0

target_point_y = 0

# 동영상 파일 열기
cap = cv2.VideoCapture(video_path)

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
                # print("bbox:", bbox, "class id:", class_id, "seg:", seg, "score:", score)
                #(x, y, x2, y2) = bbox
                if class_id == 0:

                    cv2.polylines(roi, [seg], True, (0, 255, 0), 4)
                    
                    condition = (seg[:, 1] >= 0) & (seg[:, 1] < 10)
                    filtered_arr = seg[condition, 0]
                    try:
                        center_pt_orange = (np.max(filtered_arr) + np.min(filtered_arr)) // 2
                    except:
                        pass

                if class_id == 1:

                    cv2.polylines(roi, [seg], True, (0, 255, 0), 4)

                    condition = (seg[:, 1] >= 0) & (seg[:, 1] < 10)
                    filtered_arr = seg[condition, 0]
                    try:
                        center_pt_blue = (np.max(filtered_arr) + np.min(filtered_arr)) // 2
                    except:
                        pass

            target_point_x = (center_pt_orange + center_pt_blue) // 2
            target_point_y = 0

            # 앵글 타겟 포인트
            cv2.circle(frame, (target_point_x, height//2), 10, (0, 255, 0), 3)
            
            # 가로 중앙선
            cv2.line(frame, (0, int(height/2)), (width, int(height/2)), (255, 0, 0), 5)
            
            # 로봇 중심점
            cv2.circle(frame, (int(width/2), int(height/2)), 10, (50, 50, 50), -1)
            
            # Twist msg
            cv2.putText(frame, "linear_x : {:.2f} m/s".format(compensate_y(height, height//2)), (width//2 - 150, 340), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, "angular_z : {:.2f} rad/s".format(compensate_x(width//2, target_point_x)), (width//2 - 150, 370), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 3, cv2.LINE_AA)
            
            # 동영상 화면에 표시
            cv2.imshow('frame', frame)
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
