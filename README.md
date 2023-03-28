# 로봇 가족
##### 부제 : 미래형 라스트마일 배송로봇 구현

__3월 23일까지 최신화 및 Readme 수정이 있을 예정입니다.__

<br>
<img src="https://user-images.githubusercontent.com/110883172/218902218-3180dd52-6303-4bcb-adc9-3e61bb6fd40e.png" width="200" height="200">

###### 팀원 : 권용민, 김두엽, 류도현, 박민제, 박인, 손훈민, 정재욱, 최선민
<br>
<br>

## 프로젝트 개요
- 20232년 2월 23일 발표된 정부 정책인 '[스마트 물류 인프라 구축방안](https://www.korea.kr/news/visualNewsView.do?newsId=148912027&pWise=sub&pWiseSub=I1)'
<img src="https://user-images.githubusercontent.com/110883172/220928275-43f94830-4d69-4403-b48b-676e44df54b4.png" width="400" height="400">

 국토부는 차세대 물류 서비스 구현, 세계 최고 수준의 물류 네트워크 구축, 첨단기술 기반 물류 안전망 강화를 위해 투자를 하겠다고 발표를 했습니다. 이러한 흐름 속에서 우리 팀은 다음 두가지 키워드에 집중합니다. __차세대 물류 서비스 구현__ 과 __첨단기술 기반 물류 안정망 강화__ .
 
__차세대 물류 서비스 구현__ 을 위해 저희는 __배송로봇__ 을 한 번 구현해보고자 합니다. 


<br>
<br>
 



## 프로젝트 목표
#### PART 1. 엄마로봇
- 자율주행 실외 배송로봇 구현
- 라이다를 사용하지 않고 자율주행 구현 (Visual SLAM)

#### PART 2. 아기로봇
- 자율주행 실내 배송로봇 구현
- 2대 이상의 Fleet Management 구현

#### PART 3. 통합 인터페이스 구현
- PART 2의 Fleet Management 관제 소프트웨어 제작
- Unity 엔진을 통한 디지털 트윈 구현


<br>
<br>

## 프로젝트 구체화
#### PART 1. 엄마로봇 (유비쿼티 MAGNI)
- 이미지 세그멘테이션과 Visual SLAM 기술을 통해 Localization하고, 경로를 생성하여 차로를 따라간다.
- 신호등이나 보행자 등의 객체는 YOLO를 통해서 검출하고, 이 때 피하거나 정지할 수 있도록 한다.

#### PART 2. 아기로봇 (PINKBOT BASED ON ADDBOT)
- 실내용 SLAM 기술과 Localization 기술을 통해 경로를 생성한다.
- 실내에서도 장애물을 회피하거나 정지할 수 있어야 한다. 실내배송로봇이 목적지까지 도달하면 다음 목적지로 이동한다. 이동을 마치면 실외배송로봇으로 복귀를 한다.
- 여러 대의 로봇을 관제하는 Fleet Management를 구현한다.


#### PART 3. 통합 인터페이스 구현
- 실외로봇과 실내로봇을 관제하는 인터페이스가 필요하다. 이는 APP이 될 수 있고, 디지털트윈을 통해 구현할 수도 있다.
- 우선은 Unity를 통해 구현을 할 예정이다.





<br>
<br>




## 프로젝트 역할 분배
### Task 1 : RGB-D Camera를 활용한 RtabMap 으로  Visual SLAM 결과물 제시
- 장애물 탐지 및 렌드마크 탐지 알고리즘 및 ROS상에서 메시지로 발행하는 코드 작성

<br>

### Task 2 :  Visual SLAM(실외) 로봇의 ROS 패키지 설계 및 FSM 설계
- FSM 설계
- Visual SLAM(실외) 로봇의 ROS 패키지 설계  

<br>

### Task 3 : 내부 로봇의 FSM 설계 및 ROS 패키지 설계
- FSM 설계
- SLAM(실내) 로봇의 ROS 패키지 설계

<br>

### Task 4 : 언리얼 엔진을 통한 ROS simulation 결과물 제시 
- ROS simulation 결과물 제시
- Localization 된 로봇의 위치정보와 경로정보를 ROS를 통해 받아 화면에 출력하기

<br>

### Task 5 : 차선 인식 및 중앙점으로 경로를 생성하는 코드 작성 
- 차선인식 이미지 segmentation 모델 개발
- 중앙점 경로 생성 알고리즘 개발
- 터틀봇의 하드웨어적 특성 이해 및 구현
- 모니터링 정보(Localization)를 ROS상에 지속적으로 전송

<br>

### Task 6 : YOLO 를 통한 신호등 탐지 및 ROS topic 발행 
- 신호등 제작  및 YOLO detection 하기  
- ROS 상으로 YOLO detection 정보 ROS 상으로 전송
- 모니터링 정보(신호등 정보)를 ROS상에 지속적으로 전송

<br>

### Task 7 : 핑크봇 경로생성 알고리즘 작성 
- Nav2를 이용하여 긴 경로를 짧게 나누어 수행하는 코드 작성 
- SLAM에서 발생하는 문제로 파라미터를 수정해야함.
- 모니터링 정보(Localization, 교통흐름 정보)를 ROS상에 지속적으로 전송



## 프로젝트 설계
#### 실외팀
<img src="https://user-images.githubusercontent.com/110883172/228341795-3fcbdb3b-26ab-449d-9beb-465660a1ca99.png" width="400" height="400">

###### FSM 
![image](https://user-images.githubusercontent.com/110883172/228341648-736681e1-7b41-41af-9a74-483c2c086443.png)
![image](https://user-images.githubusercontent.com/110883172/228341675-f0758a6d-a96a-416c-ba3e-0ea7e907e077.png)

###### 패키지 설계
![image](https://user-images.githubusercontent.com/110883172/228342512-ee8c8800-1093-4e8e-8993-971057276170.png)


## 실내팀
![image](https://user-images.githubusercontent.com/110883172/228341913-a3074fb2-c71b-4769-aa06-088f95717925.png)
###### FSM 
![image](https://user-images.githubusercontent.com/110883172/228341761-f30b833e-8bd3-40d6-9f30-835609e2b30b.png)

#### 패키지 설계
![image](https://user-images.githubusercontent.com/110883172/228342043-6f3ff742-0d75-47f7-93db-732849f6c4cc.png)


