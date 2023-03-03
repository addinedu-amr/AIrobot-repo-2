# Week 2 진행상황 정리

### (엄마로봇) visual SLAM 팀 진행상황 정리
###### 유니버셜 로봇 고장으로 인해 터틀봇4 스텐다드로 변경하였습니다.
<img src="https://user-images.githubusercontent.com/110883172/222607940-bf58e156-51cd-44fc-8036-e3e701bce057.png" width="200" height="200">

__터블봇 4 초기 세팅 과정__
- 터틀봇 4는 새제품이 아니라 다른 사람이 쓰던 것을 받아왔기 때문에 문제가 생겼다. 여러가지 문제로 인해 1~2일 정도 시간을 버리겠되었다.
- 터틀봇 4를 분해하여 SD카드를 분리하였고, SD카드에 새롭게 터틀봇4 이미지를 구웠다.

<img src="https://user-images.githubusercontent.com/110883172/222608491-e54989c5-960f-4ae0-9fe3-086b3abe0f22.png" width="200" height="200">

- 문제는 SLAM에서 Rviz가 map topic을 받아오지 못한다. 문제는 RPLidar 패키지를 다운받지 않아서 생긴 문제로 보이며, 해결이 가능할 것으로 예상된다.


###### Visual SLAM을 차선 그리기
<img src="https://user-images.githubusercontent.com/110883172/222609300-fca8bed4-7943-40d4-b00c-97427d25b640.jpg" width="200" height="200">

- 마스킹 테이프를 이용하여 차선을 설정함. 장애물이 생기면 다른 차선으로 이동하게 만들 예정
- 3D 프린터를 이용하여 신호등을 만들어올 예정입니다.

<img src="https://user-images.githubusercontent.com/110883172/222610236-f901fb7c-0505-4708-9d19-e25219dcf50b.png" width="200" height="200">
<img src="https://user-images.githubusercontent.com/110883172/222610274-6fe050f2-ab0d-47f8-875a-9640e9b62dd9.png" width="200" height="200">

- 현재 이미지 세그멘테이션을 위해서 차선을 라벨링하는 과정입니다. 



### (아기로봇)팀 진행상황 정리
- 맵을 제작하고 SLAM을 완료함 
<img src="https://user-images.githubusercontent.com/110883172/222610405-d6b86d5b-40cb-4dfd-86ce-5054149a0a2b.png" width="200" height="200">
<img src="https://user-images.githubusercontent.com/110883172/222610526-0e419d2b-6439-4842-9fd6-ec0cef016cc0.png" width="200" height="200">

- 네비게이션을 위해서 파라미터를 수정해야하며, 다음의 논문을 번역하려고 하고 있습니다.
- https://kaiyuzheng.me/documents/navguide.pdf
