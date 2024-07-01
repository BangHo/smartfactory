시스템의 구성은 아래와 같습니다.
![image](https://github.com/BangHo/smartfactory/assets/113181132/a033d4ea-d1f2-4948-8732-f1e0f35fd242)

시스템 인터페이스는 아래와 같습니다.
![image](https://github.com/BangHo/smartfactory/assets/113181132/7549cac7-cf0a-4a49-816a-d141d16527a7)

1. main_scada.py : 시스템의 메인 SCADA의 소스코드로 각 제어기로부터 통신 상태와 센서값, 로봇의 상태값을 받고, 명령을 내리기 위한 내용이 포함되어 있습니다.
2. controller_desktop.py : 시스템 인터페이스의 데스크탑과 같은 내용이 구현되어 있습니다.
3. controller_rp1.py : 시스템 인터페이스의 라즈베이파이1과 같은 내용이 구현되어 있습니다.
4. controller_rp2.py : 시스템 인터페이스의 라즈베이파이2과 같은 내용이 구현되어 있습니다.
5. hx711.py : loadcell을 사용하는데 참고한 소스코드입니다.
