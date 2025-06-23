# Rodando o robô

## 1 - carregue o simulador

Rode  ```ros2 run ur_client_library start_ursim.sh -m ur20```. Atenção: ele irá baixar a imagem do Docker com o ur20.

Quando estiver baixado, no terminal você verá um link semelhante a ```http://192.168.56.101:6080/vnc.html```. Abra esse link em um navegador e ligue o simulador.

## 2 - carregue o robô

Agora rode o comando ```ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur20 robot_ip:=192.168.56.101 launch_rviz:=true``` para executar a simulação.

## OBS: como rodar tf no terminal

Leia em https://docs.ros.org/en/eloquent/Tutorials/tf2.html

## Executing test with dummy publisher and robot: 20/12

1) ```ros2 run movement_test_pkg publisher_cartesian_pose.py```
2) ```ros2 run movement_test_pkg test_input_motion_apriltags_relative_pos.py```