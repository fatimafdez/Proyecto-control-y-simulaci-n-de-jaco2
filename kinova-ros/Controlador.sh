cd trabajo_control/config/
python generador_de_controlador.py
gnome-terminal
export SVGA_VGPU10=0
cd ..
cd src

chmod +x mov_cont_trabajo.py

roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2n6s300


