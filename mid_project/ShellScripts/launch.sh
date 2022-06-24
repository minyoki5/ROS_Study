    #! /bin/bash
gnome-terminal -e "roscore" &
sleep 5
gnome-terminal -e "rosrun mid_project Rear_Cam __name:=Rear_Camera" &
sleep 3
gnome-terminal -e "rosrun mid_project Follower_Vehicle2 __name:=Following_Vehicles" &
sleep 3
gnome-terminal -e "rosrun mid_project Traffic_Center __name:=Traffic_Center" &
sleep 3
gnome-terminal -e "rosrun mid_project GPS1.py __name:=GPS" &
sleep 3
gnome-terminal -e "rosrun mid_project Platoon_Control __name:=Platoon_Control" &
sleep 3
gnome-terminal -e "rosrun mid_project V2X __name:=V2X"&
sleep 3
gnome-terminal -e "rosrun mid_project Leader_Vehicle __name:=Leader_Vehicle"




