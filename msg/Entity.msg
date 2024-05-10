# Entity.msg
std_msgs/Header header
uint64 id
string name
uint8 type

uint8 NONE=0
uint8 LIGHT=1
uint8 MODEL=2
uint8 LINK=3
uint8 VISUAL=4
uint8 COLLISION=5
uint8 SENSOR=6
uint8 JOINT=7
uint8 ACTOR=8
uint8 WORLD=9
