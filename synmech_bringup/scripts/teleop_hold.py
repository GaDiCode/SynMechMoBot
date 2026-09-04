#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

msg = """
========================================
SynMech Hold-to-Move Teleop
========================================
Key bindings:
Hold key to move. Release to stop.

Movement:
        w    
   a    s    d

w/s : Linear forward/backward
a/d : Angular left/right

Speed Control:
q/z : Increase/Decrease maximum speed by 10%

Press CTRL-C to exit
========================================
"""

moveBindings = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
}

class TeleopHold:
    def __init__(self):
        rospy.init_node('teleop_hold')
        self.settings = termios.tcgetattr(sys.stdin)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.speed = rospy.get_param("~speed", 0.15)
        self.turn = rospy.get_param("~turn", 0.5)
        
        # Nếu sau 0.15s không nhận được phím nào -> Tự động dừng
        self.timeout = rospy.get_param("~timeout", 0.15) 

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        # Đợi phím bấm với một khoảng timeout
        rlist, _, _ = select.select([sys.stdin], [], [], self.timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg)
        print(f"Initial limits: Linear {self.speed:.2f} m/s | Angular {self.turn:.2f} rad/s")
        
        x = 0
        th = 0
        
        try:
            while not rospy.is_shutdown():
                key = self.getKey()
                
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    th = moveBindings[key][1]
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    print(f"Current limits: Linear {self.speed:.2f} m/s | Angular {self.turn:.2f} rad/s")
                    continue
                elif key == '\x03': # Bấm CTRL-C
                    break
                else:
                    # Nếu timeout (nhả phím) hoặc bấm phím lạ -> Dừng lại
                    x = 0
                    th = 0
                
                twist = Twist()
                twist.linear.x = x * self.speed
                twist.angular.z = th * self.turn
                self.pub.publish(twist)
                
        finally:
            # Luôn gửi lệnh dừng xe trước khi thoát chương trình
            twist = Twist()
            self.pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    try:
        app = TeleopHold()
        app.run()
    except rospy.ROSInterruptException:
        pass
