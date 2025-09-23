#1. Clearing costmaps (removes ghost obstacles). 
# 2. Backing up slightly (unstick the robot).
#  3. Spinning in place (refresh sensors and local map).

import math, time
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from nav2_msgs.action import Spin, BackUp
from std_srvs.srv import Empty
from builtin_interfaces.msg import Duration as RosDuration

class Recovery:
    
    def __init__(self, node):
    
    #Action clients - Backup, spin, clear called from RViz 
        self.node = node
        self.back = ActionClient(node, BackUp, 'backup') #tells robot to reverse
        self.spin = ActionClient(node, Spin, 'spin') #tells robot to spin
        self.clear_global = node.create_client(Empty, '/global_costmap/clear_entirely_global_costmap') #service client clears global costmap
        self.clear_local  = node.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')  #service client clears local costmap

    #Extra recovery state variables 
    #Keep track of retries and prevent infinite recovery loops
        self.retries = 0
        self.max_retries = 2  #max attempts per goal
        self.blacklist = []   #list of failed goal positions (x,y)
        self.last_recovery_time = 0

#to call whenever a navigation goal fails 
#Main recovery sequence - clear cost maps, issue a backup action, when backup finishes triggers a spin. 
#If too many retries, stop and add the failed goal to blacklist. 

    def recover(self, goal=None):
        #check retry count first 
        if self.retries >= self.max_retries:
            self.node.get_logger().error("Max recoveries reached. Goal blacklisted.")
            if goal:
                #goal may be tuple (x,y), store it safely
                if isinstance(goal, tuple):
                    self.blacklist.append(goal)
                else:
                    self.blacklist.append((goal.pose.position.x, goal.pose.position.y))
            return

        self.retries += 1
        self.node.get_logger().warn(f"Recovery attempt {self.retries}…")

        # clear both global and local cost maps
        if self.clear_global.service_is_ready():
            self.clear_global.call_async(Empty.Request())
        if self.clear_local.service_is_ready():
            self.clear_local.call_async(Empty.Request())

        # setup backup goal 
        g = BackUp.Goal()
        g.target = Point(x=-0.2, y=0.0, z=0.0) #reverse 20cm
        g.speed = 0.05                          #5cm/s
        g.time_allowance = RosDuration(sec=5)   #max 5 seconds

        #send backup goal then spin 
        self.back.send_goal_async(g).add_done_callback(
            lambda _: self.do_spin()
        )

#Spin recovery 
#rotates robot in place by pi then refresh local sensor data + free local planner 
#When spin finishes, reset retry counter so next navigation can start fresh. 
    def do_spin(self):
        g = Spin.Goal()
        g.target_yaw = math.pi   # 180° spin
        g.time_allowance = RosDuration(sec=10) #max 10 seconds to complete

    #Sends spin goal, log completion when done 
        self.spin.send_goal_async(g).add_done_callback(
            lambda _: self._spin_done()
        )

#Private helper after spin finishes 
#logs success and resets retries 
    def _spin_done(self, *_):
        self.node.get_logger().info("Spin complete, recovery finished.")
        self.last_recovery_time = time.time()
        self.retries = 0
