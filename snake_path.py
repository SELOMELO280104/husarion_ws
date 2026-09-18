import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

def create_pose(x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0 # Forward facing
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    # Y coordinates provided by you
    y_coords = [25, 22, 19, 16, 13, 10, 7, 4, 1, -2, -5, -8]
    
    waypoints = []
    
    # Build the snake path: Cube (-8) -> Cone (33)
    # Added a 2-meter buffer (X=-6 and X=31) to prevent collisions
    for i, y in enumerate(y_coords):
        if i % 2 == 0:
            waypoints.append(create_pose(-6, y))
            waypoints.append(create_pose(31, y))
        else:
            waypoints.append(create_pose(31, y))
            waypoints.append(create_pose(-6, y))
            
    navigator.followWaypoints(waypoints)
    
    while not navigator.isTaskComplete():
        pass
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
