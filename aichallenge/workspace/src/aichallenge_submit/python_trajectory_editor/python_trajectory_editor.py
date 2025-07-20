Python


import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler
import csv

class TrajectoryEditor(Node):
    def __init__(self):
        super().__init__('trajectory_editor')
        self.server = InteractiveMarkerServer(self, 'traj_editor')
        self.waypoints = []    # [{id:int, pose:Pose}, …]
        self.load_csv('trajectory.csv')
        self.make_markers()
        self.server.applyChanges()

    def load_csv(self, path):
        with open(path) as f:
            reader = csv.reader(f)
            for idx, row in enumerate(reader):
                x,y,z,yaw = map(float, row)
                q = quaternion_from_euler(0,0,yaw)
                p = Pose(position=Point(x=x,y=y,z=z),
                         orientation=Quaternion(x=q[0],y=q[1],z=q[2],w=q[3]))
                self.waypoints.append({'id':idx, 'pose':p})

    def make_markers(self):
        for wp in self.waypoints:
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = 'map'
            int_marker.name = f'wp_{wp["id"]}'
            int_marker.pose = wp['pose']
            int_marker.scale = 0.5

            # 可視化用に球マーカーを追加
            marker = Marker(type=Marker.SPHERE, scale=Point(0.2,0.2,0.2),
                            color=Point(1.0,0.0,0.0,1.0))
            ctrl = InteractiveMarkerControl()
            ctrl.always_visible = True
            ctrl.markers.append(marker)
            int_marker.controls.append(ctrl)

            # 6DoF 移動＋回転操作を追加
            for axis in ['MOVE_X','MOVE_Y','MOVE_Z','ROTATE_Z']:
                c = InteractiveMarkerControl()
                c.name = axis.lower()
                c.interaction_mode = getattr(InteractiveMarkerControl, axis)
                int_marker.controls.append(c)

            self.server.insert(int_marker, self.process_feedback)

    def process_feedback(self, feedback):
        # ユーザー操作後のPoseを内部リストに反映
        name = feedback.marker_name      # 'wp_0' など
        idx = int(name.split('_')[1])
        self.waypoints[idx]['pose'] = feedback.pose
        self.server.applyChanges()

    def save_csv(self, path='trajectory_out.csv'):
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            for wp in sorted(self.waypoints, key=lambda x:x['id']):
                p = wp['pose'].position
                # yaw は必要に応じて quaternion → euler 変換
                yaw = 0.0
                writer.writerow([p.x, p.y, p.z, yaw])
        self.get_logger().info(f'Saved to {path}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryEditor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_csv()
    finally:
        node.destroy_node(