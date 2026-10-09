import math
import struct

from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import PointField, Image
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped, TwistStamped, Point
from visualization_msgs.msg import Marker, MarkerArray


def construct_frame(se2_tuple, source_frame='base_link', child_frame='whatever'):
    t = TransformStamped()
    # IMPORTANT: node is responsible for setting timestamp
    t.header.frame_id = source_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = float(se2_tuple[0])
    t.transform.translation.y = float(se2_tuple[1])
    t.transform.translation.z = 0.0
    # Pure Z-axis rotation (yaw)
    yaw = float(se2_tuple[2])
    half_yaw = yaw / 2.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = math.sin(half_yaw)
    t.transform.rotation.w = math.cos(half_yaw)

    return t


def construct_frame_se3(se3_tuple, source_frame='base_link', child_frame='whatever'):
    t = TransformStamped()
    # IMPORTANT: node is responsible for setting timestamp
    t.header.frame_id = source_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = float(se3_tuple[0])
    t.transform.translation.y = float(se3_tuple[1])
    t.transform.translation.z = float(se3_tuple[2])
    t.transform.rotation.x = float(se3_tuple[3])
    t.transform.rotation.y = float(se3_tuple[4])
    t.transform.rotation.z = float(se3_tuple[5])
    t.transform.rotation.w = float(se3_tuple[6])

    return t


def construct_twist(vx, vy, wz, frame_id='base_link'):
    t = TwistStamped()
    # IMPORTANT: node is responsible for setting timestamp
    t.header.frame_id = frame_id
    t.twist.linear.x = float(vx)
    t.twist.linear.y = float(vy)
    t.twist.linear.z = 0.0
    t.twist.angular.x = 0.0
    t.twist.angular.y = 0.0
    t.twist.angular.z = float(wz)

    return t


def construct_image(bgr_image, frame_id='base_link'):
    # IMPORTANT: node is responsible for setting timestamp
    msg = Image()
    msg.header.frame_id = frame_id
    msg.height, msg.width = bgr_image.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = 3 * msg.width
    msg.data = bgr_image.tobytes()
    return msg


def construct_colored_cloud(points, colors):
    header = Header()
    header.frame_id = 'base_link'
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    cloud_data = []
    for i in range(len(points)):
        p = points[i]
        c = colors[i]
        r, g, b = int(c[0]), int(c[1]), int(c[2])
        rgb_int = (r << 16) | (g << 8) | b
        rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
        cloud_data.append([p[0], p[1], p[2], rgb_float])

    return point_cloud2.create_cloud(header, fields, cloud_data)


def construct_triangle(apex, right, left):
    marker_array = MarkerArray()

    # Common points
    apex_pt = Point(x=float(apex[0]), y=float(apex[1]), z=float(apex[2]))
    right_pt = Point(x=float(right[0]), y=float(right[1]), z=float(right[2]))
    left_pt = Point(x=float(left[0]), y=float(left[1]), z=float(left[2]))

    # -----------------------------
    # Marker 0: The Yellow Triangle Lines
    # -----------------------------
    m_lines = Marker()
    m_lines.header.frame_id = 'base_link'
    m_lines.ns = 'triangle'
    m_lines.id = 0
    m_lines.type = Marker.LINE_STRIP
    m_lines.action = Marker.ADD
    m_lines.pose.orientation.w = 1.0
    m_lines.scale.x = 0.01  # Line width
    m_lines.color.r = 1.0
    m_lines.color.g = 1.0
    m_lines.color.b = 0.0
    m_lines.color.a = 1.0
    m_lines.points = [apex_pt, right_pt, left_pt, apex_pt] # Close the loop

    marker_array.markers.append(m_lines)

    # -----------------------------
    # Marker 1: The Colored Corner Spheres
    # -----------------------------
    m_spheres = Marker()
    m_spheres.header.frame_id = 'base_link'
    m_spheres.ns = 'triangle_spheres'
    m_spheres.id = 1
    m_spheres.type = Marker.SPHERE_LIST
    m_spheres.action = Marker.ADD
    m_spheres.pose.orientation.w = 1.0
    m_spheres.scale.x = 0.05
    m_spheres.scale.y = 0.05
    m_spheres.scale.z = 0.05
    m_spheres.points = [apex_pt, right_pt, left_pt]

    # Colors: Apex=Red, Right=Green, Left=Blue
    c_apex = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
    c_right = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
    c_left = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
    m_spheres.colors = [c_apex, c_right, c_left]

    marker_array.markers.append(m_spheres)

    # -----------------------------
    # Markers 2, 3, 4: Text Labels
    # -----------------------------
    # Helper list to iterate through our points and labels
    labels = [
        ("Apex", apex_pt, 2),
        ("Right", right_pt, 3),
        ("Left", left_pt, 4)
    ]

    for text_str, pt, marker_id in labels:
        m_text = Marker()
        m_text.header.frame_id = 'base_link'
        m_text.ns = 'triangle_labels'
        m_text.id = marker_id
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.orientation.w = 1.0

        # Position the text slightly above the sphere (offset by 2cm in Z)
        m_text.pose.position.x = pt.x
        m_text.pose.position.y = pt.y
        m_text.pose.position.z = pt.z + 0.02

        # For TEXT_VIEW_FACING, scale.z determines the height of an uppercase "A" in meters.
        m_text.scale.z = 0.02

        # Make the text white
        m_text.color.r = 1.0
        m_text.color.g = 1.0
        m_text.color.b = 1.0
        m_text.color.a = 1.0

        m_text.text = text_str

        marker_array.markers.append(m_text)

    return marker_array


def construct_grid_marker(obstacles, cliffs, occlusions, resolution, frame_id='base_link'):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD
    marker.id = 0

    # Scale of each cube matching resolution
    marker.scale.x = resolution
    marker.scale.y = resolution
    marker.scale.z = 0.02  # flat voxel cell height

    marker.pose.orientation.w = 1.0

    # Red for obstacles, Orange for cliffs, Gray for occlusions
    for pts, color in [
        (obstacles, ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.85)),
        (cliffs, ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.85)),
        (occlusions, ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.4))
    ]:
        for pt in pts:
            p = Point()
            p.x = float(pt[0])
            p.y = float(pt[1])
            p.z = float(pt[2])
            marker.points.append(p)
            marker.colors.append(color)

    return marker
