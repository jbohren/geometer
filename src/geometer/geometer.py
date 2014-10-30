
import threading

import PyKDL as kdl
import numpy as np
import distmesh as dm

import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA

def proj(n, p, q):
    """Projects vector q onto plane n at point p"""
    return q - np.inner((q-p).transpose(), n) * n

def element_key(m):
    return '%s-%d' % (m.ns, m.id)

class Geometer(object):

    def __init__(self, topic='geometer', rate=None):
        self.topic_name = topic
        self.pub = rospy.Publisher(self.topic_name, MarkerArray)

        self.points = {}
        self.lines = {}
        self.planes = {}

        self.marker_array = MarkerArray()

        # precompute circle mesh
        fd = lambda p: np.sqrt((p**2).sum(1))-1.0
        self.mesh_p, self.mesh_t = dm.distmesh2d(fd, dm.huniform, 0.2, (-1,-1,1,1))

        self.mod_lock = threading.Lock()
        if rate:
            self.rate = rospy.Rate(rate)
            self.pub_thread = threading.Thread(target=self.pub_loop)
            self.pub_thread.start()

    def pub_loop(self):
        while not rospy.is_shutdown():
            with self.mod_lock:
                self.publish
            self.rate.sleep()

    def publish(self):
        self.marker_array.markers = self.points.values() + self.lines.values() + self.planes.values()
        for m in self.marker_array.markers:
            m.header.stamp = rospy.Time.now()
        self.pub.publish(self.marker_array)

    def point(self, frame_id, p, s, key=None):
        """ point: p
        size: s"""

        m = Marker()
        m.header.frame_id = frame_id
        m.ns = key or 'points'
        m.id = 0 if key else len(self.points)
        m.type = Marker.POINTS
        m.action = Marker.MODIFY
        m.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))

        m.scale = Vector3(s,s,1.0)

        m.points = [Point(*p)]
        m.colors = [ColorRGBA(0.8,0.8,0,1)]

        key = key or element_key(m)

        with self.mod_lock:
            self.points[key] = m

        return key

    def line(self, frame_id, p, r, t=[0.0, 0.0], key=None):
        """
        line: t r + p

        This will be drawn for t[0] .. t[1]
        """

        r = np.array(r)
        p = np.array(p)

        m = Marker()
        m.header.frame_id = frame_id
        m.ns = key or 'lines'
        m.id = 0 if key else len(self.lines)
        m.type = Marker.LINE_STRIP
        m.action = Marker.MODIFY
        m.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))

        m.scale = Vector3(0.005, 0.005, 0.005)
        m.color = ColorRGBA(0,0.8,0.8,1)

        m.points = [Point(*(p+r*t)) for t in np.linspace(t[0], t[1], 10)]
        m.colors = [m.color] * 10

        key = key or element_key(m)

        with self.mod_lock:
            self.lines[key] = m

        return key

    def plane(self, frame_id, p, n, r=1.0, c=(0.0,0.0), key=None, draw_triad=False):
        """
        plane: n . ( X - p ) = 0

        This will be drawn as a disc of radius r about p.
        """

        n = np.array(n)
        p = np.array(p)

        m = Marker()
        m.header.frame_id = frame_id
        m.ns = key or 'planes'
        m.id = 0 if key else len(self.planes)
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.MODIFY

        # Compute plane rotation
        nz = n / np.linalg.norm(n)
        nx = np.cross(nz, np.array([0,0,1])); nx = nx / np.linalg.norm(nx)
        ny = np.cross(nz, nx); ny = ny / np.linalg.norm(ny)
        R = kdl.Rotation(
            kdl.Vector(*nx),
            kdl.Vector(*ny),
            kdl.Vector(*nz))

        # offset for in-plane circle position
        c3 = np.array(list(kdl.Vector(c[0],c[1],0.0)))
        c3 = c3 - np.inner(c3,n)*n

        if draw_triad:
            self.line(frame_id, p, nx, t=[0.0,r],key=key+'-x')
            self.line(frame_id, p, ny, t=[0.0,r],key=key+'-y')
            self.line(frame_id, p, nz, t=[0.0,r],key=key+'-z')

            self.point(frame_id, p+c3, 0.005, key=key+'-center-point')

        m.pose = Pose(Point(*(p+c3)), Quaternion(*R.GetQuaternion()))

        m.scale = Vector3(1, 1, 1)

        # TODO: precompute plane

        # flatten the triangle list
        for triangle in self.mesh_t:
            m.points.extend([
                Point(r*self.mesh_p[p][0],r*self.mesh_p[p][1],0)
                for p in triangle])

        m.color = ColorRGBA(0.8,0,0.8,0.9)
        m.colors = [m.color] * len(m.points)

        key = key or element_key(m)

        with self.mod_lock:
            self.planes[key] = m

        return key
