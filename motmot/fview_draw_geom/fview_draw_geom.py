from __future__ import with_statement, division

import pkg_resources
import warnings, threading

import numpy as np

from motmot.fview.traited_plugin import *
from motmot.fview.utils import lineseg_circle, lineseg_box

class FviewDrawGeom(HasTraits_FViewPlugin):
    plugin_name = 'draw geom'

    enabled = traits.Bool(False)
    enable_which = traits.Enum('box', 'circle', cols=2, mode='list')

    send_geometry = traits.Button()

    target_x = traits.Int(0)
    target_x_max = traits.Int(5000)
    target_y = traits.Int(0)
    target_y_max = traits.Int(5000)

    target_w = traits.Int(0)
    target_w_max = traits.Int(5000)
    target_h = traits.Int(0)
    target_h_max = traits.Int(5000)
    target_r = traits.Int(0)
    target_r_max = traits.Int(5000)

    traits_view = View(Group(
                        Group(
                            Item(name='enabled'),
                            Item(name='enable_which'),
                            Item(name='send_geometry'),
                        ),
                        Group(
                            Item(name='target_x', editor=RangeEditor(
                                                         low = 0,
                                                         high_name = 'target_x_max',
                                                         mode = 'slider',
                                                         is_float = False)),
                            Item(name='target_y', editor=RangeEditor(
                                                         low = 0,
                                                         high_name = 'target_y_max',
                                                         mode = 'slider',
                                                         is_float = False)),
                            Item(name='target_r', editor=RangeEditor(
                                                         low = 0,
                                                         high_name = 'target_r_max',
                                                         mode = 'slider',
                                                         is_float = False))
                        ),
                        Group(
                            Item(name='target_w', editor=RangeEditor(
                                                         low = 0,
                                                         high_name = 'target_x_max',
                                                         mode = 'slider',
                                                         is_float = False)),
                            Item(name='target_h', editor=RangeEditor(
                                                         low = 0,
                                                         high_name = 'target_y_max',
                                                         mode = 'slider',
                                                         is_float = False)),
                        ),
    ))

    def __init__(self,wx_parent,fview_options):
        super(FviewDrawGeom, self).__init__(wx_parent)

        self._wx_parent = wx_parent

        self._pubo = None
        self._pubs = None
        self._pubpts = None
        if fview_options.get('have_ros'):

            import roslib;
            roslib.load_manifest('rospy')
            roslib.load_manifest('geometry_msgs')
            roslib.load_manifest('std_msgs')
            import rospy
            import geometry_msgs.msg
            import std_msgs.msg

            #publish the image origin and image size
            self._pubo = rospy.Publisher('/draw_geom/image_origin',
                                        std_msgs.msg.String,
                                        latch=True)
            self._pubs = rospy.Publisher('/draw_geom/image_size',
                                        geometry_msgs.msg.Point32,
                                        latch=True)

            self._ptclass = geometry_msgs.msg.Point32
            self._pubpts = rospy.Publisher('/draw_geom/poly',
                                        geometry_msgs.msg.Polygon,
                                        latch=True)

    def _draw_centre(self):
        draw_points = []

        if self.enabled:
            if self.enable_which == 'circle':
                draw_points.append( (self.target_x, self.target_y) )

            elif self.enable_which == 'box':
                xmid = self.target_x + self.target_w//2
                ymid = self.target_y + self.target_h//2
                draw_points.append( (xmid, ymid) )

        return draw_points

    def _draw_linesegs(self):
        draw_linesegs = []

        if self.enabled:
            if self.enable_which == 'circle':
                draw_linesegs.extend( lineseg_circle(self.target_x, self.target_y,self.target_r) )

            elif self.enable_which == 'box':
                xmax = self.target_x + self.target_w
                ymax = self.target_y + self.target_h

                draw_linesegs.extend( lineseg_box(self.target_x, self.target_y, xmax, ymax) )

        return draw_linesegs

    #######AAAAAAAAARRRRGGGGGHHH this decorator does not work
    #@traits.on_trait_change('enabled, enabled_which, target_*')
    def _send_new_polygon(self, *args):

        if self.enabled:
            draw_linesegs = self._draw_linesegs()
            if self._pubpts and draw_linesegs:
                rospts = []
                #convert linesegments to points
                for seg in draw_linesegs:
                    rospts.append( self._ptclass(seg[0],seg[1],0) )
                #close the polygon
                rospts.append( self._ptclass(draw_linesegs[0][0], draw_linesegs[0][1],0) )
                self._pubpts.publish(rospts)

    def _send_geometry_fired(self, *args):
        self._send_new_polygon()
    def _enabled_changed(self, *args):
        self._send_new_polygon()
    def _enable_which_changed(self, *args):
        self._send_new_polygon()
    def _target_x_changed(self, *args):
        self._send_new_polygon()
    def _target_y_changed(self, *args):
        self._send_new_polygon()
    def _target_r_changed(self, *args):
        self._send_new_polygon()
    def _target_w_changed(self, *args):
        self._send_new_polygon()
    def _target_h_changed(self, *args):
        self._send_new_polygon()

    def camera_starting_notification(self,cam_id,
                                     pixel_format=None,
                                     max_width=None,
                                     max_height=None):

        self.target_x_max = int(max_width)
        self.target_y_max = int(max_height)
        self.target_r_max = int(max(max_width,max_height))
        self.target_w_max = int(max_width)
        self.target_h_max = int(max_height)

        if self._pubs:
            self._pubs.publish(int(max_width), int(max_height), 0)
            self._pubo.publish('top left')

    def process_frame(self, cam_id, buf, buf_offset, timestamp, framenumber):
        return self._draw_centre(), self._draw_linesegs()

