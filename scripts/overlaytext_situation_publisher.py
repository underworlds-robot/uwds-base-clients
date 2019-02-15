#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from pyuwds.reconfigurable_client import ReconfigurableClient
from pyuwds.types import READER
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA


class OverlaytextSituationPublisher(ReconfigurableClient):
    """
    """
    def __init__(self):
        """
        """
        self.text_pub = rospy.Publisher('situations_overlay', OverlayText, queue_size=1)
        ReconfigurableClient.__init__(self, "overlaytext_situation_publisher", READER)

    def onReconfigure(self, worlds_names):
        """
        """
        #rospy.loginfo("reconfigure")
        text = OverlayText()
        text.action = 1
        self.text_pub.publish(text)
        pass

    def onSubscribeChanges(self, world_name):
        """
        """
        pass

    def onUnsubscribeChanges(self, world_name):
        """
        """
        pass

    def onChanges(self, world_name, header, invalidations):
        """
        """
        if len(invalidations.situation_ids_updated) > 0:
            self.publishOverlaytext(world_name, header, invalidations)

    def publishOverlaytext(self, world_name, header, invalidations):
        """
        """
        situations_text = "   *** facts ***\n\r"
        fact_sorted = {}
        for situation_id, situation in self.worlds[world_name].timeline.situations.items():
            if situation.end.data == rospy.Time(0):
                fact_sorted[situation.start.data] = situation

        for key in sorted(fact_sorted.iterkeys()):
            situations_text += fact_sorted[key].description + "\n\r"

        situations_text += "   *** events ***\n\r"
        event_sorted = {}
        for situation_id, situation in self.worlds[world_name].timeline.situations.items():
            if situation.start.data == situation.end.data:
                event_sorted[situation.start.data] = situation

        for key in sorted(event_sorted.iterkeys()):
                situations_text += event_sorted[key].description + "\n\r"

        text = OverlayText()
        text.width = 400
        text.height = 600
        text.left = 10
        text.top = 10
        text.text_size = 12
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.text = situations_text
        text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        self.text_pub.publish(text)


if __name__ == '__main__':
    rospy.init_node("overlaytext_situation_publisher")
    ovtextpub = OverlaytextSituationPublisher()
    rospy.spin()
