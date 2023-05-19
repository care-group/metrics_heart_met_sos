from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher

import rospy
from geometry_msgs.msg import PoseStamped
from ronsm_messages.msg import dm_intent

rospy.init_node('metrics_rasa_core')

pub_intent_bus = rospy.Publisher('/metrics_rasa_core/intent_bus', dm_intent, queue_size=10)

class ActionSimpleROSCommand(Action):

    def name(self) -> Text:
        return 'action_simple_ros_command'

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        intent = tracker.get_intent_of_latest_message()

        print('Intent:', intent)

        if intent == 'intent_location':
            args = tracker.get_latest_entity_values(entity_type='location')
        elif intent == 'intent_item':
            args = tracker.get_latest_entity_values(entity_type='item')
        else:
            args = []

        args = [str(i) for i in args]

        print('Args:', args)

        msg = dm_intent()
        msg.intent = str(intent)
        msg.args = args
        msg.pose = PoseStamped()

        pub_intent_bus.publish(msg)

        print('Command issued.')

        return []