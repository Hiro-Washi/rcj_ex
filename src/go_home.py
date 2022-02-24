#!/usr/bin/env/ python
#-*-coding: utf-8 -*-
#-----------------------------
# Desc:
# Author:
# Date: Feb/15/2022
#-----------------------------

import rospy
import time
import sys
from std_msgs.msg import String
import smach
import smach_ros

class DoorOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['door_transition'])

    def execute(self, userdata):
        rospy.loginfo('Enter The Room')
        speak('Start Rescue guidance.')
        enterTheRoomAC(0.8)
        return 'door_transition'

class MoveAndPick(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['success', 'failed'],
                            input_keys=['object_name_in'],
                            output_keys=['object_name_out'])
        # Service
        self.grab = rospy.ServiceProxy('/manipulation', ManipulateSrv)
        # Publisher
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)

    def execute(self, userdata):
        location_list = searchLocationName('table')
        navigationAC(location_list)
        
        rospy.wait_for_service('/object/recognize')
        recog = rospy.ServiceProxy('/object/recognize', RecognizeCount)
        res = recog('any')

        if len(res.data) >= 2:
            object_name = res.data[1]

        elif len(res.data) == 1:
            object_name = res.data[0]

        else:
            object_name = 'any'
        userdata.object_name_out = object_name
        self.pub_location.publish('table')

        result = self.grab(object_name).result  #object_nameによってif等で条件分岐
        if result == True:
            return 'success'
        else:
            return 'failed'

class MoveAndPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['completed'],
                            input_keys=['object_name_in'])

        self.object_list = ['cup','bottle','snack','dish','chips',
                            'bag','toy','smartphone','book','pen']

        #Service
        self.arm_srv = rospy.ServiceProxy('/servo/arm', ManipulateSrv)
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)

    def execute(self, userdata):
        if userdata.object_name_in  in self.object_list:
            location_list = searchLocationName('chair')
            self.pub_location.publish('chair')
        else:
            location_list = searchLocationName('couch')
            self.pub_location.publish('couch')
        navigationAC(location_list)
        self.arm_srv('place')
        return 'completed'


class AvoidThat(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_WDYS'])

    def execute(self, userdata):
        print("AvoidThat")
        speak('start Avoid That')
        location_list = searchLocationName('operator')
        navigationAC(location_list)
        return 'to_WDYS'


class TimeCount(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_PS'],
                            output_keys=['start_time_out', 'success_count_out'])
    
    def execute(self, userdata):
        speak('Staet What did you say')
        userdata.start_time_out = time.time()
        userdata.success_count_out = 0
        return 'to_PS'


class PersonSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['found'])
        self.flag = 'failed'

    def execute(self, userdata):
        result = approachPersonAC()
        if result == True:
            m6Control(0.4)
            speak('I found the questioner')
        else:
            speak('Please come in front of me')
            rospy.sleep(5.0)
            speak('Thank you')
        return 'found'



class QuestionResponse(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['continues', 'give_up', 'completed'],
                            input_keys=['success_count_in', 'start_time_in'],
                            output_keys=['success_count_out', 'start_time_out'])
        self.WDYS = rospy.ServiceProxy('/bf/conversation_srvserver', WhatDidYouSay)
        self.target_time = 160.0

    def execute(self, userdata):
        end_time = time.time()
        userdata.start_time_out = userdata.start_time_in
        if end_time - userdata.start_time_in >= self.target_time:
            speak('Cancel Q and A session')
            return 'give_up'
        speak('ready')
        result = self.WDYS().result
        count_in = userdata.success_count_in
        if result == True:
            count_in = userdata.success_count_in
            count_out = count_in + 1
            userdata.success_count_out = count_out
            if count_out == 3:
                return 'completed'
            else:
                return 'continues'  
        else:
            return 'continues'


class ExitRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['to_finish'])

    def execute(slef, userdata):
        speak('Go to the exit')
        location_list = searchLocationName('entrance')
        navigationAC(location_list)
        speak('finish what did you say')
        return 'to_finish'


def main():
    sm_top = smach.StateMachine(outcomes=['FINISH'])
    with sm_top:
        ### EnterRoom
        smach.StateMachine.add('ENTER', EnterRoom(),
                            transitions={'to_pap':'PICH_AND_PLACE'})

        ### Pick and place
        sm_pap = smach.StateMachine(outcomes=['to_AvoidThat'])
        sm_pap.userdata.sm_name = 'cup'
        with sm_pap:
            smach.StateMachine.add('pick', MoveAndPick(),
                            transitions={'success':'place',
                                         'failed':'to_AvoidThat'},
                            remapping={'object_name_out':'sm_name',
                                       'object_name_in':'sm_name'})
            smach.StateMachine.add('place', MoveAndPlace(),
                            transitions={'completed':'to_AvoidThat'},
                            remapping={'object_name_in':'sm_name'})
        smach.StateMachine.add('PICH_AND_PLACE', sm_pap,
                            transitions={'to_AvoidThat':'AVOID_THAT'})

        ### Avoid that
        smach.StateMachine.add('AVOID_THAT', AvoidThat(),
                            transitions={'to_WDYS':'WHAT_DID_YOU_SAY'})

        ### what did you say
        sm_wdys = smach.StateMachine(outcomes=['to_exit'])
        sm_wdys.userdata.sm_time = time.time()
        #sm_wdys.userdata.sm_success

        with sm_wdys:
            smach.StateMachine.add('STARTWDYS', TimeCount(),
                            transitions={'to_PS':'PersonSearch'},
                            remapping={'start_time_out':'sm_time',
                                       'success_count_out':'sm_success'})
            smach.StateMachine.add('PersonSearch', PersonSearch(),
                            transitions={'found':'QUESTION'})
            smach.StateMachine.add('QUESTION', QuestionResponse(),
                            transitions={'continues':'QUESTION',
                                        'give_up':'to_exit',
                                        'completed':'to_exit'},
                            remapping={'success_count_in':'sm_success',
                                       'success_count_out':'sm_success',
                                       'start_time_in':'sm_time',
                                       'start_time_out':'sm_time'})
        smach.StateMachine.add('WHAT_DID_YOU_SAY', sm_wdys,
                            transitions={'to_exit':'EXIT'})

        ### Go to the exit
        smach.StateMachine.add('EXIT', ExitRoom(),
                            transitions={'to_finish':'FINISH'})

    outcome = sm_top.execute()
        
if __name__ == '__main__':
    rospy.init_node('')
    main()

