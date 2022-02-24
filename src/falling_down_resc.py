#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------
# Desc:
# Author: Hiroto Washio
# Date: 17/2/2022
#-------------------------------------------------------------------
import time
import sys
import rospy
from std_msgs.msg import String
import smach
import smach_ros
sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_action_client import *
from common_function import *
from mimi_common_pkg.srv import ManipulateSrv, RecognizeCount 
sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_voice_control/src')
from voice_common_pkg.srv import WhatDidYouSay 
from happymimi_voice_msgs.srv import YesNo
from recognition_action_server import Count,Find,Localize

class Order(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['yes_sir'],
                            input_keys = ['g'])
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        #self.guest_name  = "null"
        self.bc = BaseControl()
    def yesNo(self):
        result = self.yes_no_srv().result
        return result
    def execute(self, userdata):
        rospy.loginfo('Would you like me to call?')
        self.YesNoRes = self.yes_no_srv().result
        #if self.YesNoRes = bool
        if self.yesNo():
            speak("Cought Yes")
            print("Cought yes")
            self.bc.rotateAngle(180,0.2)
            return 'yes_sir'
        else:
            print("Cought no")
            speak("do nothing")     
            pass
        
class EnterRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['finish_enter'])
        self.enter_srv = rospy.ServiceProxy('enter_room_server', EnterRoom)
    def execute(self, userdata):
        rospy.loginfo('start enterTheRoom(S)')
        speak('Start enter the room')
        self.enter_srv(distance=1.0,velocity=0.2)
        speak('Howdy, Whats up.')
        return 'finish_enter'

class SearchPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['found_lying','found_standing',
                                            'not_found1','not_found2'])
                            #input_keys=['human_state_in'],
                            #output_keys=['human_state_out'])
        self.navi_srv = rospy.serviceproxy('navi_location_server',Navigation)
        #self.multiple_localize_srv = rospy.ServiceProxy('/recognition/multiple_localize',MultipleLocalize)
        self.localize_srv = rospy.ServiceProxy('/recognition/localize',RecognitionLocalize)
        self.find_srv = rospy.ServiceProxy('/recognition/find', RecognitionFind)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)
        self.navi_srv = rospy.ServiceProxy('navi_location_server',Navigation)
        self.bc = BaseControl()
        navi_counter = 0
    def execute(self, userdata):
        rospy.loginfo("start searching a person")
        navi_counter =+ 1
        self.head_pub.publish(-15)
        self.bc.rotateAngle(45, 0.2)
        rospy.sleep(5.0)
        self.find_result = self.find_srv(RecognitionFindRequest(target_name='person')).result
        self.head_pub.publish(0)
        #　人検知したか、しないかで条件分岐
        if self.find_result == True:
            target_name = "stand_person"
            #　三次元位置を推定してぇんだなあ
            self.localize_srv(target_name="person")
            #　Z軸の値を受け取りてぇんだなあ
            self.localize_srv(RecognitionLocalizeReqest(num = [2]))
            #　Z軸の位置が基準より高い、低いで条件分岐
            if z_value >= constant_value:
                #target_name = 'standing_person'
                #speak("Come to The operator")
                return 'found_standing'
            else :
                #target_name = 'lying_person'
                return 'found_lying'
        elif self.find_result == False:
            rospy.loginfo('find no person')
            speak("search clear!")
            if navi_counter == 1:
                speak("start navigation")
                self.navi_srv('Shelf')
                self.bc.rotateAngle(90,0.2)
                rospy.sleep(0.5)
                rospy.loginfo('finish moving to another place')
                return 'not_found_1'
            elif navi_counter > 1:
                return 'not_found2'

class TalkAndAlert(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['to_observation'])
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)
    def execute(self, userdata):
        speak('Are you sleeping?')
        
class Observation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['moved','not_moved'])
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
    def execute(self, userdata):
        rospy.loginfo('Executing state: RETURN')
        self.head_pub.publish(-15)

class Exit(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['all_finish']
        self.navi_srv = rospy.ServiceProxy('navi_location_server',Navigation)
    def execute(self, userdata):
        print("Start going to operator")
        speak('Go to the exit')
        self.navi_srv("next_to_soor")
        print("finish confirming of human life safety")
        speak("finish confirming")
        return 'all_finish'

if __name__ == '__main__':
    rospy.init_node('falling_down_resc')
    #main()
    rospy.loginfo('Start informing people lying down')
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    with sm_top:
        smach.StateMachine.add(
                'EnterRoom',
                EnterRoom(),
                transitions = {'finish_enter':'SearchPerson'})
        smach.StateMachine.add(
                'SearchPerson',
                SearchPerson(),
                transitions = {'found_lying':'Observation',
                               'found_standing':'Navigation'
                               'not_found1':'SearchPerson',
                               'not_found2':'Exit'})
        smach.StateMachine.add(
                'TalkAndAlert',
                TalkAndAlert(),
                transitions = {'alert_finish':'Observation'})
        smach.StateMachine.add(
                'Observation',
                Observation(),
                transitions = {'moved':'Exit'})
        smach.StateMachine.add(
                'Exit',
                Exit(),
                transitions = {'all_finish':'finish_sm_top'},

        outcome = sm_top.execute()


#class Navigation(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes = ['finish_moveing']
#        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
#        self.save_srv = rospy.ServiceProxy('/recognition/save', StrTrg)
#        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
#        self.si = SaveInfo()
#        self.bc = BaseControl()
#    def execute(self, userdata):
#        rospy.loginfo('start navigation')
#        self.navi_srv('shelf')
#        rospy.sleep(0.5)
#        rospy.loginfo('finish navigation')
#        return 'finish_moveing'

#class Talk(smach.State):
#    def __init__(self):
#        smach.State.__init__(self,
#                             outcomes = ['dead', 'live'])
#    def execute(self, userdata):


#class Apology(smach.State):
    #def __init__(self):
        #smach.State.__init__(self,
                             #outcomes = ['decide_finish', 'all_cmd_finish'],
                             #input_keys = ['cmd_count_in'])

