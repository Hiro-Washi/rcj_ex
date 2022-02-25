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
tts_srv = rospy.ServiceProxy('/tts', StrTrg)

class Order(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['yes_sir'],
                            input_keys = ['g'])
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.bc = BaseControl()
#    def yesNo(self):
#        result = self.yes_no_srv().result
#        return result
    def execute(self, userdata):
        print('Would you like me to call a person?')
        tts_srv("Would you like me to call a person?")
        #self.Y_N_Res = self.yes_no_srv().result
        if self.yes_no_srv().result == True:
            tts_srv("Got Yes")
            print("Got yes")
            self.bc.rotateAngle(180,0.4)
            rospy.sleep(5.0)
            return 'yes_sir'
        else:
            tts_srv("Got no")
            print("Got no, My work completed. Good bye, take care.")
            pass

class EnterRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['finish_enter'])
        self.enter_srv = rospy.ServiceProxy('enter_room_server', EnterRoom)
    def execute(self, userdata):
        rospy.loginfo('start enterTheRoom(S)')
        tts_srv("Start enter the room")
        self.enter_srv(distance=1.0,velocity=0.2)
        tts_srv("Howdy? What's up! Body")
        return 'finish_enter'

class SearchPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['found_lying','found_standing',
                                            'not_found_one','not_found_two'])
        self.localize_srv = rospy.ServiceProxy('/recognition/localize',RecognitionLocalize)
        self.find_srv = rospy.ServiceProxy('/recognition/find', RecognitionFind)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.pub_location = rospy.Publisher('/navigation/move_place', String, queue_size = 1)
        self.navi_srv = rospy.ServiceProxy('navi_location_server',Navigation)
        self.bc = BaseControl()
        navi_counter = 0
    def execute(self, userdata):
        rospy.loginfo("start SearchPerson")
        navi_counter =+ 1
        self.head_pub.publish(-15)
        self.bc.rotateAngle(45, 0.3)
        rospy.sleep(5.0)
        self.find_result = self.find_srv(RecognitionFindRequest(target_name='person')).result
        rospy.sleep(5.0)
        self.head_pub.publish(0)
        if self.find_result == True:
            print("found person")
            target_name = "person"
            #　三次元位置を推定したいんだなあ
            self.localize_srv(target_name="person")
            #　Z軸の値を受け取りたいんだなあ
            localize_result = self.localize_srv(RecognitionLocalizeReqest)
            z_value = localize_result[3]
            #　Z軸の位置が基準より高い、低いで条件分岐
            if z_value >= constant_value:
                target_name = 'standing_person'
                tts_srv("come to the operator")
                return 'found_standing'
            else :
                target_name = 'lying_person'
                return 'found_lying'
        elif self.find_result == False:
            print("find no person")
            tts_srv("find no person")
            if navi_counter == 1:
                tts_srv("start navigation")
                self.navi_srv('Shelf')
                self.bc.rotateAngle(90,0.3)
                rospy.sleep(0.5)
                rospy.loginfo('finish moving to another place')
                return 'not_found_one'
            elif navi_counter > 1:
                return 'not_found_two'

class Observation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['moved','not_moved'])
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)

    def execute(self, userdata):
        rospy.loginfo('Executing state: Observation')
        self.head_pub.publish(-15)


class TalkAndAlert(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['alert_finish','to_exit'])
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)
    def execute(self, userdata):
        for i in range(2):
            tts_srv("Are you awake?")
            self.y_n_result = self.yes_no_srv().result
            if self.y_n_result == True:
                print("Got yes")
                tts_srv("Got Yes. Please come to the operator")
                break
            elif self.y_n_result == False:
                print("This guy is awake")
                tts_srv("Come to the operator")
                break
        else:
            tts_srv("This guy has no sounds.")
            tts_srv("Hellooo gooood mooorrrniiing")
            return 'alert_finish'
        return 'to_exit'

class Exit(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['all_finish'])
        self.navi_srv = rospy.ServiceProxy('navi_location_server', Navigation)
    def execute(self, userdata):
        print("Start going to operator")
        tts_srv("let's go to the operator")
        self.navi_srv("next_to_door")

        print("finish confirming of human life safety")
        tts_srv("finish confirming")
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
                               'found_standing':'Exit',
                               'not_found_one':'SearchPerson',
                               'not_found_two':'Exit'})
        smach.StateMachine.add(
                'TalkAndAlert',
                TalkAndAlert(),
                transitions = {'alert_finish':'Observation',
                               'to_exit':'Exit'})
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

