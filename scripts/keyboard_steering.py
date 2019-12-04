#!/usr/bin/env python
import rospy
import os
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from selfie_msgs.msg import MPCControl
from pynput import keyboard
import pygame

meta_path = os.path.abspath(os.path.join(__file__, '../../metadata'))
pygame.init()
pygame.mixer.init()


class msg_manager:
    # 1 -> manually     2-> semi semi_automat   3-> full automat
    def __init__(self, pause_pub, steering_mode = 1, force_stop = 0, msg_full_auto = MPCControl(),
                default_speed = 1,
                msg_semi_auto = MPCControl(),msg_manual = MPCControl(), 
                default_left = 0.3, default_right = -0.3, speed = 0, steering_angle = 0):
        self.steering_mode = steering_mode
        self.force_stop = force_stop
        self.msg_full_auto = msg_full_auto
        self.msg_semi_auto = msg_semi_auto
        self.msg_manual = msg_manual
        self.default_speed = default_speed
        self.default_left = default_left
        self.default_right = default_right
        self.speed = speed
        self.steering_angle = steering_angle
        self.pause_pub = pause_pub
        self.paused = True


    def cmd_callback(self, msg):
        if self.steering_mode == 3:
            self.msg_full_auto = msg
        # semi automat
        elif self.steering_mode == 2:
            self.msg_semi_auto = msg
            self.msg_semi_auto.speed = self.speed


    def on_press(self, key):
        try:
            # space changes steering mode
            if key == keyboard.Key.space:
                mode = ''
                if self.steering_mode == 1:
                    mode='semi_automatic_mode'
                    rospy.loginfo(mode)
                    self.steering_mode = 2

                elif self.steering_mode == 2:
                    mode='full_automatic_mode'
                    rospy.loginfo(mode)
                    self.steering_mode = 3

                else:
                    mode='manual_mode'
                    rospy.loginfo(mode)

                    self.steering_mode = 1
                sound_file = os.path.join(meta_path, mode+'.mp3')
                pygame.mixer.music.load(sound_file)
            #    pygame.mixer.music.set_volume(1)
                pygame.mixer.music.play()
            if(key.char == 'r'):
                if(self.force_stop == 1):
                    self.speed = self.default_speed/2
            elif (key.char == 'e'):
                self.speed = -self.default_speed/2

   #         if (key.char == 'f'):
   #             if(self.force_stop == 0):
   #                 rospy.logwarn('stop mode')
   #                 self.force_stop = 1
   #             else:
   #                 rospy.logwarn('ride mode')
   #                 self.force_stop = 0

            elif(key.char == 'a'):
                if(self.force_stop == 1):
                    self.steering_angle = self.default_left
                    #self.speed = 0
                else:
                    #rospy.loginfo('left')
                    self.speed = self.default_speed/2
                    self.steering_angle = self.default_left
            elif(key.char =='d'):
                if(self.force_stop == 1):
                    self.steering_angle = self.default_right
                    #self.speed = 0
                else:
                    #rospy.loginfo('right')
                    self.speed = self.default_speed/2
                    self.steering_angle = self.default_right
            elif(key.char == 'w'):
                if(self.force_stop == 1):
                    self.steering_angle = self.default_left*2
                    #self.speed = 0
                else:
                    #rospy.loginfo('forward')
                    self.speed = self.default_speed
                    self.steering_angle = 0
            elif(key.char == 's'):
                if(self.force_stop == 1):
                    self.steering_angle = self.default_right*2
                    #self.speed = 0
                else:
                    #rospy.loginfo('backwards')
                    self.speed = - self.default_speed
                    self.steering_angle = 0
            elif(key.char == 'p'):
                pause_msg = Bool()
                if(self.paused == True):
                    pause_msg.data = False
                    self.paused = False
                else:
                    pause_msg.data = True
                    self.paused = True
                self.pause_pub.publish(pause_msg)

        except AttributeError:
            pass


    def on_release(self, key):
        try:
            if key.char == 'w' or key.char == 's' or key.char == 'd' or key.char == 'a' or  key.char == 'r' or key.char == 'e' or key.char == 'f' or key == keyboard.Key.space:
                self.speed = 0
                self.steering_angle = 0
            elif key == keyboard.Key.esc:
                rospy.signal_shutdown('closed by Esc')
                # Stop listener
                return False
        except AttributeError:
            pass

    def publish(self, pub):
        #rospy.loginfo("publish")
        if self.steering_mode == 1:
            self.msg_manual.speed = self.speed
            self.msg_manual.steering_angle = self.steering_angle
            pub.publish(self.msg_manual)

        elif self.steering_mode == 2:
            #self.msg_semi_auto.drive.speed = 10
            pub.publish(self.msg_semi_auto)

        elif self.steering_mode == 3:
            pub.publish(self.msg_full_auto)

## main
if __name__ == '__main__':
    rospy.init_node('sim_manager', anonymous=True)

    default_speed = rospy.get_param('~default_speed', 1)
    publish_rate = rospy.get_param('~publish_rate', 10)

    rospy.loginfo('default_steering_speed = ' + str(default_speed))
    rospy.loginfo('sim_controls_publish_rate = ' + str(publish_rate))

    cmd_pub = rospy.Publisher('/target_control', MPCControl, queue_size=1)
    pause_pub = rospy.Publisher('sim_pause', Bool, queue_size=1)

    manager = msg_manager(default_speed=default_speed, pause_pub=pause_pub)
    rospy.Subscriber('/mpc_control', MPCControl, manager.cmd_callback)

    listener = keyboard.Listener(
                on_press=manager.on_press,
                on_release=manager.on_release)
    listener.start()
    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        manager.publish(cmd_pub)
        rate.sleep()

    rospy.signal_shutdown("manually closed")