import rospy
from sensor_msgs.msg import Image
import time
import threading

timer1 = time.time()
timer2 = time.time()

frq = 10


def cam1_pub(image):
    global timer2
    if time.time() - timer2 >= 0.1 : #10hz
        pub1.publish(image)
        timer2 = time.time()
        
def cam2_pub(image):
    global timer1
    if time.time() - timer1 >= 0.1 : #10hz
        pub2.publish(image)
        timer1 = time.time()
        
rospy.init_node('t265_slower')
def receive():
    sub1 = rospy.Subscriber('/camera/fisheye1/image_raw',Image,cam1_pub)
    sub2 = rospy.Subscriber('/camera/fisheye2/image_raw',Image,cam2_pub)


pub1 = rospy.Publisher('t265_slow1', Image, queue_size=10)
pub2 = rospy.Publisher('t265_slow2', Image, queue_size=10)

t1 = threading.Thread(target = receive)

t1.start()
    
rospy.spin()
