import rospy
import tf

def get_transform():
    # ROSノードの初期化
    rospy.init_node('tf_listener', anonymous=True)

    # Transform Listenerの作成
    listener = tf.TransformListener()

    # ループして変換を取得
    rate = rospy.Rate(10.0)  # 10Hzで更新
    while not rospy.is_shutdown():
        try:
            # base_linkからzedm_camera_centerの変換を取得
            (trans, rot) = listener.lookupTransform('base_link', 'zedm_camera_center', rospy.Time(0))
            
            # 取得した変換を表示
            rospy.loginfo("Translation: %s", trans)
            rospy.loginfo("Rotation: %s", rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        rate.sleep()

if __name__ == '__main__':
    get_transform()
