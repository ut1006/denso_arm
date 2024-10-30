import rospy
import tf

# TFリスナーを初期化
rospy.init_node('tf_listener')
listener = tf.TransformListener()

def print_tf_data():
    rate = rospy.Rate(1)  # 1Hzで実行
    while not rospy.is_shutdown():
        try:
            # base_linkからzedm_left_camera_frameまでのTFを取得
            (trans, rot) = listener.lookupTransform('base_link', 'zedm_left_camera_frame', rospy.Time(0))
            
            # 取得した座標変換を表示
            print(f"Translation: {trans}")
            print(f"Rotation: {rot}")
            print("_________________________________________")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Transform not available.")
        
        # 一定間隔で取得
        rate.sleep()

if __name__ == "__main__":
    print_tf_data()
"""
close to plants
Translation: [0.31080178659642416, 0.10599170797615597, 0.8441504761778316]
Rotation (Quaternion): [0.05286531579966582, 0.652797351214838, 0.00947469485383175, 0.7556263010177188]



all 0
Translation: [-0.010959981492226276, 0.0299629505854618, 1.0662479644125242]
Rotation (Quaternion): [-9.662191087222998e-06, 0.0026609570164876483, 0.0030596183441807556, 0.9999917789412004]
くりあ０


j1=90
Translation: [-0.030004688994989426, -0.010922088933003608, 1.0662468317290497]
Rotation: [-0.0018422074252879381, 0.0018356642418455502, 0.70793398350983, 0.706273821970274]
くりあ[ 9.01345185e+01 -8.80022046e-04 -2.98011817e-01]


j1=0, j2=37お辞儀
Translation: [0.42986457224619806, 0.030298018742749778, 0.9242367555130419]
Rotation: [0.00028957596498370663, 0.32349657028279294, 0.0015241885925268348, 0.9462280708212819]
くりあ[ 1.95438389e-01  3.77490344e+01 -3.17478062e-02]

j6=90
Translation: [-0.04126119801727501, -0.00010665571181086001, 1.066396705193538]
Rotation: [0.0018581119219741977, 0.0018552135987737373, 0.7082588422647021, 0.7059479569744865]
[ 9.01872489e+01  3.00885408e-01 -2.56817030e-04]


"""