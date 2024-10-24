import os
import glob
import shutil
# dir_name = "/home/kamiya/data/*"
dir_name = "/media/kamiya/Extreme SSD/0627/*"
files = glob.glob(dir_name)
print(files)

for f in files:
    ftitle, fext = os.path.splitext(f)
    bag_name = ftitle.split('/')[-1]
    print(bag_name)
    bashCommand = "roslaunch extract_image_from_bag.launch filename:="+bag_name
    os.system(bashCommand)

    # shutil.copy(f, '../renamed'+dir_name+'/' + date + img_name + fext)