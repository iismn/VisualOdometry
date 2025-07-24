import numpy as np
import cv2

from visual_odometry_FAST import PinholeCamera, VisualOdometry

# CAM : SET CAMERA PARAMETER
camL = PinholeCamera(1241.0, 376.0, 718.8560, 718.8560, 607.1928, 185.2157)
camR = PinholeCamera(1241.0, 376.0, 718.8560, 718.8560, 607.1928, 185.2157)
vo = VisualOdometry(camL, camR, '/home/iismn/IISMN_DATA/SLAM_SET/KITTI/02/02.txt')

# SVO : EMPTY IMAGE GEN
traj = np.zeros((1200,1200,3), dtype=np.uint8)

for img_id in range(4541):
	# SVO : INPUT IMAGE FROM DATASET
	img_L = cv2.imread('/home/iismn/IISMN_DATA/SLAM_SET/KITTI/02/image_0/'+str(img_id).zfill(6)+'.png', 0)
	img_R = cv2.imread('/home/iismn/IISMN_DATA/SLAM_SET/KITTI/02/image_1/'+str(img_id).zfill(6)+'.png', 0)

	# SVO : UPDATE KEY FRAME
	img_L = vo.update(img_L, img_R, img_id)

	# SVO : ODOMETRY DERIVE
	if(img_id > 2):
		x, y, z = vo.cur_t[0], vo.cur_t[1], vo.cur_t[2]
	else:
		x, y, z = 0., 0., 0.

	# SVO : DRAW TRAJECTORY
	draw_x, draw_y = int(x)+290, int(z)+90
	true_x, true_y = int(vo.trueX)+290, int(vo.trueZ)+90
	cv2.circle(traj, (draw_x,draw_y), 1, (img_id*255/4540,255-img_id*255/4540,0), 1)
	# cv2.circle(traj, (true_x,true_y), 1, (0,0,255), 2)
	cv2.rectangle(traj, (10, 20), (600, 60), (0,0,0), -1)
	text = "Coordinates: x=%2fm y=%2fm z=%2fm"%(x,y,z)
	cv2.putText(traj, text, (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)

	# SVO : IMAGE PLOT
	cv2.imshow('Road facing camera', img_L)
	cv2.imshow('Trajectory', traj)
	cv2.waitKey(1)

# WRITE IMAGE RESULT
cv2.imwrite('map.png', traj)
