import numpy as np
import cv2

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2
kMinNumFeature = 1500

lk_params = dict(winSize  = (21, 21),
				#maxLevel = 3,
				criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

def featureTracking(detector, image_ref, image_cur, kp1_ref, des1_ref):
	# VO : Feature Matching Based Optical Flow
	kp2, des2 = detector.detectAndCompute(image_cur,None)
	bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
	matches = bf.match(des1_ref,des2)

	pt1 = np.empty((1, 2))
	pt2 = np.empty((1, 2))
	for i in range(1,len(matches)):
		## Notice: How to get the index
		pt1 = np.append(pt1,np.array([kp1_ref[matches[i].queryIdx].pt]), axis=0)
		pt2 = np.append(pt2,np.array([kp2[matches[i].trainIdx].pt]), axis=0)

	return pt1, pt2

class PinholeCamera:
	def __init__(self, width, height, fx, fy, cx, cy, k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
		self.width = width
		self.height = height
		self.fx = fx
		self.fy = fy
		self.cx = cx
		self.cy = cy
		self.distortion = (abs(k1) > 0.0000001)
		self.d = [k1, k2, p1, p2, k3]

class VisualOdometry:
	def __init__(self, cam, annotations):
		self.frame_stage = 0
		self.cam = cam
		self.new_frame = None
		self.last_frame = None
		self.cur_R = None
		self.cur_t = None
		self.px_ref = None
		self.des_ref = None
		self.px_cur = None
		self.px_ref_point = None
		self.px_cur_point = None
		self.focal = cam.fx
		self.pp = (cam.cx, cam.cy)
		self.trueX, self.trueY, self.trueZ = 0, 0, 0
		self.color = 255
		self.visual = True
		self.detector = cv2.ORB_create(nfeatures=15000, scaleFactor=1.2, nlevels=8, edgeThreshold=31, firstLevel=0, WTA_K=2, scoreType=cv2.ORB_HARRIS_SCORE, patchSize=31, fastThreshold=20)

		with open(annotations) as f:
			self.annotations = f.readlines()

	# VO : Visualize
	def visualizeFeature(self, img):
		# draw the tracks
		if self.visual == True:
			maskT = np.zeros_like(img)
			for i in range(1,len(self.px_cur_point)):
				maskT = cv2.line(maskT, (self.px_cur_point[i,0].astype('int32'),self.px_cur_point[i,1].astype('int32')),(self.px_ref_point[i,0].astype('int32'),self.px_ref_point[i,1].astype('int32')), self.color, 2)
				# img = cv2.circle(img,(a,b),5,self.color,-1)
			img_modified = cv2.add(img,maskT)
			return img_modified
		else:
			img_modified = img
			return img_modified

	# VO : KITTI Metric Scale Recover
	def getAbsoluteScale(self, frame_id):  #specialized for KITTI odometry dataset
		ss = self.annotations[frame_id-1].strip().split()
		x_prev = float(ss[3])
		y_prev = float(ss[7])
		z_prev = float(ss[11])
		ss = self.annotations[frame_id].strip().split()
		x = float(ss[3])
		y = float(ss[7])
		z = float(ss[11])
		self.trueX, self.trueY, self.trueZ = x, y, z
		return np.sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev))

	# VO : Feature Extract First Frame
	def processFirstFrame(self):
		self.px_ref, self.des_ref = self.detector.detectAndCompute(self.new_frame,None)
		self.frame_stage = STAGE_SECOND_FRAME

	# VO : Calculate Rel. Pos between First-Second Frame (Initialization)
	def processSecondFrame(self):
		self.px_ref_point, self.px_cur_point = featureTracking(self.detector, self.last_frame, self.new_frame, self.px_ref, self.des_ref)
		E, mask = cv2.findEssentialMat(self.px_cur_point, self.px_ref_point, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
		_, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.px_cur_point, self.px_ref_point, focal=self.focal, pp = self.pp)
		self.frame_stage = STAGE_DEFAULT_FRAME
		img_modified = self.visualizeFeature(self.new_frame)
		self.px_ref_point = self.px_cur_point

		return img_modified

		# VO : Process All Frame
	def processFrame(self, img, frame_id):
		self.px_ref, self.des_ref = self.detector.detectAndCompute(self.last_frame,None)
		self.px_ref_point, self.px_cur_point = featureTracking(self.detector, self.last_frame, self.new_frame, self.px_ref, self.des_ref)



		E, maskE = cv2.findEssentialMat(self.px_cur_point, self.px_ref_point, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
		_, R, t, mask = cv2.recoverPose(E, self.px_cur_point, self.px_ref_point, focal=self.focal, pp = self.pp)

		self.px_ref_point = self.px_ref_point[maskE.ravel()==1]
		self.px_cur_point = self.px_cur_point[maskE.ravel()==1]
		img_modified = self.visualizeFeature(img)

		absolute_scale = self.getAbsoluteScale(frame_id)

		if(absolute_scale > 0.1):
			self.cur_t = self.cur_t + absolute_scale*self.cur_R.dot(t)
			self.cur_R = R.dot(self.cur_R)
		if(self.px_ref_point.shape[0] < kMinNumFeature):
			# self.px_cur = self.detector.detect(self.new_frame)
			self.px_cur, self.des_ref = self.detector.detectAndCompute(self.new_frame,None)
			# self.px_cur = np.array([x.pt for x in self.px_cur], dtype=np.float32)
		self.px_ref = self.px_cur

		return img_modified

	# VO : Process Tree
	def update(self, img, frame_id):
		assert(img.ndim==2 and img.shape[0]==self.cam.height and img.shape[1]==self.cam.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"
		self.new_frame = img
		if(self.frame_stage == STAGE_DEFAULT_FRAME):
			img = self.processFrame(img, frame_id)
		elif(self.frame_stage == STAGE_SECOND_FRAME):
			img = self.processSecondFrame()
		elif(self.frame_stage == STAGE_FIRST_FRAME):
			self.processFirstFrame()
		self.last_frame = self.new_frame

		return img
