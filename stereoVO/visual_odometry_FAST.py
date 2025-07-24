import numpy as np
import cv2

# SVO Additional Header
import inlierDetector
from scipy.optimize import least_squares
from helperFunctions import genEulerZXZMatrix, minimizeReprojection, generate3DPoints

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2
kMinNumFeature = 500

lk_params = dict(winSize  = (21, 21),
				#maxLevel = 3,
				criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

def featureTracking(image_ref, image_cur, px_ref):
	# VO : Feature Matching Based Optical Flow
	# VO : KLT Optical Flow
	kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)  #shape: [k,2] [k,1] [k,1]

	st = st.reshape(st.shape[0])
	kp1 = px_ref[st == 1]
	kp2 = kp2[st == 1]

	return kp1, kp2

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
	def __init__(self, camL, camR, annotations):
		self.frame_stage = 0
		self.camL = camL
		self.camR = camR
		self.new_frame = None
		self.last_frame = None
		self.cur_R = None
		self.cur_t = None
		self.px_ref_L = None
		self.px_cur_L = None
		self.px_ref_R = None
		self.px_cur_R = None
		self.ref_disparity = None
		self.cur_disparity = None
		self.focal = camL.fx
		self.pp = (camL.cx, camL.cy)
		self.trueX, self.trueY, self.trueZ = 0, 0, 0
		self.color = 255
		self.visual = True
		self.disparityMinThres = 0.0
		self.disparityMaxThres = 100.0
		self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
		self.useRansac = False
		#self.detector = cv2.ORB_create(nfeatures=3000, scaleFactor=1.2, nlevels=8, edgeThreshold=31, firstLevel=0, WTA_K=2, scoreType=cv2.ORB_HARRIS_SCORE, patchSize=31, fastThreshold=20)

		# SVO Parameter Set
		block = 11
		P1 = block * block * 8
		P2 = block * block * 32
		self.disparityEngine = cv2.StereoSGBM_create(minDisparity=0,numDisparities=32, blockSize=block, P1=P1, P2=P2)

		# SVO Projection Matrix Calculation
		self.left_projection = np.array([[7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 0.000000000000e+00],[0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00],[0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00]])
		self.right_projection = np.array([[7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, -3.861448000000e+02],[0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, 0.000000000000e+00],[0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 0.000000000000e+00]])

		with open(annotations) as f:
			self.annotations = f.readlines()

	# VO : Visualize ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	def visualizeFeature(self, img):
		# draw the tracks
		if self.visual == True:
			maskT = np.zeros_like(img)
			for i,(new,old) in enumerate(zip(self.px_cur,self.px_ref)):
				a,b = new.ravel()
				c,d = old.ravel()
				maskT = cv2.line(maskT, (a,b),(c,d), self.color, 2)
				# img = cv2.circle(img,(a,b),5,self.color,-1)
			img_modified = cv2.add(img,maskT)
			return img_modified
		else:
			img_modified = img
			return img_modified

	# VO : KITTI Metric Scale Recover --------------------------------------------------------------------------------------------------------------------------------------------------------------
	def getGrountTruth(self, frame_id):
		ss = self.annotations[frame_id].strip().split()
		x = float(ss[3])
		y = float(ss[7])
		z = float(ss[11])
		self.trueX, self.trueY, self.trueZ = x, y, z

	# VO : Feature Extract First Frame -------------------------------------------------------------------------------------------------------------------------------------------------------------
	def processFirstFrame(self):
		print('VO : Process First Frame \n')
		# SVO : Feature Detector
		self.px_ref_L = self.detector.detect(self.new_frame_L)
		self.px_ref_L = np.array([x.pt for x in self.px_ref_L], dtype=np.float32)

		# SVO : Disparity Calculation (for Triangulation)
		self.ref_disparity = self.disparityEngine.compute(self.new_frame_L, self.new_frame_R).astype(np.float32)
		self.ref_disparity = np.divide(self.ref_disparity, 16.0)
		self.frame_stage = STAGE_SECOND_FRAME

	# VO : Calculate Rel. Pos between First-Second Frame (Initialization) --------------------------------------------------------------------------------------------------------------------------
	def processSecondFrame(self):
		print('VO : Process Second Frame \n')

		# SVO : Feature Detector
		H,W = self.new_frame_L.shape
		self.px_ref_L, self.px_cur_L = featureTracking(self.last_frame_L, self.new_frame_L, self.px_ref_L)

		# SVO : Disparity Calculation (Triangulation)
		self.cur_disparity = self.disparityEngine.compute(self.new_frame_L, self.new_frame_R).astype(np.float32)
		self.cur_disparity = np.divide(self.cur_disparity, 16.0)

		# SVO : Filter Feature Point Inlier
		E, maskE = cv2.findEssentialMat(self.px_ref_L, self.px_cur_L, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
		self.px_ref_L = self.px_ref_L[maskE.ravel()==1]
		self.px_cur_L = self.px_cur_L[maskE.ravel()==1]

		# SVO : Filter Max Window Img Size
		hPts = np.where(self.px_cur_L[:,1] >= H)
		wPts = np.where(self.px_cur_L[:,0] >= W)
		outTrackPts = hPts[0].tolist() + wPts[0].tolist()
		outDeletePts = list(set(outTrackPts))

		if len(outDeletePts) > 0:
			self.px_ref_L = np.delete(self.px_ref_L, outDeletePts, axis=0)
			self.px_cur_L = np.delete(self.px_cur_L, outDeletePts, axis=0)
		else:
			self.px_ref_L = self.px_ref_L
			self.px_cur_L = self.px_cur_L

		# SVO : Right Feature Correction with Calculated Dispairty
		self.px_ref_R = np.copy(self.px_ref_L)
		self.px_cur_R = np.copy(self.px_cur_L)
		selectedPointMap = np.zeros(self.px_ref_L.shape[0])

		for i in range(self.px_ref_L.shape[0]):
			T1Disparity = self.ref_disparity[int(self.px_ref_L[i,1]), int(self.px_ref_L[i,0])]
			T2Disparity = self.cur_disparity[int(self.px_cur_L[i,1]), int(self.px_cur_L[i,0])]

			if (T1Disparity > self.disparityMinThres and T1Disparity < self.disparityMaxThres
				and T2Disparity > self.disparityMinThres and T2Disparity < self.disparityMaxThres):
				self.px_ref_R[i, 0] = self.px_ref_L[i, 0] - T1Disparity
				self.px_cur_R[i, 0] = self.px_cur_L[i, 0] - T2Disparity
				selectedPointMap[i] = 1

		selectedPointMap = selectedPointMap.astype(bool)
		trackPoints1_KLT_L_3d = self.px_ref_L[selectedPointMap, ...]
		trackPoints1_KLT_R_3d = self.px_ref_R[selectedPointMap, ...]
		trackPoints2_KLT_L_3d = self.px_cur_L[selectedPointMap, ...]
		trackPoints2_KLT_R_3d = self.px_cur_R[selectedPointMap, ...]

		# SVO : 3d point cloud triagulation
		numPoints = trackPoints1_KLT_L_3d.shape[0]
		d3dPointsT1 = generate3DPoints(trackPoints1_KLT_L_3d, trackPoints1_KLT_R_3d, self.left_projection, self.right_projection)
		d3dPointsT2 = generate3DPoints(trackPoints2_KLT_L_3d, trackPoints2_KLT_R_3d, self.left_projection, self.right_projection)

		# SVO : RANSAC (ONLY SECOND FRAME for Stable Initialization)
		ransacError = float('inf')
		dOut = None
		ransacSize = 6

		for ransacItr in range(250):
			sampledPoints = np.random.randint(0, numPoints, ransacSize)
			rD2dPoints1_L = trackPoints1_KLT_L_3d[sampledPoints]
			rD2dPoints2_L = trackPoints2_KLT_L_3d[sampledPoints]
			rD3dPointsT1 = d3dPointsT1[sampledPoints]
			rD3dPointsT2 = d3dPointsT2[sampledPoints]

			dSeed = np.zeros(6)
			#minimizeReprojection(d, trackedPoints1_KLT_L, trackedPoints2_KLT_L, cliqued3dPointT1, cliqued3dPointT2, Proj1)
			optRes = least_squares(minimizeReprojection, dSeed, method='lm', max_nfev=200,
								args=(rD2dPoints1_L, rD2dPoints2_L, rD3dPointsT1, rD3dPointsT2, self.left_projection))

			#error = optRes.fun
			error = minimizeReprojection(optRes.x, trackPoints1_KLT_L_3d, trackPoints2_KLT_L_3d,
											d3dPointsT1, d3dPointsT2, self.left_projection)

			eCoords = error.reshape((d3dPointsT1.shape[0]*2, 3))
			totalError = np.sum(np.linalg.norm(eCoords, axis=1))

			if (totalError < ransacError):
				ransacError = totalError
				print(ransacError)
				dOut = optRes.x

		# SVO : Derive Odometry
		Rmat = genEulerZXZMatrix(dOut[0], dOut[1], dOut[2])
		translationArray = np.array([[dOut[3]], [dOut[4]], [dOut[5]]])

		# -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		self.cur_R = Rmat
		self.cur_t = translationArray

		self.px_ref_L = self.px_cur_L
		self.ref_disparity = self.cur_disparity
		self.frame_stage = STAGE_DEFAULT_FRAME

	# VO : Process All Frame -----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	def processFrame(self, img_L, img_R, frame_id):
		# -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		print('VO : Process Main Frame \n')
		# SVO : Feature Detector
		H,W = self.new_frame_L.shape
		self.px_ref_L, self.px_cur_L = featureTracking(self.last_frame_L, self.new_frame_L, self.px_ref_L)
		E, maskE = cv2.findEssentialMat(self.px_ref_L, self.px_cur_L, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
		# SVO : Filter Feature Point Inlier
		self.px_ref_L = self.px_ref_L[maskE.ravel()==1]
		self.px_cur_L = self.px_cur_L[maskE.ravel()==1]

		# SVO : Disparity Calculation (Triangulation)
		self.cur_disparity = self.disparityEngine.compute(self.new_frame_L, self.new_frame_R).astype(np.float32)
		self.cur_disparity = np.divide(self.cur_disparity, 16.0)

		# SVO : Filter Max Window Img Size
		hPts = np.where(self.px_cur_L[:,1] >= H)
		wPts = np.where(self.px_cur_L[:,0] >= W)
		outTrackPts = hPts[0].tolist() + wPts[0].tolist()
		outDeletePts = list(set(outTrackPts))

		if len(outDeletePts) > 0:
			self.px_ref_L = np.delete(self.px_ref_L, outDeletePts, axis=0)
			self.px_cur_L = np.delete(self.px_cur_L, outDeletePts, axis=0)
		else:
			self.px_ref_L = self.px_ref_L
			self.px_cur_L = self.px_cur_L

		# SVO : Cut-Off Maximum Feature Points
		if self.px_ref_L.shape[0] < 500:
			numPoints_Selection = self.px_ref_L.shape[0]
		else:
			numPoints_Selection = 500

		sampledPoints = np.random.randint(0, self.px_ref_L.shape[0], numPoints_Selection)
		self.px_ref_L = self.px_ref_L[sampledPoints]
		self.px_cur_L = self.px_cur_L[sampledPoints]

		self.px_ref_R = np.copy(self.px_ref_L)
		self.px_cur_R = np.copy(self.px_cur_L)
		selectedPointMap = np.zeros(self.px_ref_L.shape[0])

		for i in range(self.px_ref_L.shape[0]):
			T1Disparity = self.ref_disparity[int(self.px_ref_L[i,1]), int(self.px_ref_L[i,0])]
			T2Disparity = self.cur_disparity[int(self.px_cur_L[i,1]), int(self.px_cur_L[i,0])]

			if (T1Disparity > self.disparityMinThres and T1Disparity < self.disparityMaxThres
				and T2Disparity > self.disparityMinThres and T2Disparity < self.disparityMaxThres):
				self.px_ref_R[i, 0] = self.px_ref_L[i, 0] - T1Disparity
				self.px_cur_R[i, 0] = self.px_cur_L[i, 0] - T2Disparity
				selectedPointMap[i] = 1

		selectedPointMap = selectedPointMap.astype(bool)
		trackPoints1_KLT_L_3d = self.px_ref_L[selectedPointMap, ...]
		trackPoints1_KLT_R_3d = self.px_ref_R[selectedPointMap, ...]
		trackPoints2_KLT_L_3d = self.px_cur_L[selectedPointMap, ...]
		trackPoints2_KLT_R_3d = self.px_cur_R[selectedPointMap, ...]

		# SVO : 3d point cloud triagulation
		numPoints = trackPoints1_KLT_L_3d.shape[0]
		d3dPointsT1 = generate3DPoints(trackPoints1_KLT_L_3d, trackPoints1_KLT_R_3d, self.left_projection, self.right_projection)
		d3dPointsT2 = generate3DPoints(trackPoints2_KLT_L_3d, trackPoints2_KLT_R_3d, self.left_projection, self.right_projection)

		print(numPoints)

		ransacError = float('inf')
		dOut = None

		# SVO : RANSAC Process 3D-2D Correspondence Optimization with R,t
		if self.useRansac:
			# RANSAC
			ransacSize = 6
			for ransacItr in range(250):
				sampledPoints = np.random.randint(0, numPoints, ransacSize)
				rD2dPoints1_L = trackPoints1_KLT_L_3d[sampledPoints]
				rD2dPoints2_L = trackPoints2_KLT_L_3d[sampledPoints]
				rD3dPointsT1 = d3dPointsT1[sampledPoints]
				rD3dPointsT2 = d3dPointsT2[sampledPoints]

				dSeed = np.zeros(6)
				#minimizeReprojection(d, trackedPoints1_KLT_L, trackedPoints2_KLT_L, cliqued3dPointT1, cliqued3dPointT2, Proj1)
				optRes = least_squares(minimizeReprojection, dSeed, method='lm', max_nfev=200,
									args=(rD2dPoints1_L, rD2dPoints2_L, rD3dPointsT1, rD3dPointsT2, self.left_projection))

				#error = optRes.fun
				error = minimizeReprojection(optRes.x, trackPoints1_KLT_L_3d, trackPoints2_KLT_L_3d,
												d3dPointsT1, d3dPointsT2, self.left_projection)

				eCoords = error.reshape((d3dPointsT1.shape[0]*2, 3))
				totalError = np.sum(np.linalg.norm(eCoords, axis=1))

				if (totalError < ransacError):
					ransacError = totalError
					dOut = optRes.x

				#clique size check
				# reproj error check
				# r, t generation
			Rmat = genEulerZXZMatrix(dOut[0], dOut[1], dOut[2])
			translationArray = np.array([[dOut[3]], [dOut[4]], [dOut[5]]])

		else:
			#tunable - def 0.01
			distDifference = 0.2

			lClique = 0
			clique = []
			while lClique < 6 and d3dPointsT1.shape[0] >= 6:
				# in-lier detection algorithm
				clique = inlierDetector.findClique(d3dPointsT1, d3dPointsT2, distDifference)
				lClique = len(clique)
				distDifference *= 2
			#print (frm, trackPoints1_KLT.shape[0],len(outDeletePts), len(selectedPointMap),d3dPointsT1.shape[0], len(clique))
			# pick up clique point 3D coords and features for optimization
			pointsInClique = len(clique)

			cliqued3dPointT1 = d3dPointsT1[clique]#np.zeros((pointsInClique, 3))
			cliqued3dPointT2 = d3dPointsT2[clique]

			# points = features
			trackedPoints1_KLT_L = trackPoints1_KLT_L_3d[clique]
			trackedPoints2_KLT_L = trackPoints2_KLT_L_3d[clique]

			if (trackedPoints1_KLT_L.shape[0] >= 6):
				dSeed = np.zeros(6)
				#minimizeReprojection(d, trackedPoints1_KLT_L, trackedPoints2_KLT_L, cliqued3dPointT1, cliqued3dPointT2, Proj1)
				optRes = least_squares(minimizeReprojection, dSeed, method='lm', max_nfev=200,
									args=(trackedPoints1_KLT_L, trackedPoints2_KLT_L, cliqued3dPointT1, cliqued3dPointT2, self.left_projection))

				error = optRes.fun
				pointsInClique = len(clique)
				e = error.reshape((pointsInClique*2, 3))
				errorThreshold = 0.5
				xRes1 = np.where(e[0:pointsInClique, 0] >= errorThreshold)
				yRes1 = np.where(e[0:pointsInClique, 1] >= errorThreshold)
				zRes1 = np.where(e[0:pointsInClique, 2] >= errorThreshold)
				xRes2 = np.where(e[pointsInClique:2*pointsInClique, 0] >= errorThreshold)
				yRes2 = np.where(e[pointsInClique:2*pointsInClique, 1] >= errorThreshold)
				zRes2 = np.where(e[pointsInClique:2*pointsInClique, 2] >= errorThreshold)

				pruneIdx = xRes1[0].tolist() + yRes1[0].tolist() + zRes1[0].tolist() + xRes2[0].tolist() + yRes2[0].tolist() +  zRes2[0].tolist()
				if (len(pruneIdx) > 0):
					uPruneIdx = list(set(pruneIdx))
					trackedPoints1_KLT_L = np.delete(trackedPoints1_KLT_L, uPruneIdx, axis=0)
					trackedPoints2_KLT_L = np.delete(trackedPoints2_KLT_L, uPruneIdx, axis=0)
					cliqued3dPointT1 = np.delete(cliqued3dPointT1, uPruneIdx, axis=0)
					cliqued3dPointT2 = np.delete(cliqued3dPointT2, uPruneIdx, axis=0)

					if (trackedPoints1_KLT_L.shape[0] >= 6):
						optRes = least_squares(minimizeReprojection, optRes.x, method='lm', max_nfev=200,
									args=(trackedPoints1_KLT_L, trackedPoints2_KLT_L, cliqued3dPointT1, cliqued3dPointT2, self.left_projection))

				Rmat = genEulerZXZMatrix(optRes.x[0], optRes.x[1], optRes.x[2])
				translationArray = np.array([[optRes.x[3]], [optRes.x[4]], [optRes.x[5]]])

		# -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		# SVO : Odometry Update
		self.cur_t = self.cur_t + self.cur_R.dot(translationArray)
		self.cur_R = Rmat.dot(self.cur_R)

		if(self.px_ref_L.shape[0] < kMinNumFeature):
			self.px_cur_L = self.detector.detect(self.new_frame_L)
			self.px_cur_L = np.array([x.pt for x in self.px_cur_L], dtype=np.float32)

		self.px_ref_L = self.px_cur_L
		self.ref_disparity = self.cur_disparity
		self.frame_stage = STAGE_DEFAULT_FRAME

		self.getGrountTruth(frame_id)

		img_modified = img_L
		return img_modified

	# VO : Process Tree ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	def update(self, imgL, imgR, frame_id):
		assert(imgL.ndim==2 and imgL.shape[0]==self.camL.height and imgL.shape[1]==self.camL.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"
		self.new_frame_L = imgL
		self.new_frame_R = imgR
		if(self.frame_stage == STAGE_DEFAULT_FRAME):
			imgL = self.processFrame(imgL, imgR, frame_id)
		elif(self.frame_stage == STAGE_SECOND_FRAME):
			self.processSecondFrame()
		elif(self.frame_stage == STAGE_FIRST_FRAME):
			self.processFirstFrame()

		self.last_frame_L = self.new_frame_L
		self.last_frame_R = self.new_frame_R
		return imgL
