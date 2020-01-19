import numpy as np 
import cv2
import math
import time

STAGE_FIRST_FRAME = 0
STAGE_SECOND_FRAME = 1
STAGE_DEFAULT_FRAME = 2
kMinNumFeature = 1500

lk_params = dict(winSize  = (21, 21), 
				#maxLevel = 3,
             	criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

class PinholeCamera:
	def __init__(self, width, height, fx, fy, cx, cy, 
				k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
		self.width = width
		self.height = height
		self.fx = fx
		self.fy = fy
		self.cx = cx
		self.cy = cy
		self.distortion = (abs(k1) > 0.0000001)
		self.d = [k1, k2, p1, p2, k3]

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-2

def rotationMatrixToEulerAngles(R) :
 
    # assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan(R[2,1] / R[2,2])
        y = math.atan(-R[2,0]/ sy)
        z = math.atan(R[1,0]/ R[0,0])
    else :
        x = math.atan(-R[1,2]/ R[1,1])
        y = math.atan(-R[2,0]/ sy)
        z = 0
 
    return 57.3*np.round(np.array([x, y, z]),3)

def set(R):#nonholonomic constraints
	R[0,1] = np.round(R[0,1])
	R[1,0] = np.round(R[1,0])
	R[1,2] = np.round(R[1,2])
	R[2,1] = np.round(R[2,1])
	R[1,1] = np.round(R[1,1])
	if(R[0,2]>0 and R[2,0]<0):
		R[0,2] = (R[0,2] + math.fabs(R[2,0]))/2
	elif(R[0,2]<0 and R[2,0]>0):
		R[0,2] = (R[0,2] - R[2,0])/2
	R[0,2] = -R[2,0]
	R[0,0] = (R[0,0] + R[2,2])/2
	R[2,2] = R[0,0]
	return R



class VisualOdometry:
	def __init__(self, cam, annotations):
		self.frame_stage = 0
		self.cam = cam
		self.v = 0
		self.scale = []
		self.new_frame = None
		self.last_frame = None
		self.cur_R = None
		self.cur_t = None
		self.R_f = None
		self.px_ref = None
		self.distance = 0
		self.px_cur = None
		self.focal = cam.fx
		self.pp = (cam.cx, cam.cy)
		self.trueX, self.trueY, self.trueZ = 0, 0, 0
		self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
		with open(annotations) as f:
			self.annotations = f.readlines()

	def featureTracking(self,image_ref, image_cur, px_ref):
		kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)  #shape: [k,2] [k,1] [k,1]
		st = st.reshape(st.shape[0])
		kp1 = px_ref[st == 1] #keep only those points for which the optical flow was calculated
		kp2 = kp2[st == 1] #list containing positions of points being tracked in 2D pixel coordinates

		return kp1, kp2

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

	def processFirstFrame(self):
		self.px_ref = self.detector.detect(self.new_frame)
		self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)
		self.frame_stage = STAGE_SECOND_FRAME

	def processSecondFrame(self):
		self.px_ref, self.px_cur = self.featureTracking(self.last_frame, self.new_frame, self.px_ref)
		E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
		_, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
		self.frame_stage = STAGE_DEFAULT_FRAME 
		self.px_ref = self.px_cur
		rt = np.hstack([self.cur_R,self.cur_t])
		self.R_f = np.vstack( [ rt, np.array([0,0,0,1]) ] )

	def processFrame(self, frame_id):
		# flow = cv2.calcOpticalFlowFarneback(self.last_frame, self.new_frame,None, 0.5, 3, 15, 3, 5, 1.2, 0)
		# flow_X, flow_Y = -flow[:,:,0], flow[:,:,1] #the x and y components of motion.

		self.px_ref, self.px_cur = self.featureTracking(self.last_frame, self.new_frame, self.px_ref)
		
		E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
		_, R, t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)
		distance_moved = self.getAbsoluteScale(frame_id)

		if(distance_moved > 0.1):
			self.cur_t = self.cur_t + distance_moved*self.cur_R.dot(t)
			self.cur_R = R.dot(self.cur_R)
			pyr = rotationMatrixToEulerAngles(R)
			# flow_X = ((120*flow_X/self.new_frame.shape[1]) - pyr[1])*25
			# flow_Y = ((30*flow_Y/self.new_frame.shape[0]) + pyr[0])*25
			# self.v = np.sqrt( np.add(np.square(flow_X),np.square(flow_Y)) ) #magnitude of motion at each point
			# self.v = np.array(self.v,dtype=np.uint8)
			# cv2.imshow('window',self.v)
			# R = set(R)
			# rt = np.hstack([R,distance_moved*t])
			# pose = np.vstack( [ rt, np.array([0,0,0,1]) ] )
			# self.R_f = np.matmul(self.R_f,pose)
			# self.cur_R = self.R_f[:3,:3]
			# self.cur_t = self.R_f[:3,3]		

		# print(np.round(rotationMatrixToEulerAngles(self.cur_R),2))

		if(self.px_ref.shape[0] < kMinNumFeature):
			self.px_cur = self.detector.detect(self.new_frame)
			self.px_cur = np.array([x.pt for x in self.px_cur], dtype=np.float32)
		self.px_ref = self.px_cur

	def update(self, img, frame_id):
		# assert(img.ndim==2 and img.shape[0]==self.cam.height and img.shape[1]==self.cam.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"
		self.new_frame = img
		if(self.frame_stage == STAGE_DEFAULT_FRAME):
			self.processFrame(frame_id)
		elif(self.frame_stage == STAGE_SECOND_FRAME):
			self.processSecondFrame()
		elif(self.frame_stage == STAGE_FIRST_FRAME):
			self.processFirstFrame()
		self.last_frame = self.new_frame