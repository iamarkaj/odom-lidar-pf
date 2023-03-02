#!/usr/bin/env python

import cv2
import math
import bisect
import rospy
import numpy as np
from tf import transformations
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError


#####################
# TOOL
#####################


def xyzrpy2mat(x, y, z, roll, pitch, yaw):
    rot_vec = np.zeros((3, 1), dtype=np.float32)
    rot_vec[0] = roll
    rot_vec[1] = pitch
    rot_vec[2] = yaw
    rot_mat, _ = cv2.Rodrigues(rot_vec)
    result = np.zeros((4, 4), dtype=np.float32)
    result[0:3, 0:3] = rot_mat
    result[0, 3] = x
    result[1, 3] = y
    result[2, 3] = z
    result[3, 3] = 1
    return result


def mat2xyzrpy(mat):
    result = np.zeros(6)
    result[0] = mat[0, 3]
    result[1] = mat[1, 3]
    result[2] = mat[2, 3]
    rot_mat = mat[0:3, 0:3]
    rot_vec, _ = cv2.Rodrigues(rot_mat)
    result[3] = rot_vec[0]
    result[4] = rot_vec[1]
    result[5] = rot_vec[2]
    return result


#####################
# MCL
#####################


class Particle:
    def __init__(self):
        self.pose = np.eye(4, dtype=np.float32)
        self.score = np.float32(0.0)
        self.scan = np.zeros((4, 4), dtype=np.float32)


class mcl:
    def __init__(self):
        self.imageResolution = None
        self.mapCenterX = None
        self.mapCenterY = None
        self.particles = []
        self.maxProbParticle = Particle()
        self.gridMap = None
        self.poseMap = None
        self.particlesMap = None
        self.odomBefore = None
        self.numOfParticle = 2500
        self.minOdomDistance = 0.00003
        self.minOdomAngle = 10.0
        self.repropagateCountNeeded = 1
        self.odomCovariance = np.array(
            [0.02, 0.02, 0.02, 0.02, 0.02, 0.02]).astype(np.float32)
        self.tf_laser2robot = np.array([[1, 0, 0, 0],
                                        [0, -1, 0, 0],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]]).astype(np.float32)
        self.isOdomInitialized = False
        self.predictionCounter = 0

        self.bridge = CvBridge()
        self.showmap_pub = rospy.Publisher(
            "/image/showmap", Image, queue_size=1)
        self.posemap_pub = rospy.Publisher(
            "/image/posemap", Image, queue_size=1)

        self.getMap()
        self.initializeParticles()
        self.showInMap()

    def getMap(self):
        map_msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=10)

        i = 0
        self.gridMap = np.zeros(
            (map_msg.info.height, map_msg.info.width), dtype=np.uint8)
        for x in range(map_msg.info.height):
            for y in range(map_msg.info.width):
                if map_msg.data[i] == 100:  # occupied
                    self.gridMap[y, x] = 255
                elif map_msg.data[i] == -1:  # free
                    self.gridMap[y, x] = 1
                else:  # unknown
                    self.gridMap[y, x] = 0
                i += 1

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3), (1, 1))
        self.gridMap = cv2.dilate(self.gridMap, kernel)

        _, self.poseMap = cv2.threshold(
            self.gridMap, 50, 255, cv2.THRESH_BINARY_INV)
        self.poseMap = cv2.cvtColor(self.poseMap, cv2.COLOR_GRAY2BGR)
        self.particlesMap = self.poseMap.copy()

        self.mapCenterX = int(map_msg.info.origin.position.x +
                              map_msg.info.width * map_msg.info.resolution / 2.0)
        self.mapCenterY = int(map_msg.info.origin.position.y +
                              map_msg.info.height * map_msg.info.resolution / 2.0)
        self.imageResolution = map_msg.info.resolution

    def initializeParticles(self):
        rospy.loginfo("Starting particle initialization")
        self.particles = []
        i = 0
        while i != self.numOfParticle:
            particle_temp = Particle()
            randomX = np.random.uniform(self.mapCenterX - self.gridMap.shape[1] * self.imageResolution / 2.0,
                                        self.mapCenterX + self.gridMap.shape[1] * self.imageResolution / 2.0)
            randomY = np.random.uniform(self.mapCenterY - self.gridMap.shape[0] * self.imageResolution / 2.0,
                                        self.mapCenterY + self.gridMap.shape[0] * self.imageResolution / 2.0)
            randomTheta = np.random.uniform(-math.pi, math.pi)
            ptX = int((randomX - self.mapCenterX +
                      (self.gridMap.shape[1] * self.imageResolution) / 2) / self.imageResolution)
            ptY = int((randomY - self.mapCenterY +
                      (self.gridMap.shape[0] * self.imageResolution) / 2) / self.imageResolution)
            # if occupied/unknown, skip
            if self.gridMap[ptY, ptX] == 255 or self.gridMap[ptY, ptX] == 1:
                continue
            particle_temp.pose = xyzrpy2mat(
                randomX, randomY, 0, 0, 0, randomTheta)
            particle_temp.score = 1 / self.numOfParticle
            self.particles.append(particle_temp)
            i += 1
        rospy.loginfo("Completed")

    def prediction(self, diffPose):
        diff_xyzrpy = mat2xyzrpy(diffPose)

        delta_trans = math.sqrt(
            pow(diff_xyzrpy[0], 2) + pow(diff_xyzrpy[1], 2))
        delta_rot1 = math.atan2(diff_xyzrpy[1], diff_xyzrpy[0])
        delta_rot2 = diff_xyzrpy[5] - delta_rot1

        if delta_rot1 > math.pi:
            delta_rot1 -= (2 * math.pi)
        if delta_rot1 < -math.pi:
            delta_rot1 += (2 * math.pi)
        if delta_rot2 > math.pi:
            delta_rot2 -= (2 * math.pi)
        if delta_rot2 < -math.pi:
            delta_rot2 += (2 * math.pi)

        # Add noises to trans/rot1/rot2
        trans_noise_coeff = self.odomCovariance[2] * abs(
            delta_trans) + self.odomCovariance[3] * abs(delta_rot1 + delta_rot2)
        rot1_noise_coeff = self.odomCovariance[0] * abs(
            delta_rot1) + self.odomCovariance[1] * abs(delta_trans)
        rot2_noise_coeff = self.odomCovariance[0] * abs(
            delta_rot2) + self.odomCovariance[1] * abs(delta_trans)

        for i in range(self.numOfParticle):
            gaussian_distribution = np.random.normal(0, 1)

            delta_trans = delta_trans + gaussian_distribution * trans_noise_coeff
            delta_rot1 = delta_rot1 + gaussian_distribution * rot1_noise_coeff
            delta_rot2 = delta_rot2 + gaussian_distribution * rot2_noise_coeff

            x = delta_trans * \
                math.cos(delta_rot1) + gaussian_distribution * \
                self.odomCovariance[4]
            y = delta_trans * \
                math.sin(delta_rot1) + gaussian_distribution * \
                self.odomCovariance[5]
            theta = delta_rot1 + delta_rot2 + gaussian_distribution * \
                self.odomCovariance[0] * (math.pi / 180.0)

            diff_odom_w_noise = xyzrpy2mat(x, y, 0, 0, 0, -theta)
            pose_t_plus_1 = self.particles[i].pose @ diff_odom_w_noise
            self.particles[i].pose = pose_t_plus_1

    def weightning(self, laser):
        maxScore = np.float32(0.0)

        for i in range(self.numOfParticle):
            # now this is lidar sensor's frame
            transLaser = (self.particles[i].pose @ self.tf_laser2robot) @ laser
            calcedWeight = np.float32(0.0)

            for j in range(transLaser.shape[1]):
                ptX = int((transLaser[0, j] - self.mapCenterX + (
                    self.gridMap.shape[1] * self.imageResolution) / 2) / self.imageResolution)
                ptY = int((transLaser[1, j] - self.mapCenterY + (
                    self.gridMap.shape[0] * self.imageResolution) / 2) / self.imageResolution)

                if ptX < 0 or ptX >= self.gridMap.shape[1] or ptY < 0 or ptY >= self.gridMap.shape[0]:
                    continue
                elif self.gridMap[ptY, ptX] == 0:
                    continue
                elif self.gridMap[ptY, ptX] == 1:
                    calcedWeight = np.float32(
                        calcedWeight - (self.gridMap[ptY, ptX] / 255.0) / transLaser.shape[1])
                else:
                    calcedWeight = np.float32(
                        calcedWeight + (self.gridMap[ptY, ptX] / 255.0) / transLaser.shape[1])

            self.particles[i].score = self.particles[i].score + calcedWeight

            if maxScore < self.particles[i].score:
                self.maxProbParticle = self.particles[i]
                self.maxProbParticle.scan = laser
                maxScore = self.particles[i].score

        scoreArr = np.array(
            [particle.score for particle in self.particles]).astype(np.float32)
        scoreSum = np.sum(scoreArr).astype(np.float32)
        scoreNorm = scoreArr / scoreSum

        for i, particle in enumerate(self.particles):
            particle.score = scoreNorm[i]

    def resampling(self):
        particleScores = []
        particleSampled = []
        scoreBaseline = np.float32(0.0)

        for particle in self.particles:
            scoreBaseline = scoreBaseline + particle.score
            particleScores.append(scoreBaseline)

        for i in range(self.numOfParticle):
            darted = np.random.uniform(0.0, scoreBaseline)
            particleIndex = bisect.bisect_left(particleScores, darted)
            selectedParticle = self.particles[particleIndex]
            selectedParticle.score = 1.0 / self.numOfParticle
            particleSampled.append(selectedParticle)

        self.particles = particleSampled

    def showInMap(self):
        showMap = self.particlesMap.copy()

        for i in range(self.numOfParticle):
            xPos = int((self.particles[i].pose[0, 3] - self.mapCenterX + (
                self.gridMap.shape[1] * self.imageResolution) / 2) / self.imageResolution)
            yPos = int((self.particles[i].pose[1, 3] - self.mapCenterY + (
                self.gridMap.shape[0] * self.imageResolution) / 2) / self.imageResolution)

            cv2.circle(showMap, (xPos, yPos), 1, (255, 0, 0), -1)

        if self.maxProbParticle.score > 0:
            # Estimate position using all particles
            x_all = 0
            y_all = 0
            for i in range(self.numOfParticle):
                x_all += self.particles[i].pose[0, 3] * self.particles[i].score
                y_all += self.particles[i].pose[1, 3] * self.particles[i].score

            xPos = int((x_all - self.mapCenterX +
                       (self.gridMap.shape[1] * self.imageResolution) / 2) / self.imageResolution)
            yPos = int((y_all - self.mapCenterY +
                       (self.gridMap.shape[0] * self.imageResolution) / 2) / self.imageResolution)

            cv2.circle(showMap, (xPos, yPos), 2, (0, 0, 255), -1)
            cv2.circle(self.poseMap, (xPos, yPos), 1, (0, 0, 255), -1)

            transLaser = (self.maxProbParticle.pose @
                          self.tf_laser2robot) @ self.maxProbParticle.scan

            for i in range(transLaser.shape[1]):
                xPos = int((transLaser[0, i] - self.mapCenterX + (
                    self.gridMap.shape[1] * self.imageResolution) / 2) / self.imageResolution)
                yPos = int((transLaser[1, i] - self.mapCenterY + (
                    self.gridMap.shape[0] * self.imageResolution) / 2) / self.imageResolution)

                cv2.circle(showMap, (xPos, yPos), 1, (0, 255, 255), -1)

        SET_MAP_SIZE = 400.0
        showMapCropped = showMap[1900:2100, 1900:2100]
        poseMapCropped = self.poseMap[1900:2100, 1900:2100]
        showMapCropped = cv2.resize(showMapCropped, (int(SET_MAP_SIZE), int(
            SET_MAP_SIZE)), interpolation=cv2.INTER_LINEAR)
        poseMapCropped = cv2.resize(poseMapCropped, (int(SET_MAP_SIZE), int(
            SET_MAP_SIZE)), interpolation=cv2.INTER_LINEAR)

        try:
            self.showmap_pub.publish(
                self.bridge.cv2_to_imgmsg(showMapCropped, "bgr8"))
            self.posemap_pub.publish(
                self.bridge.cv2_to_imgmsg(poseMapCropped, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def updateData(self, pose, laser):
        if not self.isOdomInitialized:
            self.odomBefore = pose
            self.isOdomInitialized = True
            return

        diffOdom = np.linalg.inv(self.odomBefore) * pose

        self.prediction(diffOdom)
        self.weightning(laser)
        self.predictionCounter += 1
        if self.predictionCounter == self.repropagateCountNeeded:
            self.resampling()
            self.predictionCounter = 0
        self.showInMap()
        self.odomBefore = pose


#####################
# MCL NODE
#####################


class MclNode(object):
    def __init__(self):
        self.mclocalizer = mcl()
        self.vec_poses = []
        self.vec_poses_time = []
        self.vec_lasers = []
        self.vec_lasers_time = []
        self.subscribe_laser = rospy.Subscriber(
            '/laser/scan', LaserScan, self.callback_laser, queue_size=10)
        self.subscribe_pose = rospy.Subscriber(
            '/odom', Odometry, self.callback_pose, queue_size=10)

    def check_data(self):
        while (len(self.vec_poses) != 0 and len(self.vec_lasers) != 0):
            if (np.abs(self.vec_poses_time[0] - self.vec_lasers_time[0]) > 0.1):
                if (self.vec_poses_time[0] > self.vec_lasers_time[0]):
                    self.vec_lasers.pop(0)
                    self.vec_lasers_time.pop(0)
                else:
                    self.vec_poses.pop(0)
                    self.vec_poses_time.pop(0)
            else:
                mat_poses = np.asarray(self.vec_poses[0])
                mat_lasers = np.asarray(self.vec_lasers[0])
                self.mclocalizer.updateData(mat_poses, mat_lasers)
                self.vec_lasers.pop(0)
                self.vec_lasers_time.pop(0)
                self.vec_poses.pop(0)
                self.vec_poses_time.pop(0)

    def callback_laser(self, msg):
        scanQuantity = int((msg.angle_max - msg.angle_min) /
                           msg.angle_increment + 1)
        eigenLaser = np.ones((4, 1))
        scanEffective = 0
        for i in range(scanQuantity):
            dist = msg.ranges[i]
            if (dist > 0.12 and dist < 10.0):
                scanEffective += 1
                eigenLaser = np.resize(eigenLaser, (4, scanEffective))
                eigenLaser[0, scanEffective-1] = dist * \
                    np.cos(msg.angle_min + msg.angle_increment * i)
                eigenLaser[1, scanEffective-1] = dist * \
                    np.sin(msg.angle_min + msg.angle_increment * i)
                eigenLaser[2, scanEffective-1] = 0
                eigenLaser[3, scanEffective-1] = 1
        self.vec_lasers.append(eigenLaser)
        self.vec_lasers_time.append(msg.header.stamp.to_sec())
        self.check_data()

    def callback_pose(self, msg):
        position = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        orientation = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                               msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        eigenPose = transformations.quaternion_matrix(orientation)
        eigenPose[0:3, 3] = position
        self.vec_poses.append(eigenPose)
        self.vec_poses_time.append(msg.header.stamp.to_sec())


def main():
    rospy.init_node('mcl_python')
    MclNode()
    rospy.spin()


if __name__ == '__main__':
    main()
