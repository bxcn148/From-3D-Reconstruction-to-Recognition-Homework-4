Download link :https://programming.engineering/product/from-3d-reconstruction-to-recognition-homework-4/


# From-3D-Reconstruction-to-Recognition-Homework-4
From 3D Reconstruction to Recognition Homework #4
This PSET will involve concepts from lectures 14 onwards. To give you more time to finish your project, it will be the shortest of the PSETs.

Submitting

Please put together a PDF with your answers for each problem, and submit it to the appropriate assignment on Gradescope. We recommend you to add these answers to the latex template files on our website, but you can also create a PDF in any other way you prefer. For the written report, in the case of problems that just involve implementing code you can include only the final output and in some cases a brief description if requested in the problem. There will be an additional coding assignment on Gradescope that has an autograder that is there to help you double check your code. Make sure you use the provided ”.py” files to write your Python code. Submit to both the PDF and code assignment, as we will be grading the PDF submissions and using the coding assignment to check your code if needed.

Extended Kalman Filter with a Nonlinear Observation Model (60 points)

Consider the scenario depicted in Figure 1 where a robot tries to catch a fly that it tracks visually with its cameras. To catch the fly, the robot needs to estimate the 3D position pt ∈ R3 and linear



Figure 1

velocity ξt ∈ R3 of the fly with respect to its camera coordinate system. The fly is moving randomly in a way that can be modelled by a discrete time double integrator:


pt+1 = pt + ∆tξt

(1a)

ξt+1 = 0.8ξt + ∆tat

(1b)

where the constant velocity value describes the average velocity value over ∆t and is just an approximation of the true process. Variations in the fly’s linear velocity are caused by random, immeasurable accelerations at. As the accelerations are not measurable, we treat it as the pro-cess noise, w = ∆tat, and we model it as a realization of a normally-distributed white-noise random vector with zero mean and covariance Q: w ∼ N(0, Q). The covariance is given by

= diag(0.05, 0.05, 0.05)

The vision system of the robot consists of (unfortunately) only one camera. With the camera, the robot can observe the fly and receive noisy measurements z ∈ R2 which are the pixel coordinates (u, v) of the projection of the fly onto the image. We model this projection mapping of the fly’s 3D location to pixels as the observation model h:

zt = h(xt) + vt

(1c)

where x = (p, ξ)T and v is a realization of the normally-distributed, white-noise observation noise vector: v ∼ N(0, R). The covariance of the measurement noise is assumed constant and of value, R = diag(5, 5).

We assume a known 3×3 camera intrinsic matrix:

500

0

320

0

K =

0

500

240

0

(1d)

0

0

1

0

(8 pts) Let ∆t = 0.1s. Find the system matrix A for the process model, and implement the

noise covariance functions (Implement your answer in the system matrix, process noise covariance, and observation noise covariance functions in Q1.py).

(8 pts) Define the observation model h in terms of the camera parameters (Implement your answer in the observation function in Q1.py).

(8 pts) Initially, the fly is sitting on the fingertip of the robot when it is noticing it for the first time. Therefore, the robot knows the fly’s initial position from forward kinematics to be at p0 = (0.5, 0, 5.0)T (resting velocity). Simulate in Python the 3D trajectory that the fly takes as well as the measurement process. This requires generating random acceleration noise and observation noise. Simulate for 100 time steps. Attach a plot of the generated trajectories and the corresponding measurements. Please add your code to the report.

(8 pts) Find the Jacobian H of the observation model with respect to the fly’s state x. (Implement your answer of H in function observation state jacobian in Q1.py.)


(20 pts) Now let us run an Extended Kalman Filter to estimate the position and velocity of the fly relative to the camera. You can assume the aforementioned initial position and the following initial error covariance matrix: P0 = diag(0.1, 0.1, 0.1). The measurements can be found in data/Q1E measurement.npy. Plot the mean and error ellipse of the predicted mea-surements over the true measurements. Plot the means and error ellipsoids of the estimated positions over the true trajectory of the fly. The true states are in data/Q1E state.npy

(8 pts) Discuss the difference in magnitude of uncertainty in the different dimensions of the state.

Extra Credit – From Monocular to Stereo Vision (30 points)

Now let us assume that our robot got an upgrade: Someone installed a stereo camera and calibrated it. Let us assume that this stereo camera is perfectly manufactured, i.e., the two cameras are perfectly parallel with a baseline of b = 0.2. The camera intrinsics are the same as before in Question 1.

Now the robot receives as measurement z a pair of pixel coordinates in the left image (uL, vL) and right image (uR, vR) of the camera. Since our camera system is perfectly parallel, we will assume a measurement vector z = (uL, vL, dL) where dL is the disparity between the projection of the fly on the left and right image. We define the disparity to be positive. The fly’s states are represented in the left camera’s coordinate system.

a. (6 pts) Find the observation model h in terms of the camera parameters (Implement your answer in function observation in Q2.py).

b. (6 pts) Find the Jacobian H of the observation model with respect to the fly’s state x. (Implement H in function observation state jacobian in Q2.py)

(6 pts) What is the new observation noise covariance matrix R? Assume the noise on (uL, vL), and (uR, vR) to be independent and to have the same distribution as the observation noise given in Question 1, respectively. (Implement R in function observation noise covariance in Q2.py).

(6 pts) Now let us run an Extended Kalman Filter to estimate the position and velocity of the fly relative to the left camera. You can assume the same initial position and the initial error covariance matrix as in the previous questions. Plot the means and error ellipses of the predicted measurements over the true measurement trajectory in both the left and right images. The measurements can be found in data/Q2D measurement.npy. Plot the means and error ellipsoids of the estimated positions over the true trajectory of the fly. The true states are in data/Q2D state.npy Include these plots here.


(6 pts) In this Question, we are defining z = (uL, vL, dL)T . Alternatively, we could recon-struct the 3D position p of the fly from its left and right projection (uL, vL, uR, vR) through triangulation and use z = (x, y, z)T directly. Discuss the pros and cons of using (uL, vL, dL) over (x, y, z)!

Linear Kalman Filter with a Learned Inverse Observation Model (40 points)

Now the robot is trying to catch a ball. So far, we assumed that there was some vision module that would detect the object in the image and thereby provide a noisy observation. In this part of the assignment, let us learn such a detector from annotated training data and treat the resulting detector as a sensor.

If we assume the same process as in the first task, but we have a measurement model that observes directly noisy 3D locations of the ball, we end up with a linear model whose state can be estimated with a Kalman filter. Note that since you are modifying code from previous parts and are implementing your own outlier detection for part C, there is no autograder for this problem – we will be grading based on your plots.

a. (10 pts) In the folder data/Q3A data you will find a training set of 1000 images in the subfolder training set and the file Q3A positions train.npy that contains the ground truth 3D position of the red ball in the image. We have provided you with the notebook LearnedObservationModel.ipynb that can be used to train a noisy observation model. As in PSET 3, use this notebook with Google Colab to do this – note that you’ll need to upload the data directory onto a location of your choosing in Drive first. Report the training and test set mean squared error in your write-up.

(30 pts) In the folder data/Q3B data you will find a set of 1000 images that show a new tra-jectory of the red ball. Run your linear Kalman Filter using this sequence of images as input, where your learned model provides the noisy measurements (the logic for this is provided in LearnedObservationModel.ipynb). Now you can work on using the model by completing p3.py. Tune a constant measurement noise covariance appropriately, assuming it is a zero mean Gaussian and the covariance matrix is a diagonal matrix. Plot the resulting estimated trajectory from the images, along with the detections and the ground truth trajectory (the logic for this is provided in the starter code). Please add your code to the report.

Extra Credit (10 pts) Because the images are quite noisy and the red ball may be partially or completely occluded, your detector is likely to produce some false detections. In the folder data/Q3D data you will find a set of 1000 images that show a trajectory of the red ball where some images are blank (as if the ball is occluded by a white object). Discuss what happens if you do not reject these outliers but instead use them to update the state estimate. Like in the previous question, run your linear Kalman Filter using the sequence of images as input that are corrupted by occlusions (this is also provided in the notebook). Plot the resulting estimated trajectory of the ball over the ground truth trajectory. Also plot the 3-D trajectory in 2-D (x vs. z) and (y vs. z) to better visualize what happens to your filter.


Extra Credit (10 pts) Design an outlier detector and use the data from data/Q3D data. Provide the same plots as in part c with filter outliers=True. Explain how you imple-mented your outlier detector and add your code to the report. Hint: Your observation model predicts where your measurement is expected to occur and its uncertainty.
