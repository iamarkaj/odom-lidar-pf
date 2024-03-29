#include "gpf.h"

gpf::gpf()
{
  gen.seed(1); // Set random seed for random engine

  gpf::getMap(); // Get map from ROS

  numOfParticle = 2500;       // Number of Particles
  odomCovariance[0] = 0.02;   // Rotation to Rotation
  odomCovariance[1] = 0.02;   // Translation to Rotation
  odomCovariance[2] = 0.02;   // Translation to Translation
  odomCovariance[3] = 0.02;   // Rotation to Translation
  odomCovariance[4] = 0.02;   // X
  odomCovariance[5] = 0.02;   // Y

  tf_laser2robot << 1, 0, 0, 0,
      0, -1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1; // TF (laser frame to robot frame)

  isOdomInitialized = false; // Will be true when first data incoming.

  initializeParticles(); // Initialize particles
  showInMap();
}

gpf::~gpf()
{
}

void gpf::getMap()
{
  nav_msgs::OccupancyGridConstPtr map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", ros::Duration(10));

  // Set gridMap
  gridMap.create(map_msg->info.height, map_msg->info.width, CV_8SC1);
  int i = 0;
  for (int x = 0; x < map_msg->info.height; x++)
  {
    for (int y = 0; y < map_msg->info.width; y++)
    {
      gridMap.at<signed char>(y, x) = (int)map_msg->data[i++];
    }
  }

  // Set gridMapCV
  gridMap.convertTo(gridMapCV, CV_8UC1);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
  cv::dilate(gridMapCV, gridMapCV, element);
  cv::GaussianBlur(gridMapCV, gridMapCV, cv::Size(3, 3), 1.0);

  // Set poseMap and particlesMap
  cv::threshold(gridMapCV, poseMap, 50, 255, cv::THRESH_BINARY_INV);
  cv::cvtColor(poseMap, poseMap, cv::COLOR_GRAY2BGR);
  particlesMap = poseMap.clone();

  mapCenterX = map_msg->info.origin.position.x + map_msg->info.width * map_msg->info.resolution / 2.0;
  mapCenterY = map_msg->info.origin.position.y + map_msg->info.height * map_msg->info.resolution / 2.0;
  imageResolution = map_msg->info.resolution;
}

void gpf::initializeParticles()
{
  particles.clear();
  std::uniform_real_distribution<float> x_pos(mapCenterX - gridMapCV.cols * imageResolution / 2.0,
                                              mapCenterX + gridMapCV.cols * imageResolution / 2.0);
  std::uniform_real_distribution<float> y_pos(mapCenterY - gridMapCV.rows * imageResolution / 2.0,
                                              mapCenterY + gridMapCV.rows * imageResolution / 2.0); // heuristic setting (to put particles into the map)
  std::uniform_real_distribution<float> theta_pos(-M_PI, M_PI);                                     // -180 ~ 180 Deg

  // Set particles by random distribution
  int i = 0;
  while (i != numOfParticle)
  {
    particle particle_temp;
    float randomX = x_pos(gen);
    float randomY = y_pos(gen);
    float randomTheta = theta_pos(gen);
    int ptX = static_cast<int>((randomX - mapCenterX + (gridMapCV.cols * imageResolution) / 2) / imageResolution);
    int ptY = static_cast<int>((randomY - mapCenterY + (gridMapCV.rows * imageResolution) / 2) / imageResolution);
    if (gridMap.at<signed char>(ptY, ptX) != 0) // initialize particles only in free space
      continue;
    particle_temp.pose = tool::xyzrpy2eigen(randomX, randomY, 0, 0, 0, randomTheta);
    particle_temp.score = 1 / (float)numOfParticle;
    particles.push_back(particle_temp);
    i++;
  }
}

void gpf::prediction(Eigen::Matrix4f diffPose)
{
  Eigen::VectorXf diff_xyzrpy = tool::eigen2xyzrpy(diffPose); // {x,y,z,roll,pitch,yaw} (z,roll,pitch assume to 0)

  /*
   * Input : diffPose,diff_xyzrpy (difference of odometry pose).
   * TODO : update(propagate) particle's pose.
   */

  // Using odometry model
  float delta_trans = sqrt(pow(diff_xyzrpy(0), 2) + pow(diff_xyzrpy(1), 2));
  float delta_rot1 = atan2(diff_xyzrpy(1), diff_xyzrpy(0));
  float delta_rot2 = diff_xyzrpy(5) - delta_rot1;

  std::default_random_engine generator;
  if (delta_rot1 > M_PI)
    delta_rot1 -= (2 * M_PI);
  if (delta_rot1 < -M_PI)
    delta_rot1 += (2 * M_PI);
  if (delta_rot2 > M_PI)
    delta_rot2 -= (2 * M_PI);
  if (delta_rot2 < -M_PI)
    delta_rot2 += (2 * M_PI);

  // Add noises to trans/rot1/rot2
  float trans_noise_coeff = odomCovariance[2] * fabs(delta_trans) + odomCovariance[3] * fabs(delta_rot1 + delta_rot2);
  float rot1_noise_coeff = odomCovariance[0] * fabs(delta_rot1) + odomCovariance[1] * fabs(delta_trans);
  float rot2_noise_coeff = odomCovariance[0] * fabs(delta_rot2) + odomCovariance[1] * fabs(delta_trans);

  for (int i = 0; i < numOfParticle; i++)
  {
    std::normal_distribution<float> gaussian_distribution(0, 0.2);
    
    delta_trans = delta_trans + gaussian_distribution(gen) * trans_noise_coeff;
    delta_rot1 = delta_rot1 + gaussian_distribution(gen) * rot1_noise_coeff * (M_PI / 180.0);
    delta_rot2 = delta_rot2 + gaussian_distribution(gen) * rot2_noise_coeff * (M_PI / 180.0);

    float x = delta_trans * cos(delta_rot1) + gaussian_distribution(gen) * odomCovariance[4];
    float y = delta_trans * sin(delta_rot1) + gaussian_distribution(gen) * odomCovariance[5];
    float theta = delta_rot1 + delta_rot2 + gaussian_distribution(gen) * odomCovariance[0] * (M_PI / 180.0);

    Eigen::Matrix4f diff_odom_w_noise = tool::xyzrpy2eigen(y, x, 0, 0, 0, -theta);
    Eigen::Matrix4f pose_t_plus_1 = particles.at(i).pose * diff_odom_w_noise;
    particles.at(i).pose = pose_t_plus_1;
  }
}

void gpf::weightning(Eigen::Matrix4Xf laser)
{
  float maxScore = 0;
  float scoreSum = 0;

  /*
   * Input : laser measurement data
   * TODO : update particle's weight(score)
   */

  for (int i = 0; i < numOfParticle; i++)
  {
    // TODO : Transform laser data into global frame to map matching
    // Input : laser (4 x N matrix of laser points in lidar sensor's frame)
    //         particles.at(i).pose (4 x 4 matrix of robot pose)
    //         tf_laser2robot (4 x 4 matrix of transformatino between robot and sensor)
    // Output : transLaser (4 x N matrix of laser points in global frame)

    Eigen::Matrix4Xf transLaser = particles.at(i).pose * tf_laser2robot * laser; // now this is lidar sensor's frame

    float calcedWeight = 0;

    for (int j = 0; j < transLaser.cols(); j++)
    {
      // TODO :  translate each laser point (in [m]) to pixel frame.  (transLaser(0,i) 's unit is [m]) (You will use it in GPF too! remember!)
      // Input :  transLaser(0,j), transLaser(1,j)  (laser point's pose in global frame)
      //          imageResolution
      //          gridMap.rows , gridMap.cols (size of image)
      //          mapCenterX, mapCenterY (center of map's position)
      // Output : ptX, ptY (laser point's pixel position)

      int ptX = static_cast<int>((transLaser(0, j) - mapCenterX + (gridMapCV.cols * imageResolution) / 2) / imageResolution);
      int ptY = static_cast<int>((transLaser(1, j) - mapCenterY + (gridMapCV.rows * imageResolution) / 2) / imageResolution);

      if (ptX < 0 || ptX >= gridMapCV.cols || ptY < 0 || ptY >= gridMapCV.rows)
        continue; // dismiss if the laser point is at the outside of the map
      else
      {
        float img_val = gridMapCV.at<uchar>(ptY, ptX) / 100.0; // calculate the score
        calcedWeight += img_val;                               // sum up the score
      }
    }

    // Adding score to particle
    particles.at(i).score = particles.at(i).score + (calcedWeight / transLaser.cols());

    scoreSum += particles.at(i).score;

    // To check which particle has max score
    if (maxScore < particles.at(i).score)
    {
      maxProbParticle = particles.at(i);
      maxProbParticle.scan = laser;
      maxScore = particles.at(i).score;
    }
  }

  // normalize the score
  for (int i = 0; i < numOfParticle; i++)
  {
    particles.at(i).score = particles.at(i).score / scoreSum;
  }
}

void gpf::gaussianUpdate()
{
  // TODO : Regenerate particles based on the mean and covariance 

  // Calculate the mean of the particles
  meanX = 0.0;
  meanY = 0.0;
  float meanTheta = 0;
  for (int i = 0; i < numOfParticle; i++)
  {
    Eigen::VectorXf xyzrpy = tool::eigen2xyzrpy(particles.at(i).pose); 
    meanX += xyzrpy(0) * particles.at(i).score;
    meanY += xyzrpy(1) * particles.at(i).score;
    meanTheta += xyzrpy(5) * particles.at(i).score;
  }

  // Calculate the covariance matrix
  Eigen::MatrixXf cov = Eigen::MatrixXf::Zero(3,3);
  for (int i = 0; i < numOfParticle; i++)
  {
    Eigen::VectorXf xyzrpy = tool::eigen2xyzrpy(particles.at(i).pose);
    float dTheta = xyzrpy(5) - meanTheta;
    
    if (dTheta > M_PI)
      dTheta -= (2 * M_PI);
    if (dTheta < -M_PI)
      dTheta += (2 * M_PI);

    Eigen::Vector3f dPose(xyzrpy(0) - meanX, xyzrpy(1) - meanY, dTheta);
    cov += particles.at(i).score * (dPose * dPose.transpose());     
  }

  if (cov(2,2) > M_PI)
    cov(2,2) -= (2 * M_PI);
  if (cov(2,2) < -M_PI)
    cov(2,2) += (2 * M_PI);

  cov = cov.llt().matrixL(); 
  std::normal_distribution<float> gaussian_distribution(0, 1);

  // Regenerate particles
  for (int i = 0; i < numOfParticle; i++)
  {
    float new_x = meanX + cov(0,0) * gaussian_distribution(gen);
    float new_y = meanY + cov(1,1) * gaussian_distribution(gen);
    float new_theta = meanTheta + cov(2,2) * gaussian_distribution(gen) * (M_PI / 180.0);

    if (new_theta > M_PI)
      new_theta -= (2 * M_PI);
    if (new_theta < -M_PI)
      new_theta += (2 * M_PI);
  
    particles.at(i).pose = tool::xyzrpy2eigen(new_x, new_y, 0, 0, 0, new_theta);
    particles.at(i).score = 1.0 / numOfParticle;
  }
}

void gpf::showInMap()
{
  cv::Mat showMap = particlesMap.clone();

  for (int i = 0; i < numOfParticle; i++)
  {
    // Todo : Convert robot pose in image frame
    // Input :  particles[i].pose(0,3), particles[i].pose(1,3) (x,y position in [m])
    //          imageResolution
    //          gridMap.rows , gridMap.cols (size of image)
    //          mapCenterX, mapCenterY (center of map's position)
    // Output : xPos, yPos (pose in pixel value)

    int xPos = static_cast<int>((particles.at(i).pose(0, 3) - mapCenterX + (gridMapCV.cols * imageResolution) / 2) / imageResolution);
    int yPos = static_cast<int>((particles.at(i).pose(1, 3) - mapCenterY + (gridMapCV.rows * imageResolution) / 2) / imageResolution);

    cv::circle(showMap, cv::Point(xPos, yPos), 1, cv::Scalar(255, 0, 0), -1);
  }
  if (maxProbParticle.score > 0)
  {
    // Todo : Convert robot pose in image frame
    // Input :  maxProbParticle.pose(0,3), maxProbParticle.pose(1,3) (x,y position in [m])
    //          imageResolution
    //          gridMap.rows , gridMap.cols (size of image)
    //          mapCenterX, mapCenterY (center of map's position)
    // Output : xPos, yPos (pose in pixel value)

    // Estimate position using all particles
    float x_all = 0;
    float y_all = 0;
    for (int i = 0; i < particles.size(); i++)
    {
      x_all = x_all + particles.at(i).pose(0, 3) * particles.at(i).score;
      y_all = y_all + particles.at(i).pose(1, 3) * particles.at(i).score;
    }
    int xPos = static_cast<int>((x_all - mapCenterX + (gridMapCV.cols * imageResolution) / 2) / imageResolution);
    int yPos = static_cast<int>((y_all - mapCenterY + (gridMapCV.rows * imageResolution) / 2) / imageResolution);

    cv::circle(showMap, cv::Point(xPos, yPos), 2, cv::Scalar(0, 0, 255), -1);
    cv::circle(poseMap, cv::Point(xPos, yPos), 1, cv::Scalar(0, 0, 255), -1);
  }

  // Modify maps for better display
  float SET_MAP_SIZE = 400.0;
  cv::Mat showMapCropped = showMap.clone();
  cv::Mat poseMapCropped = poseMap.clone();
  cv::resize(showMapCropped, showMapCropped, cv::Size(SET_MAP_SIZE, SET_MAP_SIZE), cv::INTER_LINEAR);
  cv::resize(poseMapCropped, poseMapCropped, cv::Size(SET_MAP_SIZE, SET_MAP_SIZE), cv::INTER_LINEAR);

  cv::imshow("GPF", showMapCropped);
  cv::imshow("POSEMAP", poseMapCropped);
  cv::waitKey(1);
}

void gpf::updateData(Eigen::Matrix4f pose, Eigen::Matrix4Xf laser)
{
  if (!isOdomInitialized)
  {
    odomBefore = pose; // Odom used at last prediction
    isOdomInitialized = true;
    return;
  }

  // Calculate difference of distance and angle.
  Eigen::Matrix4f diffOdom = odomBefore.inverse() * pose;

  prediction(diffOdom);
  weightning(laser);
  gaussianUpdate();
  showInMap();
  odomBefore = pose;
}
