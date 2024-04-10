/* * @Author: zhaoyu  * @Date: 2024-04-05 16:33:58  * @Last Modified by:   zhaoyu  * @Last Modified time: 2024-04-05 19:33:58  */ #include "mcl.h"

mcl::mcl()
{
    m_sync_cnt_ = 0;
    gen_ = std::mt19937(rd_());

    // initialize random number generator
    gridMap_ = cv::imread("/home/mywork/ROS-MCL-2D-LIDAR/gridmap.png", cv::IMREAD_GRAYSCALE);
    gridMapCV_ = cv::imread("/home/mywork/ROS-MCL-2D-LIDAR/erodedGridmap.png", cv::IMREAD_GRAYSCALE);

    numParticles_ = 2500;

    minOdomAngle_ = 30;

    minOdomDistance_ = 0.1;

    repropagateCountNeeded_ = 1;

    odomCovariance_[0] = 0.02;
    odomCovariance_[1] = 0.02;
    odomCovariance_[2] = 0.02;
    odomCovariance_[3] = 0.02;
    odomCovariance_[4] = 0.02;
    odomCovariance_[5] = 0.02;

    imageResolution_ = 0.05;

    tf_laser2robot_ << -1, 0, 0, 0.2,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;

    mapCenterX_ = 0;
    mapCenterY_ = -2;

    isOdomInitialized_ = false;

    predictionCounter_ = 0;

    initializeParticles();

    showInMap();
}

void mcl::initializeParticles()
{
    particles_.clear();

    std::uniform_real_distribution<float> x_pos(mapCenterX_ - gridMapCV_.cols * imageResolution_ / 4.0,
                                                mapCenterX_ + gridMapCV_.cols * imageResolution_ / 4.0);

    std::uniform_real_distribution<float> y_pos(mapCenterY_ - gridMapCV_.rows * imageResolution_ / 4.0,
                                                mapCenterY_ + gridMapCV_.rows * imageResolution_ / 4.0);
    std::uniform_real_distribution<float> theta_pos(-M_PI, M_PI);

    for (int i = 0; i < numParticles_; i++)
    {
        Particle particle_; // create a new particle object

        float randomX_ = x_pos(gen_);
        float randomY_ = y_pos(gen_);
        float randomTheta_ = theta_pos(gen_);

        particle_.pose_ = tool::xyzrpy2eigen(randomX_, randomY_, 0, 0, 0, randomTheta_);
        particle_.weight_ = 1.0 / numParticles_;

        particles_.push_back(particle_);
    }

    showInMap();
}

void mcl::showInMap()
{
    cv::Mat map_;

    cv::cvtColor(gridMap_, map_, cv::COLOR_GRAY2BGR);

    for (int i = 0; i < numParticles_; i++)
    {
        int xPos_ = static_cast<int>((particles_.at(i).pose_(0, 3) - mapCenterX_ + (300.0 * imageResolution_) / 2) / imageResolution_);
        int yPos_ = static_cast<int>((particles_.at(i).pose_(1, 3) - mapCenterY_ + (300.0 * imageResolution_) / 2) / imageResolution_);

        cv::circle(map_, cv::Point(xPos_, yPos_), 1, cv::Scalar(255, 0, 0), -1);
    }

    if (maxProbParicle_.weight_ > 0)
    {
        float x_all_ = 0;
        float y_all_ = 0;

        for (int i = 0; i < particles_.size(); i++)
        {

            x_all_ = x_all_ + particles_.at(i).pose_(0, 3) * particles_.at(i).weight_;
            y_all_ = y_all_ + particles_.at(i).pose_(1, 3) * particles_.at(i).weight_;
        }

        int xPos_ = static_cast<int>((x_all_ - mapCenterX_ + (300.0 * imageResolution_) / 2) / imageResolution_);
        int yPos_ = static_cast<int>((y_all_ - mapCenterY_ + (300.0 * imageResolution_) / 2) / imageResolution_);

        cv::circle(map_, cv::Point(xPos_, yPos_), 1, cv::Scalar(0, 0, 255), -1);

        Eigen::Matrix4Xf transLaser_ = maxProbParicle_.pose_ * tf_laser2robot_ * maxProbParicle_.laser_;

        for (int i = 0; i < transLaser_.cols(); ++i)
        {

            int xPos_ = static_cast<int>((transLaser_(0, i) - mapCenterX_ + (300.0 * imageResolution_) / 2) / imageResolution_);
            int yPos_ = static_cast<int>((transLaser_(1, i) - mapCenterY_ + (300.0 * imageResolution_) / 2) / imageResolution_);

            cv::circle(map_, cv::Point(xPos_, yPos_), 1, cv::Scalar(0, 255, 255), -1);
        }
    }

    cv::imshow("MCL", map_);

    cv::waitKey(1);
}

void mcl::prediction(Eigen::Matrix4f diffPose_)
{
    std::cout << "Prediction: " << m_sync_cnt_ << std::endl;

    Eigen::VectorXf diff_xyzrpy = tool::eigen2xyzrpy(diffPose_);

    double delta_trans_ = sqrt(pow(diff_xyzrpy(0), 2) + pow(diff_xyzrpy(1), 2));

    double delta_rot1_ = atan2(diff_xyzrpy(1), diff_xyzrpy(0));

    double delta_rot2_ = diff_xyzrpy(5) - delta_rot1_;

    std::default_random_engine generator_;

    if (delta_rot1_ > M_PI)
        delta_rot1_ -= 2 * M_PI;
    if (delta_rot1_ < -M_PI)
        delta_rot1_ += 2 * M_PI;
    if (delta_rot2_ > M_PI)
        delta_rot2_ -= 2 * M_PI;
    if (delta_rot2_ < -M_PI)
        delta_rot2_ += 2 * M_PI;

    double trans_noise_coeff_ = odomCovariance_[2] * fabs(delta_trans_) + odomCovariance_[3] * fabs(delta_rot1_ + delta_rot2_);

    double rot1_noise_coeff_ = odomCovariance_[0] * fabs(delta_rot1_) + odomCovariance_[1] * fabs(delta_trans_);

    double rot2_noise_coeff_ = odomCovariance_[0] * fabs(delta_rot2_) + odomCovariance_[1] * fabs(delta_trans_);

    float weightSum = 0.f;

    for (int i = 0; i < particles_.size(); i++)
    {
        std::normal_distribution<double> gaussian_distribution_(0, 1);

        delta_trans_ += gaussian_distribution_(gen_) * trans_noise_coeff_;
        delta_rot1_ += gaussian_distribution_(gen_) * rot1_noise_coeff_;
        delta_rot2_ += gaussian_distribution_(gen_) * rot2_noise_coeff_;

        double x_ = delta_trans_ * std::cos(delta_rot1_) + gaussian_distribution_(gen_) * odomCovariance_[4];

        double y_ = delta_trans_ * std::sin(delta_rot1_) + gaussian_distribution_(gen_) * odomCovariance_[5];

        double theta_ = delta_rot1_ + delta_rot2_ + gaussian_distribution_(gen_) * odomCovariance_[0] * (M_PI / 180.f);

        Eigen::Matrix4f diff_odom_w_noise_ = tool::xyzrpy2eigen(x_, y_, 0, 0, 0, theta_);

        weightSum += particles_[i].weight_;

        particles_[i].pose_ = particles_[i].pose_ * diff_odom_w_noise_;
    }

    for (int i = 0; i < particles_.size(); i++)
    {
        particles_[i].weight_ = particles_[i].weight_ / weightSum;
    }
    showInMap();
}

void mcl::correction(Eigen::Matrix4Xf laser_)
{
    std::cout << "Correction: " << m_sync_cnt_ << std::endl;
    float maxWeight_ = 0.f;
    float WeightSum_ = 0.f;

    for (int i = 0; i < particles_.size(); i++)
    {
        Eigen::Matrix4Xf transLaser_ = particles_[i].pose_ * tf_laser2robot_ * laser_;

        float calcedWeight = 0.f;

        for (int j = 0; j < transLaser_.cols(); j++)
        {
            int ptX_ = static_cast<int>((transLaser_(0, j) - mapCenterX_ + (300.0 * imageResolution_) / 2) / imageResolution_);
            int ptY_ = static_cast<int>((transLaser_(1, j) - mapCenterY_ + (300.0 * imageResolution_) / 2) / imageResolution_);

            if (ptX_ < 0 || ptX_ >= gridMapCV_.cols || ptY_ < 0 || ptY_ >= gridMapCV_.rows)
                continue;

            double img_val = gridMapCV_.at<uchar>(ptY_, ptX_) / 255.0;
            calcedWeight += img_val;
        }

        particles_[i].weight_ = calcedWeight / transLaser_.cols() + particles_.at(i).weight_;

        WeightSum_ += particles_[i].weight_;

        if (particles_[i].weight_ > maxWeight_)
        {
            // maxWeight_ = particles_[i].weight_;
            maxProbParicle_ = particles_[i];
            maxProbParicle_.laser_ = laser_;

            maxWeight_ = particles_[i].weight_;
        }
    }

    for (int i = 0; i < particles_.size(); i++)
    {
        particles_[i].weight_ = particles_[i].weight_ / WeightSum_;
    }
}

void mcl::resampling()
{
    std::cout << "Resampling:..." << this->m_sync_cnt_ << std::endl;

    std::vector<double> particleWeights_;
    std::vector<Particle> particleSampled_;

    double WeightBaseLine_ = 0.f;

    for (int i = 0; i < particles_.size(); i++)
    {
        WeightBaseLine_ += particles_[i].weight_;
        particleWeights_.push_back(WeightBaseLine_);
    }

    std::uniform_real_distribution<float> uniform_distribution_(0, WeightBaseLine_);

    for (int i = 0; i < particles_.size(); i++)
    {
        double darted = uniform_distribution_(gen_);

        auto lowerBound = std::lower_bound(particleWeights_.begin(), particleWeights_.end(), darted);

        particleSampled_.push_back(particles_[lowerBound - particleWeights_.begin()]);
    }

    particles_ = particleSampled_;
}

void mcl::update(Eigen::Matrix4f pose_, Eigen::Matrix4Xf laser_)
{
    std::cout << "Update:..." << this->m_sync_cnt_ << std::endl;

    if (!this->isOdomInitialized_)
    {
        odomBefore_ = pose_;
        isOdomInitialized_ = true;
    }

    Eigen::Matrix4f diff_odom_ = odomBefore_.inverse() * pose_;

    Eigen::VectorXf diffxyzrpy_ = tool::eigen2xyzrpy(diff_odom_);

    float diffDistance_ = std::sqrt(pow(diffxyzrpy_[0], 2) + pow(diffxyzrpy_[1], 2));

    float diffAngle_ = fabs(diffxyzrpy_[5]) * 180.0 / M_PI;

    if (diffDistance_ < minOdomDistance_ || diffAngle_ > minOdomAngle_)
    {

        prediction(diff_odom_);

        correction(laser_);

        predictionCounter_++;

        if (predictionCounter_ == repropagateCountNeeded_)
        {
            resampling();
            predictionCounter_ = 0;
        }

        m_sync_cnt_ += 1;

        odomBefore_ = pose_;
    }
}