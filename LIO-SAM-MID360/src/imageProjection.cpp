#include "utility.h"
#include "lio_sam/cloud_info.h"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct LiovxPointCustomMsg
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float time;
    uint16_t ring;
    uint16_t tag;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (LiovxPointCustomMsg,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (float, time, time)
    (uint16_t, ring, ring) (uint16_t, tag, tag)
)

struct RoboSensePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (RoboSensePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (double, timestamp, timestamp)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = LiovxPointCustomMsg;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;
    std::mutex lidarCalibLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;
    ros::Subscriber subLidarCalibOffset;
    double lidarZOffsetDynamic = 0.0;

#if LIO_SAM_HAS_LIVOX
    std::deque<livox_msg::CustomMsg> livoxCloudQueue;
    livox_msg::CustomMsg currentLivoxCloudMsg;
#endif
    std::deque<sensor_msgs::PointCloud2> airyCloudQueue;
    sensor_msgs::PointCloud2 currentAiryCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<RoboSensePointXYZIRT>::Ptr tmpAiryCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;


public:
    ImageProjection():
    deskewFlag(0)
    {
        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        if (sensor == SensorType::AIRY)
        {
            subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
                pointCloudTopic, 5, &ImageProjection::airyCloudHandler, this,
                ros::TransportHints().tcpNoDelay());
        }
        else if (sensor == SensorType::LIVOX)
        {
#if LIO_SAM_HAS_LIVOX
            subLaserCloud = nh.subscribe<livox_msg::CustomMsg>(
                pointCloudTopic, 5, &ImageProjection::livoxCloudHandler, this,
                ros::TransportHints().tcpNoDelay());
#else
            ROS_FATAL("sensor=livox but no Livox ROS1 CustomMsg header was found at build time.");
            ros::shutdown();
#endif
        }
        else
        {
            ROS_FATAL("This imageProjection build accepts only sensor=livox or sensor=airy inputs.");
            ros::shutdown();
        }
        subLidarCalibOffset = nh.subscribe<std_msgs::Float64>("lio_sam/lidar_calib/z_offset", 20, &ImageProjection::lidarCalibOffsetHandler, this, ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 1);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpAiryCloudIn.reset(new pcl::PointCloud<RoboSensePointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);
        maybePrintImuDebug(*imuMsg, thisImu, "imageProjection");

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void lidarCalibOffsetHandler(const std_msgs::Float64::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(lidarCalibLock);
        lidarZOffsetDynamic = msg->data;
    }

#if LIO_SAM_HAS_LIVOX
    void livoxCloudHandler(const livox_msg::CustomMsgConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        publishClouds();

        resetParameters();
    }
#endif

    void airyCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!cacheAiryPointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();
        cloudExtraction();
        publishClouds();
        resetParameters();
    }

#if LIO_SAM_HAS_LIVOX
    void moveFromCustomMsg(livox_msg::CustomMsg &Msg, pcl::PointCloud<PointXYZIRT> & cloud)
    {
        cloud.clear();
        cloud.reserve(Msg.point_num);
        PointXYZIRT point;
        double lidarZOffsetLocal = 0.0;
        {
            std::lock_guard<std::mutex> lock(lidarCalibLock);
            lidarZOffsetLocal = lidarZOffsetDynamic;
        }

        cloud.header.frame_id=Msg.header.frame_id;
        cloud.header.stamp=Msg.header.stamp.toNSec()/1000;
        cloud.header.seq=Msg.header.seq;

        for(uint i=0;i<Msg.point_num-1;i++)
        {
            if (pointCloudTransformEnable)
            {
                Eigen::Vector3d p(Msg.points[i].x, Msg.points[i].y, Msg.points[i].z);
                p = pointCloudRot * p + pointCloudTrans;
                point.x = p.x();
                point.y = p.y();
                point.z = p.z() + lidarZOffsetLocal;
            }
            else
            {
                point.x=Msg.points[i].x; 
                point.y=Msg.points[i].y; 
                point.z=Msg.points[i].z + lidarZOffsetLocal;
            }
            point.intensity=Msg.points[i].reflectivity; 
            point.tag=Msg.points[i].tag; 
            point.time=Msg.points[i].offset_time*1e-9; 
            point.ring=Msg.points[i].line; 
            cloud.push_back(point);
        }
    }

    bool cachePointCloud(const livox_msg::CustomMsgConstPtr& laserCloudMsg)
    {
        // cache point cloud
        livoxCloudQueue.push_back(*laserCloudMsg);
        if (livoxCloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentLivoxCloudMsg = std::move(livoxCloudQueue.front());
        livoxCloudQueue.pop_front();
        if (sensor == SensorType::LIVOX)
        {
            moveFromCustomMsg(currentLivoxCloudMsg, *laserCloudIn);
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // get timestamp
        if (laserCloudIn->empty())
            return false;
        cloudHeader = currentLivoxCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        return true;
    }
#endif

    bool hasAiryFields(const sensor_msgs::PointCloud2& msg) const
    {
        const std::array<std::pair<std::string, uint8_t>, 6> required = {{
            {"x", sensor_msgs::PointField::FLOAT32},
            {"y", sensor_msgs::PointField::FLOAT32},
            {"z", sensor_msgs::PointField::FLOAT32},
            {"intensity", sensor_msgs::PointField::FLOAT32},
            {"ring", sensor_msgs::PointField::UINT16},
            {"timestamp", sensor_msgs::PointField::FLOAT64}
        }};

        for (const auto& expected : required)
        {
            const auto field = std::find_if(msg.fields.begin(), msg.fields.end(),
                [&](const sensor_msgs::PointField& candidate) {
                    return candidate.name == expected.first &&
                           candidate.datatype == expected.second &&
                           candidate.count == 1;
                });
            if (field == msg.fields.end())
            {
                ROS_ERROR_STREAM_THROTTLE(2.0,
                    "Airy PointCloud2 requires field " << expected.first
                    << " with datatype=" << static_cast<int>(expected.second));
                return false;
            }
        }
        return true;
    }

    bool moveFromAiryMsg(
        const sensor_msgs::PointCloud2& msg,
        pcl::PointCloud<PointXYZIRT>& cloud)
    {
        if (!hasAiryFields(msg))
            return false;

        tmpAiryCloudIn->clear();
        pcl::fromROSMsg(msg, *tmpAiryCloudIn);
        if (tmpAiryCloudIn->empty())
            return false;

        double lidarZOffsetLocal = 0.0;
        {
            std::lock_guard<std::mutex> lock(lidarCalibLock);
            lidarZOffsetLocal = lidarZOffsetDynamic;
        }

        const double scanStamp = msg.header.stamp.toSec();
        cloud.clear();
        cloud.reserve(tmpAiryCloudIn->size());
        cloud.header.frame_id = msg.header.frame_id;
        cloud.header.stamp = msg.header.stamp.toNSec() / 1000;
        cloud.header.seq = msg.header.seq;
        cloud.is_dense = true;

        for (const auto& raw : tmpAiryCloudIn->points)
        {
            if (!std::isfinite(raw.x) || !std::isfinite(raw.y) ||
                !std::isfinite(raw.z) || !std::isfinite(raw.timestamp))
                continue;

            const double relativeTime = raw.timestamp - scanStamp;
            if (relativeTime < -1e-3 || relativeTime > 1.0)
            {
                ROS_WARN_STREAM_THROTTLE(2.0,
                    "Airy point timestamp is outside scan window: rel=" << relativeTime);
                continue;
            }

            Eigen::Vector3d position(raw.x, raw.y, raw.z);
            if (pointCloudTransformEnable)
                position = pointCloudRot * position + pointCloudTrans;

            PointXYZIRT point;
            point.x = static_cast<float>(position.x());
            point.y = static_cast<float>(position.y());
            point.z = static_cast<float>(position.z() + lidarZOffsetLocal);
            point.intensity = raw.intensity;
            point.ring = raw.ring;
            point.time = static_cast<float>(std::max(0.0, relativeTime));
            point.tag = 0;
            cloud.push_back(point);
        }
        return !cloud.empty();
    }

    bool cacheAiryPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!laserCloudMsg)
            return false;

        airyCloudQueue.push_back(*laserCloudMsg);
        if (airyCloudQueue.size() <= 2)
            return false;

        currentAiryCloudMsg = std::move(airyCloudQueue.front());
        airyCloudQueue.pop_front();
        if (!moveFromAiryMsg(currentAiryCloudMsg, *laserCloudIn))
            return false;

        cloudHeader = currentAiryCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur;
        for (const auto& point : laserCloudIn->points)
            timeScanEnd = std::max(timeScanEnd, timeScanCur + static_cast<double>(point.time));

        if (timeScanEnd <= timeScanCur)
        {
            ROS_WARN_THROTTLE(2.0, "Airy scan has no valid per-point time span.");
            return false;
        }
        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0 ; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;
            // thisPoint.intensity = (laserCloudIn->points[i].ring+1) * 17;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER)
            {
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0/float(Horizon_SCAN);
                columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            }
            else if (sensor == SensorType::LIVOX || sensor == SensorType::AIRY)
            {
                columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn] += 1;
            }
            
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }
    
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
