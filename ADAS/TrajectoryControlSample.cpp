#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneEvaluationAPI.h"
#include "SimOneSensorAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilMath.h"
#include "UtilDriver.h"
#include "hdmap/SampleGetNearMostLane.h"
#include "hdmap/SampleGetLaneST.h"
#include <memory>
#include <limits>
#include <iostream>
#include<fstream>
#include "UtilMath.h"
#include "SimOneV2XAPI.h"
#include "GetSignType.h"
#include <chrono>
#include <queue>


// ����һ��fstream����
std::fstream file;
float TargetSpeed;
int runningCase = 0;
double runLenth = 0;

class utilTargetLane {
public:
	//����·��������
	static SSD::SimPoint3D CreateKnot(const SSD::SimPoint3D& from, const SSD::SimPoint3D& to, const double& k)
	{
		return SSD::SimPoint3D(from.x + k * (to.x - from.x), from.y + k * (to.y - from.y), from.z + k * (to.z - from.z));
	}

	static SSD::SimString GetNearMostLane(const SSD::SimPoint3D& pos)
	{
		SSD::SimString laneId;
		double s, t, s_toCenterLine, t_toCenterLine;
		if (!SimOneAPI::GetNearMostLane(pos, laneId, s, t, s_toCenterLine, t_toCenterLine))
		{
			std::cout << "Error: lane is not found." << std::endl;
		}
		return std::move(laneId);
	}

	static SSD::SimString GetLeftNeighborLane(const SSD::SimString& laneId)
	{
		SSD::SimString leftNeighborLaneName;
		HDMapStandalone::MLaneLink laneLink;
		if (SimOneAPI::GetLaneLink(laneId, laneLink))
		{
			leftNeighborLaneName = laneLink.leftNeighborLaneName;
		}
		return std::move(leftNeighborLaneName);
	}

	static SSD::SimString GetRightNeighborLane(const SSD::SimString& laneId)
	{
		SSD::SimString GetRightNeighborLane;
		HDMapStandalone::MLaneLink laneLink;
		if (SimOneAPI::GetLaneLink(laneId, laneLink))
		{
			GetRightNeighborLane = laneLink.rightNeighborLaneName;
		}
		return std::move(GetRightNeighborLane);
	}
	static void GetLaneSampleFromS(const SSD::SimString& laneId, const double& s, SSD::SimPoint3DVector& targetPath)
	{
		HDMapStandalone::MLaneInfo info;
		if (SimOneAPI::GetLaneSample(laneId, info))
		{
			double accumulated = 0.0;
			int startIndex = -1;
			for (unsigned int i = 0; i < info.centerLine.size() - 1; i++)
			{
				auto& pt = info.centerLine[i];
				auto& ptNext = info.centerLine[i + 1];
				double d = UtilMath::distance(pt, ptNext);
				accumulated += d;
				if (accumulated >= s)
				{
					startIndex = i + 1;
					break;
				}
			}
			for (unsigned int i = startIndex; i < info.centerLine.size(); i++)
			{
				SSD::SimPoint3D item = info.centerLine[i];
				targetPath.push_back(info.centerLine[i]);
			}
		}
	}

	//����ֱ�����·����
	static SSD::SimPoint3DVector CreateChangeLanePath(const SSD::SimPoint3D& from, const SSD::SimPoint3D& to)
	{
		SSD::SimPoint3DVector path;
		double d = UtilMath::distance(from, to);
		const double kDistancePerSample = 1;  //every 1 meter, add a knot
		double k = kDistancePerSample / d;
		int addKnotCount = (int)std::floor(d / kDistancePerSample);
		for (int i = 1; i <= addKnotCount; i++)
		{
			path.push_back(CreateKnot(from, to, i * k));
		}
		return std::move(path);
	}
	//��ȡ�������Ĳ�����
	static SSD::SimPoint3DVector GetLaneSample(const SSD::SimString& laneId)
	{
		SSD::SimPoint3DVector targetPath;
		HDMapStandalone::MLaneInfo info;
		if (SimOneAPI::GetLaneSample(laneId, info))
		{
			for (auto& pt : info.centerLine)
			{
				targetPath.push_back(SSD::SimPoint3D(pt.x, pt.y, pt.z));
			}
		}
		return std::move(targetPath);
	}
};

class utilTargetObstacle {
public:
	struct ObstacleStruct {
		SSD::SimPoint3D pt;
		SSD::SimString ownerLaneId;
	};

	static std::vector<utilTargetObstacle::ObstacleStruct> GetObstacleList()
	{
		std::vector<utilTargetObstacle::ObstacleStruct> allObstacle;
		std::unique_ptr<SimOne_Data_Obstacle> pSimOne_Data_Obstacle = std::make_unique<SimOne_Data_Obstacle>();
		const char* MainVehicleID = "0";
		//��Ѱ���������ϰ��� ��ά�����복����
		if (SimOneAPI::GetGroundTruth(MainVehicleID, pSimOne_Data_Obstacle.get()))
		{
			for (int i = 0; i < pSimOne_Data_Obstacle->obstacleSize; i++)
			{
				float posX = pSimOne_Data_Obstacle->obstacle[i].posX;
				float posY = pSimOne_Data_Obstacle->obstacle[i].posY;
				float posZ = pSimOne_Data_Obstacle->obstacle[i].posZ;
				SSD::SimPoint3D pos(posX, posY, posZ);
				SSD::SimString laneId = utilTargetLane::GetNearMostLane(pos);
				allObstacle.push_back(utilTargetObstacle::ObstacleStruct{ SSD::SimPoint3D(posX, posY, posZ), laneId });
			}
		}
		else
		{
			file << "x:" << std::endl;
		}
		return std::move(allObstacle);
	}
	//����ϰ��ﳢ�Ա��
	static bool DetectObstacle(const SSD::SimPoint3D& vehiclePos, double mainVehicleSpeed, const std::vector<ObstacleStruct>& allObstacles, const HDMapStandalone::MLaneType& laneType,
		const SSD::SimString& laneId, const SSD::SimString& leftNeighborLaneName, SSD::SimPoint3D& changeToPoint, const SSD::SimString& rightNeighborLaneName, SSD::SimPoint3DVector& targetPath, int& warningObstacleIndex,
		std::vector<SimString> nextroadid)
	{
		warningObstacleIndex = -1;
		//file << "allObstacles.size()" << allObstacles.size() << endl;
		double kDistanceSafety;//�������
		// �����ٶ�Ӱ��İ�ȫǰհ����
		double reactionTime = 1.;
		double maxDecel = 6.0;
		double safetyMargin = 2.0;
		double mainVehicleS, mainVehicleT, obstacleS, obstacleT;
		kDistanceSafety = mainVehicleSpeed * reactionTime
			//+ (mainVehicleSpeed * mainVehicleSpeed) / (2.0 * maxDecel)
			+ safetyMargin;
		kDistanceSafety = std::max(10.0, std::min(kDistanceSafety, 50.0));

		//get closest obstacle
		//double minDist = std::numeric_limits<double>::max();
		std::vector<int>obstacleClosestIndex;
		SSD::SimPoint2D vehiclePos2D(vehiclePos.x, vehiclePos.y);
		std::queue<SimString> roadid;
		for (unsigned int i = 0; i < allObstacles.size(); i++)
		{
			auto& obstacle = allObstacles[i];
			double distanceSign = UtilMath::distance(vehiclePos2D, SSD::SimPoint2D(obstacle.pt.x, obstacle.pt.y));
			if (distanceSign < kDistanceSafety)
			{
				obstacleClosestIndex.push_back(i);
			}
		}
		if (laneType != HDMapStandalone::MLaneType::driving)
		{
			return false;
		}
		SimOneAPI::GetLaneST(laneId, vehiclePos, mainVehicleS, mainVehicleT);
		//file << "minDist:" << minDist << endl;
		for (int i = 0; i < obstacleClosestIndex.size(); i++)
		{
			auto& obstacleClosest = allObstacles[obstacleClosestIndex[i]];
			bool flagsuccessor = false;
			SimOneAPI::GetLaneST(obstacleClosest.ownerLaneId, obstacleClosest.pt, obstacleS, obstacleT);
			SSD::SimString leftLaneName = utilTargetLane::GetLeftNeighborLane(obstacleClosest.ownerLaneId);
			SSD::SimString rightLaneName = utilTargetLane::GetRightNeighborLane(obstacleClosest.ownerLaneId);
			file << "laneId" << laneId.GetString() << endl;
			file << "obstacleClosest.ownerLaneId" << obstacleClosest.ownerLaneId.GetString() << endl;
			if (nextroadid.size() != 0)//�ϰ�������һ������
			{
				for (unsigned int j = 0; j < nextroadid.size(); j++)
				{
					if (obstacleClosest.ownerLaneId == nextroadid[j])
					{
						if (j == 0 && obstacleS - mainVehicleS < 0)
						{
							break;
						}
						flagsuccessor = true;
						break;
					}
				}
			}
			/*if (obstacleClosest.ownerLaneId == laneId)//�ϰ����ڵ�ǰ������
			{
				flagsuccessor = true;
			}*/
			if (!flagsuccessor && i != obstacleClosestIndex.size()) {
				continue;
			}
			if (!flagsuccessor)
			{
				return false;
			}
			warningObstacleIndex = obstacleClosestIndex[i];
			SSD::SimPoint3D dir;
			// ����������
			if (SimOneAPI::GetLaneMiddlePoint(obstacleClosest.pt, leftLaneName, changeToPoint, dir))
			{
				targetPath.clear();  //Important
				file << "Left Change: changeToPoint x:" << changeToPoint.x << " changeToPoint y:" << changeToPoint.y << "changeToPoint z:" << changeToPoint.z << std::endl;
				targetPath.push_back(vehiclePos);
				SSD::SimPoint3DVector changeLanePath = utilTargetLane::CreateChangeLanePath(vehiclePos, changeToPoint);
				for (auto& knot : changeLanePath)
				{
					file << "knot: (" << knot.x << ", " << knot.y << ", " << knot.z << ")" << std::endl;
					targetPath.push_back(knot);
				}
				targetPath.push_back(changeToPoint);
				return true;
			}
			else if (SimOneAPI::GetLaneMiddlePoint(obstacleClosest.pt, rightLaneName, changeToPoint, dir)) // �������ұ��
			{
				targetPath.clear();  //Important
				file << "Right Change: changeToPoint x:" << changeToPoint.x << " changeToPoint y:" << changeToPoint.y << "changeToPoint z:" << changeToPoint.z << std::endl;
				targetPath.push_back(vehiclePos);
				SSD::SimPoint3DVector changeLanePath = utilTargetLane::CreateChangeLanePath(vehiclePos, changeToPoint);
				for (auto& knot : changeLanePath)
				{
					file << "knot: (" << knot.x << ", " << knot.y << ", " << knot.z << ")" << std::endl;
					targetPath.push_back(knot);
				}
				targetPath.push_back(changeToPoint);
				return true;
			}
			else
			{
				std::cout << "No valid lane to change." << std::endl;
				return false;
			}
		}
		return false;
	}

	static bool PassedObstacle(const SSD::SimPoint3D& vehiclePos, const ObstacleStruct& obstacle, const SSD::SimString& laneId)
	{
		double s, t;
		bool found = SimOneAPI::GetLaneST(laneId, obstacle.pt, s, t);
		double s_vehicle, t_vehicle;
		found = SimOneAPI::GetLaneST(laneId, vehiclePos, s_vehicle, t_vehicle);
		return s_vehicle > s;
	}

	static bool my_DetectSpeedLimitSign(SimOne_Data_Gps *gps, int& warningSignIndex)
	{
		SSD::SimVector<HDMapStandalone::MSignal> TrafficSignList;
		SimOneAPI::GetTrafficSignList(TrafficSignList);
		float mainVehicleVel = sqrtf(pow(gps->velX, 2) + pow(gps->velY, 2)) * 3.6f;

		const double kAlertDistance = 40.0;
		//get closest sign
		double minDist = std::numeric_limits<double>::max();
		int closestSignIndex = -1;

		SSD::SimPoint2D gps2D(gps->posX, gps->posY);

		for (unsigned int i = 0; i < TrafficSignList.size(); i++)
		{
			auto& sign = TrafficSignList[i];
			double distanceSign = UtilMath::distance(gps2D, SSD::SimPoint2D(sign.pt.x, sign.pt.y));
			if (distanceSign < minDist)
			{
				minDist = distanceSign;
				closestSignIndex = i;
			}
		}
		if (minDist > kAlertDistance)
		{
			return false;
		}
		warningSignIndex = closestSignIndex;
		auto& targetSign = TrafficSignList[closestSignIndex];
		TrafficSignType sign = GetTrafficSignType(targetSign.type);
		if (sign == TrafficSignType::SpeedLimit_Sign)
		{
			//std::string speedLimitValue = targetSign.value.GetString();
			//double distanceSign = minDist;

			//double speedLimit;
			//std::stringstream ss;
			//ss << speedLimitValue;
			//ss >> speedLimit;

			return true;
		}
		return false;
	}
};

// ת����㺯��
double my_calculateSteering(const SSD::SimPoint3DVector& targetPath, SimOne_Data_Gps *pGps)
{
	std::vector<float> pts;
	for (size_t i = 0; i < targetPath.size(); ++i)
	{
		pts.push_back(pow((pGps->posX - (float)targetPath[i].x), 2) + pow((pGps->posY - (float)targetPath[i].y), 2));
		// file << "i:" << i << " targetPath[i].x" << (float)targetPath[i].x << " targetPath[i].y:" << (float)targetPath[i].y << endl;
	}
	//file << "pts:";
	//for (int i = 0; i < pts.size(); i++) {
		//file << pts[i] << " ";
	//}
	file << std::endl;
	size_t index = std::min_element(pts.begin(), pts.end()) - pts.begin();
	size_t forwardIndex = 0;
	float minProgDist = 3.f;
	float progTime = 0.8f;
	float mainVehicleSpeed = sqrtf(pGps->velX * pGps->velX + pGps->velY * pGps->velY + pGps->velZ * pGps->velZ);
	float progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;

	for (; index < targetPath.size(); ++index)
	{
		forwardIndex = index;
		float distance = sqrtf(((float)pow(targetPath[index].x - pGps->posX, 2) + pow((float)targetPath[index].y - pGps->posY, 2)));
		if (distance >= progDist)
		{
			break;
		}
	}
	double psi = (double)pGps->oriZ;
	file << "psi:" << psi << std::endl;
	double alfa = atan2(targetPath[forwardIndex].y - pGps->posY, targetPath[forwardIndex].x - pGps->posX) - psi;
	file << "alfa:" << alfa << std::endl;
	double ld = sqrt(pow(targetPath[forwardIndex].y - pGps->posY, 2) + pow(targetPath[forwardIndex].x - pGps->posX, 2));
	file << "ld:" << ld << std::endl;
	double k = 2.;//�ֶ��˸�ϵ��
	if (runningCase == 41) k = 3;
	double steering = -atan2(2. * (1.3 + 1.55) * sin(alfa), ld) * 36. / (7. * M_PI) * k;
	if (index == targetPath.size()) {
		steering = 0;
	}
	file << "steering:" << steering << std::endl;
	return steering;
}

SSD::SimPoint3DVector GenerateTargetPath(const char* MainVehicleId, SimOne_Data_WayPoints_Entry wayPoints1, SimOne_Data_WayPoints_Entry wayPoints2) {
	// �洢·����
	SSD::SimPoint3DVector inputPoints;
	//std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();

	SSD::SimPoint3D inputWayPoints1(wayPoints1.posX, wayPoints1.posY, 0);
	SSD::SimPoint3D inputWayPoints2(wayPoints2.posX, wayPoints2.posY, 0);
	//file << "wayPoint x: " << specwayPoints[i].posX << " y: " << specwayPoints[i].posY << endl;
	inputPoints.push_back(inputWayPoints1);
	inputPoints.push_back(inputWayPoints2);
	// ����Ŀ��·��
	SSD::SimPoint3DVector targetPath;

	SSD::SimVector<int> indexOfValidPoints;
	if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
		return {};
	}

	return targetPath;
}

SSD::SimPoint3DVector GenerateTargetPath2(const char* MainVehicleId, SimOne_Data_WayPoints_Entry wayPoints1, SimOne_Data_WayPoints_Entry wayPoints2, SimOne_Data_WayPoints_Entry wayPoints3) {
	// �洢·����
	SSD::SimPoint3DVector inputPoints;
	//std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();

	SSD::SimPoint3D inputWayPoints1(wayPoints1.posX, wayPoints1.posY, 0);
	SSD::SimPoint3D inputWayPoints2(wayPoints2.posX, wayPoints2.posY, 0);
	SSD::SimPoint3D inputWayPoints3(wayPoints3.posX, wayPoints3.posY, 0);
	//file << "wayPoint x: " << specwayPoints[i].posX << " y: " << specwayPoints[i].posY << endl;
	inputPoints.push_back(inputWayPoints1);
	inputPoints.push_back(inputWayPoints2);
	inputPoints.push_back(inputWayPoints3);
	// ����Ŀ��·��
	SSD::SimPoint3DVector targetPath;

	SSD::SimVector<int> indexOfValidPoints;
	if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
		return {};
	}

	return targetPath;
}

SSD::SimPoint3DVector GenerateStraightTargetPath(const char* MainVehicleId, int wayPointsSize, SimOne_Data_WayPoints_Entry WayPoints[]) {
	SSD::SimPoint3DVector targetPath;
	for (int i = 0; i < wayPointsSize - 1; ++i) {
		double distence = std::sqrt(pow(WayPoints[i].posX - WayPoints[i + 1].posX, 2) + pow(WayPoints[i].posY - WayPoints[i + 1].posY, 2));
		int points = int(distence * 10);
		file << "WayPoints:" << WayPoints[i].posX << ", " << WayPoints[i].posY << endl;
		for (int j = 0; j < points; j++) {
			SSD::SimPoint3D inputWayPoints((WayPoints[i].posX*(points - j) + WayPoints[i + 1].posX * j) / points, (WayPoints[i].posY*(points - j) + WayPoints[i + 1].posY * j) / points, 0);

			targetPath.push_back(inputWayPoints);
		}
	}
	return targetPath;
}

static double PlanarDistance(const SSD::SimPoint3D& pt1, const SSD::SimPoint3D& pt2)
{
	return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
}



long fromLaneIdGetRoadId(SSD::SimString LaneId) {
	std::string s = LaneId.GetString();
	stringstream ss(s);
	char delim;
	long ans[3];
	ss >> ans[0] >> delim >> ans[1] >> delim >> ans[2];
	return ans[0];
}

long fromLaneIdGetLaneId(SSD::SimString LaneId) {
	std::string s = LaneId.GetString();
	stringstream ss(s);
	char delim;
	long ans[3];
	ss >> ans[0] >> delim >> ans[1] >> delim >> ans[2];
	return ans[2];
}

SSD::SimString getLaneId(long roadId, int sectionIndex, int laneId) {
	stringstream ss;
	ss << roadId << '_' << sectionIndex << '_' << laneId;
	SSD::SimString ans;
	return SSD::SimString(ss.str().c_str());
}

void GetCrossroad2(SSD::SimVector<HDMapStandalone::MObject>& mainCrosswalkList,
	SSD::SimVector<SSD::SimPoint3D>& intersectionVertices, SSD::SimPoint3D mainVehiclePos, size_t successorLaneSize)
{
	double ToleranceRate = 1.0;
	double maxDistance = -1.0;
	double Tolerance = 0.0;//����һ��������
	double smin = 0, smax = 0, tmin = 0, tmax = 0;
	double smin1 = 0, smax1 = 0, tmin1 = 0, tmax1 = 0;//���ڲ���·�ڵ�ͼ
	double xmin1 = 0, xmax1 = 0, ymin1 = 0, ymax1 = 0;
	double distanceA, distanceB, shortDistance, longDistance;
	size_t roadFlag = (successorLaneSize > 0) ? (successorLaneSize - 1) : 0;//����������( successorLaneSize=1)���Ͳ���չ·�ڷ�Χ�������ʮ��·�ڣ�����չ


	//����4���������ڼ�¼ʮ��·�ڵķ�Χ��s�Գ�����ǰΪ��,t�Գ����Ҳ�Ϊ����
	double xmin = std::numeric_limits<double>::max();
	double ymin = std::numeric_limits<double>::max();
	double xmax = -std::numeric_limits<double>::max();
	double ymax = -std::numeric_limits<double>::max();



	//����������
	for (auto& item : mainCrosswalkList) {
		for (auto& item1 : item.boundaryKnots) {
			if (item1.x < xmin)xmin = item1.x;
			if (item1.x > xmax)xmax = item1.x;
			if (item1.y < ymin)ymin = item1.y;
			if (item1.y > ymax)ymax = item1.y;
		}
	}

	if (!mainCrosswalkList.empty() && mainCrosswalkList[0].boundaryKnots.size() >= 3) {
		distanceA = std::sqrt(std::pow(mainCrosswalkList[0].boundaryKnots[0].x - mainCrosswalkList[0].boundaryKnots[1].x, 2) +
			std::pow(mainCrosswalkList[0].boundaryKnots[0].y - mainCrosswalkList[0].boundaryKnots[1].y, 2));
		distanceB = std::sqrt(std::pow(mainCrosswalkList[0].boundaryKnots[1].x - mainCrosswalkList[0].boundaryKnots[2].x, 2) +
			std::pow(mainCrosswalkList[0].boundaryKnots[1].y - mainCrosswalkList[0].boundaryKnots[2].y, 2));
	}
	else {
		file << "����: mainCrosswalkList Ϊ�գ��� boundaryKnots �������㣡" << std::endl;
	}
	shortDistance = std::min(distanceA, distanceB);
	longDistance = std::max(distanceA, distanceB);
	file << "���е��Ķ̱߳���: " << shortDistance << std::endl;
	file << "���е��ĳ��߳���: " << longDistance << std::endl;

	/*
	if (mainVehiclePos.x >= xmin)smin1 -= 2 * shortDistance;
	if (mainVehiclePos.x <= xmax)smax1 += 2 * shortDistance;
	if (mainVehiclePos.y >= ymin)tmin1 -= 2 * shortDistance;
	if (mainVehiclePos.y <= ymax)tmax1 += 2 * shortDistance;
	*/
	//��ʱ�ȿ���ֻ��һ�ߣ�����λ��С��xmin���������ʱ�ȿ����ҵ���Ŀ
	if (mainVehiclePos.x < xmin)
	{
		smin1 = shortDistance + Tolerance;
		smax1 = 0;//xmax�ɼӿɲ���
		tmin1 -= shortDistance;
		tmax1 = 0;
	}
	/*
	if (mainVehiclePos.x > xmax)
	if (mainVehiclePos.y < ymin)
	if (mainVehiclePos.y > ymax)
	*/

	if (roadFlag) {
		if (mainVehiclePos.x < xmin)smax += shortDistance + longDistance;
		if (mainVehiclePos.x > xmax)smin -= shortDistance + longDistance;
		if (mainVehiclePos.y < ymin)tmax += shortDistance + longDistance;
		if (mainVehiclePos.y > ymax)tmin -= shortDistance + longDistance;
	}
	else {
		if (mainVehiclePos.x < xmin)smax += -1 * shortDistance + longDistance;
		if (mainVehiclePos.x > xmax)smin -= -1 * shortDistance + longDistance;
		if (mainVehiclePos.y < ymin)tmax += -1 * shortDistance + longDistance;
		if (mainVehiclePos.y > ymax)tmin -= -1 * shortDistance + longDistance;
	}

	xmin += smin * ToleranceRate;
	xmax += smax * ToleranceRate;
	ymin += tmin * ToleranceRate;
	ymax += tmax * ToleranceRate;
	//Ĭ�����е�ƽ��������ϵ���������


	file << "·�ڷ�Χxmin: " << xmin << std::endl;
	file << "·�ڷ�Χxmax: " << xmax << std::endl;
	file << "·�ڷ�Χymin: " << ymin << std::endl;
	file << "·�ڷ�Χymax: " << ymax << std::endl;

	intersectionVertices.clear();
	intersectionVertices.reserve(8);  // Ԥ����ռ䣬��ֹ push_back() ��������
	intersectionVertices.push_back(SSD::SimPoint3D(xmin, ymin, 0));
	intersectionVertices.push_back(SSD::SimPoint3D(xmin, ymax, 0));
	intersectionVertices.push_back(SSD::SimPoint3D(xmax, ymin, 0));
	intersectionVertices.push_back(SSD::SimPoint3D(xmax, ymax, 0));


	xmin1 = xmin + smin1;
	xmax1 = xmax + smax1;
	ymin1 = ymin + tmin1;
	ymax1 = ymin + tmax1;//ע���д��

	file << "��·�ڷ�Χxmin1: " << xmin1 << std::endl;
	file << "��·�ڷ�Χxmax1: " << xmax1 << std::endl;
	file << "��·�ڷ�Χymin1: " << ymin1 << std::endl;
	file << "��·�ڷ�Χymax1: " << ymax1 << std::endl;

	intersectionVertices.push_back(SSD::SimPoint3D(xmin1, ymin1, 0));
	intersectionVertices.push_back(SSD::SimPoint3D(xmin1, ymax1, 0));
	intersectionVertices.push_back(SSD::SimPoint3D(xmax1, ymin1, 0));
	intersectionVertices.push_back(SSD::SimPoint3D(xmax1, ymax1, 0));
}

// ���㵥λ����********
void Normalize(double& x, double& y) {
	double length = std::sqrt(x * x + y * y);
	if (length > 1e-6) { // �������
		x /= length;
		y /= length;
	}
}

void SetAccidentPath(SimOne_Data_WayPoints_Entry accidentPoints[], const std::unique_ptr<SimOne_Data_WayPoints>& caraccidentPoints) {
	accidentPoints[0] = caraccidentPoints->wayPoints[0];
	accidentPoints[1] = { 1, -292.12, -13.5, 0, 0, 0, 1 };
	accidentPoints[2] = { 2, -275.60, -13.5, 0, 0, 0, 1 };
	accidentPoints[3] = { 3, -270.60, -13.5, 0, 0, 0, 1 };
	accidentPoints[4] = caraccidentPoints->wayPoints[1];
}

// �� (xP, yP) �� XY ����ϵת���� ST ����ϵ
SSD::SimPoint3D ConvertToST(const SSD::SimPoint3D A, const SSD::SimPoint3D B, const SSD::SimPoint3D P) {
	// ���� S �ᵥλ���� (AB����)
	double s_x = B.x - A.x;
	double s_y = B.y - A.y;
	Normalize(s_x, s_y);

	// ���������� T �ᵥλ��������ʱ����ת 90�㣩
	double t_x = -s_y;
	double t_y = s_x;

	// ���� P �� ST ����ϵ�е�����
	double s = (P.x - A.x) * s_x + (P.y - A.y) * s_y;
	double t = (P.x - A.x) * t_x + (P.y - A.y) * t_y;

	return SSD::SimPoint3D(s, t, 0.0);  // ���� ST ���꣬Z ����Ȼ�� 0
}

// �� (sP, tP) �� ST ����ϵת���� XY ����ϵ
SSD::SimPoint3D ConvertToXY(const SSD::SimPoint3D A, const SSD::SimPoint3D B, const SSD::SimPoint3D P_st) {
	// ���� S �ᵥλ���� (AB����)
	double s_x = B.x - A.x;
	double s_y = B.y - A.y;
	Normalize(s_x, s_y);

	// ���� T �ᵥλ��������ʱ����ת 90�㣩
	double t_x = -s_y;
	double t_y = s_x;

	// ���� XY ����
	double x = A.x + P_st.x * s_x + P_st.y * t_x;
	double y = A.y + P_st.x * s_y + P_st.y * t_y;

	return SSD::SimPoint3D(x, y, 0.0);  // ���� XY ���꣬Z ����Ȼ�� 0
}

//type=0,����ʹ�õ��ǡ���׼��·�ڷ�Χ��type=1����ʹ�á��Ӵ�·�ڷ�Χ�� type=2�������������·��
void GetCrossroad(double shortDistance, double longDistance, SSD::SimVector<SSD::SimPoint3D>& intersectionVertices, int type = 0)
{
	double A_s, A_t, F_s, F_t, G_s, G_t, J_s, J_t;
	double tolerance = 0.2;
	double spacing;

	if (type == 0) {
		spacing = shortDistance;
		//����·�ڵ��������Σ�A��FΪ����1�Խǵ㣻G��JΪ����2�Խǵ�
		A_s = 0.;
		A_t = 0.;
		F_s = 2. * shortDistance + 2. * spacing + longDistance;//����1�ĳ���,����ӵ�³���
		F_t = longDistance + tolerance;
		G_s = shortDistance;
		G_t = -shortDistance - spacing;
		J_s = shortDistance + spacing + longDistance;
		J_t = shortDistance + spacing + longDistance;
	}
	else if (type == 1) {
		spacing = shortDistance;
		//����·�ڵ��������Σ�A��FΪ����1�Խǵ㣻G��JΪ����2�Խǵ�
		A_s = 0.;
		A_t = 0.;
		F_s = 2. * shortDistance + 2. * spacing + 2. * longDistance + tolerance;//����1�ĳ���,����ӵ�³���
		F_t = longDistance;
		G_s = shortDistance;
		G_t = -shortDistance - spacing;
		J_s = shortDistance + spacing + 2. * longDistance;
		J_t = shortDistance + spacing + longDistance;
	}
	else {
		spacing = shortDistance;
		A_s = 0.;
		A_t = 0.;
		F_s = shortDistance;
		F_t = longDistance;
		G_s = spacing;
		G_t = -shortDistance - spacing;
		J_s = spacing + longDistance;
		J_t = -spacing;
	}

	file << "·�ڷ�ΧAst:" << A_s << ", " << A_t << endl;
	file << "·�ڷ�ΧFst:" << F_s << ", " << F_t << endl;
	file << "·�ڷ�ΧGst:" << G_s << ", " << G_t << endl;
	file << "·�ڷ�ΧJst:" << J_s << ", " << J_t << endl;

	intersectionVertices.clear();
	//intersectionVertices.reserve(8);  // Ԥ����ռ䣬��ֹ push_back() ��������
	intersectionVertices.push_back(SSD::SimPoint3D(A_s, A_t, 0.));//A
	intersectionVertices.push_back(SSD::SimPoint3D(A_s, F_t, 0.));//D
	intersectionVertices.push_back(SSD::SimPoint3D(F_s, A_t, 0.));//E
	intersectionVertices.push_back(SSD::SimPoint3D(F_s, F_t, 0.));//F

	intersectionVertices.push_back(SSD::SimPoint3D(G_s, G_t, 0.));//G
	intersectionVertices.push_back(SSD::SimPoint3D(G_s, J_t, 0.));//H
	intersectionVertices.push_back(SSD::SimPoint3D(J_s, G_t, 0.));//I
	intersectionVertices.push_back(SSD::SimPoint3D(J_s, J_t, 0.));//J
}

// �����ϰ�����������ʻ������ʱ����ת90�ȷ���ļ��ٶ�
double computePerpendicularAcceleration(const SimOne_Data_Obstacle_Entry& obstacle, const SimOne_Data_Gps& mainVehicle) {
	// ���������ٶȵĴ�С
	double mainSpeed = std::sqrt(mainVehicle.velX * mainVehicle.velX + mainVehicle.velY * mainVehicle.velY);

	// �����ٶ�Ϊ������
	if (mainSpeed == 0)
	{
		file << "��ǰ�����ٶ�Ϊ0���޷�����������ٶȣ�" << endl;
		return 0;
	}

	// ����������ʻ����ĵ�λ����
	double unitMainVelX = mainVehicle.velX / mainSpeed;
	double unitMainVelY = mainVehicle.velY / mainSpeed;
	// �����ϰ�����ٶ���������ʻ�����ϵ�ͶӰ
	//double tangentialAcc = (obstacle.velX * unitMainVelX + obstacle.velY * unitMainVelY) *3.6;
	//file << "����ʻ�����ٶ�" << tangentialAcc << endl;

	// ������ʱ����ת90�ȵĵ�λ��������������Ĵ�ֱ����
	double unitPerpVelX = -unitMainVelY;
	double unitPerpVelY = unitMainVelX;


	// �����ϰ�����ٶ��ڴ�ֱ�����ϵ�ͶӰ
	double perpAcc = (obstacle.velX  * unitPerpVelX + obstacle.velY * unitPerpVelY) * 3.6;

	return perpAcc;
}

// �жϵ�index����־�Ƿ���Ŀ���־��Ĭ�ϲ��ҡ���һ�����Ƿ�Ϊ��ѧУ��־����������У���������浽matchedSign
// ��index==-1��ʱ�򣬱�������TrafficSignList�����Ƿ����Ŀ���־
bool detectTargetSign(const SSD::SimVector<HDMapStandalone::MSignal>& TrafficSignList, HDMapStandalone::MSignal& matchedSign, const SSD::SimPoint3D& mainVehiclePos, int index = 0, const TrafficSignType& TargetSign = School_Sign)
{
	if (index == -1) {
		// ���������б����
		for (const auto& signal : TrafficSignList) {
			if (GetTrafficSignType(signal.type) == TargetSign) {
				matchedSign = signal;
				return true;
			}
		}
		return false;
	}
	else if (index >= 0 && index < TrafficSignList.size()) {
		// ֻ���ָ�� index ��Ԫ��
		if (GetTrafficSignType(TrafficSignList[index].type) == TargetSign) {
			matchedSign = TrafficSignList[index];
			return true;
		}
		return false;
	}
	// index �Ƿ�
	return false;
}

SSD::SimPoint3D FindLookaheadPoint(const SSD::SimPoint3DVector& targetPath, const std::unique_ptr<SimOne_Data_Gps>& pGps, double distanceTarget = 30.0)
{
	std::vector<float> pts;
	for (size_t i = 0; i < targetPath.size(); ++i)
	{
		pts.push_back(pow((pGps->posX - (float)targetPath[i].x), 2) + pow((pGps->posY - (float)targetPath[i].y), 2));
	}
	size_t index = std::min_element(pts.begin(), pts.end()) - pts.begin();
	size_t forwardIndex = 0;

	for (; index < targetPath.size(); ++index)
	{
		forwardIndex = index;
		float distance = sqrtf(((float)pow(targetPath[index].x - pGps->posX, 2) + pow((float)targetPath[index].y - pGps->posY, 2)));
		if (distance >= distanceTarget)
		{
			break;
		}
	}
	return SSD::SimPoint3D(targetPath[forwardIndex].x, targetPath[forwardIndex].y, 0.0);
}
void UpdateNextRoadId(const SSD::SimPoint3DVector& targetPath, const std::unique_ptr<SimOne_Data_Gps>& pGps, std::vector<SSD::SimString>& nextroadid, double distanceTarget = 30.0)
{
	SSD::SimPoint3D lookaheadPoint = FindLookaheadPoint(targetPath, pGps, distanceTarget);
	SSD::SimString lookaheadlaneId = SampleGetNearMostLane(lookaheadPoint);
	SSD::SimString mainVehicleId = SampleGetNearMostLane({ pGps->posX ,pGps->posY ,0. });

	if (nextroadid.size() == 0) {
		nextroadid.push_back(mainVehicleId);
	}
	int size = nextroadid.size();
	file << "size:" << size << endl;
	file << "mainVehicleId" << mainVehicleId.GetString() << endl;
	for (auto& item : nextroadid)
	{
		file << "nextroadid:" << item.GetString() << std::endl;
	}
	for (int i = 0; i < size; ++i)
	{
		if (mainVehicleId == nextroadid[i])
		{
			for (int j = 0; j < size - i; j++)
			{
				nextroadid[j] = nextroadid[j + i];
			}
			for (int j = 0; j < i; j++)
			{
				file << "pop:" << nextroadid.back().GetString() << endl;
				nextroadid.pop_back();
			}
			break;
		}
	}
	if (nextroadid.back() != lookaheadlaneId)
	{
		nextroadid.push_back(lookaheadlaneId);
	}
}
//Main function
int main()
{
	// ��ʼ������ļ�
	file.open("output.txt", std::ios::out | std::ios::trunc);//�������ԣ������������D:\Sim-One\Common\Foundation\output.txt
	file << "Hello, World!" << std::endl;
	bool inAEBState = false;
	bool isSimOneInitialized = false;
	const char* MainVehicleId = "0";
	bool isJoinTimeLoop = true;
	bool iscalculate = false;
	TargetSpeed = 40;
	float followingminSpeed = 10.;//������С�ٶ�
	// ��ʼ��SimOne API
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	SimOneAPI::SetDriverName(MainVehicleId, "1");
	SimOneAPI::SetDriveMode(MainVehicleId, ESimOne_Drive_Mode_API);
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);
	std::unique_ptr<SimOne_Data_Gps> gpsPtr = std::make_unique<SimOne_Data_Gps>();

	// ���ظ߾���ͼ
	int timeout = 20;
	while (true) {
		if (SimOneAPI::LoadHDMap(timeout)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}

	// ��ȡ���********

	SimOne_Data_CaseInfo caseInfo;
	SimOneAPI::GetCaseInfo(&caseInfo);

	std::cout << "caseID: " << caseInfo.caseId << std::endl;
	std::string casePrefix(caseInfo.caseName, 2);// ֱ��ʹ�� std::string ����ת���������������
	runningCase = std::stol(casePrefix);
	std::cout << "CASEnum: " << runningCase << std::endl;

	if (runningCase == 12)TargetSpeed = 30;
	if (runningCase == 15)TargetSpeed = 20;
	// ����Ŀ��·��
	SSD::SimPoint3DVector targetPath; // Ŀ��·����
	SSD::SimPoint3DVector turnPath; // ת��·����
	// SSD::SimPoint3D endPoint; 
	SSD::SimPoint3D changeendPoint; // ����յ�
	SSD::SimPoint3D turnPoint;
	SSD::SimPoint3D backPoint;
	SimOne_Data_Signal_Lights signal;

	bool stop1flag = false; // ��ֹ���ⰸ�����޷������Ӱ��
	bool turnFlag = false;
	bool avoidaccident = false; // ǰ������
	// double turnDistence = 10.0;
	int k = 0;//��¼���ڼ���·����
	bool createroadFlag = false; // ����·�����־
	bool createacciroadFlag = false; // ���쳵��·�����־
	bool s21 = false;
	bool start = false; // ��ȡ��ʼ�����־
	bool end = false;
	bool end1 = false;
	bool cd = false;
	SSD::SimPoint3D startPt; //��ʼλ��
	SimOne_Data_WayPoints_Entry wayPoints[39];
	SimOne_Data_WayPoints_Entry accidentPoints[5];
	SimOne_Data_WayPoints_Entry inPoints[3] = {};
	std::vector<SimString> nextroadid;
	HDMapStandalone::MSignal light;
	SSD::SimVector<HDMapStandalone::MSignal> lightList;
	SimOneAPI::GetTrafficLightList(lightList);

	auto startTime = std::chrono::system_clock::now();

	/*std::ifstream fileway("D:\\Sim-One\\SimOneAPI\\ADAS\\TrajectoryControl\\way.txt"); // ��ȡ waypoints.txt �ļ�

	for (int i = 0; i < 38; ++i)
	{
		fileway >> wayPoints[i].posX >> wayPoints[i].posY;
		wayPoints[i].index = i;
		wayPoints[i].heading_x = 0.0f;
		wayPoints[i].heading_y = 0.0f;
		wayPoints[i].heading_z = 0.0f;
		wayPoints[i].heading_w = 0.0f;
	}
	fileway.close();*/
	if (runningCase == 27) // ǰ�������¹�
	{
		avoidaccident = true;
		std::unique_ptr<SimOne_Data_WayPoints> caraccidentPoints = std::make_unique<SimOne_Data_WayPoints>();
		if (!SimOneAPI::GetWayPoints(MainVehicleId, caraccidentPoints.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
			return {};
		}
		SetAccidentPath(accidentPoints, caraccidentPoints);
	}

	int wayPointsSize = 0;
	if (runningCase == 41 || runningCase == 39)
	{
		std::unique_ptr<SimOne_Data_WayPoints> carPoints = std::make_unique<SimOne_Data_WayPoints>();
		// ��ȡ����·����
		if (!SimOneAPI::GetWayPoints(MainVehicleId, carPoints.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
			return {};
		}
		if (runningCase == 39) {
			carPoints->wayPoints[carPoints->wayPointsSize - 1] = { carPoints->wayPointsSize - 1, 35.f, -1.6f, 0.0f, 0.0f, 0.0f, 0.0f };
			//carPoints->wayPointsSize += 1;
		}
		for (int i = 0; i < carPoints->wayPointsSize; ++i)
		{
			wayPoints[i] = carPoints->wayPoints[i];
			file << "wayPoints[i].posX:" << wayPoints[i].posX << "wayPoints[i].posY:" << wayPoints[i].posY << endl;
		}
		wayPointsSize = carPoints->wayPointsSize;
		file << "carPoints:" << carPoints->wayPointsSize << endl;
	}

	std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
	// ��ȡ����·����
	if (!SimOneAPI::GetWayPoints(MainVehicleId, pWayPoints.get())) {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
		return {};
	}

	double preThrottle = 0;
	double preFollowingDistence = 0;
	double preSpeed41 = 0;
	double preTargetSpeed = 0.;
	double speedSum = 0;
	bool flag = false;

	// ·����Ҫ��ȫ�ֱ���
	//std::vector<double> crossingRoad;
	SSD::SimVector<SSD::SimPoint3D> intersectionVertices1;//���̵�
	SSD::SimVector<SSD::SimPoint3D> intersectionVertices;//�޺��̵�
	bool crossingRoadCreateFlag = false;//���̵��õı���
	bool crossingRoadCreateFlagF = false;
	//const int extraPoint = 2;

	std::unique_ptr<SimOne_Data_Gps> pre_pGps = std::make_unique<SimOne_Data_Gps>();
	if (!SimOneAPI::GetGps(MainVehicleId, pre_pGps.get())) {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
	}

	while (true) {
		file << std::endl;
		file << std::endl;
		//��ȡ��һ֡�ķ�������
		int frame = SimOneAPI::Wait();


		//�����永����״̬����״̬ΪStop���򱣴�������¼���˳�ѭ����
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		//��ȡGPS���ݣ�����ȡʧ�����˳�ѭ����
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}
		if (flag) {
			runLenth += UtilMath::planarDistance({ pGps->posX,pGps->posY,0. }, { pre_pGps->posX,pre_pGps->posY,0. });
		}
		else {
			flag = true;
		}
		pre_pGps->posX = pGps->posX;
		pre_pGps->posY = pGps->posY;
		file << "runLenth:" << runLenth << endl;

		//��ȡ�ϰ������ݣ������ȡʧ�ܣ����¼������־��
		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
		}
		SimOne_Data_Gps Gps = SimOne_Data_Gps();//��ʼgps
		if (SimOneAPI::GetGps(MainVehicleId, &Gps) && !start)//��ȡ��ʼֵ�ж��Ƿ������������
		{
			start = true;
			startPt.x = Gps.posX;
			startPt.y = Gps.posY;
			startPt.z = Gps.posZ;
			file << "startPt.x:" << startPt.x << std::endl;
			file << "startPt.y:" << startPt.y << std::endl;
		}



		if (!createroadFlag)
		{
			createroadFlag = true;
			//file << "startPt.x:" << startPt.x << std::endl;
			//file << "startPt.y:" << startPt.y << std::endl;
			//if (UtilMath::planarDistance({ -3.916309 ,420.042295 ,0. }, { startPt.x, startPt.y, 0. }) < 3)
			if (runningCase == 41)
			{
				stop1flag = true;
				//file << "����:" << std::endl;
				if (wayPointsSize == k + 2)
				{
					inPoints[0] = wayPoints[k];
					inPoints[1] = wayPoints[k + 1];
					targetPath = GenerateTargetPath(MainVehicleId, inPoints[0], inPoints[1]);
				}
				else {
					inPoints[0] = wayPoints[k];
					inPoints[1] = wayPoints[k + 1];
					inPoints[2] = wayPoints[k + 2];
					targetPath = GenerateTargetPath2(MainVehicleId, inPoints[0], inPoints[1], inPoints[2]);
				}
				//endPoint = targetPath.back();  // ��ȡ·�������һ����
				//runningCase = 41;
			}
			else if (runningCase == 39) {
				stop1flag = true;
				//file << "����:" << std::endl;
				if (wayPointsSize == k + 2)
				{
					inPoints[0] = wayPoints[k];
					inPoints[1] = wayPoints[k + 1];
					targetPath = GenerateTargetPath(MainVehicleId, inPoints[0], inPoints[1]);
				}
				else {
					inPoints[0] = wayPoints[k];
					inPoints[1] = wayPoints[k + 1];
					inPoints[2] = wayPoints[k + 2];
					targetPath = GenerateTargetPath2(MainVehicleId, inPoints[0], inPoints[1], inPoints[2]);
				}
			}
			else
			{
				inPoints[0] = pWayPoints->wayPoints[0];
				inPoints[1] = pWayPoints->wayPoints[1];
				targetPath = GenerateTargetPath(MainVehicleId, inPoints[0], inPoints[1]);
			}
		}
		else
		{
			if (avoidaccident && !createacciroadFlag)// ��ܳ�������
			{
				//file << "createacciroadFlag:" << std::endl;
				createacciroadFlag = true;
				inPoints[0] = accidentPoints[k];
				inPoints[1] = accidentPoints[k + 1];

				targetPath = GenerateTargetPath(MainVehicleId, inPoints[0], inPoints[1]);
				startTime = std::chrono::system_clock::now();
			}
		}
		if (UtilMath::planarDistance({ pGps->posX,pGps->posY,0. }, { wayPoints[k + 1].posX,wayPoints[k + 1].posY, 0. }) < 3)
		{
			if (!avoidaccident)
			{
				createroadFlag = false;
			}
			k += 1;
		}
		else if (UtilMath::planarDistance({ pGps->posX,pGps->posY,0. }, { accidentPoints[k + 1].posX,accidentPoints[k + 1].posY, 0. }) < 5)
		{
			if (avoidaccident)
			{
				createacciroadFlag = false;
				k += 1;
			}
		}

		UpdateNextRoadId(targetPath, pGps, nextroadid);

		if (runningCase == 41) {
			//41���ٶ�����
			TargetSpeed = 80;

			if (runLenth > 100)TargetSpeed = 25;
			if (runLenth > 130)TargetSpeed = 55;
			if (runLenth > 160)TargetSpeed = 45;//��ֻ��45
			if (runLenth > 210)TargetSpeed = 55;
			//if (runLenth > 240)TargetSpeed = 35;//����ǰ��ǰ����
			if (runLenth > 250)TargetSpeed = 45;//����!!!�����45���ԣ�ע������һ�У�����45��Ϊ28��
			//if (runLenth > 330)TargetSpeed = 30;//������С��һ�ѣ�С��һ��Ҳ����;����ط�ֻ��45/30/28��Ψ������29������Ȼ���������Ѷȼ�ǿ֮��ֻ��28�ˣ�
			if (runLenth > 400)TargetSpeed = 60;//50��60��70���У�����һ�ε��ٶ��ƺ�û�ĵ����о���Ϊ����·�ڵĺ����ˣ�80
			if (runLenth > 450)TargetSpeed = 70;//70������ȫ���У����ܸĵ��ˣ���ѹ��
			if (runLenth > 500)TargetSpeed = 60;
			if (runLenth > 620)TargetSpeed = 55;//�Ǹ���·id���˵��ϰ�·�ڣ�����runlenth��//��֪���۲�����ѹ�ߣ�55�����൱����
			//if (runLenth > 660)TargetSpeed = 20;
			if (runLenth > 690)TargetSpeed = 45;//�����Ǹ�·�ڣ���Ȼ45����һ��ѹ�ߣ�����û�۷֣�����ѹ�߿��Ըĳ�40
			if (runLenth > 840)TargetSpeed = 100;//840������������
			if (runLenth > 1060)TargetSpeed = 82;
			if (runLenth > 1340)TargetSpeed = 120;
			if (runLenth > 1480)TargetSpeed = 70;//������һ�У�������Ȳ��ӿ�һ��
			if (runLenth > 1520)TargetSpeed = 50;//���к���һ��һ�㶼���ܸģ���ȻС������һ�£�����runlenth���˲���ǡ��û����һ��·���㣬�ᵼ��1106�еġ�75���ٶ��ߴ�
			if (runLenth > 1570)TargetSpeed = 65;//������ϼ�����
			if (runLenth > 1640)TargetSpeed = 70;//ԭ����60������
			if (runLenth > 1690)TargetSpeed = 100;//�ᵼ�¹���ѹ�ߵ��ǲ��۷֣���Ҫ��ѹ�����Ϊ90
			if (runLenth > 1880)TargetSpeed = 80;
			if (runLenth > 2160)TargetSpeed = 100;//ԭ����50�����ϣ�60����
			if (runLenth > 2220)TargetSpeed = 120;
			if (runLenth > 2470)TargetSpeed = 80;//2410,70
			//if (runLenth > 2540)TargetSpeed = 70;//45������
			if (runLenth > 2600)TargetSpeed = 80;
			if (runLenth > 2710)TargetSpeed = 75;
			if (runLenth > 3000)TargetSpeed = 60;
			if (runLenth > 3080)TargetSpeed = 60;//�ټӿ�һ��㣬55����
			if (runLenth > 3210)TargetSpeed = 50;//��������
			if (runLenth > 3400)TargetSpeed = 55;//����
			if (runLenth > 3600)TargetSpeed = 50;//����
		}

		//�����永��������״̬�����״̬Ϊ Running�������ִ�к����߼���
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
			if (!isSimOneInitialized) {
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}

			//����������λ�ú��ٶ�
			// In this system, laneName is organized as string(SSD::SimString) with this format roadId_sectionIndex_laneId.(���ע����Դ��MHDMap.h)
			SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
			double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

			//Ѱ��������
			static int warningSignIndex = -1;
			if (utilTargetObstacle::my_DetectSpeedLimitSign(pGps.get(), warningSignIndex)) {
				SSD::SimVector<HDMapStandalone::MSignal> TrafficSignList;
				SimOneAPI::GetTrafficSignList(TrafficSignList);
				auto& sign = TrafficSignList[warningSignIndex];
				std::string speedLimitValue = sign.value.GetString();
				double speedLimit;
				std::stringstream ss;
				ss << speedLimitValue;
				ss >> speedLimit;
				if (runningCase != 41)
					TargetSpeed = (int)(speedLimit * 0.95) - 1;//��Ŀ���ٶ����������ٵ�%95

			}
			SSD::SimVector<HDMapStandalone::MSignal> TrafficSignList;
			SimOneAPI::GetTrafficSignList(TrafficSignList);
			//���������仰���ҿ��ǵ���һ�£������δ���ֻ��Ҫ����һ��
			HDMapStandalone::MSignal SchoolSign;
			bool SchoolFlag = false;
			if (SchoolFlag = detectTargetSign(TrafficSignList, SchoolSign, mainVehiclePos))
			{
				TargetSpeed = 9.0f;
				double mainVehicleSpeed_kmh = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ) * 3.6;
				float speedErrors = TargetSpeed - mainVehicleSpeed_kmh;
				double distanceToSchool = UtilMath::planarDistance(SchoolSign.pt, mainVehiclePos);
				bool shouldSlowDown = distanceToSchool < 60.0;

				std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();
				SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());

				if (speedErrors > 3.0f && !shouldSlowDown)
				{
					pControl->throttle = 1.0;
					pControl->brake = 0.0;
				}
				else if (speedErrors > 0.0f && !shouldSlowDown)
				{
					pControl->throttle = 0.3;
					pControl->brake = 0.0;
				}
				else if (speedErrors > 3.0f && shouldSlowDown)//���ҽ����ж�·�ε����ҵ��ٶ�̫С�ˣ����ٲ�ɲ����
				{
					pControl->throttle = 0.2;
					pControl->brake = 0.0;
				}
				else
				{
					pControl->throttle = 0.1;
					pControl->brake = 0.05;
				}

				preThrottle = pControl->throttle;
				SimOneAPI::SetDrive(MainVehicleId, pControl.get());
				SimOneAPI::NextFrame(frame);
				continue;
			}
			if (runningCase == 11) { // �޿�·��ͨ��
				SimOne_Data_WayPoints_Entry wayPoints[39] = {
					{0, 158.f, 67.f, 0.0f, 0.0f, 0.0f, 0.0f},
					{1, 174.f, 66.f, 0.0f, 0.0f, 0.0f, 0.0f},
					{2, 199.f, 68.f, 0.0f, 0.0f, 0.0f, 0.0f},
					{3, 224.f, 66.f, 0.0f, 0.0f, 0.0f, 0.0f},
					{4, 244.f, 67.f, 0.0f, 0.0f, 0.0f, 0.0f} };
				targetPath = GenerateStraightTargetPath(MainVehicleId, 5, wayPoints);
				double mainVehicleSpeed_kmh = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ) * 3.6;
				float speedKi = 0.00001;
				float speedKp = 0.60;
				float speedErrors = TargetSpeed - mainVehicleSpeed_kmh;
				float Imax = 500.;
				float Imin = -500.;
				float throttlePoint = 0;
				speedSum += speedErrors;
				if (speedSum > Imax)speedSum = Imax;
				if (speedSum < Imin)speedSum = Imin;

				throttlePoint += speedSum * speedKi;
				throttlePoint += speedErrors * speedKp;
				// file << "speedi:" << speedSum * speedKi << std::endl;
				// file << "speedp:" << speedErrors * speedKp << std::endl;

				if (throttlePoint > 1)throttlePoint = 1;
				if (throttlePoint < -1)throttlePoint = -1;

				std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();
				// Control mainVehicle with SimOneDriver
				SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());
				pControl->steering = my_calculateSteering(targetPath, pGps.get());
				if (throttlePoint > 0) {
					pControl->throttle = throttlePoint;
					pControl->brake = 0.;
				}
				else {
					pControl->throttle = 0.;
					pControl->brake = -throttlePoint;
				}
				preThrottle = pControl->throttle;
				SimOneAPI::SetDrive(MainVehicleId, pControl.get());
				SimOneAPI::NextFrame(frame);
				continue;
			}

			if (runningCase == 29) { // խ���ᳵ
				SimOne_Data_WayPoints_Entry wayPoints[4] = {
					{0, 145.f, 67.f, 0.0f, 0.0f, 0.0f, 0.0f},
					{1, 168.f, 65.f, 0.0f, 0.0f, 0.0f, 0.0f},
					{2, 230.f, 65.f, 0.0f, 0.0f, 0.0f, 0.0f},
					{3, 238.f, 67.f, 0.0f, 0.0f, 0.0f, 0.0f} };
				SSD::SimPoint3DVector targetPath;
				targetPath = GenerateStraightTargetPath(MainVehicleId, 4, wayPoints);
				double mainVehicleSpeed_kmh = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ) * 3.6;
				float speedKi = 0.00001;
				float speedKp = 0.60;
				float speedErrors = TargetSpeed - mainVehicleSpeed_kmh;
				float Imax = 500.;
				float Imin = -500.;
				float throttlePoint = 0;
				speedSum += speedErrors;
				if (speedSum > Imax)speedSum = Imax;
				if (speedSum < Imin)speedSum = Imin;

				throttlePoint += speedSum * speedKi;
				throttlePoint += speedErrors * speedKp;
				// file << "speedi:" << speedSum * speedKi << std::endl;
				// file << "speedp:" << speedErrors * speedKp << std::endl;

				if (throttlePoint > 1)throttlePoint = 1;
				if (throttlePoint < -1)throttlePoint = -1;

				std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();
				// Control mainVehicle with SimOneDriver
				SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());
				pControl->steering = my_calculateSteering(targetPath, pGps.get());
				if (throttlePoint > 0) {
					pControl->throttle = throttlePoint;
					pControl->brake = 0.;
				}
				else {
					pControl->throttle = 0.;
					pControl->brake = -throttlePoint;
				}
				preThrottle = pControl->throttle;
				SimOneAPI::SetDrive(MainVehicleId, pControl.get());
				SimOneAPI::NextFrame(frame);
				continue;
			}
			//file << "TargetSpeed:" << TargetSpeed << std::endl;


			//���������ϰ���ҵ�����������ͬһ�����Ҿ���������ϰ��
			double minDistance = std::numeric_limits<double>::max();
			int potentialObstacleIndex = pObstacle->obstacleSize;
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
			static SSD::SimString premainVehicleLaneId;
			long RoadId, LaneId;
			SSD::SimString potentialObstacleLaneId = "";
			RoadId = fromLaneIdGetRoadId(mainVehicleLaneId);
			LaneId = fromLaneIdGetLaneId(mainVehicleLaneId);



			//����ռ�������Ϣ

			// file << std::endl;
			// file << std::endl;
			// file << std::endl;
			// file << std::endl;
			//file << "��������:" << mainVehicleLaneId.GetString() << std::endl;
			//file << "������·:" << RoadId << std::endl;
			//file << "����������:" << LaneId << std::endl;
			//file << "����λ��:" << "x" << mainVehiclePos.x << "y" << mainVehiclePos.y << "z" << mainVehiclePos.z << std::endl;
			// file << "�����ٶ�:" << "x" << pGps->velX << "y" << pGps->velY << "z" << pGps->velZ << std::endl;
			double mainLineWidth, mainVehicleS, mainVehicleT, obstacleS, obstacleT;
			SimOneAPI::GetLaneWidth(mainVehicleLaneId, mainVehiclePos, mainLineWidth);
			// file << "�����������:" << mainLineWidth << std::endl;
			SimOneAPI::GetLaneST(mainVehicleLaneId, mainVehiclePos, mainVehicleS, mainVehicleT);
			file << "�����ڳ����е�λ��	s:" << mainVehicleS << "t:" << mainVehicleT << std::endl;

			//���̵Ʋ���
			//���Ժ��̵�
			static int crossState;//����ͨ��ʮ��·�ڵ�״̬��
			bool findlightflag = false;
			bool slowdown31 = false;
			// file << "all light size:" << lightList.size() << std::endl;
			for (auto& item : lightList)
			{
				// file << "light id " << item.id << std::endl;
				// file << "validities size " << item.validities.size() << std::endl;
				auto& ptValidities = item.validities[0];

				//current lane
				if (ptValidities.roadId == RoadId && ptValidities.fromLaneId == LaneId)
				{
					light = item;
					findlightflag = true;
					break;
				}
			}
			double margin = 0.5;
			double stoplineS, stoplineT;//���ڼ�¼ֹͣ�ߵ�λ��
			if (findlightflag) {
				SSD::SimVector<HDMapStandalone::MObject> stoplineList;
				SimOneAPI::GetStoplineList(light, mainVehicleLaneId, stoplineList);
				file << "���̵�ֹͣ�ߵ�λ��	x:" << stoplineList[0].pt.x << "y:" << stoplineList[0].pt.y << std::endl;
				SimOneAPI::GetLaneST(mainVehicleLaneId, stoplineList[0].pt, stoplineS, stoplineT);
				stoplineS = stoplineS + margin;
				file << "���̵�ֹͣ���ڳ����е�λ��	s:" << stoplineS << "t:" << stoplineT << std::endl;
				file << "֮ǰ�ĵ�·ID" << premainVehicleLaneId.GetString() << endl;
				file << "���ڵĵ�·ID" << mainVehicleLaneId.GetString() << endl;
				if (stoplineS - mainVehicleS <= 70 && stoplineS >= mainVehicleS) {
					crossState = 1;
					premainVehicleLaneId = mainVehicleLaneId;
				}
				if (stoplineS - mainVehicleS <= 65 && runningCase == 31)
				{
					slowdown31 = true;
				}
			}

			//�����ϰ���
			bool AEBflag = false;
			bool stopFlag = false;
			bool followingFlag = false;
			bool findobstacleflag = false;  // ˫���ϰ������
			double minCarDistence = std::numeric_limits<double>::max();
			int minDistenceCarId = 0;
			double minCollisionTime = std::numeric_limits<double>::max();
			HDMapStandalone::MLaneType laneType;
			SimOneAPI::GetLaneType(mainVehicleLaneId, laneType);

			/*SSD::SimStringVector lanes; // ������г���������
			SimOneAPI::GetLaneType(mainVehicleLaneId, laneType);
			SimOneAPI::GetNearLanes(mainVehiclePos, mainLineWidth*0.9, lanes);
			int laneCount = lanes.size(); // ��ȡ��������
			file << "���ڳ���: ";
			for (const auto& lane : lanes) {
				file << lane.GetString() << " ";  // ʹ�� GetString() ��ȡʵ���ַ���
			}
			file << std::endl;*/
			double s1, t1, s_toCenterLine1, t_toCenterLine1;
			double s2, t2, s_toCenterLine2, t_toCenterLine2;
			SSD::SimString laneName1;
			SSD::SimString laneName2;

			double ObstacleSpeed1;
			double ObstacleSpeed2;
			std::vector<utilTargetObstacle::ObstacleStruct> allObstacles;
			allObstacles = utilTargetObstacle::GetObstacleList();

			//if (pObstacle->obstacleSize == 2 && !findobstacleflag)
			if (runningCase == 21)
			{
				// ����Ѿ�ֹ���ϰ��������·�ߵġ�ûӰ��ģ�������Ҫ��в�������ȿ��ǡ��������������������˶����ϰ����
				if (!findobstacleflag)
				{
					SSD::SimPoint3D obstaclePos1(pObstacle->obstacle[0].posX, pObstacle->obstacle[0].posY, pObstacle->obstacle[0].posZ);
					ObstacleSpeed1 = UtilMath::calculateSpeed(pObstacle->obstacle[0].velX, pObstacle->obstacle[0].velY, pObstacle->obstacle[0].velZ);
					SimOneAPI::GetNearMostLane(obstaclePos1, laneName1, s1, t1, s_toCenterLine1, t_toCenterLine1);
					SSD::SimPoint3D obstaclePos2(pObstacle->obstacle[1].posX, pObstacle->obstacle[1].posY, pObstacle->obstacle[1].posZ);
					ObstacleSpeed2 = UtilMath::calculateSpeed(pObstacle->obstacle[1].velX, pObstacle->obstacle[1].velY, pObstacle->obstacle[1].velZ);
					SimOneAPI::GetNearMostLane(obstaclePos2, laneName2, s2, t2, s_toCenterLine2, t_toCenterLine2);
					if (laneName1 == mainVehicleLaneId && ObstacleSpeed1 == 0 && laneName2 != mainVehicleLaneId && ObstacleSpeed2 != 0)
					{
						findobstacleflag = true;
						s21 = true;
						laneName1 = laneName2;
					}
					else if (laneName1 != mainVehicleLaneId && ObstacleSpeed1 != 0 && laneName2 == mainVehicleLaneId && ObstacleSpeed2 == 0)
					{
						findobstacleflag = true;
						s21 = true;
					}
				}
				// ɾ������в�����ϰ���
				if (s21) {
					signal.signalLights = ESimOne_Signal_Light_LeftBlinker;
					allObstacles.erase(std::remove_if(allObstacles.begin(), allObstacles.end(),
						[&](const utilTargetObstacle::ObstacleStruct& obs) {  // ���������ⲿ����
						return obs.ownerLaneId == laneName1;
					}),
						allObstacles.end());
					file << "allObstacles.size():" << allObstacles.size() << std::endl;
				}
			}

			SSD::SimString leftNeighborLaneName = utilTargetLane::GetLeftNeighborLane(mainVehicleLaneId);
			SSD::SimString rightNeighborLaneName = utilTargetLane::GetRightNeighborLane(mainVehicleLaneId);
			int warningObstacleIndex = -1;
			bool isObstacleDetected = false;
			double s, t, s_toCenterLine, t_toCenterLine;
			SSD::SimString laneName;

			for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
				SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);

				// double lineWidth;
				//file << "�ϰ��ﳵ��:" << obstacleLaneId.GetString() << std::endl;
				// file << "�ϰ���λ��:" << "x" << obstaclePos.x << "y" << obstaclePos.y << "z" << obstaclePos.z << std::endl;
				// SimOneAPI::GetLaneWidth(obstacleLaneId, obstaclePos, lineWidth);
				// file << "�ϰ��ﳵ�����:" << lineWidth << std::endl;
				SimOneAPI::GetLaneST(obstacleLaneId, obstaclePos, obstacleS, obstacleT);
				// file << "�ϰ����ڳ����е�λ��	s:" << obstacleS << "t:" << obstacleT << std::endl;
				double defaultDistanceOfPedestrian = 10.f;
				double defautlTimeToCollision = 5.f;
				double defaultDistanceOfCar = 7.2f;
				double defaultDistanceOfStatic = 7.2f;
				double defautlTimeToCollisionOfCar = 5.f;
				double defautlTimeToCollisionOfStatic = 4.8f;

				double ObstacleSpeed = UtilMath::calculateSpeed(pObstacle->obstacle[i].velX, pObstacle->obstacle[i].velY, pObstacle->obstacle[i].velZ);
				double ObstacleDistence = UtilMath::planarDistance(obstaclePos, mainVehiclePos);//�ϰ������������ֱ�߾���

				// ����ٶ����������λ������֮���ͶӰ��ϵ
				// dir < 0������ٶ�ָ���������ϰ���������������˶�������
				// dir > 0������ٶ�Զ���������ϰ���������໥Զ��
				double dir = (pObstacle->obstacle[i].velX - pGps->velX) * (pObstacle->obstacle[i].posX - pGps->posX) +
					(pObstacle->obstacle[i].velY - pGps->velY) * (pObstacle->obstacle[i].posY - pGps->posY);


				switch (pObstacle->obstacle[i].type)
				{
				case ESimOne_Obstacle_Type_Pedestrian://����
					SimOneAPI::GetLaneST(mainVehicleLaneId, obstaclePos, obstacleS, obstacleT);//��ȡ��������������λ��
					file << "��������" << endl;
					//���ڲ���ͬһ����ʱ�������������󣨹�����������жϳ��������Ƿ���ڵ���ʵ�ʾ��룬Ϊ��ֹ����������������һ��
					if (abs(obstacleT) < mainLineWidth / 2 + pObstacle->obstacle[i].width + 1 && obstacleS > mainVehicleS && ObstacleDistence <= obstacleS - mainVehicleS + 1 && dir <= 1e-6)
					{
						double timeToCollision = std::abs((obstacleS - mainVehicleS)) / (mainVehicleSpeed);
						if (obstacleS - mainVehicleS < defaultDistanceOfPedestrian || (defautlTimeToCollision > timeToCollision))
						{
							if (timeToCollision < minCollisionTime)
							{
								minCollisionTime = timeToCollision;
								potentialObstacleIndex = i;
							}
							if (obstacleS - mainVehicleS < defaultDistanceOfPedestrian) {
								stopFlag = true;
							}
							AEBflag = true;
						}
					}
					break;

				case ESimOne_Obstacle_Type_Car://����
					SimOneAPI::GetLaneST(mainVehicleLaneId, obstaclePos, obstacleS, obstacleT);//��ȡ��������������λ��
					file << "�ϰ��������������е�λ��	s:" << obstacleS << "t:" << obstacleT << std::endl;
					if (abs(obstacleT) < mainLineWidth / 2 + pObstacle->obstacle[i].width / 2 && obstacleS > mainVehicleS && ObstacleDistence <= obstacleS - mainVehicleS + 1)//�ж��Ƿ��ڱ�������ǰ��
					{
						//�ж��Ƿ�������ģʽ
						if (obstacleS - mainVehicleS < 50 && obstacleS - mainVehicleS > 0 && ObstacleSpeed != 0) {
							followingFlag = true;
							if (minCarDistence > obstacleS - mainVehicleS) {
								minCarDistence = obstacleS - mainVehicleS;
								minDistenceCarId = i;
							}
						}
						file << "followingFlag:" << followingFlag << std::endl;
						file << "minCarDistence:" << minCarDistence << std::endl;
						//file << "findobstacleflag:" << findobstacleflag << std::endl;
						//file << "s2 - mainVehicleS:" << s2 - mainVehicleS << std::endl;
						// file << "avoidaccident:" << avoidaccident << std::endl;
						if (findobstacleflag) // 
						{
							if (s2 - mainVehicleS < 7)
							{
								end = false;
								stopFlag = true;
								AEBflag = true;
							}
							else
							{
								end = false;
								AEBflag = false;
							}
						}
						else
						{
							double timeToCollision = std::abs(obstacleS - mainVehicleS) / (mainVehicleSpeed - ObstacleSpeed);
							if (timeToCollision < 0) {
								timeToCollision = std::numeric_limits<double>::max();
							}
							if (obstacleS - mainVehicleS < defaultDistanceOfCar || (defautlTimeToCollisionOfCar > timeToCollision))
							{
								if (timeToCollision < minCollisionTime)
								{
									minCollisionTime = timeToCollision;
									potentialObstacleIndex = i;
								}
								if (obstacleS - mainVehicleS < defaultDistanceOfCar) {
									stopFlag = true;
								}
								AEBflag = true;

							}
						}
					}
					break;

				case ESimOne_Obstacle_Type_Static://׶Ͱ
					//SimOneAPI::GetNearMostLane(startPt, laneName, s, t, s_toCenterLine, t_toCenterLine);
					if (leftNeighborLaneName == ""&& rightNeighborLaneName == "" && !stop1flag)
					{
						SimOneAPI::GetNearMostLane(obstaclePos, laneName, s, t, s_toCenterLine, t_toCenterLine);
						if (mainVehicleLaneId == laneName && abs(t) < mainLineWidth / 4)
						{
							double timeToCollision = std::abs(obstacleS - mainVehicleS) / (mainVehicleSpeed - ObstacleSpeed);
							if (timeToCollision < 0) {
								timeToCollision = std::numeric_limits<double>::max();
							}
							if (obstacleS - mainVehicleS < defaultDistanceOfCar || (defautlTimeToCollisionOfCar > timeToCollision))
							{
								if (timeToCollision < minCollisionTime)
								{
									minCollisionTime = timeToCollision;
									potentialObstacleIndex = i;
								}
								if (obstacleS - mainVehicleS < defaultDistanceOfCar) {
									stopFlag = true;
								}
								AEBflag = true;
							}
						}
					}
					break;
				default:
					break;
				}

			}

			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();
			// Control mainVehicle with SimOneDriver
			SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());

			SSD::SimPoint3D changeToPoint;
			turnPath = targetPath;
			isObstacleDetected = utilTargetObstacle::DetectObstacle(mainVehiclePos, mainVehicleSpeed, allObstacles, laneType, mainVehicleLaneId, leftNeighborLaneName, changeToPoint, rightNeighborLaneName, turnPath, warningObstacleIndex, nextroadid);

			//file << "isObstacleDetected:" << isObstacleDetected << endl;
			// changeendPoint = targetPath.back();  // ��ȡ·�������һ����
			file << "allObstacles.size():" << allObstacles.size() << endl;
			//����13
			if (allObstacles.size() > 1)
			{
				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[1].posX, pObstacle->obstacle[1].posY, pObstacle->obstacle[1].posZ);
				if (UtilMath::planarDistance({ startPt.x ,startPt.y ,startPt.z }, obstaclePos) < 10 && pObstacle->obstacle[1].type == ESimOne_Obstacle_Type_Static && !end1)
				{
					end = true;
					pControl->gear = ESimOne_Gear_Mode_Reverse;
					pControl->steering = 0.;
					pControl->brake = 1.;
					pControl->throttle = 0.;

				}
				if (end && (mainVehicleSpeed == 0 || end1))
				{
					end1 = true;
					pControl->gear = ESimOne_Gear_Mode_Reverse;
					pControl->steering = 0.;
					pControl->brake = 0.;
					pControl->throttle = -0.4;
					if (!cd)
					{
						cd = true;
						backPoint.x = mainVehiclePos.x;
					}
				}
				if (UtilMath::planarDistance({ mainVehiclePos.x,0.,0. }, { backPoint.x,0.,0. }) > 10 && end1)
				{
					end = false;
					pControl->gear = ESimOne_Gear_Mode_Drive;
				}

			}

			double steering;
			//file << "end:" << end << endl;
			//file << "targetPath1:" << targetPath[1].x << endl;
			//file << "targetPath0:" << targetPath[0].x << endl;
			if (!end)
			{
				if (isObstacleDetected && !avoidaccident && !followingFlag) {
					turnPoint = mainVehiclePos;
					turnFlag = true;
				}
				file << "turnFlag:" << turnFlag << endl;
				if (turnFlag) {
					AEBflag = false;
					if (UtilMath::planarDistance(mainVehiclePos, changeToPoint) < 5) {
						turnFlag = false;
						file << "k:" << k << endl;
						// file << "changeToPoint.x:" << changeToPoint.x << endl;
						// file << "changeToPoint.y:" << changeToPoint.y << endl;
						if (wayPointsSize == 0) {
							pWayPoints->wayPoints[k] = { k, static_cast<float>(changeToPoint.x),static_cast<float>(changeToPoint.y), 0.0f, 0.0f, 0.0f, 0.0f };
							targetPath = GenerateTargetPath(MainVehicleId, pWayPoints->wayPoints[k], pWayPoints->wayPoints[k + 1]);
						}
						else if (wayPointsSize == k + 2) {
							wayPoints[k] = { k, static_cast<float>(changeToPoint.x),static_cast<float>(changeToPoint.y), 0.0f, 0.0f, 0.0f, 0.0f };
							targetPath = GenerateTargetPath(MainVehicleId, wayPoints[k], wayPoints[k + 1]);
							file << "targetPath size: " << targetPath.size() << endl;
							for (int i = 0; i < targetPath.size(); ++i) {
								const SSD::SimPoint3D& pt = targetPath[i];
								file << "targetPath[" << i << "]: x=" << pt.x
									<< ", y=" << pt.y
									<< ", z=" << pt.z << endl;
							}
						}
						else {
							wayPoints[k] = { k, static_cast<float>(changeToPoint.x),static_cast<float>(changeToPoint.y), 0.0f, 0.0f, 0.0f, 0.0f };
							targetPath = GenerateTargetPath2(MainVehicleId, wayPoints[k], wayPoints[k + 1], wayPoints[k + 2]);
						}
						steering = my_calculateSteering(targetPath, pGps.get());
						// file << "steering1:" << steering << endl;
					}
					else {
						steering = my_calculateSteering(turnPath, pGps.get());
						file << "steering2:" << steering << endl;
					}
				}
				else {
					steering = my_calculateSteering(targetPath, pGps.get());
					/*file << "targetPath size: " << targetPath.size() << endl;
					for (int i = 0; i < targetPath.size(); ++i) {
						const SSD::SimPoint3D& pt = targetPath[i];
						file << "targetPath[" << i << "]: x=" << pt.x
							<< ", y=" << pt.y
							<< ", z=" << pt.z << endl;
					}*/
				}
				pControl->steering = (float)steering;
				file << "turnFlag:" << turnFlag << endl;

				auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
			}

			if (isObstacleDetected)
			{
				if (steering < -0.4)
				{
					signal.signalLights = ESimOne_Signal_Light_LeftBlinker;
				}
				if (steering > 0.4)
				{

					signal.signalLights = ESimOne_Signal_Light_RightBlinker;
				}
			}
			if (allObstacles.size() == 0)
			{
				signal.signalLights = ESimOne_Signal_Light_LeftBlinker;
			}
			file << "crossState:" << crossState << endl;
			//���к��̵�
			if (crossState) {
				file << "֮ǰ�ĵ�·ID" << premainVehicleLaneId.GetString() << endl;
				file << "���ڵĵ�·ID" << mainVehicleLaneId.GetString() << endl;
				if (premainVehicleLaneId != mainVehicleLaneId) {
					crossState++;
					premainVehicleLaneId = mainVehicleLaneId;
				}
				file << "crossState:" << crossState << endl;
				if (crossState == 3) {
					//crossingRoad.clear();
					intersectionVertices1.clear();
					crossingRoadCreateFlag = false;
					crossState = 0;
				}
				if (crossState == 1) {
					bool CrossstopFlag = false;//�����ж��Ƿ�Ҫ�ڰ�����ǰͣ��
					if (findlightflag) {/*
						SimOne_Data_TrafficLight trafficLight;//�����жϺ�ƻ����̵�
						SimOneAPI::GetTrafficLight(0, light.id, &trafficLight);
						SSD::SimVector<HDMapStandalone::MObject>CrossWalkList;//��ǰ����ǰ���İ�����
						SimOneAPI::GetSpecifiedLaneCrosswalkList(mainVehicleLaneId, CrossWalkList);//��ȡ��ǰ����ǰ���İ�����
						SSD::SimVector<HDMapStandalone::MObject>mainVehicleLightCrossWalkList;//ǰ�������İ����ߣ�����İ�����λ�ã�
						SSD::SimString nextLaneId = getLaneId(light.validities[1].roadId, light.validities[1].sectionIndex, light.validities[1].fromLaneId);//��ȡǰ��������Ϣ
						SimOneAPI::GetSpecifiedLaneCrosswalkList(nextLaneId, mainVehicleLightCrossWalkList);//��ȡ����İ�����λ��
						//����4���������ڼ�¼ʮ��·�ڵķ�Χ��ͨ�����������ߵ�������µõ���
						double xmin = std::numeric_limits<double>::max();
						double ymin = std::numeric_limits<double>::max();
						double xmax = -std::numeric_limits<double>::max();
						double ymax = -std::numeric_limits<double>::max();

						file << "���к������:" << mainVehicleLightCrossWalkList[0].type.GetString() << endl;
						//�������������
						for (auto& item : mainVehicleLightCrossWalkList) {
							//file << "���к������x:" << item.pt.x << " y:" << item.pt.y << endl;
							//file << "���к������:" << item.type.GetString() << endl;
							//file << "boundaryKnots" << endl;;
							for (auto& item1 : item.boundaryKnots) {
								//file << "x:" << item1.x << " y:" << item1.y << endl;
								if (item1.x < xmin)xmin = item1.x;
								if (item1.x > xmax)xmax = item1.x;
								if (item1.y < ymin)ymin = item1.y;
								if (item1.y > ymax)ymax = item1.y;
							}
						}
						//ͬ����ʽ������ǰǰ���İ�����
						for (auto& item : CrossWalkList) {
							for (auto& item1 : item.boundaryKnots) {
								if (item1.x < xmin)xmin = item1.x;
								if (item1.x > xmax)xmax = item1.x;
								if (item1.y < ymin)ymin = item1.y;
								if (item1.y > ymax)ymax = item1.y;
							}
						}
						//��ʮ��·�ڵķ�Χ��������
						float enlargeRate = 1.5;
						file << "xmin:" << xmin << " xmax:" << xmax << endl;
						file << "ymin:" << ymin << " ymax:" << ymax << endl;
						//�����������x�����С����һ�࣬���x������������ʣ��ͬ��
						if (mainVehiclePos.x >= xmin)xmin = mainVehiclePos.x + (xmin - mainVehiclePos.x) * enlargeRate;
						if (mainVehiclePos.x <= xmax)xmax = mainVehiclePos.x + (xmax - mainVehiclePos.x) * enlargeRate;
						if (mainVehiclePos.y >= ymin)ymin = mainVehiclePos.y + (ymin - mainVehiclePos.y) * enlargeRate;
						if (mainVehiclePos.y <= ymax)ymax = mainVehiclePos.y + (ymax - mainVehiclePos.y) * enlargeRate;
						file << "xmin:" << xmin << " xmax:" << xmax << endl;
						file << "ymin:" << ymin << " ymax:" << ymax << endl;

						if (!crossingRoadCreateFlag)
						{
							file << "��ǰ�к��̵ƣ����ڽ���·������ֵ����" << endl;
							crossingRoadCreateFlag = true;
							crossingRoad.clear();  // ��վ�ֵ
							crossingRoad.push_back(xmin);
							crossingRoad.push_back(xmax);
							crossingRoad.push_back(ymin);
							crossingRoad.push_back(ymax);
							//̫���ˣ����ĸ�����ֵ����׼ȷ
						}

						//�����Ϊ�̵ƣ���Ҫͣ��
						if (trafficLight.status != ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Green) {
							CrossstopFlag = true;
						}
						//���·�����г������ˣ���Ҫͣ��
						for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
							if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Pedestrian || pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Car) {
								if (pObstacle->obstacle[i].posX >= xmin && pObstacle->obstacle[i].posX <= xmax && pObstacle->obstacle[i].posY >= ymin && pObstacle->obstacle[i].posY <= ymax) {
									CrossstopFlag = true;
								}
							}
						}*/

						//****************************************************88
						SimOne_Data_TrafficLight trafficLight;//�����жϺ�ƻ����̵�
						SimOneAPI::GetTrafficLight(0, light.id, &trafficLight);
						SSD::SimVector<HDMapStandalone::MObject>CrossWalkList;//��ǰ����ǰ���İ�����
						SimOneAPI::GetSpecifiedLaneCrosswalkList(mainVehicleLaneId, CrossWalkList);//��ȡ��ǰ����ǰ���İ�����
						SSD::SimPoint3D Axy2(CrossWalkList[0].boundaryKnots[0].x, CrossWalkList[0].boundaryKnots[0].y, 0.);
						SSD::SimPoint3D Bxy1(CrossWalkList[0].boundaryKnots[3].x, CrossWalkList[0].boundaryKnots[3].y, 0.);
						SSD::SimPoint3D Dxy1(CrossWalkList[0].boundaryKnots[1].x, CrossWalkList[0].boundaryKnots[1].y, 0.);
						double shortDistance1 = std::sqrt(std::pow(Bxy1.x - Axy2.x, 2) + std::pow(Axy2.y - Bxy1.y, 2));
						double longDistance1 = std::sqrt(std::pow(Axy2.x - Dxy1.x, 2) + std::pow(Axy2.y - Dxy1.y, 2));
						if (shortDistance1 > longDistance1)
						{
							std::swap(shortDistance1, longDistance1);
							std::swap(Bxy1, Dxy1); // ȷ�����꽻��ͬ��***************************************88
							//Bxy1.x = mainVehicleLaneCrossWalkList[0].boundaryKnots[1].x;
							//Bxy1.y = mainVehicleLaneCrossWalkList[0].boundaryKnots[1].y;
							//Dxy1.x = mainVehicleLaneCrossWalkList[0].boundaryKnots[3].x;
							//Dxy1.y = mainVehicleLaneCrossWalkList[0].boundaryKnots[3].y;
						}
						file << "�к��̵�ʱsdistance��" << shortDistance1 << "  longdistance: " << longDistance1 << endl;
						//SSD::SimVector<SSD::SimPoint3D> intersectionVertices1;
						//GetCrossroad(shortDistance1, longDistance1, intersectionVertices1);
						if (!crossingRoadCreateFlag) {
							crossingRoadCreateFlag = true;
							GetCrossroad(shortDistance1, longDistance1, intersectionVertices1);
							intersectionVertices1.push_back(Axy2);
							intersectionVertices1.push_back(Bxy1);
							file << "intersectionVertices1.size:" << intersectionVertices1.size() << endl;
						}

						//����
						SSD::SimPoint3D Axy3 = ConvertToXY(Axy2, Bxy1, intersectionVertices1[0]);
						SSD::SimPoint3D Fxy1 = ConvertToXY(Axy2, Bxy1, intersectionVertices1[3]);
						SSD::SimPoint3D Gxy1 = ConvertToXY(Axy2, Bxy1, intersectionVertices1[4]);
						SSD::SimPoint3D Jxy1 = ConvertToXY(Axy2, Bxy1, intersectionVertices1[7]);
						file << "�к��̵�·��" << endl;
						file << "·�ڷ�ΧAxy:" << Axy3.x << ", " << Axy3.y << endl;
						file << "·�ڷ�ΧFxy:" << Fxy1.x << ", " << Fxy1.y << endl;
						file << "·�ڷ�ΧGxy:" << Gxy1.x << ", " << Gxy1.y << endl;
						file << "·�ڷ�ΧJxy:" << Jxy1.x << ", " << Jxy1.y << endl;

						//�����Ϊ�̵ƣ���Ҫͣ��
						if (trafficLight.status != ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Green) {
							CrossstopFlag = true;
						}

						if (!intersectionVertices1.empty()) {
							//file << "intersectionVetices�ǿ��ҳߴ�Ϊ" << intersectionVertices1.size() << endl;
							//���·�����г������ˣ���Ҫͣ��
							for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
								if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Pedestrian || pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Car) {
									SSD::SimPoint3D ObstacleXY(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, 0.);
									SSD::SimPoint3D ObstacleST = ConvertToST(Axy2, Bxy1, ObstacleXY);
									file << "�е��ϰ���st���꣺" << ObstacleST.x << ", " << ObstacleST.y << endl;
									if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Pedestrian)
										file << "�е��ϰ�����xy���꣺" << pObstacle->obstacle[i].posX << ", " << pObstacle->obstacle[i].posY << endl;
									else file << "�е��ϰ��ﳵxy���꣺" << pObstacle->obstacle[i].posX << ", " << pObstacle->obstacle[i].posY << endl;
									if (ObstacleST.x >= intersectionVertices1[0].x && ObstacleST.x <= (intersectionVertices1[3].x + 10.0f)
										&& ObstacleST.y >= intersectionVertices1[0].y && ObstacleST.y <= intersectionVertices1[3].y) {
										file << "�е�·�����ϰ�� " << std::endl;
										CrossstopFlag = true;
									}
									else if (ObstacleST.x >= intersectionVertices1[4].x && ObstacleST.x <= intersectionVertices1[7].x
										&& ObstacleST.y >= intersectionVertices1[4].y && ObstacleST.y <= intersectionVertices1[7].y) {
										file << "·�����ϰ�� " << std::endl;
										CrossstopFlag = true;
									}
								}
							}
						}


					}

					if (CrossstopFlag) {
						double targetDistence = 4.8f;
						double currentDistence = stoplineS - mainVehicleS;
						double stopTime = (stoplineS - mainVehicleS) / (mainVehicleSpeed);

						pControl->throttle = 0.;
						if (stopTime != 0)
							pControl->brake = 0.8 / stopTime;
						else pControl->brake = 1.;
						if (targetDistence > currentDistence)
						{
							pControl->brake = 1;
						}
						if (runningCase == 31)
						{
							pControl->throttle = 0.;
							pControl->brake = 0.01;
						}

						preThrottle = pControl->throttle;
						SimOneAPI::SetDrive(MainVehicleId, pControl.get());
						SimOneAPI::NextFrame(frame);
						continue;
					}
					else {
						//����Ŀ���ٶ���ʻ
						double mainVehicleSpeed_kmh = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ) * 3.6;
						//******
						file << "////////////////////////////////////////////////////////////////" << endl;
						file << "��ǰ�ĺ��̵�״̬�ǣ�" << findlightflag << endl;
						//if (pGps->accelX > 0 && pGps->posX > -56.1 && mainVehicleS <= stoplineS) file << "�����涨���٣�" << endl;

						//file << "crossstate::" << crossState << endl;
						file << "��ǰ����crossState=1����crossFlag=0ʱ�Σ��ٶ�Ϊ��" << mainVehicleSpeed_kmh << "km/h" << endl;
						file << "////////////////////////////////////////////////////////////////" << endl;
						float TargetSpeedInCross = 0.8 * TargetSpeed;
						float speedKi = 0.00001;
						float speedKp = 0.60;
						float speedErrors = TargetSpeedInCross - mainVehicleSpeed_kmh;
						float Imax = 500.;
						float Imin = -500.;
						float throttlePoint = 0;
						speedSum += speedErrors;
						if (speedSum > Imax)speedSum = Imax;
						if (speedSum < Imin)speedSum = Imin;

						throttlePoint += speedSum * speedKi;
						throttlePoint += speedErrors * speedKp;
						// file << "speedi:" << speedSum * speedKi << std::endl;
						// file << "speedp:" << speedErrors * speedKp << std::endl;

						if (throttlePoint > 1)throttlePoint = 1;
						if (throttlePoint < -1)throttlePoint = -1;

						if (throttlePoint > 0) {
							pControl->throttle = throttlePoint;
							pControl->brake = 0.;
						}
						else {
							pControl->throttle = 0.;
							pControl->brake = -throttlePoint;
						}

						if (slowdown31)//31��ֹͣ��ǰ����
						{
							pControl->throttle = 0.;
							pControl->brake = 0.01f;
						}

						preThrottle = pControl->throttle;
						SimOneAPI::SetDrive(MainVehicleId, pControl.get());
						SimOneAPI::NextFrame(frame);
						continue;
					}
				}
				else {
					bool slowdown = false;
					float slowDownDistence = 12.0;
					/* //ԭ�����
					for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {

						if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Pedestrian || pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Car) {
							SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
							double ObstacleDistence = UtilMath::planarDistance(obstaclePos, mainVehiclePos);//�ϰ������������ֱ�߾���
							if ((ObstacleDistence < slowDownDistence))  slowdown = true;
						}
					}*/

					/* //һ��·�ڵĴ���
					for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
						if (crossingRoad.size() == 4) {
							if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Pedestrian || pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Car) {
								if (pObstacle->obstacle[i].posX >= crossingRoad[0] && pObstacle->obstacle[i].posX <= crossingRoad[1]
									&& pObstacle->obstacle[i].posY >= crossingRoad[2] && pObstacle->obstacle[i].posY <= crossingRoad[3]) {
									file << "��ǰ������·�ڣ�ͬʱ����⵽·�����ϰ���" << endl;
									slowdown = true;
								}
							}
						}
						else file << "ERROR: crossingRoad.size()������4��С������Խ��" << endl;
					}*/
					//**TIPS! ���Ҫ�����ҵĴ��룬��ȡ��ע������for���

					for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
						if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Pedestrian || pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Car) {
							SSD::SimPoint3D ObstacleXY(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, 0.);
							SSD::SimPoint3D ObstacleST = ConvertToST(intersectionVertices1[8], intersectionVertices1[9], ObstacleXY);
							file << "�е��ϰ���st���꣺" << ObstacleST.x << ", " << ObstacleST.y << endl;
							if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Pedestrian)
								file << "�е��ϰ�����xy���꣺" << pObstacle->obstacle[i].posX << ", " << pObstacle->obstacle[i].posY << endl;
							else file << "�е��ϰ��ﳵxy���꣺" << pObstacle->obstacle[i].posX << ", " << pObstacle->obstacle[i].posY << endl;
							if (ObstacleST.x >= intersectionVertices1[0].x && ObstacleST.x <= (intersectionVertices1[3].x + 15.0f)
								&& ObstacleST.y >= intersectionVertices1[0].y && ObstacleST.y <= intersectionVertices1[3].y) {
								file << "�е�·�����ϰ�� " << std::endl;
								slowdown = true;
							}
							else if (ObstacleST.x >= intersectionVertices1[4].x && ObstacleST.x <= intersectionVertices1[7].x
								&& ObstacleST.y >= intersectionVertices1[4].y && ObstacleST.y <= intersectionVertices1[7].y) {
								file << "·�����ϰ�� " << std::endl;
								slowdown = true;
							}
						}
					}


					if (slowdown) {
						pControl->throttle = 0.;
						pControl->brake = 0.5;
						preThrottle = pControl->throttle;
						if (runningCase == 31) pControl->brake = 0.01; //31�����ϰ������
						SimOneAPI::SetDrive(MainVehicleId, pControl.get());
						SimOneAPI::NextFrame(frame);
						continue;
					}
					else {
						//����Ŀ���ٶ���ʻ
						double mainVehicleSpeed_kmh = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ) * 3.6;
						float TargetSpeedInCross = 0.9 * TargetSpeed;
						float speedKi = 0.0001;
						float speedKp = 0.01;
						float speedErrors = TargetSpeedInCross - mainVehicleSpeed_kmh;
						float Imax = 500.;
						float Imin = -500.;
						float throttlePoint = 0;
						speedSum += speedErrors;
						if (speedSum > Imax)speedSum = Imax;
						if (speedSum < Imin)speedSum = Imin;

						throttlePoint += speedSum * speedKi;
						throttlePoint += speedErrors * speedKp;
						// file << "speedi:" << speedSum * speedKi << std::endl;
						// file << "speedp:" << speedErrors * speedKp << std::endl;

						if (throttlePoint > 1)throttlePoint = 1;
						if (throttlePoint < -1)throttlePoint = -1;

						if (throttlePoint > 0) {
							pControl->throttle = throttlePoint;
							pControl->brake = 0.;
						}
						else {
							pControl->throttle = 0.;
							pControl->brake = -throttlePoint;
						}

						if (runningCase == 31)//31��·�����棬δ���ϰ������
						{
							pControl->throttle = 0.;
							pControl->brake = 0.05;
						}

						preThrottle = pControl->throttle;
						SimOneAPI::SetDrive(MainVehicleId, pControl.get());
						SimOneAPI::NextFrame(frame);
						continue;

					}
				}
				SimOneAPI::NextFrame(frame);
				continue;
			}
			//����Ϊ���̵ƣ���������ʻ
			//�������к��
			SSD::SimVector<HDMapStandalone::MObject>mainVehicleLaneCrossWalkList;
			SimOneAPI::GetSpecifiedLaneCrosswalkList(mainVehicleLaneId, mainVehicleLaneCrossWalkList);

			//****************************************
			static int crossStateF;//����ͨ�����źŵ�ʮ��·�ڵ�״̬��
			static SSD::SimString premainVehicleLaneIdF;
			//size_t successorLaneSize = 0;//�����ж�������ڻ���ʮ��·��

			// ���źŵƲ���
			if (!findlightflag && !mainVehicleLaneCrossWalkList.empty()) {
				double margin2 = 5;
				std::cout << "�������źŵ�ģʽ(�к��̵Ƶ�·�ڴ�Ҳ���ܴ����� " << mainVehicleLaneCrossWalkList.size() << std::endl;//*******************************************************************************88

				SSD::SimVector<HDMapStandalone::MObject> stoplineList;
				SimOneAPI::GetSpecifiedLaneStoplineList(mainVehicleLaneId, stoplineList);
				SimOneAPI::GetLaneST(mainVehicleLaneId, stoplineList[0].pt, stoplineS, stoplineT);
				//if (runningCase > 40)stoplineS += margin2;//���41����Ҫ��һ��Ҫ��ֹͣ��ͣ������41�⳵�ٽϸ�ɲ��ס�����Կ��ǲ�ע��
				stoplineS = stoplineS + margin;

				/*
				//��ȡ��ǰ������ǰ����·��Ϣ
				HDMapStandalone::MLaneLink laneLink;
				SimOneAPI::GetLaneLink(mainVehicleLaneId, laneLink); //��ȡ��ǰ���������ӹ�ϵ
				auto& successorLaneNameList = laneLink.successorLaneNameList;//��ȡ�������ܵĳ��� ID����һ����·��


				for (const auto& laneId : successorLaneNameList) {
					file << "Successor Lane ID: " << laneId.GetString() << std::endl;
					successorLaneSize++;
				}
				*/

				//SSD::SimPoint3D stoplinept;
				/*
				if (runningCase < 41) {
					//����4���������ڼ�¼ʮ��·�ڵķ�Χ��s�Գ�����ǰΪ��,t�Գ����Ҳ�Ϊ����
					double xmin = std::numeric_limits<double>::max();
					double ymin = std::numeric_limits<double>::max();
					double xmax = -std::numeric_limits<double>::max();
					double ymax = -std::numeric_limits<double>::max();
					double margin = 1.0;//ֹͣ��ԣ��
					//����������
					for (auto& item : mainVehicleLaneCrossWalkList) {
						for (auto& item1 : item.boundaryKnots) {
							if (item1.x < xmin)xmin = item1.x;
							if (item1.x > xmax)xmax = item1.x;
							if (item1.y < ymin)ymin = item1.y;
							if (item1.y > ymax)ymax = item1.y;
						}
					}
					stoplinept = SSD::SimPoint3D(0.1, 0.1, 0);
					if (mainVehiclePos.x < xmin)  stoplinept = SSD::SimPoint3D(xmin - margin, (ymin + ymax) * 0.5, 0);
					if (mainVehiclePos.x > xmax) stoplinept = SSD::SimPoint3D(xmax + margin, (ymin + ymax) * 0.5, 0);
					if (mainVehiclePos.y < ymin) stoplinept = SSD::SimPoint3D((xmin + xmax)*0.5, ymin - margin, 0);
					if (mainVehiclePos.y > ymax) stoplinept = SSD::SimPoint3D((xmin + xmax)*0.5, ymax + margin, 0);
				}
				else {//!!!!!!!!!!�ȴջ����ã������ٸģ����Ա�ɼ�ס����

					SSD::SimPoint3D Axy1(mainVehicleLaneCrossWalkList[0].boundaryKnots[0].x, mainVehicleLaneCrossWalkList[0].boundaryKnots[0].y, 0.);
					SSD::SimPoint3D Bxy1(mainVehicleLaneCrossWalkList[0].boundaryKnots[1].x, mainVehicleLaneCrossWalkList[0].boundaryKnots[1].y, 0.);
					SSD::SimPoint3D Dxy1(mainVehicleLaneCrossWalkList[0].boundaryKnots[3].x, mainVehicleLaneCrossWalkList[0].boundaryKnots[3].y, 0.);
					SSD::SimPoint3D stoplineST(0., -1.1, 0.);
					stoplinept = ConvertToXY(Axy1, Bxy1, stoplineST);
				}
				*/
				file << "�޺��̵�ֹͣ�ߵ�λ��	x:" << stoplineList[0].pt.x << "y:" << stoplineList[0].pt.y << "z:" << stoplineList[0].pt.z << std::endl;
				//SimOneAPI::GetLaneST(mainVehicleLaneId, stoplinept, stoplineS, stoplineT);
				file << "�޺��̵�ֹͣ���ڳ����е�λ��	s:" << stoplineS << "t:" << stoplineT << std::endl;
				if (stoplineS - mainVehicleS <= 90 && stoplineS >= mainVehicleS) {
					//if (runningCase != 41)
					crossStateF = 1;
					premainVehicleLaneIdF = mainVehicleLaneId;
				}
			}//****************************************
			else {
				std::cout << "��ǰ���źŵ�·�ڲ��������к����" << std::endl;
			}

			file << "��ǰlane��Ӧ�����к��" << endl;
			for (const auto& item : mainVehicleLaneCrossWalkList) {
				// file << "���к������x:" << item.pt.x << " y:" << item.pt.y << endl;
				// file << "���к������:" << item.type.GetString() << endl;
				// file << "boundaryKnots" << endl;;
				for (const auto& item1 : item.boundaryKnots) {
					file << "x:" << item1.x << " y:" << item1.y << endl;
				}

			}
			file << "car pos x:" << mainVehiclePos.x << ", " << mainVehiclePos.y << endl;
			file << "crossStateF:" << crossStateF << endl;
			if (crossStateF) {
				if (premainVehicleLaneIdF != mainVehicleLaneId) {
					crossStateF++;
					premainVehicleLaneIdF = mainVehicleLaneId;
				}
				if (crossStateF == 3) {
					intersectionVertices.clear();
					crossingRoadCreateFlagF = false;
					crossStateF = 0;
				}
				if (crossStateF == 1) {
					bool CrossstopFlagF = false;//�����ж��Ƿ�Ҫ�ڰ�����ǰͣ��
					if (1) {
						//if (successorLaneSize != 0) {
						SSD::SimPoint3D Axy(mainVehicleLaneCrossWalkList[0].boundaryKnots[0].x, mainVehicleLaneCrossWalkList[0].boundaryKnots[0].y, 0.);
						SSD::SimPoint3D Bxy(mainVehicleLaneCrossWalkList[0].boundaryKnots[3].x, mainVehicleLaneCrossWalkList[0].boundaryKnots[3].y, 0.);
						SSD::SimPoint3D Dxy(mainVehicleLaneCrossWalkList[0].boundaryKnots[1].x, mainVehicleLaneCrossWalkList[0].boundaryKnots[1].y, 0.);
						double shortDistance = std::sqrt(std::pow(Bxy.x - Axy.x, 2) + std::pow(Axy.y - Bxy.y, 2));
						double longDistance = std::sqrt(std::pow(Axy.x - Dxy.x, 2) + std::pow(Axy.y - Dxy.y, 2));
						if (shortDistance > longDistance)
						{
							std::swap(shortDistance, longDistance);
							Bxy.x = mainVehicleLaneCrossWalkList[0].boundaryKnots[1].x;
							Bxy.y = mainVehicleLaneCrossWalkList[0].boundaryKnots[1].y;
							Dxy.x = mainVehicleLaneCrossWalkList[0].boundaryKnots[3].x;
							Dxy.y = mainVehicleLaneCrossWalkList[0].boundaryKnots[3].y;
						}
						file << "sdistance" << shortDistance << "longdistance: " << longDistance << endl;
						//!!! SSD::SimVector<SSD::SimPoint3D> intersectionVertices;
						// //GetCrossroad2(mainVehicleLaneCrossWalkList, intersectionVertices, mainVehiclePos, successorLaneSize);
						//GetCrossroad(shortDistance, longDistance, intersectionVertices);
						if (!crossingRoadCreateFlagF)
						{
							crossingRoadCreateFlagF = true;
							intersectionVertices.clear();
							GetCrossroad(shortDistance, longDistance, intersectionVertices);
						}
						//!!! ���·�������ˣ��Ͱ������ifע���ˣ������������У��Լ��������!!!ע����

						//����
						SSD::SimPoint3D Axy1 = ConvertToXY(Axy, Bxy, intersectionVertices[0]);
						SSD::SimPoint3D Fxy = ConvertToXY(Axy, Bxy, intersectionVertices[3]);
						SSD::SimPoint3D Gxy = ConvertToXY(Axy, Bxy, intersectionVertices[4]);
						SSD::SimPoint3D Jxy = ConvertToXY(Axy, Bxy, intersectionVertices[7]);
						file << "·�ڷ�ΧAxy:" << Axy1.x << ", " << Axy1.y << endl;
						file << "·�ڷ�ΧFxy:" << Fxy.x << ", " << Fxy.y << endl;
						file << "·�ڷ�ΧGxy:" << Gxy.x << ", " << Gxy.y << endl;
						file << "·�ڷ�ΧJxy:" << Jxy.x << ", " << Jxy.y << endl;

						if (!intersectionVertices.empty()) {
							//file << "intersectionVetices�ǿ��ҳߴ�Ϊ" << intersectionVertices.size() << endl;
							//���·�����г������ˣ���Ҫͣ��
							for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
								if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Pedestrian || pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Car) {
									SSD::SimPoint3D ObstacleXY(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, 0.);
									SSD::SimPoint3D ObstacleST = ConvertToST(Axy, Bxy, ObstacleXY);
									file << "�ϰ���st���꣺" << ObstacleST.x << ", " << ObstacleST.y << endl;
									if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Pedestrian)
										file << "�ϰ�����xy���꣺" << pObstacle->obstacle[i].posX << ", " << pObstacle->obstacle[i].posY << endl;
									else file << "�ϰ��ﳵxy���꣺" << pObstacle->obstacle[i].posX << ", " << pObstacle->obstacle[i].posY << endl;
									if (ObstacleST.x >= intersectionVertices[0].x && ObstacleST.x <= intersectionVertices[3].x
										&& ObstacleST.y >= intersectionVertices[0].y && ObstacleST.y <= intersectionVertices[3].y) {
										file << "·�����ϰ�� " << std::endl;
										CrossstopFlagF = true;
									}
									else if (ObstacleST.x >= intersectionVertices[4].x && ObstacleST.x <= intersectionVertices[7].x
										&& ObstacleST.y >= intersectionVertices[4].y && ObstacleST.y <= intersectionVertices[7].y) {
										file << "·�����ϰ�� " << std::endl;
										CrossstopFlagF = true;
									}
									//ʵ�ڲ��У�elseif���жϷ������ifע�͵�pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Pedestrian
								}
							}
						}
					}
					if (CrossstopFlagF) {
						double targetDistence = 4.8f;
						double currentDistence = stoplineS - mainVehicleS;
						double stopTime = (stoplineS - mainVehicleS) / (mainVehicleSpeed);

						pControl->throttle = 0.;
						pControl->brake = 0.8 / stopTime;
						if (targetDistence > currentDistence)
						{
							pControl->brake = 1;
						}

						preThrottle = pControl->throttle;
						if (runningCase == 18) {
							pControl->throttle = 0.0f;//���18����ٵ�����
							pControl->brake = 0.0f;
						}
						SimOneAPI::SetDrive(MainVehicleId, pControl.get());
						SimOneAPI::NextFrame(frame);
						continue;
					}
					else {
						//����Ŀ���ٶ���ʻ
						double mainVehicleSpeed_kmh = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ) * 3.6;
						file << "////////////////////////////////////////////////////////////////" << endl;
						//if (pGps->accelX > 0 && pGps->posX > -56.1 && mainVehicleS <= stoplineS) file << "�����涨���٣�" << endl;
						file << "��ǰ����λ�� x:" << pGps->posX << endl;
						if (pGps->posX > -94 && pGps->posX <= -19.2) file << "�޺��̵Ƶ�ǰ���ļ��ٶ�Ϊ��" << pGps->accelX << endl;
						file << "��ǰ����crossStateF=1����crossFlagF=0ʱ�Σ��ٶ�Ϊ��" << mainVehicleSpeed_kmh << "km/h" << endl;
						file << "////////////////////////////////////////////////////////////////" << endl;

						float TargetSpeedInCross = 0.9 * TargetSpeed;
						float speedKi = 0.0003;	//0.0005
						float speedKp = 0.02;	//0.08
						float speedErrors = TargetSpeedInCross - mainVehicleSpeed_kmh;
						float Imax = 500.;
						float Imin = -500.;
						float throttlePoint = 0;
						speedSum += speedErrors;
						if (speedSum > Imax)speedSum = Imax;
						if (speedSum < Imin)speedSum = Imin;

						throttlePoint += speedSum * speedKi;
						throttlePoint += speedErrors * speedKp;
						// file << "speedi:" << speedSum * speedKi << std::endl;
						// file << "speedp:" << speedErrors * speedKp << std::endl;

						if (throttlePoint > 1)throttlePoint = 1;
						if (throttlePoint < -1)throttlePoint = -1;

						if (throttlePoint > 0) {
							pControl->throttle = throttlePoint;
							pControl->brake = 0.;
						}
						else {
							pControl->throttle = 0.;
							pControl->brake = -throttlePoint;
						}
						preThrottle = pControl->throttle;
						if (runningCase == 18) pControl->throttle = 0.0f;//���18����ٵ�����
						SimOneAPI::SetDrive(MainVehicleId, pControl.get());
						SimOneAPI::NextFrame(frame);
						continue;

					}
				}
				else {
					bool slowdown = false;
					float slowDownDistence = 12.0;
					for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {

						if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Pedestrian || pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Car) {
							SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
							double ObstacleDistence = UtilMath::planarDistance(obstaclePos, mainVehiclePos);//�ϰ������������ֱ�߾���
							//double obstacleSpeed = UtilMath::calculateSpeed(pObstacle->obstacle[i].velX, pObstacle->obstacle[i].velY, pObstacle->obstacle[i].velZ) * 3.6;
							//file << "�ϰ���ʱ��" << obstacleSpeed << endl;
							//if (obstacleSpeed > 1) file << "�ϰ��ﳵλ�ã�" << obstaclePos.x << ", " << obstaclePos.y << endl;
							double perpAcc = computePerpendicularAcceleration(pObstacle->obstacle[i], *pGps) * 3.6;
							// file << "�ϰ��ﳵ��ֱʱ��" << perpAcc << "km/h" << endl;
							if ((abs(perpAcc) > 15) && (ObstacleDistence < 2.0 * slowDownDistence)) slowdown = true;
							else if ((abs(perpAcc) > 5) && (ObstacleDistence < slowDownDistence)) slowdown = true;

							//if ((ObstacleDistence < 2.0 * slowDownDistence) && runningCase == 18)  slowdown = true;//********
							/*
							if ((ObstacleDistence < 2.0 * slowDownDistence) && obstacleSpeed > 30)  slowdown = true;
							else if ((ObstacleDistence < slowDownDistence) && obstacleSpeed > 15) slowdown = true;*/
							// file << "slowdown Flag:" << slowdown << endl;
						}
					}
					if (slowdown) {
						pControl->throttle = 0.;
						pControl->brake = 0.5;
						preThrottle = pControl->throttle;
						SimOneAPI::SetDrive(MainVehicleId, pControl.get());
						SimOneAPI::NextFrame(frame);
						continue;
					}
					else {
						//����Ŀ���ٶ���ʻ
						double mainVehicleSpeed_kmh = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ) * 3.6;
						float TargetSpeedInCross = 0.9 * TargetSpeed;
						float speedKi = 0.0001;
						float speedKp = 0.01;
						float speedErrors = TargetSpeedInCross - mainVehicleSpeed_kmh;
						float Imax = 500.;
						float Imin = -500.;
						float throttlePoint = 0;
						speedSum += speedErrors;
						if (speedSum > Imax)speedSum = Imax;
						if (speedSum < Imin)speedSum = Imin;

						throttlePoint += speedSum * speedKi;
						throttlePoint += speedErrors * speedKp;
						// file << "speedi:" << speedSum * speedKi << std::endl;
						// file << "speedp:" << speedErrors * speedKp << std::endl;

						if (throttlePoint > 1)throttlePoint = 1;
						if (throttlePoint < -1)throttlePoint = -1;

						if (throttlePoint > 0) {
							pControl->throttle = throttlePoint;
							pControl->brake = 0.;
						}
						else {
							pControl->throttle = 0.;
							pControl->brake = -throttlePoint;
						}
						preThrottle = pControl->throttle;
						SimOneAPI::SetDrive(MainVehicleId, pControl.get());
						SimOneAPI::NextFrame(frame);
						continue;

					}
				}
				SimOneAPI::NextFrame(frame);
				continue;
			}
			//·�ڱ��ϣ�Ϊ�˽������������ͼ��ƴ�ӵ�����
			//if (!mainVehicleLaneCrossWalkList.empty() && runningCase > 40) {
			if (!mainVehicleLaneCrossWalkList.empty()) {
				bool slowdown = false;
				float slowDownDistence = 12.0;

				for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {

					if (pObstacle->obstacle[i].type == ESimOne_Obstacle_Type_Car) {
						SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
						double ObstacleDistence = UtilMath::planarDistance(obstaclePos, mainVehiclePos);//�ϰ������������ֱ�߾���
						double obstacleSpeed = UtilMath::calculateSpeed(pObstacle->obstacle[i].velX, pObstacle->obstacle[i].velY, pObstacle->obstacle[i].velZ) * 3.6;
						//file << "�ϰ���ʱ��" << obstacleSpeed << endl;
						double perpAcc = computePerpendicularAcceleration(pObstacle->obstacle[i], *pGps);
						// file << "�ϰ��ﳵ��ֱ���ٶ�" << perpAcc << "km/h" << endl;
						if ((perpAcc > 15) && (ObstacleDistence < 2.0 * slowDownDistence)) slowdown = true;
						else if ((perpAcc > 5) && (ObstacleDistence < slowDownDistence)) slowdown = true;
					}
				}
				if (slowdown) {
					pControl->throttle = 0.;
					pControl->brake = 0.5;
					preThrottle = pControl->throttle;
					SimOneAPI::SetDrive(MainVehicleId, pControl.get());
					SimOneAPI::NextFrame(frame);
					continue;
				}
			}
			// file << "AEBflag:" << AEBflag << std::endl;
			if (AEBflag)
			{
				inAEBState = true;
			}
			else
			{
				inAEBState = false;
			}

			if (inAEBState)
			{
				pControl->throttle = 0.;
				pControl->brake = 1 / minCollisionTime;
				// file << "minCollisionTime" << minCollisionTime << endl;
				if (stopFlag)
				{
					pControl->brake = 1;
				}

				preThrottle = pControl->throttle;
				SimOneAPI::SetDrive(MainVehicleId, pControl.get());
				SimOneAPI::NextFrame(frame);
				continue;
			}

			double mainVehicleSpeed_kmh = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ) * 3.6;
			file << "followingFlag:" << followingFlag << std::endl;
			// file << "mainVehicleSpeed_kmh:" << mainVehicleSpeed_kmh << std::endl;
			//����
			if (followingFlag) {
				float TargetDistence = 35.;//Ŀ���������
				float followingKp = 0.1;//0.01//PID��Ϊ0.5��PD��0.1
				float followingKd = 10;//10
				float throttlePoint = 0;

				if (runningCase == 25)
				{
					followingKp = 0.5;
					followingKd = 40;
				}
				float distenceErrors = minCarDistence / TargetDistence - 1.0f;
				float predistenceErrors = preFollowingDistence - 1.0f;

				throttlePoint = followingKp * distenceErrors + followingKd * (distenceErrors - predistenceErrors);
				//throttlePoint += followingKi * distanceSum;
				//throttlePoint += followingKd * (distenceErrors - predistenceErrors);

				//file << "P:" << followingKp * distenceErrors << endl;
				//file << "D:" << followingKd * (distenceErrors - predistenceErrors) << endl;

				if (throttlePoint > 1)throttlePoint = 1;
				if (throttlePoint < -1)throttlePoint = -1;

				if (throttlePoint > 0) {
					pControl->throttle = throttlePoint;
					pControl->brake = 0.;

				}
				else {
					pControl->throttle = 0.;
					pControl->brake = -throttlePoint;
				}
				preFollowingDistence = minCarDistence / TargetDistence;
			}
			else {
				if (!end)
				{
					if (runningCase == 41)
					{
						float speedKp = 0.55;
						float speedKi = 0.00001;
						float speedKd = 1.;
						double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);
						double mainVehicleSpeed_kmh = mainVehicleSpeed * 3.6;
						//float speedErrors = 1.0f - mainVehicleSpeed/ TargetSpeed;//mainVehicleSpeed_kmh
						float prespeedErrors = preTargetSpeed - preSpeed41 ;//
						float speedErrors = TargetSpeed - mainVehicleSpeed_kmh ;
						float Imax = 20000.;
						float Imin = -20000.;
						float throttlePoint = 0;
						speedSum += speedErrors;
						if (speedSum > Imax)speedSum = Imax;
						if (speedSum < Imin)speedSum = Imin;
						throttlePoint += speedSum * speedKi;
						throttlePoint += speedErrors * speedKp;
						//throttlePoint += - speedKd * (speedErrors - prespeedErrors);//

						// file << "speedi:" << speedSum * speedKi << std::endl;
						// file << "speedp:" << speedErrors * speedKp << std::endl;
						// file << "k:" << k << endl;
						// file << "TargetSpeed" << TargetSpeed << endl;
						// file << "mainVehicleSpeed_kmh:" << mainVehicleSpeed_kmh << endl;

						file << "P:" << speedErrors * speedKp << std::endl;
						file << "I:" << speedSum * speedKi << std::endl;
						file << "D:" << speedKd * (speedErrors - prespeedErrors) << endl;

						if (TargetSpeed == 28 || TargetSpeed == 30)
						{
							file << "С���ٶ�: " << mainVehicleSpeed<<" m/s" << std::endl;
							file << "С��λ�ã�"<< mainVehiclePos.x << ", " << mainVehiclePos.y << endl;
							if (mainVehicleSpeed > 8.33)
							{
								file << "���٣�" << endl;
							}
						}
						if (throttlePoint > 1)throttlePoint = 1;
						if (throttlePoint < -1)throttlePoint = -1;

						if (throttlePoint > 0) {
							pControl->throttle = throttlePoint;
							pControl->brake = 0.;

						}
						else {
							pControl->throttle = 0.;
							pControl->brake = -throttlePoint;
						}
						preSpeed41 = mainVehicleSpeed_kmh;//
						preTargetSpeed = TargetSpeed;
						file << "pControl->throttle:" << pControl->throttle << endl;
						file << "pControl->brake:" << pControl->brake << endl;
					}
					else {
						//ǰ��û��������Ŀ���ٶ���ʻ
						float speedKi = 0.0001;
						float speedKp = 0.01;
						float speedErrors = TargetSpeed - mainVehicleSpeed_kmh;
						float Imax = 500.;
						float Imin = -500.;
						float throttlePoint = 0;
						speedSum += speedErrors;

						if (speedSum > Imax)speedSum = Imax;
						if (speedSum < Imin)speedSum = Imin;

						throttlePoint += speedSum * speedKi;
						throttlePoint += speedErrors * speedKp;
						// file << "speedi:" << speedSum * speedKi << std::endl;
						// file << "speedp:" << speedErrors * speedKp << std::endl;

						if (throttlePoint > 1)throttlePoint = 1;
						if (throttlePoint < -1)throttlePoint = -1;

						if (throttlePoint > 0) {
							pControl->throttle = throttlePoint;
							pControl->brake = 0.;

						}
						else {
							pControl->throttle = 0.;
							pControl->brake = -throttlePoint;
						}
					}

				}

			}
			//file << "pControl->throttle:" << pControl->throttle << std::endl;

			preThrottle = pControl->throttle;
			SimOneAPI::SetDrive(MainVehicleId, pControl.get());

		}
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}
		SimOneAPI::SetSignalLights(MainVehicleId, &signal);
		//������һ֡����
		SimOneAPI::NextFrame(frame);
	}
	// �ر��ļ�
	file.close();
	return 0;
}