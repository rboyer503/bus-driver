/*
 * LaneTransform.cpp
 *
 *  Created on: Oct 18, 2019
 *      Author: rboyer
 */
#include <cmath>
#include <fstream>
#include <iostream>

#include "LaneTransform.h"

#define PI 3.14159265


using namespace std;
using namespace cv;


const std::string LaneTransform::c_indexFileName("index_file.txt");
const std::string LaneTransform::c_voteArrayFileName("vote_array_file.txt");
const std::string LaneTransform::c_laneFileName("lane_file.txt");


LaneTransform::LaneTransform()
{
	// TODO: Remove weight array?
	const int divisionSize = c_voteArrayHeight / 3;
	for (int y = 0; y < divisionSize; ++y)
	{
		m_weightArray[y] = 1;
		m_weightArray[y + divisionSize] = 2;
		m_weightArray[y + (divisionSize * 2)] = 3;
	}

	for (int laneId = 0; laneId < c_laneVariants; ++laneId)
		m_slopeArray[laneId] = tan( GetLaneAngle(laneId) * PI / 180.0 );
}

bool LaneTransform::Load()
{
	ifstream indexFile(c_indexFileName);
	if (indexFile.is_open())
	{
		for (int y = 0; y < c_voteArrayHeight; ++y)
			for (int x = 0; x < c_voteArrayWidth; ++x)
				indexFile >> m_voteIndex[y][x];
		indexFile.close();
	}
	else
	{
		cerr << "ERROR: Cannot open index file." << endl;
		return false;
	}

	ifstream voteArrayFile(c_voteArrayFileName);
	if (voteArrayFile.is_open())
	{
		for (int i = 0; i < c_packedVoteBufferSize; ++i)
			voteArrayFile >> m_packedVoteArray[i];
		voteArrayFile.close();
	}
	else
	{
		cerr << "ERROR: Cannot open vote array file." << endl;
		return false;
	}

	ifstream laneFile(c_laneFileName);
	if (laneFile.is_open())
	{
		for (int laneId = 0; laneId < c_laneVariants; ++laneId)
			for (int y = 0; y < c_voteArrayHeight; ++y)
				laneFile >> m_lanes[laneId][y];
		laneFile.close();
	}
	else
	{
		cerr << "ERROR: Cannot open lane file." << endl;
		return false;
	}

	return true;
}

bool LaneTransform::LaneSearch(const vector<Vec3i> & edges, const eLane lane, const cv::Vec2i searchRange, LaneInfo & laneInfo, const int maxAngleDeviation,
							   const int angleLimit, int laneVoteThreshold, const bool debug /* = false */) const
{
	const int cVoteArrayMedianX = c_voteArrayWidth / 2;
	bool limitAngleDeviation = false;

	if (laneInfo.isActive())
	{
		limitAngleDeviation = true;
		laneVoteThreshold >>= 1;
	}

	LaneInfo tempLaneInfo;
	eLaneSearchState lss = LSS_WAIT_FOR_LOW_THRES;
	bool done[2] = {false, false};

	int offset = (searchRange[0] + searchRange[1]) / 2;
	int range = abs(searchRange[0] - searchRange[1]);
	int jump = 1;
	int jumpDir = 1;
	int voteArrayOffset;
	for (int i = 0; i < range; i++)
	{
		voteArrayOffset = cVoteArrayMedianX - offset;

		short laneVoteTable[c_laneVariants + 1] = {};
		const short * pVote;
		int currMaxVotes = 0;
		int currBestLaneId = 0;

		for (const Vec3i & edge : edges)
		{
			int index;
			int x = edge[0] + voteArrayOffset - 1;
			if ( (x < 0) || ((x + 2) >= c_voteArrayWidth) )
				continue;

			for (int xCount = 0; xCount < 3; ++xCount, ++x)
			{
				index = m_voteIndex[edge[1]][x];
				if (index)
				{
					pVote = m_packedVoteArray + index;
					while (*pVote)
						laneVoteTable[*pVote++] += (edge[2] * m_weightArray[edge[1]]);
				}
			}
		}

		for (int i = 1; i <= c_laneVariants; ++i)
		{
			if (laneVoteTable[i] > currMaxVotes)
			{
				currMaxVotes = laneVoteTable[i];
				currBestLaneId = i - 1;
			}
		}

		if (debug)
			cout << "    Debug xTarget=" << (cVoteArrayMedianX - voteArrayOffset) << ", maxVotes=" << currMaxVotes << ", bestLaneId=" << currBestLaneId <<
					", angle=" << GetLaneAngle(currBestLaneId) << endl;

		if (currMaxVotes > tempLaneInfo.votes)
		{
			int tempAngle = GetLaneAngle(currBestLaneId);

			if ( ( (lane == LEFT_LANE) && (tempAngle >= -angleLimit) ) ||
				 ( (lane == RIGHT_LANE) && (tempAngle <= angleLimit) ) )
			{
				if ( !limitAngleDeviation ||
					 ( abs(tempAngle - laneInfo.angle) <= maxAngleDeviation ) )
				{
					tempLaneInfo.votes = currMaxVotes;
					tempLaneInfo.laneId = currBestLaneId;
					tempLaneInfo.xTarget = cVoteArrayMedianX - voteArrayOffset;
					tempLaneInfo.angle = tempAngle;
				}
			}
		}

		switch (lss)
		{
		case LSS_WAIT_FOR_LOW_THRES:
			if (currMaxVotes >= laneVoteThreshold)
				lss = LSS_WAIT_FOR_DECAY;
			break;

		case LSS_WAIT_FOR_DECAY:
			if ( currMaxVotes <= ((tempLaneInfo.votes << 1) / 3) )
			{
				if (done[0])
					done[1] = true;
				else
					done[0] = true;
			}
			else
				done[0] = false;
			break;
		}

		if (done[1])
			break;

		offset += (jump++ * jumpDir);
		jumpDir *= -1;
	}

	if (tempLaneInfo.votes < laneVoteThreshold)
	{
		if (debug)
			cout << "    Debug final: lane not found" << endl;

		if (laneInfo.isActive())
		{
			if ( (lane == LEFT_LANE) && (laneInfo.xTarget >= 120) )
				;
			else if ( (lane == RIGHT_LANE) && (laneInfo.xTarget <= 40) )
				;
			else
			{
				laneInfo.deactivate();
				return false;
			}
		}
		else
		{
			laneInfo.deactivate();
			return false;
		}
	}

	if (debug)
		cout << "    Debug final: xTarget=" << tempLaneInfo.xTarget << ", maxVotes=" << tempLaneInfo.votes << ", bestLaneId=" << tempLaneInfo.laneId <<
				", angle=" << tempLaneInfo.angle << endl;

	laneInfo = tempLaneInfo;
	return true;
}

void LaneTransform::RenderLane(Mat & frame, const LaneInfo & laneInfo) const
{
	const short * pLane = m_lanes[laneInfo.laneId];
	int offset = (c_voteArrayWidth / 2) - laneInfo.xTarget;

	for (int y = 0; y < c_voteArrayHeight; ++y)
	{
		int x = (*pLane++) - offset;
		if ( (x >= 0) && (x < frame.cols) )
			frame.at<uchar>(y, x, 0) = 255;
	}
}

int LaneTransform::GetLaneAngle(const int laneId) const
{
	return ( (laneId / c_numSlopeAdjust) - (c_numStartAngles >> 1) ) * 4;
}

int LaneTransform::GetLaneXPos(const LaneInfo & laneInfo, int yPos) const
{
	int offset = (c_voteArrayWidth / 2) - laneInfo.xTarget;
	return m_lanes[laneInfo.laneId][yPos] - offset;
}
