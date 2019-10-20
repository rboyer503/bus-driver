/*
 * LaneTransform.cpp
 *
 *  Created on: Oct 18, 2019
 *      Author: rboyer
 */
#include <fstream>
#include <iostream>

#include "LaneTransform.h"

using namespace std;
using namespace cv;


const std::string LaneTransform::c_indexFileName("index_file.txt");
const std::string LaneTransform::c_voteArrayFileName("vote_array_file.txt");
const std::string LaneTransform::c_laneFileName("lane_file.txt");
const cv::Vec2i LaneTransform::c_offsetRange[MAX_LANES] = { {85, 211}, {125, -1} };


LaneTransform::LaneTransform()
{
	for (int y = 0; y < ROI_HEIGHT; ++y)
		m_weightArray[y] = (y / 20) + 1;
}

bool LaneTransform::Load()
{
	ifstream indexFile(c_indexFileName);
	if (indexFile.is_open())
	{
		for (int y = 0; y < ROI_HEIGHT; ++y)
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
			for (int y = 0; y < ROI_HEIGHT; ++y)
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

bool LaneTransform::LaneSearch(vector<Vec2i> & edges, eLane lane, LaneInfo & laneInfo, bool debug /* = false */) const
{
	const int cVoteArrayMedianX = c_voteArrayWidth / 2;

	LaneInfo tempLaneInfo;
	int step = (lane == LEFT_LANE ? 1 : -1);
	//eLaneSearchState lss = LSS_INIT;
	//int lsLowThreshold;
	//bool lsDone = false;

	for (int offset = c_offsetRange[lane][0]; offset != c_offsetRange[lane][1]; offset += step)
	{
		short laneVoteTable[c_laneVariants] = {};
		const short * pVote;
		int currMaxVotes = 0;
		int currBestLaneId = 0;
		//bool done = false;

		for (Vec2i & edge : edges)
		{
			int index = m_voteIndex[edge[1]][edge[0] + offset];
			if (index)
			{
				pVote = m_packedVoteArray + index;
				while (*pVote)
					laneVoteTable[*pVote++] += m_weightArray[edge[1]];
			}

			index = m_voteIndex[edge[1]][edge[0] + offset - 1];
			if (index)
			{
				pVote = m_packedVoteArray + index;
				while (*pVote)
					laneVoteTable[*pVote++] += m_weightArray[edge[1]];
			}

			index = m_voteIndex[edge[1]][edge[0] + offset + 1];
			if (index)
			{
				pVote = m_packedVoteArray + index;
				while (*pVote)
					laneVoteTable[*pVote++] += m_weightArray[edge[1]];
			}
		}

		for (int i = 0; i < c_laneVariants; ++i)
		{
			if (laneVoteTable[i] > currMaxVotes)
			{
				currMaxVotes = laneVoteTable[i];
				currBestLaneId = i;
			}
		}

		currMaxVotes *= ( 1.0f - ( abs(GetSlopeAdjust(currBestLaneId)) / 30.0f ) );

		if (debug)
			cout << "    Debug xTarget=" << (cVoteArrayMedianX - offset) << ", maxVotes=" << currMaxVotes << ", bestLaneId=" << currBestLaneId << endl;

		if (currMaxVotes > tempLaneInfo.votes)
		{
			tempLaneInfo.votes = currMaxVotes;
			tempLaneInfo.laneId = currBestLaneId;
			tempLaneInfo.xTarget = cVoteArrayMedianX - offset;
		}

		/*
		switch (lss)
		{
		case LSS_INIT:
			lsLowThreshold = currMaxVotes * 4;
			if (lsLowThreshold < 20)
				lsLowThreshold = 20;
			lss = LSS_WAIT_FOR_LOW_THRES;
			break;

		case LSS_WAIT_FOR_LOW_THRES:
			if (currMaxVotes >= lsLowThreshold)
				lss = LSS_WAIT_FOR_DECAY;
			break;

		case LSS_WAIT_FOR_DECAY:
			if (currMaxVotes <= (tempLaneInfo.votes >> 1))
				done = true;
			break;
		}

		if (done)
			break;
		*/
	}

	//if (lss == LSS_WAIT_FOR_LOW_THRES)
	if (tempLaneInfo.votes < 10)
	{
		if (debug)
			cout << "    Debug final: lane not found" << endl;

		return false;
	}

	if (debug)
		cout << "    Debug final: xTarget=" << tempLaneInfo.xTarget << ", maxVotes=" << tempLaneInfo.votes << ", bestLaneId=" << tempLaneInfo.laneId << endl;

	laneInfo = tempLaneInfo;
	return true;
}

void LaneTransform::RenderLane(Mat & frame, LaneInfo & laneInfo) const
{
	const short * pLane = m_lanes[laneInfo.laneId];
	int offset = (c_voteArrayWidth / 2) - laneInfo.xTarget;

	for (int y = 0; y < ROI_HEIGHT; ++y)
	{
		int x = (*pLane++) - offset;
		if ( (x >= 0) && (x < frame.cols) )
			frame.at<uchar>(y, x, 0) = 192;
	}
}

int LaneTransform::GetLaneAngle(int laneId) const
{
	return ( (laneId / c_numSlopeAdjust) - 15 ) * 4;
}

int LaneTransform::GetSlopeAdjust(int laneId) const
{
	return ( (laneId % c_numSlopeAdjust) - 15 );
}
