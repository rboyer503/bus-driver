/*
 * LaneTransform.h
 *
 *  Created on: Oct 18, 2019
 *      Author: rboyer
 */

#ifndef LANETRANSFORM_H_
#define LANETRANSFORM_H_

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "Constants.h"


enum eLane
{
	LEFT_LANE,
	RIGHT_LANE,
	MAX_LANES
};

struct LaneInfo
{
	int xTarget;
	int laneId;
	int votes;

	LaneInfo() : xTarget(0), laneId(0), votes(0)
	{}
};


class LaneTransform
{
private:
	static const std::string c_indexFileName;
	static const std::string c_voteArrayFileName;
	static const std::string c_laneFileName;
	static constexpr int c_voteArrayWidth = 370;
	static constexpr int c_packedVoteBufferSize = 64000; // (ROI_HEIGHT * c_laneVariants) + 1 + extra room for null terminator values for each grid position with > 0 votes.
	static constexpr int c_numStartAngles = 31;
	static constexpr int c_numSlopeAdjust = 31;
	static constexpr int c_laneVariants = (c_numStartAngles * c_numSlopeAdjust);
	static const cv::Vec2i c_offsetRange[MAX_LANES];

	enum eLaneSearchState
	{
		LSS_INIT,
		LSS_WAIT_FOR_LOW_THRES,
		LSS_WAIT_FOR_DECAY
	};

	// Map pixels representing closer edges (high y value) to larger weight to help counteract "overfitting" effect in which highly curved lines happen to include several votes.
	int m_weightArray[ROI_HEIGHT];

	// m_voteIndex provides offset into m_packedVoteArray for corresponding pixel.
	// m_packedVoteArray provides the list of all associated lane variants terminated with a 0.
	int m_voteIndex[ROI_HEIGHT][c_voteArrayWidth];
	short m_packedVoteArray[c_packedVoteBufferSize];

	short m_lanes[c_laneVariants][ROI_HEIGHT];

public:
	LaneTransform();

	bool Load();
	bool LaneSearch(std::vector<cv::Vec2i> & edges, eLane lane, LaneInfo & laneInfo, bool debug = false) const;
	void RenderLane(cv::Mat & frame, LaneInfo & laneInfo) const;
	int GetLaneAngle(int laneId) const;
	int GetSlopeAdjust(int laneId) const;
};

#endif /* LANETRANSFORM_H_ */
