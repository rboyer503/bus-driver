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
	int angle;

	LaneInfo() : xTarget(0), laneId(0), votes(0), angle(0)
	{}

	void deactivate() { votes = 0; }
	bool isActive() const { return (votes > 0); }
};


class LaneTransform
{
private:
	static const std::string c_indexFileName;
	static const std::string c_voteArrayFileName;
	static const std::string c_laneFileName;
	static constexpr int c_voteArrayWidth = 230;
	static constexpr int c_voteArrayHeight = 30;
	static constexpr int c_packedVoteBufferSize = 2142; // (ROI_HEIGHT * c_laneVariants) + 1 + extra room for null terminator values for each grid position with > 0 votes.
	static constexpr int c_numStartAngles = 41;
	static constexpr int c_numSlopeAdjust = 1;
	static constexpr int c_laneVariants = (c_numStartAngles * c_numSlopeAdjust);

	enum eLaneSearchState
	{
		LSS_INIT,
		LSS_WAIT_FOR_LOW_THRES,
		LSS_WAIT_FOR_DECAY
	};

	// Map pixels representing closer edges (high y value) to larger weight to help counteract "overfitting" effect in which highly curved lines happen to include several votes.
	int m_weightArray[c_voteArrayHeight];

	// Precalculate slopes for each lane variant.
	float m_slopeArray[c_laneVariants];

	// m_voteIndex provides offset into m_packedVoteArray for corresponding pixel.
	// m_packedVoteArray provides the list of all associated lane variants terminated with a 0.
	int m_voteIndex[c_voteArrayHeight][c_voteArrayWidth];
	short m_packedVoteArray[c_packedVoteBufferSize];

	short m_lanes[c_laneVariants][c_voteArrayHeight];

public:
	LaneTransform();

	bool Load();
	bool LaneSearch(const std::vector<cv::Vec3i> & edges, const eLane lane, const cv::Vec2i searchRange, LaneInfo & laneInfo, const bool debug = false) const;
	void RenderLane(cv::Mat & frame, const LaneInfo & laneInfo) const;
	int GetLaneAngle(const int laneId) const;
	float GetLaneSlope(const int laneId) const { return m_slopeArray[laneId]; }
};

#endif /* LANETRANSFORM_H_ */
