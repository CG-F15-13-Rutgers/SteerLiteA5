//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}
// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	int length = controlPoints.size();
	//DrawLib::drawLine(controlPoints[0].position, controlPoints[length - 1].position, curveColor, curveThickness);
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
	{
		return;
	}
	window = 1;
	unsigned int nextPoint;
	int max = controlPoints.size();
	Point from = controlPoints[0].position;
	Point to;
	for (int t = 0; t < controlPoints[max - 1].time; t += window)
	{
		if (findTimeInterval(nextPoint, t))
		{
			calculatePoint(to, t);
			DrawLib::drawLine(from, to, curveColor, curveThickness);
			from = to;
		}
	} 
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	int length = controlPoints.size();
	//bubble sort
	for (int i = 0; i < length; i++)
	{
		for (int j = 0; j < length - i - 1; j++)
		{
			if (controlPoints[j].time > controlPoints[j + 1].time)//ascending order
			{
				CurvePoint temp = controlPoints[j];
				controlPoints[j] = controlPoints[j + 1];
				controlPoints[j + 1] = temp;
			}
		}
	}
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Robustness
bool Curve::checkRobust()
{
	//hermitecurve needs at least 2 points
	if (type == 0)//hermiteCurve
	{
		if (controlPoints.size()<2)
		{
			return false;
		}
		return true;
	}
	//catmillcurve needs at least 4 points.. right?
	else if (type == 1)// catmullCurve
	{
		if (controlPoints.size() < 4)
		{
			return false;
		}
		return true;
	} 
	return false;// false if type is neither hermiteCurve nor catmullCurve
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	//The way I see it: we are supposed to find the control point that has a greater time value 
	//than the given time. Since the the list is sorted
	//index (nextPoint -1) will be the other control point used to calculate the point on the curve at a the given time
	for (int i = 0; i < controlPoints.size(); i++)
	{
		if (controlPoints[i].time > time)
		{
			nextPoint = i;
			return true;
		}
	} 
	return false; 
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	// Calculate time interval, and normal time required for later curve calculations
	int max = nextPoint;
	int min = nextPoint - 1;
	// normalized time, I used the formula provided here https://en.wikipedia.org/wiki/Feature_scaling
	float t = time - controlPoints[min].time;

	intervalTime = controlPoints[max].time - controlPoints[min].time;
	normalTime = t / intervalTime;

	float s;

	s = normalTime;    // 0<=s<=1
	float h1 = 2 * pow(s, 3) - 3 * pow(s, 2) + 1;
	float h2 = -2 * pow(s, 3) + 3 * pow(s, 2);
	float h3 = pow(s, 3) - 2 * pow(s, 2) + s;
	float h4 = pow(s, 3) - pow(s, 2);

	CurvePoint curvePoint1 = controlPoints[nextPoint - 1];
	CurvePoint curvePoint2 = controlPoints[nextPoint];

	newPosition = h1*curvePoint1.position + h2*curvePoint2.position + (h3*curvePoint1.tangent)*intervalTime + h4*curvePoint2.tangent*intervalTime; 
	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	// Calculate time interval, and normal time required for later curve calculations
	//intervalTime = controlPoints[nextPoint].time - time;

	float t = time - controlPoints[0].time;// given time - minimum time
    	int max = controlPoints.size() - 1;
	int min = 0;
	float intervalTime = controlPoints[max].time - controlPoints[min].time; 
	float s = t / intervalTime;
	// Calculate position at t = time on Catmull-Rom curve
	
	//For the CurvePoints, using nextPoint[-2] or lower will cause a crash. I moved everything up one though we're not going to get the result we want.
	CurvePoint p0 = controlPoints[nextPoint-1]; //n-1  /  
	CurvePoint p1 = controlPoints[nextPoint]; //n    t0 & y0
	CurvePoint p2 = controlPoints[nextPoint+1]; //n+1    t1 & y1
	CurvePoint p3 = controlPoints[nextPoint+2]; //n+2  t2 & y2
	
	Vector v, v2;	

	float changeinTime = p1.time - p0.time; //t_i - t_i-1 - from slides
	
	//using formula from catmull-rom spline: www.en.wikipedia.org/wiki/Cubic_Hermite_spline#Interpolating_a_data_set
	if(nextPoint == 0)
	{
		v = (p3.position - p1.position)/intervalTime;
		v2 = (p3.position - p1.position)/(p3.time - p1.time); //in formula
	}
	else if (nextPoint == max)
	{
		v = (p2.position - p0.position)/(p2.time - p0.time); //switched around formula
		v2 = (p2.position - p0.position) / intervalTime;
	}
	else //not first or last point
	{
		v = (p2.position - p0.position)/(p2.time - p0.time);
		v2 = (p3.position - p1.position)/(p3.time - p1.time); 
	}

	p1.tangent = v;
	p2.tangent = v2;
	//from hermite:
	float h1 = 2 * pow(s, 3) - 3 * pow(s, 2) + 1;
	float h2 = -2 * pow(s, 3) + 3 * pow(s, 2);
	float h3 = pow(s, 3) - 2 * pow(s, 2) + s;
	float h4 = pow(s, 3) - pow(s, 2);

	newPosition = h1*p1.position + h2*p2.position + (h3*p1.tangent)*intervalTime + (h4*p2.tangent)*intervalTime;

	// Return result
	return newPosition;
}