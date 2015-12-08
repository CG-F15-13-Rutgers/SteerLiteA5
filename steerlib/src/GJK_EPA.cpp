/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


	SteerLib::GJK_EPA::GJK_EPA()//constructor
	{
	}

	//Look at the GJK_EPA.h header file for documentation and instructions
	bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
	{
		//std::cout << "inside intersect" << std::endl;
		std::vector<Util::Vector> simplex;

		if (SteerLib::GJK_EPA::GJK(_shapeA, _shapeB, simplex)) {
			//std::cout << "inside the function interesect. returning true. simplex length: " << simplex.size() << std::endl;
			/*std::vector<Util::Vector> simplex2;
			Util::Vector point1, point2, point3;
			point1.x = 4;
			point1.y = 0;
			point1.z = 2;
			simplex2.push_back(point1);
			point2.x = -8;
			point2.y = 0;
			point2.z = -2;
			simplex2.push_back(point2);
			point3.x = -1;
			point3.y = 0;
			point3.z = -2;
			simplex2.push_back(point3); */
			SteerLib::GJK_EPA::EPA(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB, simplex);
			return true;
		}
		return false; // There is no collision
	}



	using namespace SteerLib;
	
	Util::Vector GJK_EPA::GetFarthestPoint(std::vector<Util::Vector>& polygon, Util::Vector direction) {

		int farthestIndex = 0;
		float tmp;
		//soultion to possible problem is to use dot(); 
		float farthestDistance = polygon[0].operator*(direction); // dot product . for more info : steerlib\include\util\Geometry.h
		for (int i = 1; i < polygon.size(); i++) {
			tmp = polygon[i].operator*(direction);
			if (tmp > farthestDistance) {
				farthestDistance = tmp;
				farthestIndex = i;
			}
		}

		return polygon[farthestIndex];
	}

     Util::Vector GJK_EPA::Support(std::vector<Util::Vector> polygonA, std::vector<Util::Vector> polygonB, Util::Vector direction) {

		Util::Vector p1 = GJK_EPA::GetFarthestPoint(polygonA, direction);
		//possible problem with -(direction)
		direction = -(direction); // or multiply each value by -1
		Util::Vector p2 = GJK_EPA::GetFarthestPoint(polygonB, direction);
		Util::Vector p3 = p1.operator-(p2);

		return p3;
	}



	 Util::Vector GJK_EPA::GetClosestPointToOrigin(Util::Vector pointA, Util::Vector pointB) {
		if (pointA.lengthSquared() > pointB.lengthSquared()) { //pointA is further away from the origin than pointB
			return pointB;
		}
		else {
			return pointA;
		}
	}

    bool GJK_EPA::containsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& direction) {
		Util::Vector a = simplex.back();
		Util::Vector ao = a.operator-(); // negate a
		Util::Vector b;
		Util::Vector c;
		//check if simplex is a triangle case
		if (simplex.size() == 3) {
			b = simplex[1];
			c = simplex[0];
			//compute edges
			Util::Vector ab = b.operator-(a);
			Util::Vector ac = c.operator-(a);
			//compute the normals
			//ABPerp = AB(AB.dot(AC) - AC(AB.dot(AB))
			float x = ab.operator*(ac);//dot product
			float x2 = ab.operator*(ab);//dot product
			Util::Vector abPerp = ab.operator*(x).operator-(ac.operator*(x2));
			//acperp = AC(AC.dot(AB)) - AB(AC.dot(AC))
			float w = ac.operator*(ab); //dot product
			float w2 = ac.operator*(ac);
			Util::Vector acPerp = ac.operator*(w).operator-(ab.operator*(w2));

			//check if origin is contained
			if (abPerp.operator*(ao) > 0) {
				simplex.erase(simplex.begin()); //remove vector c which is in index 0
												//set the new direction to abPerp
				direction = abPerp;
			}
			else {
				//check if origin is in Region 3
				if (acPerp.operator*(ao) > 0) {
					//remove point b
					simplex.erase(simplex.begin());
					//set the new direction to acPerp
					direction = acPerp;
				}
				else 
					//at this point we know the origin is in region 5
					return true;
				}
		}
		else
		{
			//its in the line segment case
			b = simplex[0];
			//compute ab
			Util::Vector ab = b.operator-(a);
			//get the perpedicular line to AB  in direction of origin
			float w = ab.operator*(ab);
			float w2 = ab.operator*(ao);
			Util::Vector abPerp = ao.operator*(w).operator-(ab.operator*(w2));

			direction = abPerp;
		}
		return false;
	}

	 bool GJK_EPA::GJK(const std::vector<Util::Vector>& polygonA, const std::vector<Util::Vector>& polygonB, std::vector<Util::Vector>& simplex) {
		//printf("inside GJK()");
		//std::cout << "inside GJK" << std::endl;

		if (polygonA.size() == 0 || polygonB.size() == 0) {
			printf("one shape is missing points.");
			return false; // no collision if an object is not there
		}
		//create simplex

		//std::vector<Util::Vector> simplex;
		//any intitial direction we want
		Util::Vector direction; //still need to intialize
		direction.x = 6;
		direction.y = 0;
		direction.z = -4.5;
		Util::Vector A = GJK_EPA::Support(polygonA, polygonB, direction);
		//add new Point or vector to the simplex
		simplex.push_back(A);
		//negate direction
		direction = direction.operator-(); //line 90 in geometry.h

		//start looping
		while (true) {
			// add a new point to the simplex
			simplex.push_back(GJK_EPA::Support(polygonA, polygonB, direction));
			//check if last point added to the simplex pass the origin
			float t = simplex.back().operator*(direction); // operator* is the dot product
			if (t <= 0) {
				//the last point in the simplex is also the point on the edge of the minkowski difference
				//if this point is less than 0 then the minkowksi sum does not contain the origin
				return false;
			}
			else {
				//determine in origin is in the current simplex
				if (GJK_EPA::containsOrigin(simplex, direction))  //passing by reference
					return true;
				}


		}
	}

	 void GJK_EPA::EPA(float& penDepth, Util::Vector& penVector, const std::vector<Util::Vector>& polygonA, const std::vector<Util::Vector>& polygonB, std::vector<Util::Vector>& simplex) {
		 Util::Vector edge;
		 //printSimplexList(simplex);
		 float eDistance;
		 Util::Vector eNormal;
		 int eIndex;
		 //std::cout << "inside EPA" << std::endl;
		 Util::Vector supportPoint;
		 while (true) {
			 //std::cout << "inside EPA while loop" << std::endl;
			 //obtain the closestEdge to the origin 
			 // param list, float, Util::Vector, int
			 edge = FindClosestEdge(simplex, eDistance,eNormal, eIndex ); //pass by reference
			 //printPoint(edge);
			 //everything works up until this point. not sure about the rest
			 supportPoint = Support(polygonA, polygonB, eNormal);
			// std::cout << "new support point:" << std::endl;
			// printPoint(supportPoint);

			 float distance = supportPoint.operator*(eNormal);// using dot product to obtain distance
			 //std::cout << "distance  " << distance << std::endl;
			 float diff = distance - eDistance;
			 //std::cout << "eDistance that came back from Find()" << eDistance << std::endl;

			 //std::cout << "inside EPA while loop" << diff<< std::endl;
			 //printSimplexList(simplex);
			 //printEdgeInfo(eIndex, eDistance, eNormal);
			 if (diff < 0.00001) {
				 //if true then we cant expand the simplex any further
				 
				 penVector = eNormal;
				 penDepth = distance;// the distance along the eNormal
				 return;
			 }
			 else {
				 //continue expanding by adding new point to simplex in between the closestEdge
				// std::cout << "else add new point" << std::endl;
				 simplex.insert(simplex.begin()+eIndex, supportPoint); // 0+ eIndex should give us the index of the last point in the current edge
				// std::cout << "%%%%%%%%%%%%%%%%" << std::endl;
				// printSimplexList(simplex);
				// std::cout << "$$$$$$$$$$$$$$$" << std::endl;

			 }


		 }
	 }

	 Util::Vector GJK_EPA::FindClosestEdge(std::vector<Util::Vector>& simplex, float& eDistance, Util::Vector& eNormal, int& eIndex) {
		 //return an "edge" or vector
		 Util::Vector edge;
		 eDistance = FLT_MAX;

		 Util::Vector a, b, originToA, n;
		 //find edge of begin() and last()
		 int j;
		 for (int i = 0; i < simplex.size(); i++) {
			 //std::cout << "inside for loop i =" << i << std::endl;//delete
			     j = i + 1;
			 if (j == simplex.size())
					 j = 0;
				 //pick two points
				 a = simplex[i];
				 //printPoint(a);//delete
				 b = simplex[j];
				 //printPoint(b);//delete
				 //create edge or vector
				 edge = b.operator-(a);
				 //a - ORIGIN = a
				 originToA = a;
				 //  n will represent the edge towards the origin(ab, oa, ab) ->( e oa e)
				 float w = edge.operator*(edge);
				 float w2 = originToA.operator*(edge); //edge and oe 
				 n = originToA.operator*(w).operator-(edge.operator*(w2)); // n is a vector

				 //normalize vector n
				 float magnitudeOfn = n.norm(); // returns sqrt(x^2 +y^2+z^2)
				 n = n.operator/(magnitudeOfn);// divides vector by a scalar
				 //now calculate distance between origin and edge
				 float distance = n.operator*(a); // using dot product (n.x* a.x+ n.y*a.y+ n.z*a.z)
				// std::cout<< "edge "<<i << "d = a.dot(n) = " << distance << std::endl;

				//compare distances
				 if (distance < eDistance) {// the first time this will always be true b/c eDistance is initialized to FLT_MAX
					 //if true then use the close edge
					 //std::cout << "******inside if statement " << std::endl;
					 //printEdgeInfo(j, distance, n);
					 eDistance = distance;
					 eNormal = n;
					 eIndex = j;

				 }
		 }
		 return edge;
	 }

	 void GJK_EPA::printSimplexList(std::vector<Util::Vector> simplex) {
		 for (int i = 0; i < simplex.size(); i++) {
			 std::cout << "simplex point " << i << " : (" << simplex[i].x << "," << simplex[i].y << "," << simplex[i].z << " )" << std::endl;
		 }
	 }


	 void GJK_EPA::printPoint(Util::Vector point) {
		 std::cout << "(" << point.x << "," << point.y << "," << point.z << ")" << std::endl;
	 }

	 void GJK_EPA::printEdgeInfo(int index, float distance, Util::Vector normal) {
		 std::cout <<"index: "<<index<<" distance:"<<distance<< "normal : " <<"("<<normal.x<<","<<normal.y<<","<<normal.z<<")"<<std::endl;
	 }

