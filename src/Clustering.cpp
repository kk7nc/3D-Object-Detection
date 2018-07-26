#include "Clustering.h"
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <limits>
#include <string>
#include "vec3.h"
#include <thread>
#include <future>
#include <ppl.h>
#include <omp.h>
// Windows
#include <Kinect.h>
#include <Windows.h>
#include <Strsafe.h>
using namespace Concurrency;
using namespace std;

#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998




inline float inner(const vec3 &v1, const vec3 &v2) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}



inline float CalculateDistance_(const vec3 &v1, const vec3 &v2)
{
	return (float)sqrt((v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y) + (v1.z - v2.z)*(v1.z - v2.z));
}
inline float Clustering::CalculateDistance(const vec3 &v1, const vec3 &v2, const vec3 &c1, const vec3 &c2, const vec3 &n1, const vec3 &n2, bool considerColor, bool considerNormal)
{
	float dist = 0.0;

	if (considerColor)
	{


		dist = sqrtf(gama*gama*((v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y) + (v1.z - v2.z)*(v1.z - v2.z)) +
			//alpha*((c1.x - c2.x)*(c1.x - c2.x) + (c1.y - c2.y)*(c1.y - c2.y) + (c1.z - c2.z)*(c1.z - c2.z)));

			alpha*alpha*((c1.x - c2.x)*(c1.x - c2.x) + (c1.y - c2.y)*(c1.y - c2.y) + (c1.z - c2.z)*(c1.z - c2.z)));
	}
	else
		dist = sqrtf(gama*(v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y) + (v1.z - v2.z)*(v1.z - v2.z));

	if (considerNormal)
	{
		float fWeight = 0.0001;
		if (!((n1.x == 0) && (n1.y == 0) && (n1.z == 0)))
			dist += fWeight * (1 - inner(n1, n2));
	}

	return dist;
}

//assigned Labels 
void Clustering::assigned_label(int i, int label)
{
	input[i].Label_c[label] += 1.0;
	float sum = 0.0;
	/*for (int j = 0; j < nNumCluster; j++)
	{
	if (input[i].Label_c[j] < 0.01) input[i].Label_c[j] = 0.01;
	sum += (input[i].Label_c[j] );
	}*/

	//sum = sqrtf(sum);
#pragma parallel

	for (int j = 0; j < nNumCluster; j++)
		input[i].Label_c[j] /= 2.0;
}

/********************************************************************
** Added by Manal
** implementation of k-means clusterin algorithm with distortion
** criteria of iteration termination.
********************************************************************/
void Clustering::Clustering_KMeans()
{
	// Added by Manal: Automatic uniformally seeding for k-means and k-means++ seeding for k-means
	// g_bAutoSeedingType values: 1: choose smart seeds, 2: choose uniformaly at random
	float fThreshold = 1.0, fChange, fMinDist, fDistance;
	// Choose the number of clusters, k.	
	// 1. Automatically generate k clusters and determine the cluster centers, or directly generate k random points as cluster centers.
	bool bAutomaticSeed = false;
	int terminationIteration = 1;

	if (center_of_cluster == NULL)
	{
		bAutomaticSeed = true;
		terminationIteration = 1;
		center_of_cluster = new vec3[nNumCluster];
		for (register int i = 0; i < IMAGESIZE; i++)
		{
			for (register int j = 0; j < nNumCluster; j++)
			{
				input[i].Label_c[j] = 0.0;

			}
			input[i].Label = 0;

		}
	}

	//to hold the automatic generated seeds
	vec3* center_of_cluster_ = new vec3[nNumCluster];
	vec3* normal_center_of_cluster = new vec3[nNumCluster];

	// copy seed to the center of the cluster
	if (bAutomaticSeed == false)
		for (register int i = 0; i<nNumCluster; i++)
			center_of_cluster_[i] = center_of_cluster[i];

	else
	{
		// intialize the labels to zero before we start

		// Output: center_of_cluster_, normal_center_of_cluster
		fThreshold = 0.1;
		if (AutoSeedingType == 1)
		{
			//ChooseSmartCenters(center_of_cluster_, 5);

			ChooseSmartCenters( center_of_cluster_, 5);//changed for parallel processing 
			//thread_ChooseSmartCenters.join();//changed for parallel processing 
			
			char * pFileName = "D:\\SEEDS.TXT";
			FILE *fc = fopen(pFileName, "w+");
			for (int l = 0; l < nNumCluster; l++)
				fprintf(fc, "%f	%f	%f\n", center_of_cluster_[l].x, center_of_cluster_[l].y, center_of_cluster_[l].z);
			fclose(fc);

		}
		else if (AutoSeedingType == 2)
		{
			ChooseUniformCenters( center_of_cluster_);
			//thread_ChooseUniformCenters.join();

			//ChooseUniformCenters(center_of_cluster_);
		}

		else
		{
			//printf("no clustering centers are specified!");
			return;
		}

		//read from file and put in center_of_cluster_[l].x, center_of_cluster_[l].y, center_of_cluster_[l].z of # of clusters

	}

	//after each iteration, check the center of the cluster to see if there is changes or not, if not, we got the optimal center
	vec3* center_of_cluster_old = new vec3[nNumCluster];
	int*  nNumPointInCluster = new int[nNumCluster];
	int* centerIndeces = new int[nNumCluster];
	int iterationCounter = 0;

	////////////////////////////////////////
	// Repeat the two steps

#pragma omp parallel
	//#pragma loop(hint_parallel(32))

	do
	{
		iterationCounter++;
#pragma omp parallel
		//#pragma loop(hint_parallel(32))
		for (register int j = 0; j < nNumCluster; j++)
		{
			//centerIndeces[j] = GetNearestNeighborIndex(center_of_cluster_[j]);

			
			centerIndeces[j] = GetNearestNeighborIndex(center_of_cluster_[j]);
			center_of_cluster_[j] = input[centerIndeces[j]].Pos3D;
			center_of_cluster_old[j] = center_of_cluster_[j];
		}
		//for all surface points
		// 2. Assign each point to the nearest cluster center, where "nearest" is defined with respect to one of the distance measures.
		// Distance Measure: Distance, Normal, Color

		//#pragma loop(hint_parallel(32))
#pragma omp parallel num_threads(32)
		{

			for (register int i = 0; i < IMAGESIZE; i++)


			{

				int new_cluster = 0;
				if (Mask[i])
				{
					fMinDist = 100000;
					float sum = 0.0;
#pragma omp parallel
#pragma loop(no_vector)
					//#pragma loop(hint_parallel(32))
					for (int j = 0; j < nNumCluster; j++)
					{
						// Distance: ecludian distance
						fDistance = CalculateDistance(input[i].Pos3D, center_of_cluster_[j], input[i].color/*input[i].color*/, input[centerIndeces[j]].color /*input[centerIndeces[j]].color*/,
							input[i].normal, input[centerIndeces[j]].normal, true, true);


						if (fDistance < fMinDist)
						{
							fMinDist = fDistance;
							new_cluster = j;
							input[i].Label = j;

						}
					}
					assigned_label(i, new_cluster);



				}
			}
		}
		////////////////////////////////////////
		// 3. Recompute the new cluster centers.
		// initilaization
#pragma omp parallel
		//#pragma loop(hint_parallel(32))

		parallel_for(int(0), nNumCluster, [&](int j)
		{
			center_of_cluster_[j] = vec3(0, 0, 0);
			normal_center_of_cluster[j] = vec3(0, 0, 0);
			nNumPointInCluster[j] = 0;
		});
		// averging
#pragma omp parallel

#pragma loop(no_vector)
#pragma loop(hint_parallel(32))
		for (int i = 0; i < IMAGESIZE; i++)

		{

			if (Mask[i])
			{
#pragma omp parallel
				//#pragma loop(hint_parallel(32))
				for (register int j = 0; j < nNumCluster; j++)
				{

					if ((int)input[i].Label == j)
					{
						center_of_cluster_[j].x += input[i].Pos3D.x;
						center_of_cluster_[j].y += input[i].Pos3D.y;
						center_of_cluster_[j].z += input[i].Pos3D.z;

						nNumPointInCluster[j]++;
					}
				}
			}

		}
#pragma omp parallel
		//#pragma loop(hint_parallel(32))

		for (register int j = 0; j < nNumCluster; j++)
		{
			if (nNumPointInCluster[j])
			{
				center_of_cluster_[j].x /= (float)nNumPointInCluster[j];
				center_of_cluster_[j].y /= (float)nNumPointInCluster[j];
				center_of_cluster_[j].z /= (float)nNumPointInCluster[j];
			}
			else
			{
				center_of_cluster_[j] = vec3(0, 0, 0);
			}
		}
		///////////////////////////////////////
		// Calculate the termination condition
		// if the centers of the cluster change less than threshold, the iteration stops. 
		//fChange = CalculateClusterChange(center_of_cluster_old, center_of_cluster_);

	} while (iterationCounter < terminationIteration);//(fChange != 0);

													  // save the center_of_cluster_ in the same array of the seeds
#pragma omp parallel
													  //#pragma loop(hint_parallel(32))
	for (int i = 0; i < nNumCluster; i++)
		center_of_cluster[i] = center_of_cluster_[i];

	delete[]center_of_cluster_;
	delete[]normal_center_of_cluster;
	delete[]center_of_cluster_old;
	delete[]nNumPointInCluster;

}
/********************************************************************
** Added by Manal
** used in k-means clusterin algorithm to calculate the distortion change.
*********************************************************************/
float CalculateClusterChange(vec3* center_of_cluster_old, vec3* center_of_cluster)
{
	//for termination: all clusteres in account
	float fChange = 0;
#pragma omp parallel
	//#pragma loop(hint_parallel(32))
	for (int j = 0; j < nNumCluster; j++)
		fChange += fabs(center_of_cluster_old[j].x - center_of_cluster[j].x)
		+ fabs(center_of_cluster_old[j].y - center_of_cluster[j].y)
		+ fabs(center_of_cluster_old[j].z - center_of_cluster[j].z);
	//printf("center change = %f\n", fChange);
	return fChange;
}
/********************************************************************
** Added by Manal
** Chooses random color for each label.
********************************************************************/
bool color_first = false;
void Clustering::AssignLabelColor()
{
	//using binary of 3 bits assignment
	if (clustersColors == NULL)
	{
		float r, g, b;
		clustersColors = new vec3[nNumCluster + 1];

		if (nNumCluster > 1 && nNumCluster < 8)
			for (int j = 1; j <= nNumCluster; j++)
				clustersColors[j - 1] = vec3(r = (j % 2) * 100, g = ((int)(j / 2) % 2) * 100, b = ((int)((int)(j / 2) / 2) % 2) * 100);

		else if (nNumCluster > 0)
		{
#pragma omp parallel
			//#pragma loop(hint_parallel(32))
			for (int j = 0; j < nNumCluster; j++)
			{
				r = rand() % 255; g = rand() % 255; b = rand() % 255;
				// make sure that this random generated color does not generated before
#pragma omp parallel
				//#pragma loop(hint_parallel(32))
				for (register int i = 0; i < j; i++)
				{
					if ((clustersColors[i].x == r) && (clustersColors[i].y == g) && (clustersColors[i].z == b))
					{
						r = rand() % 255; g = rand() % 255;
					}
				}
				clustersColors[j] = vec3(r, g, b);
			}
		}
	}
#pragma omp parallel
	//#pragma loop(hint_parallel(32))
	//for (register int i = 0; i < IMAGESIZE; i++)*/
	parallel_for(int(0), IMAGESIZE, [&](int i)
	{
		if (Mask[i])
		{
			vec3 Color_2;
			//	vec3 Color_1 = clustersColors[(int)input[i].Label] ;
			if (input[i].color_buffer.x == 0 && input[i].color_buffer.y == 0 && input[i].color_buffer.z == 0)
			{
				Color_2.x = 255.0; Color_2.y = 255.0; Color_2.z = 255.0;
			}
			else	Color_2 = (input[i].color_buffer);

			vec3 AA;
			//	AA.set(1.0, 1.0, 1.0);
			if (M_OBJECT_DETECTING)
			{
				AA.x += input[i].Label_c[ML_OBJECT_DETECTING] * input[i].color.x;// clustersColors[S_OBJECT_DETECTING].x;
				AA.y += input[i].Label_c[ML_OBJECT_DETECTING] * input[i].color.y;//clustersColors[S_OBJECT_DETECTING].y;
				AA.z += input[i].Label_c[ML_OBJECT_DETECTING] * input[i].color.z;//clustersColors[S_OBJECT_DETECTING].z;

			}
			else
			{

				if (!B_OBJECT_DETECTING)
				{
					for (register int ll = 0; ll < nNumCluster; ll++)
					{
						AA.x += input[i].Label_c[ll] * clustersColors[ll].x;
						AA.y += input[i].Label_c[ll] * clustersColors[ll].y;
						AA.z += input[i].Label_c[ll] * clustersColors[ll].z;

					}
				}
				else
				{


					AA.x += input[i].Label_c[S_OBJECT_DETECTING] * input[i].color.x;// clustersColors[S_OBJECT_DETECTING].x;
					AA.y += input[i].Label_c[S_OBJECT_DETECTING] * input[i].color.y;//clustersColors[S_OBJECT_DETECTING].y;
					AA.z += input[i].Label_c[S_OBJECT_DETECTING] * input[i].color.z;//clustersColors[S_OBJECT_DETECTING].z;

				}
			}
			vec3 CC = input[i].color;
			input[i].color = (AA + input[i].color_buffer) / 2.0;




			input[i].color_buffer = AA;




			//	j = nNumCluster;
			//}
			//}
			//color_first = true;
		}
	});
}
/********************************************************************
** Added by Manal
** Chooses a number of centers uniformly at random from the data set.
********************************************************************/
void Clustering::ChooseUniformCenters(vec3* center_of_cluster)
{
	int i;

	std::vector<int> centerIndices;
#pragma omp parallel
	//#pragma loop(hint_parallel(32))
	for (i = 0; i < IMAGESIZE; i++)
		centerIndices.push_back(i);

	// Choose each center one at a time, keeping the list up to date
#pragma omp parallel
	//#pragma loop(hint_parallel(32))
	for (i = 0; i < nNumCluster; i++)
	{
		int index = (int)(/*getRandomScalar*/(double(rand()) / RAND_MAX) * centerIndices.size());
		center_of_cluster[i] = input[centerIndices[index]].Pos3D;
		centerIndices[index] = centerIndices[int(centerIndices.size()) - 1];
		centerIndices.pop_back();
	}
}
/*******************************************************************************************
Added by Manal
to generate random index for k-means++ Algorithm
*********************************************************************************************/
int getRandomIndex()
{
	register int randomIndex = 0;
#pragma omp parallel
	//#pragma loop(hint_parallel(32))
	/*while (randomIndex == 0)
	{
	randomIndex = 10;// (int)(double(rand()) / RAND_MAX) * IMAGESIZE;
	if(!Mask[randomIndex])
	randomIndex = 0;
	}*/
	return 959549;
}
/*******************************************************************************************
Added by Manal
k-means++ Algorithm:
* Chooses a number of centers from the data set as follows:
*  - One center is chosen randomly.
*  - Now repeat numCenters-1 times:
*      - Repeat numLocalTries times:
*          - Add a point x with probability proportional to the distance squared from x
*            to the closest existing center
*      - Add the point chosen above that results in the smallest potential.
*******************************************************************************************/
void Clustering::ChooseSmartCenters(vec3* center_of_cluster, int numLocalTries)
{
	register int i;
	double currentPot = 0;
	std::vector<double> closestDistSq;

	// Choose one random center and set the closestDistSq values
	int index = getRandomIndex();
	center_of_cluster[0] = input[index].Pos3D;
	//	printf("%f, %f, %f\n", center_of_cluster[0].x, center_of_cluster[0].y, center_of_cluster[0].z);
#pragma omp parallel
	//#pragma loop(hint_parallel(32))
	for (i = 0; i < IMAGESIZE; i++)
	{

		closestDistSq.push_back(CalculateDistance_(input[i].Pos3D, input[index].Pos3D));
		currentPot += closestDistSq[i];
	}

	// Choose each center
#pragma omp parallel
	//#pragma loop(hint_parallel(32))
	for (register int centerCount = 1; centerCount < nNumCluster; centerCount++)
	{
		// Repeat several trials
		double bestNewPot = -1;
		int bestNewIndex;
#pragma omp parallel
		//#pragma loop(hint_parallel(32))
		for (int localTrial = 0; localTrial < numLocalTries; localTrial++)
		{
			// Choose our center - have to be slightly careful to return a valid answer even accounting
			// for possible rounding errors
			double randVal = /*getRandomScalar*/(double(rand()) / RAND_MAX) * currentPot;
#pragma omp parallel
			//#pragma loop(hint_parallel(32))
			for (index = 0; index < IMAGESIZE - 1; index++)
			{
				if (randVal <= closestDistSq[index])
					break;
				else
					randVal -= closestDistSq[index];
			}

			// Compute the new potential
			double newPot = 0;
#pragma omp parallel
			//#pragma loop(hint_parallel(32))
			for (i = 0; i < IMAGESIZE; i++)
				newPot += min((double)CalculateDistance_(input[i].Pos3D, input[index].Pos3D), closestDistSq[i]);

			// Store the best result
			if (bestNewPot < 0 || newPot < bestNewPot)
			{
				bestNewPot = newPot;
				bestNewIndex = index;
			}
		}

		// Add the appropriate center
		center_of_cluster[centerCount] = input[bestNewIndex].Pos3D;
		//printf("%f, %f, %f\n", center_of_cluster[centerCount].x, center_of_cluster[centerCount].y, center_of_cluster[centerCount].z);

		currentPot = bestNewPot;
#pragma parallel
#pragma omp parallel
		//#pragma loop(hint_parallel(32))
		for (i = 0; i < IMAGESIZE; i++)
			closestDistSq[i] = min((double)CalculateDistance_(input[i].Pos3D, input[bestNewIndex].Pos3D), closestDistSq[i]);
	}
}
int Clustering::GetNearestNeighborIndex(vec3 center_of_cluster)
{
	int nnIndex = 0;
	float minDist = CalculateDistance_(input[0].Pos3D, center_of_cluster);
	float currentDistance;
#pragma omp parallel
	//#pragma loop(hint_parallel(32))
	for (register int i = 0; i< IMAGESIZE; i++)
	{

		if (Mask[i])
		{
			currentDistance = CalculateDistance_(input[i].Pos3D, center_of_cluster);
			if (currentDistance < minDist)
			{
				minDist = currentDistance;
				nnIndex = i;
			}
		}
	}
	return nnIndex;
}

void Clustering::update() {
	Clustering_KMeans();
}
