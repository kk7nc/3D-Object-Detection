#pragma once

#include "vec3.h"
#define IMAGESIZE 1920*1080//961*412
#define nNumCluster 4


typedef struct Image_buffer
{
	vec3 Pos3D;
	vec3 color;
	vec3 normal;
	float   Label_c[nNumCluster];
	int Label;
	vec3 color_buffer;
	

} Image_buffer;




class Clustering
{
public:
	inline void set_frame(Image_buffer* frame_points) {
		input = frame_points;
	}

	inline void set_mask(bool* valid_frame_points) {
		Mask = valid_frame_points;
	}

	void update();

	void assigned_label(int i, int label);
	void	AssignLabelColor();
	void	Clustering_KMeans();
	void ChooseUniformCenters(vec3* center_of_cluster);
	void ChooseSmartCenters(vec3* center_of_cluster, int numLocalTries);
	int GetNearestNeighborIndex(vec3 center_of_cluster);
	inline float CalculateDistance(const vec3 &v1, const vec3 &v2, const vec3 &c1, const vec3 &c2, const vec3 &n1, const vec3 &n2, bool considerColor, bool considerNormal);
private:
	bool *Mask;
	Image_buffer *input;
	float alpha = 0.00332931578291761926961249526;
	float gama = 1.0 - alpha;


	int S_OBJECT_DETECTING = -1;
	bool B_OBJECT_DETECTING = false;
	bool M_OBJECT_DETECTING = false;
	int ML_OBJECT_DETECTING;
	vec3* center_of_cluster = NULL;
	vec3* clustersColors = NULL;
	int AutoSeedingType = 1;

};