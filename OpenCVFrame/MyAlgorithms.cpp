#include "MyAlgorithms.h"

float CalChiSquareDis(vector<float> vA, vector<float> vB)
{
	float sum = 0;

	if (vA.size() != vB.size())
	{
		return -1;
	}

	for (int i = 0; i < vA.size(); i++)
	{
		if (vA[i] == 0 && vB[i] == 0)
		{
			continue;
		}
		else
		{
			sum += powf(vA[i] - vB[i], 2) / (vA[i] + vB[i]);
		}
	}
	return sum;
}




