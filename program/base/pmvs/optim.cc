#define _USE_MATH_DEFINES
#include <cmath>

#include <algorithm>
#include <numeric>
#include "findMatch.h"
#include "optim.h"
#include <cstdio>

#include "nlopt.hpp"

using namespace Patch;
using namespace PMVS3;
using namespace std;

//其实就是this
Coptim* Coptim::m_one = NULL;

Coptim::Coptim(CfindMatch& findMatch) : m_fm(findMatch) {
	
	m_one = this;

	m_status.resize(35);
	fill(m_status.begin(), m_status.end(), 0);
}

void Coptim::init(void) {
	m_vect0T.resize(m_fm.m_CPU);
	m_centersT.resize(m_fm.m_CPU);
	m_raysT.resize(m_fm.m_CPU);
	m_indexesT.resize(m_fm.m_CPU);
	m_dscalesT.resize(m_fm.m_CPU);
	m_ascalesT.resize(m_fm.m_CPU);
	m_paramsT.resize(m_fm.m_CPU);
	m_xaxisT.resize(m_fm.m_CPU);
	m_yaxisT.resize(m_fm.m_CPU);

	m_texsT.resize(m_fm.m_CPU);
	m_weightsT.resize(m_fm.m_CPU);

	for (int c = 0; c < m_fm.m_CPU; ++c) {
		m_texsT[c].resize(m_fm.m_num);
		m_weightsT[c].resize(m_fm.m_num);
		for (int j = 0; j < m_fm.m_tau; ++j)
			m_texsT[c][j].resize(3 * m_fm.m_wsize * m_fm.m_wsize);
	}

	setAxesScales();
}

void Coptim::setAxesScales(void) {
	m_xaxes.resize(m_fm.m_num);
	m_yaxes.resize(m_fm.m_num);
	m_zaxes.resize(m_fm.m_num);
	for (int index = 0; index < m_fm.m_num; ++index) 
	{
		m_zaxes[index] = Vec3f(m_fm.m_pss.m_photos[index].m_oaxis[0],
			m_fm.m_pss.m_photos[index].m_oaxis[1],
			m_fm.m_pss.m_photos[index].m_oaxis[2]);
		m_xaxes[index] = Vec3f(m_fm.m_pss.m_photos[index].m_projection[0][0][0],
			m_fm.m_pss.m_photos[index].m_projection[0][0][1],
			m_fm.m_pss.m_photos[index].m_projection[0][0][2]);
		m_yaxes[index] = cross(m_zaxes[index], m_xaxes[index]);
		unitize(m_yaxes[index]);
		m_xaxes[index] = cross(m_yaxes[index], m_zaxes[index]);
	}

	m_ipscales.resize(m_fm.m_num);
	for (int index = 0; index < m_fm.m_num; ++index) {
		const Vec4f xaxe(m_xaxes[index][0], m_xaxes[index][1], m_xaxes[index][2], 0.0);
		const Vec4f yaxe(m_yaxes[index][0], m_yaxes[index][1], m_yaxes[index][2], 0.0);

		const float fx = xaxe * m_fm.m_pss.m_photos[index].m_projection[0][0];
		const float fy = yaxe * m_fm.m_pss.m_photos[index].m_projection[0][1];
		m_ipscales[index] = fx + fy;
	}
}

void Coptim::collectImages(const int index, std::vector<int>& indexes) const{

	indexes.clear();
	Vec4f ray0 = m_fm.m_pss.m_photos[index].m_oaxis;
	ray0[3] = 0.0f;

	vector<Vec2f> candidates;
	// Search for only related images
	for (int i = 0; i < (int)m_fm.m_visdata2[index].size(); ++i) {
		const int indextmp = m_fm.m_visdata2[index][i];

		//if (m_fm.m_tnum <= indextmp)，此处没有阈值 不会出现超出阈值的情况
		//continue;
		if (m_fm.m_sequenceThreshold != -1 &&
			m_fm.m_sequenceThreshold < abs(index - indextmp))
			continue;

		Vec4f ray1 = m_fm.m_pss.m_photos[indextmp].m_oaxis;
		ray1[3] = 0.0f;

		if (ray0 * ray1 < cos(m_fm.m_angleThreshold0))
			continue;

		candidates.push_back(Vec2f(m_fm.m_pss.m_distances[index][indextmp], indextmp));
	}

	sort(candidates.begin(), candidates.end(), Svec2cmp<float>());
	for (int i = 0; i < min(m_fm.m_tau, (int)candidates.size()); ++i)
		indexes.push_back((int)candidates[i][1]);
}


int Coptim::preProcess(Cpatch& patch, const int id, const int seed) {
	addImages(patch);

	// Here define reference images, and sort images.
	// Something similar to constraintImages is done inside.
	constraintImages(patch, m_fm.m_nccThresholdBefore, id);

	// Fix the reference image and sort the other  m_tau - 1 images.
	sortImages(patch);

	// Pierre Moulon (it avoid crash in some case)
	if ((int)patch.m_images.size() > 0)
	{
		// setSscales should be here to avoid noisy output
		m_fm.m_pos.setScales(patch);
	}

	// Check minimum number of images
	if ((int)patch.m_images.size() < m_fm.m_minImageNumThreshold)
		return 1;

	const int flag =
		m_fm.m_pss.checkAngles(patch.m_coord, patch.m_images,
		m_fm.m_maxAngleThreshold,
		m_fm.m_angleThreshold1,
		m_fm.m_minImageNumThreshold);

	if (flag) {
		patch.m_images.clear();
		return 1;
	}

	return 0;
}


void Coptim::filterImagesByAngle(Cpatch& patch) {
	vector<int> newindexes;

	vector<int>::iterator bimage = patch.m_images.begin();
	vector<int>::iterator eimage = patch.m_images.end();

	while (bimage != eimage) {
		const int index = *bimage;
		Vec4f ray = m_fm.m_pss.m_photos[index].m_center - patch.m_coord;
		unitize(ray);
		if (ray * patch.m_normal < cos(m_fm.m_angleThreshold1)) {
			// if reference image is deleted, over
			if (bimage == patch.m_images.begin()) {
				patch.m_images.clear();
				return;
			}
		}
		else
			newindexes.push_back(index);
		++bimage;
	}

	patch.m_images.swap(newindexes);
}

//seed根本用不上！
int Coptim::postProcess(Cpatch& patch, const int id, const int seed) 
{
	if ((int)patch.m_images.size() < m_fm.m_minImageNumThreshold)
		return 1;

	if (m_fm.m_pss.getMask(patch.m_coord, m_fm.m_level) == 0 ||
		m_fm.insideBimages(patch.m_coord) == 0)
		return 1;

	addImages(patch);

	constraintImages(patch, m_fm.m_nccThreshold, id);
	filterImagesByAngle(patch);

	if ((int)patch.m_images.size() < m_fm.m_minImageNumThreshold)
		return 1;

	m_fm.m_pos.setGrids(patch);

	setRefImage(patch, id);
	//实际constraintimages可以集成在setrefimage中，避免重复计算ncc
	constraintImages(patch, m_fm.m_nccThreshold, id);

	if ((int)patch.m_images.size() < m_fm.m_minImageNumThreshold)
		return 1;

	m_fm.m_pos.setGrids(patch);

	// set m_timages
	patch.m_timages = 0;
	vector<int>::const_iterator begin = patch.m_images.begin();
	vector<int>::const_iterator end = patch.m_images.end();
	while (begin != end) {
		if (*begin < m_fm.m_tnum)
			++patch.m_timages;
		++begin;
	}

	patch.m_tmp = patch.score2(m_fm.m_nccThreshold);
	// Set vimages vgrids.seed阶段没有深度信息
	if (m_fm.m_depth) 
	{
		m_fm.m_pos.setVImagesVGrids(patch);

		if (2 <= m_fm.m_depth && check(patch))
			return 1;
	}
	return 0;
}

int Coptim::postProcess2(Patch::Cpatch& patch, const int id, const int seed)
{
	if ((int)patch.m_images.size() < m_fm.m_minImageNumThreshold)
		return 1;

	if (m_fm.m_pss.getMask(patch.m_coord, m_fm.m_level) == 0 ||
		m_fm.insideBimages(patch.m_coord) == 0)
		return 1;

	addImages(patch);

	constraintImages(patch, m_fm.m_nccThreshold, id);
	filterImagesByAngle(patch);

	if ((int)patch.m_images.size() < m_fm.m_minImageNumThreshold)
		return 1;

	m_fm.m_pos.setGrids(patch);

	setRefImage(patch, id);
	//实际constraintimages可以集成在setrefimage中，避免重复计算ncc
	constraintImages(patch, m_fm.m_nccThreshold, id);

	if ((int)patch.m_images.size() < m_fm.m_minImageNumThreshold)
		return 1;

	m_fm.m_pos.setGrids(patch);

	// set m_timages
	patch.m_timages = 0;
	vector<int>::const_iterator begin = patch.m_images.begin();
	vector<int>::const_iterator end = patch.m_images.end();
	while (begin != end) {
		if (*begin < m_fm.m_tnum)
			++patch.m_timages;
		++begin;
	}

	patch.m_tmp = patch.score2(m_fm.m_nccThreshold);
	// Set vimages vgrids.seed阶段没有深度信息
	if (m_fm.m_depth)
	{
		m_fm.m_pos.setVImagesVGrids(patch);

		if (2 <= m_fm.m_depth && check(patch))
			return 1;
	}
	return 0;

}

void Coptim::constraintImages(Cpatch& patch, const float nccThreshold,
	const int id) {
	vector<float> inccs;
// 	setINCCs(patch, inccs, patch.m_images, id, 0);

	if (bOrigin)
	{
		setINCCs(patch, inccs, patch.m_images, id, 0);
	}
	else
	{
		setINCCs2(patch, inccs, patch.m_images, id, 0);
	}
	//----------------------------------------------------------------------
	// Constraint images
	vector<int> newimages;
	newimages.push_back(patch.m_images[0]);
	for (int i = 1; i < (int)patch.m_images.size(); ++i) {
		if (inccs[i] < 1.0f - nccThreshold)
			newimages.push_back(patch.m_images[i]);
	}
	patch.m_images.swap(newimages);
}

void Coptim::setRefImage(Cpatch& patch, const int id) {
#ifdef DEBUG
	if (patch.m_images.empty()) {
		cerr << "empty images" << endl;    exit (1);
	}
#endif
	//----------------------------------------------------------------------
	// Set the reference image
	// Only for target images
	vector<int> indexes;
	vector<int>::const_iterator begin = patch.m_images.begin();
	vector<int>::const_iterator end = patch.m_images.end();
	while (begin != end) 
	{
		if (*begin < m_fm.m_tnum)
			indexes.push_back(*begin);
		++begin;
	}
	// To avoid segmentation error on alley dataset. (this code is necessary because of the use of filterExact)
	if (indexes.empty()) {
		patch.m_images.clear();
		return;
	}

	vector<vector<float> > inccs;
	if (bOrigin)
	{
		setINCCs(patch, inccs, indexes, id, 1);

	}
	else
	{
		setINCCs2(patch, inccs, indexes, id, 1);

	}

	
	int refindex = -1;
	float refncc = INT_MAX / 2;
	for (int i = 0; i < (int)indexes.size(); ++i) {
		const float sum = accumulate(inccs[i].begin(), inccs[i].end(), 0.0f);
		if (sum < refncc) {
			refncc = sum;
			refindex = i;
		}
	}

	const int refIndex = indexes[refindex];
	//这个循环是在太逗，直接用refindex索引不就好了？
	for (int i = 0; i < (int)patch.m_images.size(); ++i) {
		if (patch.m_images[i] == refIndex) {
			const int itmp = patch.m_images[0];
			patch.m_images[0] = refIndex;
			patch.m_images[i] = itmp;
			break;
		}
	}
}

// When no sampling was done, this is used
void Coptim::setRefConstraintImages(Cpatch& patch, const float nccThreshold,
	const int id) {
	//----------------------------------------------------------------------
	// Set the reference image
	vector<vector<float> > inccs;
	setINCCs(patch, inccs, patch.m_images, id, 1);

	int refindex = -1;
	float refncc = INT_MAX / 2;
	for (int i = 0; i < (int)patch.m_images.size(); ++i) {
		const float sum = accumulate(inccs[i].begin(), inccs[i].end(), 0.0f);
		if (sum < refncc) {
			refncc = sum;
			refindex = i;
		}
	}

	//refindex = 0;

	const float robustThreshold = robustincc(1.0f - nccThreshold);
	vector<int> newimages;
	newimages.push_back(patch.m_images[refindex]);
	for (int i = 0; i < (int)patch.m_images.size(); ++i)
	if (i != refindex && inccs[refindex][i] < robustThreshold)
		newimages.push_back(patch.m_images[i]);
	patch.m_images.swap(newimages);
}

void Coptim::sortImages(Cpatch& patch) const{
	const int newm = 1;
	if (newm == 1) {
		const float threshold = 1.0f - cos(10.0 * M_PI / 180.0);
		vector<int> indexes, indexes2;
		vector<float> units, units2;
		vector<Vec4f> rays, rays2;

		computeUnits(patch, indexes, units, rays);

		patch.m_images.clear();
		if (indexes.size() < 2)
			return;

		units[0] = 0.0f;

		while (!indexes.empty()) {
			vector<float>::iterator ite = min_element(units.begin(), units.end());
			const int index = ite - units.begin();

			patch.m_images.push_back(indexes[index]);

			// Remove other images within 5 degrees
			indexes2.clear();    units2.clear();
			rays2.clear();
			for (int j = 0; j < (int)rays.size(); ++j) {
				if (j == index)
					continue;

				indexes2.push_back(indexes[j]);
				rays2.push_back(rays[j]);
				const float ftmp = min(threshold,
					max(threshold / 2.0f,
					1.0f - rays[index] * rays[j]));

				units2.push_back(units[j] * (threshold / ftmp));
			}
			indexes2.swap(indexes);
			units2.swap(units);
			rays2.swap(rays);
		}
	}
	else {
		//----------------------------------------------------------------------
		//Sort and grab the best m_tau images. All the other images don't
		//matter.  First image is the reference and fixed
		const float threshold = cos(5.0 * M_PI / 180.0);
		vector<int> indexes, indexes2;
		vector<float> units, units2;
		vector<Vec4f> rays, rays2;

		computeUnits(patch, indexes, units, rays);

		patch.m_images.clear();
		if (indexes.size() < 2)
			return;

		units[0] = 0.0f;

		while (!indexes.empty()) {
			//for (int i = 0; i < size; ++i) {
			vector<float>::iterator ite = min_element(units.begin(), units.end());
			const int index = ite - units.begin();

			patch.m_images.push_back(indexes[index]);

			// Remove other images within 5 degrees
			indexes2.clear();    units2.clear();
			rays2.clear();
			for (int j = 0; j < (int)rays.size(); ++j) {
				if (rays[index] * rays[j] < threshold) {
					indexes2.push_back(indexes[j]);
					units2.push_back(units[j]);
					rays2.push_back(rays[j]);
				}
			}
			indexes2.swap(indexes);
			units2.swap(units);
			rays2.swap(rays);
		}
	}
}

int Coptim::check(Cpatch& patch) {
	const float gain = m_fm.m_filter.computeGain(patch, 1);
	patch.m_tmp = gain;

	if (gain < 0.0) {
		patch.m_images.clear();
		return 1;
	}

	{
		vector<Ppatch> neighbors;
		m_fm.m_pos.findNeighbors(patch, neighbors, 1, 4, 2);
		// Only check when enough number of neighbors
		if (6 < (int)neighbors.size() &&
			//if (8 < (int)neighbors.size() &&
			m_fm.m_filter.filterQuad(patch, neighbors)) {
			patch.m_images.clear();
			return 1;
		}
	}

	return 0;
}

void Coptim::removeImagesEdge(Patch::Cpatch& patch) const{
	vector<int> newindexes;
	vector<int>::const_iterator bimage = patch.m_images.begin();
	vector<int>::const_iterator eimage = patch.m_images.end();
	while (bimage != eimage) {
		if (m_fm.m_pss.getEdge(patch.m_coord, *bimage, m_fm.m_level))
			newindexes.push_back(*bimage);
		++bimage;
	}
	patch.m_images.swap(newindexes);
}

//reference visdata图片中，“添加”可以看见patch并且视角在60°之内的图片
void Coptim::addImages(Patch::Cpatch& patch) const{
	vector<int> used;
	used.resize(m_fm.m_num);
	for (int index = 0; index < m_fm.m_num; ++index)
		used[index] = 0;

	vector<int>::const_iterator bimage = patch.m_images.begin();
	vector<int>::const_iterator eimage = patch.m_images.end();
	while (bimage != eimage) {
		used[*bimage] = 1;
		++bimage;
	}

	bimage = m_fm.m_visdata2[patch.m_images[0]].begin();
	eimage = m_fm.m_visdata2[patch.m_images[0]].end();

	const float athreshold = cos(m_fm.m_angleThreshold0);
	while (bimage != eimage) {
		if (used[*bimage]) {
			++bimage;
			continue;
		}

		const Vec3f icoord = m_fm.m_pss.project(*bimage, patch.m_coord, m_fm.m_level);
		if (icoord[0] < 0.0f || m_fm.m_pss.getWidth(*bimage, m_fm.m_level) - 1 <= icoord[0] ||
			icoord[1] < 0.0f || m_fm.m_pss.getHeight(*bimage, m_fm.m_level) - 1 <= icoord[1]) {
			++bimage;
			continue;
		}

		//真尼玛蛋炸，又求一次投影然后判断是否在edge之内，为何不直接在上一步判断？！！
		if (m_fm.m_pss.getEdge(patch.m_coord, *bimage, m_fm.m_level) == 0) {
			++bimage;
			continue;
		}

		Vec4f ray = m_fm.m_pss.m_photos[*bimage].m_center - patch.m_coord;
		unitize(ray);
		const float ftmp = ray * patch.m_normal;

		if (athreshold <= ftmp)
			patch.m_images.push_back(*bimage);

		++bimage;
	}
}

void Coptim::computeUnits(const Patch::Cpatch& patch,std::vector<float>& units) const
{
	const int size = (int)patch.m_images.size();
	units.resize(size);

	vector<int>::const_iterator bimage = patch.m_images.begin();
	vector<int>::const_iterator eimage = patch.m_images.end();

	vector<float>::iterator bfine = units.begin();

	while (bimage != eimage) {
		*bfine = INT_MAX / 2;

		*bfine = getUnit(*bimage, patch.m_coord);
		Vec4f ray = m_fm.m_pss.m_photos[*bimage].m_center - patch.m_coord;
		unitize(ray);
		const float denom = ray * patch.m_normal;
		if (0.0 < denom)
			*bfine /= denom;
		else
			*bfine = INT_MAX / 2;

		++bimage;
		++bfine;
	}
}

void Coptim::computeUnits(const Patch::Cpatch& patch,
	std::vector<int>& indexes,
	std::vector<float>& units,
	std::vector<Vec4f>& rays) const{
	vector<int>::const_iterator bimage = patch.m_images.begin();
	vector<int>::const_iterator eimage = patch.m_images.end();

	while (bimage != eimage) {
		Vec4f ray = m_fm.m_pss.m_photos[*bimage].m_center - patch.m_coord;
		unitize(ray);
		const float dot = ray * patch.m_normal;
		if (dot <= 0.0f) {
			++bimage;
			continue;
		}

		const float scale = getUnit(*bimage, patch.m_coord);
		const float fine = scale / dot;

		indexes.push_back(*bimage);
		units.push_back(fine);
		rays.push_back(ray);
		++bimage;
	}
}

void Coptim::refinePatch(Cpatch& patch, const int id,
	const int time) {
	if (!refinePatchBFGS(patch, id, 1000, 1))
		std::cout << "refinePatchBFGS failed!" << std::endl;

	if (patch.m_images.empty())
		return;
}

void Coptim::refinePatch2(Cpatch& patch, const int id,
	const int time) {
	if (!refinePatchBFGS2(patch, id, 1000))
		std::cout << "refinePatchBFGS failed!" << std::endl;

	if (patch.m_images.empty())
		return;
}


//----------------------------------------------------------------------
// BFGS functions
//----------------------------------------------------------------------

// 2 是最大值，函数中返回2表示绝对不可取。如果*vp数量少，也不可取
double Coptim::my_f(unsigned n, const double *x, double *grad, void *my_func_data)
{
	double xs[3] = { x[0], x[1], x[2] };
	const int id = *((int*)my_func_data);

	const float angle1 = xs[1] * m_one->m_ascalesT[id];
	const float angle2 = xs[2] * m_one->m_ascalesT[id];

	double ret = 0.0;

	//?????
	const double bias = 0.0f;//2.0 - exp(- angle1 * angle1 / sigma2) - exp(- angle2 * angle2 / sigma2);

	Vec4f coord, normal;
	m_one->decode(coord, normal, xs, id);

	const int index = m_one->m_indexesT[id][0];
	Vec4f pxaxis, pyaxis;
	m_one->getPAxes(index, coord, normal, pxaxis, pyaxis);

	const int size = min(m_one->m_fm.m_tau, (int)m_one->m_indexesT[id].size());
	const int mininum = min(m_one->m_fm.m_minImageNumThreshold, size);

	for (int i = 0; i < size; ++i) {
		int flag;
		flag = m_one->grabTex(coord, pxaxis, pyaxis, normal, m_one->m_indexesT[id][i], m_one->m_fm.m_wsize, m_one->m_texsT[id][i]);

		if (flag == 0)
			m_one->normalize(m_one->m_texsT[id][i]);
	}

	//光照差异度的另一种形式，*vp每一对都进行判断！
	const int pairwise = 0;
	if (pairwise) {
		double ans = 0.0f;
		int denom = 0;
		for (int i = 0; i < size; ++i) {
			for (int j = i + 1; j < size; ++j) {
				if (m_one->m_texsT[id][i].empty() || m_one->m_texsT[id][j].empty())
					continue;

				ans += robustincc(1.0 - m_one->dot(m_one->m_texsT[id][i], m_one->m_texsT[id][j]));
				denom++;
			}
		}
		if (denom <
			//m_one->m_fm.m_minImageNumThreshold *
			//(m_one->m_fm.m_minImageNumThreshold - 1) / 2)
			mininum * (mininum - 1) / 2)
			ret = 2.0f;
		else
			ret = ans / denom + bias;
	}
	else {
		if (m_one->m_texsT[id][0].empty())
			return 2.0;

		double ans = 0.0f;
		int denom = 0;
		for (int i = 1; i < size; ++i) {
			if (m_one->m_texsT[id][i].empty())
				continue;
			ans +=
				robustincc(1.0 - m_one->dot(m_one->m_texsT[id][0], m_one->m_texsT[id][i]));
			denom++;
		}
		//if (denom < m_one->m_fm.m_minImageNumThreshold - 1)
		if (denom < mininum - 1)
			ret = 2.0f;
		else
			ret = ans / denom + bias;
	}

	return ret;
}

double Coptim::my_f2(unsigned n, const double *x, double *grad, void *my_func_data)
{
	double xs[5] = { x[0], x[1], x[2] ,x[3],x[4]};
	const int id = *((int*)my_func_data);

	const float angle1 = xs[1] * m_one->m_ascalesT[id];
	const float angle2 = xs[2] * m_one->m_ascalesT[id];

	double ret = 0.0;

	//?????
	const double bias = 0.0f;//2.0 - exp(- angle1 * angle1 / sigma2) - exp(- angle2 * angle2 / sigma2);

	Vec4f coord, normal,xaxis,yaxis,negxaxis;
	const int index = m_one->m_indexesT[id][0];
	m_one->decode2(coord, normal, xaxis,negxaxis,yaxis,xs, id);
	const float xdis = norm(m_one->m_fm.m_pss.project(index, coord + xaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
	const float nxdis = norm(m_one->m_fm.m_pss.project(index, coord + negxaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
	const float ydis = norm(m_one->m_fm.m_pss.project(index, coord + yaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
	xaxis /= xdis;
	negxaxis /= nxdis;
	yaxis /= ydis;



// 	const int index = m_one->m_indexesT[id][0];
// 	Vec4f pxaxis, pyaxis;
// 	m_one->getPAxes(index, coord, normal, pxaxis, pyaxis);

	const int size = min(m_one->m_fm.m_tau, (int)m_one->m_indexesT[id].size());
	const int mininum = min(m_one->m_fm.m_minImageNumThreshold, size);

	for (int i = 0; i < size; ++i) {
		int flag;
		flag = m_one->grabTex2(coord, xaxis, negxaxis,yaxis, normal, m_one->m_indexesT[id][i], m_one->m_fm.m_wsize, m_one->m_texsT[id][i]);

		if (flag == 0)
			m_one->normalize(m_one->m_texsT[id][i]);
	}

	//光照差异度的另一种形式，*vp每一对都进行判断！
	const int pairwise = 0;
	if (pairwise) {
		double ans = 0.0f;
		int denom = 0;
		for (int i = 0; i < size; ++i) {
			for (int j = i + 1; j < size; ++j) {
				if (m_one->m_texsT[id][i].empty() || m_one->m_texsT[id][j].empty())
					continue;

				ans += robustincc(1.0 - m_one->dot(m_one->m_texsT[id][i], m_one->m_texsT[id][j]));
				denom++;
			}
		}
		if (denom <
			//m_one->m_fm.m_minImageNumThreshold *
			//(m_one->m_fm.m_minImageNumThreshold - 1) / 2)
			mininum * (mininum - 1) / 2)
			ret = 2.0f;
		else
			ret = ans / denom + bias;
	}
	else {
		if (m_one->m_texsT[id][0].empty())
			return 2.0;

		double ans = 0.0f;
		int denom = 0;
		for (int i = 1; i < size; ++i) {
			if (m_one->m_texsT[id][i].empty())
				continue;
			ans +=
				robustincc(1.0 - m_one->dot(m_one->m_texsT[id][0], m_one->m_texsT[id][i]));
			denom++;
		}
		//if (denom < m_one->m_fm.m_minImageNumThreshold - 1)
		if (denom < mininum - 1)
			ret = 2.0f;
		else
			ret = ans / denom + bias;
	}

	return ret;

}
// refine结果 ， *vp数量大于一定值情况下的最优ncc，更新了位置、法向量 以及优化之前的权重
bool Coptim::refinePatchBFGS(Cpatch& patch, const int id,
	const int time, const int ncc)
{
	int idtmp = id;

	m_centersT[id] = patch.m_coord;
	m_raysT[id] = patch.m_coord - m_fm.m_pss.m_photos[patch.m_images[0]].m_center;
	unitize(m_raysT[id]);
	m_indexesT[id] = patch.m_images;

	m_dscalesT[id] = patch.m_dscale;
	//算了半天，最后还是毛用没有啊！
	m_ascalesT[id] = M_PI / 48.0f;//patch.m_ascale;

	//在第一波，reference的weight确实是最大的（然而此变量根本没有被用上）
	computeUnits(patch, m_weightsT[id]);
	for (int i = 1; i < (int)m_weightsT[id].size(); ++i)
		m_weightsT[id][i] = min(1.0f, m_weightsT[id][0] / m_weightsT[id][i]);
	m_weightsT[id][0] = 1.0f;

	double p[3];
	encode(patch.m_coord, patch.m_normal, p, id);

	//角度偏移范围 在 24个角度单位之间（即 +-90°范围）
	double min_angle = -23.99999;	//(- M_PI / 2.0) / m_one->m_ascalesT[id];
	double max_angle = 23.99999;	//(M_PI / 2.0) / m_one->m_ascalesT[id];

	std::vector<double> lower_bounds(3);
	lower_bounds[0] = -HUGE_VAL;		// Not bound
	lower_bounds[1] = min_angle;
	lower_bounds[2] = min_angle;
	std::vector<double> upper_bounds(3);
	upper_bounds[0] = HUGE_VAL;		// Not bound
	upper_bounds[1] = max_angle;
	upper_bounds[2] = max_angle;

	bool success = false;

	try
	{
		// LN_NELDERMEAD: Corresponds to the N-Simplex-Algorithm of GSL, that was used originally here
		// LN_SBPLX
		// LN_COBYLA
		// LN_BOBYQA
		// LN_PRAXIS
		nlopt::opt opt(nlopt::LN_BOBYQA, 3);
		opt.set_min_objective(my_f, &idtmp);
		opt.set_xtol_rel(1.e-7);
		opt.set_maxeval(time);

		opt.set_lower_bounds(lower_bounds);
		opt.set_upper_bounds(upper_bounds);

		std::vector<double> x(3);
		for (int i = 0; i < 3; i++)
		{
			// NLOPT returns an error if x is not within the bounds
			x[i] = max(min(p[i], upper_bounds[i]), lower_bounds[i]);
		}

		double minf;
		nlopt::result result = opt.optimize(x, minf);

		p[0] = x[0];
		p[1] = x[1];
		p[2] = x[2];

		success = (result == nlopt::SUCCESS
			|| result == nlopt::STOPVAL_REACHED
			|| result == nlopt::FTOL_REACHED
			|| result == nlopt::XTOL_REACHED);
	}
	catch (std::exception &e)
	{
		success = false;
	}

	if (success) {
		decode(patch.m_coord, patch.m_normal, p, id);
		//竟然不直接用minf计算 (⊙﹏⊙)b
		patch.m_ncc = 1.0 - unrobustincc(computeINCC(patch.m_coord, patch.m_normal, patch.m_images, id, 1));

	}
	else {
		return false;
	}

	return true;
}

bool Coptim::refinePatchBFGS2(Patch::Cpatch& patch, const int id, const int time)
{
	int idtmp = id;

	m_centersT[id] = patch.m_coord;
	m_raysT[id] = patch.m_coord - m_fm.m_pss.m_photos[patch.m_images[0]].m_center;
	unitize(m_raysT[id]);
	m_indexesT[id] = patch.m_images;

	m_dscalesT[id] = patch.m_dscale;
	//算了半天，最后还是毛用没有啊！
	m_ascalesT[id] = M_PI / 48.0f;//patch.m_ascale;

	//在第一波，reference的weight确实是最大的（然而此变量根本没有被用上）
	computeUnits(patch, m_weightsT[id]);
	for (int i = 1; i < (int)m_weightsT[id].size(); ++i)
		m_weightsT[id][i] = min(1.0f, m_weightsT[id][0] / m_weightsT[id][i]);
	m_weightsT[id][0] = 1.0f;

	double p[5];
	encode2(patch.m_coord, patch.m_normal,patch.m_xaxis,patch.m_yaxis, p, id);

	
// 	decode2(patch.m_coord, patch.m_normal, patch.m_xaxis, patch.m_negxaxis,patch.m_yaxis, p, id);

	//角度偏移范围 在 24个角度单位之间（即 +-90°范围）
	double min_angle = -23.99999;	//(- M_PI / 2.0) / m_one->m_ascalesT[id];
	double max_angle = 23.99999;	//(M_PI / 2.0) / m_one->m_ascalesT[id];
	double min_folder = -11.9999;
	double max_folder = 11.9999;

	std::vector<double> lower_bounds(5);
	lower_bounds[0] = -HUGE_VAL;		// Not bound
	lower_bounds[1] = min_angle;
	lower_bounds[2] = min_angle;
	lower_bounds[3] = min_angle;
	lower_bounds[4] = min_folder;


	std::vector<double> upper_bounds(5);
	upper_bounds[0] = HUGE_VAL;		// Not bound
	upper_bounds[1] = max_angle;
	upper_bounds[2] = max_angle;
	upper_bounds[3] = max_angle;
	upper_bounds[4] = max_folder;


	bool success = false;

	try
	{
		// LN_NELDERMEAD: Corresponds to the N-Simplex-Algorithm of GSL, that was used originally here
		// LN_SBPLX
		// LN_COBYLA
		// LN_BOBYQA
		// LN_PRAXIS
		nlopt::opt opt(nlopt::LN_BOBYQA, 5);
		opt.set_min_objective(my_f2, &idtmp);
		opt.set_xtol_rel(1.e-7);
		opt.set_maxeval(time * 2);

		opt.set_lower_bounds(lower_bounds);
		opt.set_upper_bounds(upper_bounds);

		std::vector<double> x(5);
		for (int i = 0; i < 5; i++)
		{
			// NLOPT returns an error if x is not within the bounds
			x[i] = max(min(p[i], upper_bounds[i]), lower_bounds[i]);
		}

		double minf;
		nlopt::result result = opt.optimize(x, minf);

		p[0] = x[0];
		p[1] = x[1];
		p[2] = x[2];
		p[3] = x[3];
		p[4] = x[4];

		success = (result == nlopt::SUCCESS
			|| result == nlopt::STOPVAL_REACHED
			|| result == nlopt::FTOL_REACHED
			|| result == nlopt::XTOL_REACHED);
	}
	catch (std::exception &e)
	{
		success = false;
	}

	if (success) {
		decode2(patch.m_coord, patch.m_normal,patch.m_xaxis,patch.m_negxaxis,patch.m_yaxis,patch.m_angle,patch.m_folder, p, id);
		//竟然不直接用minf计算 (⊙﹏⊙)b
		patch.m_ncc = 1.0 - unrobustincc(computeINCC(patch.m_coord, patch.m_normal, patch.m_images, id, 1));

	}
	else {
		return false;
	}

	return true;

}

//vect 0 为patch沿轴移动之后，对应*vp中移动像素距离（即patch的像素移动距离）
void Coptim::encode(const Vec4f& coord, double* const vect, const int id) const 
{
	vect[0] = (coord - m_centersT[id]) * m_raysT[id] / m_dscalesT[id];
}

//vect1 vect2 分别为像素的x轴、y轴角度移动距离（角度 / 角度单位），patch的xy初始与r相同，z反向
void Coptim::encode(const Vec4f& coord, const Vec4f& normal, double* const vect, const int id) const 
{
	encode(coord, vect, id);

	const int image = m_indexesT[id][0];
	const float fx = m_xaxes[image] * proj(normal); // projects from 4D to 3D, divide by last value
	const float fy = m_yaxes[image] * proj(normal);
	const float fz = m_zaxes[image] * proj(normal);

	//真应该用inside函数代替max、min系列
	vect[2] = asin(max(-1.0f, min(1.0f, fy)));
	const float cosb = cos(vect[2]);

	if (cosb == 0.0)
		vect[1] = 0.0;
	else {
		const float sina = fx / cosb;
		const float cosa = -fz / cosb;
		vect[1] = acos(max(-1.0f, min(1.0f, cosa)));
		if (sina < 0.0)
			vect[1] = -vect[1];
	}

	vect[1] = vect[1] / m_ascalesT[id];
	vect[2] = vect[2] / m_ascalesT[id];
}

void Coptim::encode2(const Vec4f& coord, const Vec4f& normal, const Vec4f& xaxis, const Vec4f& yaxis, double* const vect, const int id) const
{
	encode(coord, vect, id);

	const int image = m_indexesT[id][0];
	const float fx = m_xaxes[image] * proj(normal); // projects from 4D to 3D, divide by last value
	const float fy = m_yaxes[image] * proj(normal);
	const float fz = m_zaxes[image] * proj(normal);

	//真应该用inside函数代替max、min系列
	vect[2] = asin(max(-1.0f, min(1.0f, fy)));
	const float cosb = cos(vect[2]);

	if (cosb == 0.0)
		vect[1] = 0.0;
	else {
		const float sina = fx / cosb;
		const float cosa = -fz / cosb;
		vect[1] = acos(max(-1.0f, min(1.0f, cosa)));
		if (sina < 0.0)
			vect[1] = -vect[1];
	}

	vect[1] = vect[1] / m_ascalesT[id];
	vect[2] = vect[2] / m_ascalesT[id];

	//folder和angle
	Vec4f refxaxis, refyaxis;
	getPAxes(m_one->m_indexesT[id][0], coord, normal, refxaxis, refyaxis);
	unitize(refxaxis);
	unitize(refyaxis);

	//实际上angle是限定在 -90 到 90°之间的
	float angle = acos(yaxis * refyaxis);
	if ((yaxis - refyaxis) * refxaxis > 0)
	{
		angle = -angle;
	}
	vect[3] = angle / m_ascalesT[id];

	refxaxis += tan(angle) * refyaxis;
	unitize(refxaxis);
	float folder = acos(xaxis * refxaxis);
	if ((xaxis - refxaxis) * normal > 0)
	{
		folder = -folder;
	}
	vect[4] = folder;
}


//将code还原为patch的位置、方向（code即降维编码）
void Coptim::decode(Vec4f& coord, Vec4f& normal,const double* const vect, const int id) const 
{
	decode(coord, vect, id);
	const int image = m_indexesT[id][0];

	const float angle1 = vect[1] * m_ascalesT[id];
	const float angle2 = vect[2] * m_ascalesT[id];

	const float fx = sin(angle1) * cos(angle2);
	const float fy = sin(angle2);
	const float fz = -cos(angle1) * cos(angle2);

	Vec3f ftmp = m_xaxes[image] * fx + m_yaxes[image] * fy + m_zaxes[image] * fz;
	normal = Vec4f(ftmp[0], ftmp[1], ftmp[2], 0.0f);
}

void Coptim::decode2(Vec4f& coord, Vec4f& normal, Vec4f& xaxis, Vec4f& negxaxis, Vec4f& yaxis, float& angle,float& folder,const double* const vect, const int id) const
{
	decode(coord, vect, id);
	const int image = m_indexesT[id][0];

	const float angle1 = vect[1] * m_ascalesT[id];
	const float angle2 = vect[2] * m_ascalesT[id];

	const float fx = sin(angle1) * cos(angle2);
	const float fy = sin(angle2);
	const float fz = -cos(angle1) * cos(angle2);

	Vec3f ftmp = m_xaxes[image] * fx + m_yaxes[image] * fy + m_zaxes[image] * fz;
	normal = Vec4f(ftmp[0], ftmp[1], ftmp[2], 0.0f);
	unitize(normal);

	Vec4f refxaxis, refyaxis;
	getPAxes(image, coord, normal, refxaxis, refyaxis);
	unitize(refxaxis);
	unitize(refyaxis);
	angle = vect[3] * m_ascalesT[id];
	yaxis = refyaxis - tan(vect[3] * m_ascalesT[id]) * refxaxis;
	xaxis = refxaxis + tan(vect[3] * m_ascalesT[id]) * refyaxis;
	negxaxis = -xaxis;
	unitize(yaxis);
	unitize(xaxis);
	unitize(negxaxis);
	folder = vect[4] * m_ascalesT[id];
	xaxis -= normal * tan(vect[4] * m_ascalesT[id]);
	negxaxis -= normal * tan(vect[4] * m_ascalesT[id]);
	unitize(xaxis);
	unitize(yaxis);
	unitize(negxaxis);
}

void Coptim::decode2(Vec4f& coord, Vec4f& normal, Vec4f& xaxis, Vec4f& negxaxis, Vec4f& yaxis, const double* const vect, const int id) const
{
	decode(coord, vect, id);
	const int image = m_indexesT[id][0];

	const float angle1 = vect[1] * m_ascalesT[id];
	const float angle2 = vect[2] * m_ascalesT[id];

	const float fx = sin(angle1) * cos(angle2);
	const float fy = sin(angle2);
	const float fz = -cos(angle1) * cos(angle2);

	Vec3f ftmp = m_xaxes[image] * fx + m_yaxes[image] * fy + m_zaxes[image] * fz;
	normal = Vec4f(ftmp[0], ftmp[1], ftmp[2], 0.0f);

	Vec4f refxaxis, refyaxis;
	getPAxes(image, coord, normal, refxaxis, refyaxis);
	unitize(refxaxis);
	unitize(refyaxis);
	yaxis = refyaxis - tan(vect[3] * m_ascalesT[id]) * refxaxis;
	xaxis = refxaxis + tan(vect[3] * m_ascalesT[id]) * refyaxis;
	negxaxis = -xaxis;
	unitize(yaxis);
	unitize(xaxis);
	unitize(negxaxis);
	xaxis -= normal * tan(vect[4] * m_ascalesT[id]);
	negxaxis -= normal * tan(vect[4] * m_ascalesT[id]);

}
void Coptim::decode(Vec4f& coord, const double* const vect, const int id) const {
	coord = m_centersT[id] + m_dscalesT[id] * vect[0] * m_raysT[id];
}

//robust为0

void Coptim::setINCCs(const Patch::Cpatch& patch, std::vector<float> & inccs,const std::vector<int>& indexes,const int id, const int robust) 
{
	const int index = indexes[0];

	Vec4f pxaxis, pyaxis;
	getPAxes(index, patch.m_coord, patch.m_normal, pxaxis, pyaxis);
// 
// 	Vec4f xaxis = patch.m_xaxis;
// 	Vec4f	negxaxis = patch.m_negxaxis;
// 	Vec4f	yaxis = patch.m_yaxis;
// 	Vec4f coord = patch.m_coord;
// 	const float xdis = norm(m_one->m_fm.m_pss.project(index, coord + xaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
// 	const float nxdis = norm(m_one->m_fm.m_pss.project(index, coord + negxaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
// 	const float ydis = norm(m_one->m_fm.m_pss.project(index, coord + yaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
// 	xaxis /= xdis;
// 	negxaxis /= nxdis;
// 	yaxis /= ydis;

	vector<vector<float> >& texs = m_texsT[id];

	const int size = (int)indexes.size();
	for (int i = 0; i < size; ++i) {
		const int flag = grabTex(patch.m_coord, pxaxis,pyaxis, patch.m_normal,
			indexes[i], m_fm.m_wsize, texs[i]);
		if (flag == 0) {
			normalize(texs[i]);
		}
	}

	inccs.resize(size);
	if (texs[0].empty()) {
		fill(inccs.begin(), inccs.end(), 2.0f);
		return;
	}

	for (int i = 0; i < size; ++i) {
		if (i == 0)
			inccs[i] = 0.0f;
		else if (!texs[i].empty()) {
			if (robust == 0)
				inccs[i] = 1.0f - dot(texs[0], texs[i]);
			else
				inccs[i] = robustincc(1.0f - dot(texs[0], texs[i]));
		}
		else
			inccs[i] = 2.0f;
	}
}
void Coptim::setINCCs2(const Patch::Cpatch& patch,
	std::vector<float> & inccs,
	const std::vector<int>& indexes,
	const int id, const int robust)
{
	const int index = indexes[0];

	// 	Vec4f pxaxis, pyaxis;
	// 	getPAxes(index, patch.m_coord, patch.m_normal, pxaxis, pyaxis);
	// 
	Vec4f xaxis = patch.m_xaxis;
	Vec4f	negxaxis = patch.m_negxaxis;
	Vec4f	yaxis = patch.m_yaxis;
	Vec4f coord = patch.m_coord;
	const float pscale = getUnit(index, coord);
	xaxis *= pscale;
	negxaxis *= pscale;
	yaxis *= pscale;

	const float xdis = norm(m_one->m_fm.m_pss.project(index, coord + xaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
	const float nxdis = norm(m_one->m_fm.m_pss.project(index, coord + negxaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
	const float ydis = norm(m_one->m_fm.m_pss.project(index, coord + yaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
	xaxis /= xdis;
	negxaxis /= nxdis;
	yaxis /= ydis;

	vector<vector<float> >& texs = m_texsT[id];

	const int size = (int)indexes.size();
	for (int i = 0; i < size; ++i) {
		const int flag = grabTex2(patch.m_coord, xaxis, negxaxis, yaxis, patch.m_normal,
			indexes[i], m_fm.m_wsize, texs[i]);
		if (flag == 0) {
			normalize(texs[i]);
		}
	}

	inccs.resize(size);
	if (texs[0].empty()) {
		fill(inccs.begin(), inccs.end(), 2.0f);
		return;
	}

	for (int i = 0; i < size; ++i) {
		if (i == 0)
			inccs[i] = 0.0f;
		else if (!texs[i].empty()) {
			if (robust == 0)
				inccs[i] = 1.0f - dot(texs[0], texs[i]);
			else
				inccs[i] = robustincc(1.0f - dot(texs[0], texs[i]));
		}
		else
			inccs[i] = 2.0f;
	}

}


void Coptim::setINCCs(const Patch::Cpatch& patch,
	std::vector<std::vector<float> >& inccs,
	const std::vector<int>& indexes,
	const int id, const int robust) 
{
	const int index = indexes[0];
	Vec4f pxaxis, pyaxis;
	getPAxes(index, patch.m_coord, patch.m_normal, pxaxis, pyaxis);

// 	Vec4f xaxis = patch.m_xaxis;
// 	Vec4f	negxaxis = patch.m_negxaxis;
// 	Vec4f	yaxis = patch.m_yaxis;
// 	Vec4f coord = patch.m_coord;
// 	const float xdis = norm(m_one->m_fm.m_pss.project(index, coord + xaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
// 	const float nxdis = norm(m_one->m_fm.m_pss.project(index, coord + negxaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
// 	const float ydis = norm(m_one->m_fm.m_pss.project(index, coord + yaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
// 	xaxis /= xdis;
// 	negxaxis /= nxdis;
// 	yaxis /= ydis;


	vector<vector<float> >& texs = m_texsT[id];

	const int size = (int)indexes.size();
	for (int i = 0; i < size; ++i) {
		const int flag = grabTex(patch.m_coord, pxaxis, pyaxis, patch.m_normal,
			indexes[i], m_fm.m_wsize, texs[i]);

		if (flag == 0)
			normalize(texs[i]);
	}

	inccs.resize(size);
	for (int i = 0; i < size; ++i)
		inccs[i].resize(size);

	for (int i = 0; i < size; ++i) {
		inccs[i][i] = 0.0f;
		for (int j = i + 1; j < size; ++j) {
			if (!texs[i].empty() && !texs[j].empty()) {
				if (robust == 0)
					inccs[j][i] = inccs[i][j] = 1.0f - dot(texs[i], texs[j]);
				else
					inccs[j][i] = inccs[i][j] = robustincc(1.0f - dot(texs[i], texs[j]));
			}
			else
				inccs[j][i] = inccs[i][j] = 2.0f;
		}
	}
}

void Coptim::setINCCs2(const Patch::Cpatch& patch,
	std::vector<std::vector<float> >& inccs,
	const std::vector<int>& indexes,
	const int id, const int robust)
{
	const int index = indexes[0];
	// 	Vec4f pxaxis, pyaxis;
	// 	getPAxes(index, patch.m_coord, patch.m_normal, pxaxis, pyaxis);

	Vec4f xaxis = patch.m_xaxis;
	Vec4f	negxaxis = patch.m_negxaxis;
	Vec4f	yaxis = patch.m_yaxis;
	Vec4f coord = patch.m_coord;
	const float pscale = getUnit(index, coord);
	xaxis *= pscale;
	negxaxis *= pscale;
	yaxis *= pscale;

	const float xdis = norm(m_one->m_fm.m_pss.project(index, coord + xaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
	const float nxdis = norm(m_one->m_fm.m_pss.project(index, coord + negxaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
	const float ydis = norm(m_one->m_fm.m_pss.project(index, coord + yaxis, m_one->m_fm.m_level) - m_one->m_fm.m_pss.project(index, coord, m_one->m_fm.m_level));
	xaxis /= xdis;
	negxaxis /= nxdis;
	yaxis /= ydis;


	vector<vector<float> >& texs = m_texsT[id];

	const int size = (int)indexes.size();
	for (int i = 0; i < size; ++i) {
		const int flag = grabTex2(patch.m_coord, xaxis, negxaxis, yaxis, patch.m_normal,
			indexes[i], m_fm.m_wsize, texs[i]);

		if (flag == 0)
			normalize(texs[i]);
	}

	inccs.resize(size);
	for (int i = 0; i < size; ++i)
		inccs[i].resize(size);

	for (int i = 0; i < size; ++i) {
		inccs[i][i] = 0.0f;
		for (int j = i + 1; j < size; ++j) {
			if (!texs[i].empty() && !texs[j].empty()) {
				if (robust == 0)
					inccs[j][i] = inccs[i][j] = 1.0f - dot(texs[i], texs[j]);
				else
					inccs[j][i] = inccs[i][j] = robustincc(1.0f - dot(texs[i], texs[j]));
			}
			else
				inccs[j][i] = inccs[i][j] = 2.0f;
		}
	}

}


int Coptim::grabSafe(const int index, const int size, const Vec3f& center, const Vec3f& dx, const Vec3f& dy, const int level) const 
{
	const int margin = size / 2;

	const Vec3f tl = center - dx * margin - dy * margin;
	const Vec3f tr = center + dx * margin - dy * margin;

	const Vec3f bl = center - dx * margin + dy * margin;
	const Vec3f br = center + dx * margin + dy * margin;

	const float minx = min(tl[0], min(tr[0], min(bl[0], br[0])));
	const float maxx = max(tl[0], max(tr[0], max(bl[0], br[0])));
	const float miny = min(tl[1], min(tr[1], min(bl[1], br[1])));
	const float maxy = max(tl[1], max(tr[1], max(bl[1], br[1])));

	// 1 should be enough
	const int margin2 = 3;
	// ??? may need to change if we change interpolation method
	if (minx < margin2 ||
		m_fm.m_pss.getWidth(index, level) - 1 - margin2 <= maxx ||
		miny < margin2 ||
		m_fm.m_pss.getHeight(index, level) - 1 - margin2 <= maxy)
		return 0;
	return 1;
}

int Coptim::grabSafe2(const int index, const int size, const Vec3f& center, const Vec3f& dx, const Vec3f& dnx, const Vec3f& dy, const int level) const
{
	const int margin = size / 2;

	const Vec3f tl = center + dnx * margin - dy * margin;
	const Vec3f tr = center + dx * margin - dy * margin;

	const Vec3f bl = center + dnx * margin + dy * margin;
	const Vec3f br = center + dx * margin + dy * margin;

	const float minx = min(tl[0], min(tr[0], min(bl[0], br[0])));
	const float maxx = max(tl[0], max(tr[0], max(bl[0], br[0])));
	const float miny = min(tl[1], min(tr[1], min(bl[1], br[1])));
	const float maxy = max(tl[1], max(tr[1], max(bl[1], br[1])));

	// 1 should be enough
	const int margin2 = 3;
	// ??? may need to change if we change interpolation method
	if (minx < margin2 ||
		m_fm.m_pss.getWidth(index, level) - 1 - margin2 <= maxx ||
		miny < margin2 ||
		m_fm.m_pss.getHeight(index, level) - 1 - margin2 <= maxy)
		return 0;
	return 1;

}
// My own optimisaton，2的指数（数组存取，快速计算）
float MyPow2(int x)
{
	const float answers[] = { 0.0625, 0.125, 0.25, 0.5, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 };

	return answers[x + 4];
}

static float Log2 = log(2.0f);

//过滤图片夹角过大的图，获取newlevel，判断在newlevel下是 patch投影是否越界，若不越界，则获取text
int Coptim::grabTex(const Vec4f& coord, const Vec4f& pxaxis, const Vec4f& pyaxis, const Vec4f& pzaxis, const int index, const int size,std::vector<float>& tex) const 
{
	

	tex.clear();

	Vec4f ray = m_fm.m_pss.m_photos[index].m_center - coord;
	unitize(ray);
	
	
	const float weight = max(0.0f, ray * pzaxis);

	//??????? 
	//if (weight < cos(m_fm.m_angleThreshold0))
	if (weight < cos(m_fm.m_angleThreshold0))
		return 1;

	const int margin = size / 2;

	Vec3f center = m_fm.m_pss.project(index, coord, m_fm.m_level);
	Vec3f dx = m_fm.m_pss.project(index, coord + pxaxis, m_fm.m_level) - center;
	Vec3f dy = m_fm.m_pss.project(index, coord + pyaxis, m_fm.m_level) - center;

	const float ratio = (norm(dx) + norm(dy)) / 2.0f;
	//int leveldif = (int)floor(log(ratio) / log(2.0f) + 0.5f);
	int leveldif = (int)floor(log(ratio) / Log2 + 0.5f);

	// Upper limit is 2
	leveldif = max(-m_fm.m_level, min(2, leveldif));

	//const float scale = pow(2.0f, (float)leveldif);

	const float scale = MyPow2(leveldif);
	const int newlevel = m_fm.m_level + leveldif;

	center /= scale;  dx /= scale;  dy /= scale;

// 	if (grabSafe(index, size, center, dx, dy, newlevel) == 0)
// 		return 1;

	Vec3f left = center - dx * margin - dy * margin;

	tex.resize(3 * size * size);
// 	float* texp = &tex[0] - 1;
// 	for (int y = 0; y < size; ++y) {
// 		//为什么不直接用vftmp += dy 啊！难道也是为了调试？！！！
// 		Vec3f vftmp = left;
// 		left += dy;
// 		for (int x = 0; x < size; ++x) {
// 			Vec3f color = m_fm.m_pss.getColor(index, vftmp[0], vftmp[1], newlevel);
// 			*(++texp) = color[0];
// 			*(++texp) = color[1];
// 			*(++texp) = color[2];
// 			vftmp += dx;
// 		}
// 	}

	//尝试采用真正的投影看看！！！
	for (int y = -margin; y <= margin;y++)
	{
		for (int x = -margin; x <= margin;x++)
		{
			int nTexIndex = (y + margin) * size * 3 + (x + margin) * 3;
			Vec4f pos3D = coord + y * pyaxis + x * pxaxis;
			Vec3f pos2D = m_fm.m_pss.project(index, pos3D, newlevel);
			if (pos2D[0] < margin || pos2D[0] >= m_fm.m_pss.getWidth(index, newlevel) - margin || pos2D[1] < margin || pos2D[1] >= m_fm.m_pss.getHeight(index, newlevel) - margin)
			{
				return 1;
			}

			Vec3f color = m_fm.m_pss.getColor(index, pos2D[0], pos2D[1], newlevel);
			tex[nTexIndex] = color[0];
			tex[nTexIndex + 1] = color[1];
			tex[nTexIndex + 2] = color[2];
		}
	}


	return 0;
}
int Coptim::grabTex2(const Vec4f& coord, const Vec4f& pxaxis, const Vec4f& pnegxaxis, const Vec4f& pyaxis, const Vec4f& pzaxis, const int index, const int size, std::vector<float>& tex) const
{
	tex.clear();

	Vec4f ray = m_fm.m_pss.m_photos[index].m_center - coord;
	unitize(ray);


	const float weight = max(0.0f, ray * pzaxis);

	//??????? 
	//if (weight < cos(m_fm.m_angleThreshold0))
	if (weight < cos(m_fm.m_angleThreshold1))
		return 1;

	const int margin = size / 2;

	Vec3f center = m_fm.m_pss.project(index, coord, m_fm.m_level);
	Vec3f dx = m_fm.m_pss.project(index, coord + pxaxis, m_fm.m_level) - center;
	Vec3f dy = m_fm.m_pss.project(index, coord + pyaxis, m_fm.m_level) - center;
	Vec3f dnx = m_fm.m_pss.project(index, coord + pnegxaxis, m_fm.m_level) - center;

	const float ratio = (norm(dx) + norm(dy)) / 2.0f;
	//int leveldif = (int)floor(log(ratio) / log(2.0f) + 0.5f);
	int leveldif = (int)floor(log(ratio) / Log2 + 0.5f);

	// Upper limit is 2
	leveldif = max(-m_fm.m_level, min(2, leveldif));

	//const float scale = pow(2.0f, (float)leveldif);

	const float scale = MyPow2(leveldif);
	const int newlevel = m_fm.m_level + leveldif;

	center /= scale;  dx /= scale;  dy /= scale; dnx /= scale;

	if (grabSafe2(index, size, center, dx, dnx,dy, newlevel) == 0)
		return 1;

// 	Vec3f left = center - dx * margin - dy * margin;
// 
// 	tex.resize(3 * size * size);
// 	float* texp = &tex[0] - 1;
// 	for (int y = 0; y < size; ++y) {
// 		//为什么不直接用vftmp += dy 啊！难道也是为了调试？！！！
// 		Vec3f vftmp = left;
// 		left += dy;
// 		for (int x = 0; x < size; ++x) {
// 			Vec3f color = m_fm.m_pss.getColor(index, vftmp[0], vftmp[1], newlevel);
// 			*(++texp) = color[0];
// 			*(++texp) = color[1];
// 			*(++texp) = color[2];
// 			vftmp += dx;
// 		}
// 	}


	//变量命名，我自己都醉了(其实text的顺序未必都要按照这个来，目前修改的话能不改动的尽量不改避免后面出错)
 	tex.resize(3 * size * size);
// 	int nsize = size / 2 + 1;
// 	for (int y = 0; y < nsize;y++)
// 	{
// 		for (int x = 0; x < nsize;x++)
// 		{
// 			int rt = nsize - y - 1;
// 			int rb = nsize + y - 1;
// 			int cl = nsize - x - 1;
// 			int cr = nsize + x - 1;
// 			if (x == 0 && y == 0)
// 			{
// 				Vec3f color = m_fm.m_pss.getColor(index, center[0], center[1], newlevel);
// 				tex[(rt * size + cl) * 3] = color[0];
// 				tex[(rt * size + cl) * 3 + 1] = color[1];
// 				tex[(rt * size + cl) * 3 + 2] = color[2];
// 				continue;
// 			}
// 
// 			Vec3f color;
// 			Vec3f lttmp, lbtmp, rttmp, rbtmp;
// 			lttmp = center + x * dnx - y * dy;
// 			lbtmp = center + x * dnx + y * dy;
// 			rttmp = center + x * dx - y * dy;
// 			rbtmp = center + x * dx + y * dy;
// 
// 			color = m_fm.m_pss.getColor(index, lttmp[0], lttmp[1], newlevel);
// 			tex[(rt * size + cl) * 3] = color[0];
// 			tex[(rt * size + cl) * 3 + 1] = color[1];
// 			tex[(rt * size + cl) * 3 + 2] = color[2];
// 
// 			color = m_fm.m_pss.getColor(index, lbtmp[0], lbtmp[1], newlevel);
// 			tex[(rb * size + cl) * 3] = color[0];
// 			tex[(rb * size + cl) * 3 + 1] = color[1];
// 			tex[(rb * size + cl) * 3 + 2] = color[2];
// 
// 			color = m_fm.m_pss.getColor(index, rttmp[0], rttmp[1], newlevel);
// 			tex[(rt * size + cr) * 3] = color[0];
// 			tex[(rt * size + cr) * 3 + 1] = color[1];
// 			tex[(rt * size + cr) * 3 + 2] = color[2];
// 
// 			color = m_fm.m_pss.getColor(index, rbtmp[0], rbtmp[1], newlevel);
// 			tex[(rb * size + cr) * 3] = color[0];
// 			tex[(rb * size + cr) * 3 + 1] = color[1];
// 			tex[(rb * size + cr) * 3 + 2] = color[2];
// 
// 
// 
// 		}
// 	}
	float* curTex = &tex[0];
	for (int y = -margin; y <= margin;y++)
	{
		for (int x = -margin; x <= margin;x++)
		{
			if (x < 0)
			{
				Vec4f pos3D = coord + y * pyaxis - x * pnegxaxis;
				Vec3f pos2D = m_fm.m_pss.project(index, pos3D, newlevel);
				Vec3f color = m_fm.m_pss.getColor(index, pos2D[0], pos2D[1], newlevel);
				*(curTex++) = color[0];
				*(curTex++) = color[1];
				*(curTex++) = color[2];
			}
			else
			{
				Vec4f pos3D = coord + y * pyaxis + x * pxaxis;
				Vec3f pos2D = m_fm.m_pss.project(index, pos3D, newlevel);
				Vec3f color = m_fm.m_pss.getColor(index, pos2D[0], pos2D[1], newlevel);
				*(curTex++) = color[0];
				*(curTex++) = color[1];
				*(curTex++) = color[2];
			}
		}
	}

	return 0;

}

//最后一位robust位 ，判断是否采用rubustincc
double Coptim::computeINCC(const Vec4f& coord, const Vec4f& normal,
	const std::vector<int>& indexes, const int id,
	const int robust) {
	if ((int)indexes.size() < 2)
		return 2.0;

	const int index = indexes[0];
	Vec4f pxaxis, pyaxis;
	getPAxes(index, coord, normal, pxaxis, pyaxis);

	return computeINCC(coord, normal, indexes, pxaxis, pyaxis, id, robust);
}

double Coptim::computeINCC(const Vec4f& coord, const Vec4f& normal,
	const std::vector<int>& indexes, const Vec4f& pxaxis,
	const Vec4f& pyaxis, const int id,
	const int robust) {
	if ((int)indexes.size() < 2)
		return 2.0;

	const int size = min(m_fm.m_tau, (int)indexes.size());
	vector<vector<float> >& texs = m_texsT[id];

	for (int i = 0; i < size; ++i) {
		int flag;
		flag = grabTex(coord, pxaxis, pyaxis, normal,
			indexes[i], m_fm.m_wsize, texs[i]);

		if (flag == 0)
			normalize(texs[i]);
	}

	if (texs[0].empty())
		return 2.0;

	double score = 0.0;

	// pure pairwise of reference based
#ifdef PMVS_PAIRNCC
	float totalweight = 0.0;
	for (int i = 0; i < size; ++i) {
		for (int j = i+1; j < size; ++j) {
			if (!texs[i].empty() && !texs[j].empty()) {
				const float ftmp = m_weightsT[id][i] * m_weightsT[id][j];
				totalweight += ftmp;
				if (robust)
					score += robustincc(1.0 - dot(texs[i], texs[j])) * ftmp;
				else
					score += (1.0 - dot(texs[i], texs[j])) * ftmp;
			}	
		}
	}

	if (totalweight == 0.0)
		score = 2.0;
	else
		score /= totalweight;
#else
	float totalweight = 0.0;
	for (int i = 1; i < size; ++i) {
		if (!texs[i].empty()) {
			totalweight += m_weightsT[id][i];
			if (robust)
				score += robustincc(1.0 - dot(texs[0], texs[i])) * m_weightsT[id][i];
			else
				score += (1.0 - dot(texs[0], texs[i])) * m_weightsT[id][i];
		}
	}
	if (totalweight == 0.0)
		score = 2.0;
	else
		score /= totalweight;
#endif  

	return score;
}

void Coptim::lfunc(double* p, double* hx, int m, int n, void* adata) {
	int iflag;
	m_one->func(n, m, p, hx, &iflag, adata);
}

void Coptim::func(int m, int n, double* x, double* fvec, int* iflag, void* arg) {
	const int id = *((int*)arg);
	double xs[3] = { x[0], x[1], x[2] };

	for (int i = 0; i < m; ++i)
		fvec[i] = 2.0;

	Vec4f coord, normal;
	decode(coord, normal, xs, id);

	const int index = m_indexesT[id][0];
	Vec4f pxaxis, pyaxis;
	getPAxes(index, coord, normal, pxaxis, pyaxis);

	const int size = min(m_fm.m_tau, (int)m_indexesT[id].size());

	for (int i = 0; i < size; ++i) {
		int flag;
		flag = grabTex(coord, pxaxis, pyaxis, normal, m_indexesT[id][i],
			m_fm.m_wsize, m_texsT[id][i]);

		if (flag == 0)
			normalize(m_texsT[id][i]);
	}

	int count = -1;
	for (int i = 0; i < size; ++i) {
		for (int j = i + 1; j < size; ++j) {
			count++;
			if (m_texsT[id][i].empty() || m_texsT[id][j].empty())
				continue;

			fvec[count] = robustincc(1.0 - dot(m_texsT[id][i], m_texsT[id][j]));
		}
	}
}

// Normalize only scale for each image
void Coptim::normalize(std::vector<std::vector<float> >& texs,
	const int size) {
	// compute average rgb
	Vec3f ave;
	int denom = 0;

	vector<Vec3f> rgbs;
	rgbs.resize(size);
	for (int i = 0; i < size; ++i) {
		if (texs[i].empty())
			continue;

		int count = 0;
		while (count < (int)texs[i].size()) {
			rgbs[i][0] += texs[i][count++];
			rgbs[i][1] += texs[i][count++];
			rgbs[i][2] += texs[i][count++];
		}
		rgbs[i] /= (int)texs[i].size() / 3;

		ave += rgbs[i];
		++denom;
	}

	// overall average
	if (denom == 0)
		return;

	ave /= denom;

	// Scale all the colors
	for (int i = 0; i < size; ++i) {
		if (texs[i].empty())
			continue;
		int count = 0;
		// compute scale
		Vec3f scale;
		for (int j = 0; j < 3; ++j)
		if (rgbs[i][j] != 0.0f)
			scale[j] = ave[j] / rgbs[i][j];

		while (count < (int)texs[i].size()) {
			texs[i][count++] *= scale[0];
			texs[i][count++] *= scale[1];
			texs[i][count++] *= scale[2];
		}
	}
}

void Coptim::normalize(std::vector<float>& tex) {
	const int size = (int)tex.size();
	const int size3 = size / 3;
	Vec3f ave;

	float* texp = &tex[0] - 1;
	for (int i = 0; i < size3; ++i) {
		ave[0] += *(++texp);
		ave[1] += *(++texp);
		ave[2] += *(++texp);
	}

	ave /= size3;

	float ave2 = 0.0;
	texp = &tex[0] - 1;
	for (int i = 0; i < size3; ++i) {
		const float f0 = ave[0] - *(++texp);
		const float f1 = ave[1] - *(++texp);
		const float f2 = ave[2] - *(++texp);

		ave2 += f0 * f0 + f1 * f1 + f2 * f2;
	}

	ave2 = sqrt(ave2 / size);

	if (ave2 == 0.0f)
		ave2 = 1.0f;

	texp = &tex[0] - 1;
	for (int i = 0; i < size3; ++i) {
		*(++texp) -= ave[0];    *texp /= ave2;
		*(++texp) -= ave[1];    *texp /= ave2;
		*(++texp) -= ave[2];    *texp /= ave2;
	}
}

float Coptim::dot(const std::vector<float>& tex0,
	const std::vector<float>& tex1) const{
#ifndef PMVS_WNCC
	// Pierre Moulon (use classic access to array, windows STL do not like begin()-1)
	const int size = (int)tex0.size();
	float ans = 0.0f;
	for (int i = 0; i < size; ++i) {
		ans += tex0[i] * tex1[i];
	}
	return ans / size;
#else
	const int size = (int)tex0.size();
	vector<float>::const_iterator i0 = tex0.begin();
	vector<float>::const_iterator i1 = tex1.begin();
	float ans = 0.0f;
	for (int i = 0; i < size; ++i, ++i0, ++i1) {
		ans += (*i0) * (*i1) * m_template[i];
	}
	return ans;
#endif
}

float Coptim::ssd(const std::vector<float>& tex0,
	const std::vector<float>& tex1) const{
	const float scale = 0.01;

#ifndef PMVS_WNCC
	// Pierre Moulon (use classic access to array, windows STL do not like begin()-1)
	const int size = (int)tex0.size();
	float ans = 0.0f;
	for (int i = 0; i < size; ++i) {
		ans += fabs(tex0[i] - tex1[i]);
	}

	return scale * ans / size;
#else
	const int size = (int)tex0.size();
	vector<float>::const_iterator i0 = tex0.begin();
	vector<float>::const_iterator i1 = tex1.begin();
	float ans = 0.0f;
	for (int i = 0; i < size; ++i, ++i0, ++i1) {
		const float ftmp = fabs((*i0) - (*i1));
		//ans += (*i0) * (*i1) * m_template[i];
		ans += ftmp * m_template[i];
	}
	return scale * ans;
#endif
}


float Coptim::getUnit(const int index, const Vec4f& coord) const {
	const float fz = norm(coord - m_fm.m_pss.m_photos[index].m_center);
	const float ftmp = m_ipscales[index];
	if (ftmp == 0.0)
		return 1.0;

	return 2.0 * fz * (0x0001 << m_fm.m_level) / ftmp;
}

// get x and y axis to collect textures given reference image and normal，index表示reference图片id
void Coptim::getPAxes(const int index, const Vec4f& coord, const Vec4f& normal,Vec4f& pxaxis, Vec4f& pyaxis) const
   {
	// yasu changed here for fpmvs
	const float pscale = getUnit(index, coord);

	Vec3f normal3(normal[0], normal[1], normal[2]);
	Vec3f yaxis3 = cross(normal3, m_xaxes[index]);
	unitize(yaxis3);
	Vec3f xaxis3 = cross(yaxis3, normal3);
	pxaxis[0] = xaxis3[0];  pxaxis[1] = xaxis3[1];  pxaxis[2] = xaxis3[2];  pxaxis[3] = 0.0;
	pyaxis[0] = yaxis3[0];  pyaxis[1] = yaxis3[1];  pyaxis[2] = yaxis3[2];  pyaxis[3] = 0.0;

	pxaxis *= pscale;
	pyaxis *= pscale;

	//特么傻吊，前面getUnit等 level缩放 步骤全都没毛用
	const float xdis = norm(m_fm.m_pss.project(index, coord + pxaxis, m_fm.m_level) -
		m_fm.m_pss.project(index, coord, m_fm.m_level));
	const float ydis = norm(m_fm.m_pss.project(index, coord + pyaxis, m_fm.m_level) -
		m_fm.m_pss.project(index, coord, m_fm.m_level));

	
	pxaxis /= xdis;
	pyaxis /= ydis;
}

void Coptim::setWeightsT(const Patch::Cpatch& patch, const int id) {
	computeUnits(patch, m_weightsT[id]);
	for (int i = 1; i < (int)m_weightsT[id].size(); ++i)
		m_weightsT[id][i] = min(1.0f, m_weightsT[id][0] / m_weightsT[id][i]);
	m_weightsT[id][0] = 1.0f;
}

//若成功则返回1，否则返回0(适用于右手坐标系)
int Coptim::getRotateAngle(const Vec4f& xaxis1, const Vec4f& yaxis1, const Vec4f& zaxis1, const Vec4f& xaxis2, const Vec4f& yaxis2, const Vec4f& zaxis2, float& anglex, float& angley, float &anglez)
{
	//判断两个坐标系是否为直角坐标系，是否都为右手系
	if (xaxis1 * yaxis1 || yaxis1 * zaxis1 || zaxis1 * xaxis1 || xaxis2 * yaxis2 || yaxis2 * zaxis2 || zaxis2 * xaxis2)
	{
		return 0;
	}
	if ((cross(xaxis1, yaxis1) * zaxis1) * (cross(xaxis2, yaxis2) * zaxis2) < 0)
	{
		//坐标系种类不同
		return 0;
	}

	//还是这种命名短一点
	Vec4f x1, x2, y1, y2, z1, z2;
	x1 = xaxis1;
	x2 = xaxis2;
	y1 = yaxis1;
	y2 = yaxis2;
	z1 = yaxis1;
	z2 = yaxis2;
	x1[3] = 0; x2[3] = 0; y1[3] = 0; y2[3] = 0; z1[3] = 0; z2[3] = 0;
	unitize(x1);
	unitize(y1);
	unitize(z1);
	unitize(x2);
	unitize(y2);
	unitize(z2);

	float dzx = z2 * x1;
	float dzy = z2 * y1;
	float dzz = z2 * z1;
	Vec4f z2yz = dzy * y1 + dzz * z1;

	anglex = acos(dzz / norm(z2yz));
	if (cross(z1,z2yz) * x1 < 0)
	{
		anglex = -anglex;
	}
	angley = asin(dzy);
	unitize(z2yz);
	Vec4f curX = x1 * cos(angley) - dzx * z2yz;
	anglez = acos(curX * x2);
	if (cross(curX, x2) * z2 < 0)
	{
		anglez = -anglez;
	}

}

void Coptim::RotateWithAngle(const float& anglex, const float& angley, const float &anglez, const Vec4f& xaxis1, const Vec4f& yaxis1, const Vec4f& zaxis1, Vec4f& xaxis2, Vec4f& yaxis2, Vec4f& zaxis2)
{
	for (int i = 0; i < 4; i++)
	{
		xaxis2[i] = 0;
		yaxis2[i] = 0;
		zaxis2[i] = 0;
	}
	vector<vector<float>> RotateMat;
	getRotateMetrix(anglex, angley, anglez, RotateMat);
	for (int i = 0; i < 3;i++)
	{
		for (int j = 0; j < 3;j++)
		{
			xaxis2[i] += xaxis1[j] * RotateMat[i][j];
			yaxis2[i] += yaxis1[j] * RotateMat[i][j];
			zaxis2[i] += zaxis1[j] * RotateMat[i][j];
		}
	}
}

void Coptim::getRotateMetrix(const float& anglex, const float& angley, const float &anglez, vector<vector<float>>& matR)
{
	vector<vector<float>> RotX(3, vector<float>(3,0));
	vector<vector<float>> RotY(3, vector<float>(3, 0));
	vector<vector<float>> RotZ(3, vector<float>(3, 0));
	RotX[0][0] = 1;
	RotX[1][1] = cos(anglex);
	RotX[2][2] = cos(anglex);
	RotX[1][2] = -sin(anglex);
	RotX[2][1] = sin(anglex);

	RotY[1][1] = 1;
	RotY[0][0] = cos(angley);
	RotY[2][2] = cos(angley);
	RotY[0][2] = sin(angley);
	RotY[2][0] = -sin(angley);

	RotZ[2][2] = 1;
	RotZ[0][0] = cos(anglez);
	RotZ[1][1] = cos(anglez);
	RotZ[0][1] = -sin(anglez);
	RotZ[1][0] = sin(anglez);

	matR = DotMat(DotMat(RotX, RotY), RotZ);
}

vector<vector<float>> Coptim::DotMat(const vector<vector<float>>& A, const vector<vector<float>>& B)
{
	if (!A.size() || !B.size())
	{
		return vector<vector<float>>(0);
	}
	if (A[0].size() != B.size())
	{
		return vector<vector<float>>(0);
	}
	vector<vector<float>> C(A.size(), vector<float>(B[0].size(), 0));
	for (int i = 0; i < A.size();i++)
	{
		for (int j = 0; j < B[0].size();j++)
		{
			for (int k = 0; k < B.size();k++)
			{
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}
	return C;
}


