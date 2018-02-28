#ifndef MYALGORITHMS_H
#define MYALGORITHMS_H

#include <stdlib.h>
#include <iostream>
#include <map>
#include <vector>
#include <assert.h>

using namespace std;

float CalChiSquareDis(vector<float> vA, vector<float> vB);
float aaa();

class MyNode
{
public:
	MyNode(int i) :next(nullptr),Index(i){};
	~MyNode(){};
	int Index;
	MyNode* next;

	
protected:
	//dynamic_cast ��Ҫ�麯������
// 	virtual void what();

private:

};


//T��ʾ��ʹ�ýڵ������(����̳���MyNode)
template<typename T>
class MyGraph
{
public:
	MyGraph(int size);
	~MyGraph();

	//��ӽڵ�
	bool addNode();

	//����ҵ������ߣ��򷵻� true �Լ��ڵ�λ��(i��j)�����򷵻�false�Լ�����ڵ�ָ���λ��
	pair<bool, T**> FindEdge(int i,int j);

	//��ӱ�
	bool AddEdge(int i, int j);

	//ɾ����
	bool DelEdge(int i, int j);

	void aaa();


private:


public:
	vector<T*> nodes;
	int nSize;
};


template<typename T>
inline MyGraph<T>::MyGraph(int size)
{
	//�ж�T�Ƿ� Ϊmynode ���� �̳���
	T* a = new T(0);
	assert(dynamic_cast<MyNode*>(a));

	for (int i = 0; i < size; i++)
	{
		T* cur = new T(i);
		nodes.push_back(cur);
	}

	nSize = size;
}


template<typename T>
inline MyGraph<T>::~MyGraph()
{
	for (int i = 0; i < nodes.size();i++)
	{
		if (nodes[i])
		{
			MyNode* mhead = dynamic_cast<MyNode*>(nodes[i]);
			while (mhead->next)
			{
				MyNode* curNode = dynamic_cast<MyNode*>(mhead->next);
				mhead->next = curNode->next;
				delete curNode;				
			}
		}
	}
}


template<typename T>
bool MyGraph<T>::addNode()
{
	nodes.push_back(new T(nSize));
	nSize++;
	return true;
}

template<typename T>
pair<bool, T**> MyGraph<T>::FindEdge(int i, int j)
{
	if (i >= nSize || j >= nSize || i < 0 || j < 0)
	{
		return pair<bool, T**>(false, NULL);
	}
	else
	{
		MyNode** curNode = &(dynamic_cast<MyNode*>(nodes[i]));
		while (*curNode != NULL)
		{
			if ((*curNode)->Index == j)
			{
				return pair<bool, T**>(true, curNode);
			}
			curNode = &((*curNode)->next);
		}
		return pair<bool, T**>(false, curNode);
	}
}


template<typename T>
bool MyGraph<T>::AddEdge(int i, int j)
{
	if (i >= nSize || j >= nSize || i < 0 || j < 0)
	{
		return false;
	}
	pair<bool, T**> result = FindEdge(i, j);
	if (result.first)
	{
		return true;
	}
	else
	{
		*(result.second) = new T(j);
		return true;
	}


}


template<typename T>
bool MyGraph<T>::DelEdge(int i, int j)
{
	if (i >= nSize || j >= nSize || i < 0 || j < 0)
	{
		return false;
	}
	pair<bool, T**> result = FindEdge(i, j);
	if (!result.first)
	{
		return true;
	}
	else
	{
		MyNode* curNode = dynamic_cast<MyNode*>(*(result.second));
		*(result.second) = curNode->next;
		delete curNode;
		return true;
	}
}





#endif // !MYALGORITHMS_H
