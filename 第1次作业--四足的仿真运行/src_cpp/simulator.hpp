#pragma once
#include<cstdio>
#include<mujoco/mujoco.h>
#include<iostream>
#include<cmath>
#include<string>

class MuJoCoSimulator
{
public:
explicit MuJoCoSimulator(const std::string& scene_path);//构造函数声明
~MuJoCoSimulator();//析构函数声明
//禁止通过已有对象复制出新的对象
MuJoCoSimulator(const MuJoCoSimulator&)=delete;
//禁止把一个对象赋值给另一个对象
MuJoCoSimulator&operator=(const MuJoCoSimulator&)=delete;
double timestep() const;
int nq() const;
int nu() const;
int nv() const;
void step();
void reset();
void reset_to_keyframe(const std::string& name);
const mjModel* model() const;
mjData* data() const;
private:
mjModel* model_=nullptr;
mjData* data_=nullptr;
};