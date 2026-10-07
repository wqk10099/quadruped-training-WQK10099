#include "simulator.hpp"
#include<stdexcept>
#include <filesystem>
MuJoCoSimulator:: MuJoCoSimulator(const std::string& scene_path)
{
auto absolute_scene_path=std::filesystem::absolute(scene_path);//将相对路径转换为绝对路径
char error[1000]={};    //预留的错误缓冲区
model_=mj_loadXML(
    absolute_scene_path.string().c_str(), //C风格字符串的XML文件路径
    nullptr,            //不使用虚拟文件系统
    error,              //出错信息写入这里
    sizeof(error)       //错误缓冲区大小
);
if(model_==nullptr)     //检查返回的模型指针
{
    throw std::runtime_error(error);
}
data_=mj_makeData(model_);
if(data_==nullptr)     //检查返回的data指针
{
    mj_deleteModel(model_);
    model_=nullptr;
    throw std::runtime_error("data指针创建出错");
}
}
MuJoCoSimulator::~MuJoCoSimulator()
{
    mj_deleteData(data_);
    data_=nullptr;
    mj_deleteModel(model_);
    model_=nullptr;
}
double MuJoCoSimulator::timestep() const
{
    return model_->opt.timestep;
}
int MuJoCoSimulator::nq() const
{
    return model_->nq;
}
int MuJoCoSimulator::nv() const
{
    return model_->nv;
}
int MuJoCoSimulator::nu() const
{
    return model_->nu;
}
void MuJoCoSimulator::step()
{
    mj_step(model_,data_);
}
void MuJoCoSimulator::reset()
{
    mj_resetData(model_,data_);
}
void MuJoCoSimulator::reset_to_keyframe(const std::string& name)
{
    int keyframe_index=mj_name2id(model_,mjOBJ_KEY,name.c_str());
    if(keyframe_index<0)
    {
        throw std::runtime_error("找不到指定的关键帧");
    }
    mj_resetDataKeyframe(model_,data_,keyframe_index);
}
//由于model_和data_是私有成员变量，外部无法直接访问它们，因此提供了两个公有成员函数来获取它们的指针
const mjModel* MuJoCoSimulator::model() const
{
    return model_;
}
mjData* MuJoCoSimulator::data() const
{
    return data_;
}
